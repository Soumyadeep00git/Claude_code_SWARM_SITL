"""Mission-dispatch architecture for the follower drone.

Missions:
    IDLE     — pre-takeoff, waiting for GPS/leader
    TAKEOFF  — climbing to target altitude
    HOVER    — hold position (default after takeoff)
    FOLLOW   — guidance loop with leader
    RTL      — return to launch (terminal)
    LAND     — land at current position (terminal)
    KILL     — force disarm (terminal)

Main loop (called from follower_main.py):
    1. check_status()    — GPS, battery, heartbeat snapshot
    2. get_mission()     — THE ONLY place transitions happen
    3. execute_mission() — dispatch to per-mission handler (pure side-effects)

All transitions flow through get_mission(). Handlers never mutate
current_mission — they only set flags that get_mission reads next tick.
"""

import logging
import time
from dataclasses import dataclass, field
from enum import Enum

from docker_sim.config import CONTROL_HZ, TAKEOFF_ALT_M
from docker_sim.mavlink_conn import MavlinkConn
from docker_sim.mavlink_bridge import (
    MavlinkBridge,
    CMD_RTL, CMD_LAND, CMD_KILL, CMD_FOLLOW, CMD_HOVER, CMD_TAKEOFF,
)
from docker_sim.takeoff import TakeoffManager

from guidance_lib import GuidanceConfig
from failsafe_lib import FailsafeConfig, FailsafeState, compute_failsafe

log = logging.getLogger("follower")


# ═══════════════════════════════════════════════════════════════
# Mission enum
# ═══════════════════════════════════════════════════════════════

class Mission(Enum):
    IDLE    = "IDLE"
    TAKEOFF = "TAKEOFF"
    HOVER   = "HOVER"
    FOLLOW  = "FOLLOW"
    RTL     = "RTL"
    LAND    = "LAND"
    KILL    = "KILL"


# ═══════════════════════════════════════════════════════════════
# Data structures
# ═══════════════════════════════════════════════════════════════

@dataclass
class FlightStatus:
    """Snapshot of drone state, rebuilt each tick."""
    pos: dict | None        = None   # {lat, lon, alt, vx, vy, vz, heading}
    heartbeat: dict | None  = None   # {mode, armed}
    gps_valid: bool         = False
    leader: dict | None     = None   # peer state from bridge
    leader_fresh: bool      = False


@dataclass
class MissionContext:
    """Long-lived objects shared across missions."""
    conn: MavlinkConn
    bridge: MavlinkBridge
    cfg: GuidanceConfig
    fs_cfg: FailsafeConfig
    fs_state: FailsafeState
    ctrl: object = None              # FollowerController, created after takeoff
    takeoff: TakeoffManager = None   # created at TAKEOFF enter
    leader_id: int = None            # drone_id of assigned leader (from hierarchy)

    # Mission state
    current_mission: Mission = Mission.IDLE
    no_leader_count: int = 0
    last_guidance_mode: str = ""
    rtl_sent: bool = False
    has_gps: bool = False
    has_leader: bool = False
    wants_follow: bool = False   # True = user commanded Pursuit, auto-resume after failsafe clears
    is_airborne: bool = False    # True once TAKEOFF completes (guards FOLLOW)


# ═══════════════════════════════════════════════════════════════
# 1. check_status
# ═══════════════════════════════════════════════════════════════

def check_status(ctx: MissionContext) -> FlightStatus:
    """Read own state + leader state, return a FlightStatus snapshot."""
    data = ctx.conn.drain_latest()
    pos = data.get('position')
    hb = data.get('heartbeat')

    if pos:
        ctx.bridge.update_own_state(pos)
        if ctx.ctrl is not None:
            ctx.ctrl.update_own_state(pos)

    leader = ctx.bridge.get_peer_state(ctx.leader_id)

    return FlightStatus(
        pos=pos,
        heartbeat=hb,
        gps_valid=bool(pos and pos['lat'] != 0),
        leader=leader,
        leader_fresh=leader is not None,
    )


# ═══════════════════════════════════════════════════════════════
# 2. get_mission — THE ONLY place transitions happen
# ═══════════════════════════════════════════════════════════════

_RC_CMD_MAP = {
    CMD_TAKEOFF: Mission.TAKEOFF,
    CMD_FOLLOW:  Mission.FOLLOW,
    CMD_HOVER:   Mission.HOVER,
    CMD_RTL:     Mission.RTL,
    CMD_LAND:    Mission.LAND,
    CMD_KILL:    Mission.KILL,
}


def get_mission(status: FlightStatus, ctx: MissionContext) -> Mission:
    """Determine the mission for this tick.

    Priority (highest first):
        1. RC commands (override anything except physical constraints)
        2. Global failsafe (GPS deadman, follower geofence, altitude ceiling)
        3. Auto-promotions (IDLE→TAKEOFF, TAKEOFF→HOVER, FOLLOW leader-lost→RTL)
        4. Current mission continues
    """
    mission = ctx.current_mission

    # ── RC commands (drain all, last wins) ──
    rc_mission = None
    while True:
        gcs_cmd = ctx.bridge.get_pending_command()
        if gcs_cmd is None:
            break
        mapped = _RC_CMD_MAP.get(gcs_cmd['cmd'])
        if mapped is not None:
            rc_mission = mapped

    if rc_mission is not None:
        # Physical constraints
        if rc_mission == Mission.FOLLOW and not status.gps_valid:
            log.warning("RC: FOLLOW denied — no GPS, staying %s", mission.value)
        elif rc_mission == Mission.FOLLOW and not ctx.is_airborne:
            log.warning("RC: FOLLOW denied — not airborne yet (currently %s)", mission.value)
        elif rc_mission == Mission.TAKEOFF and mission != Mission.IDLE:
            log.warning("RC: TAKEOFF denied — not in IDLE (currently %s)", mission.value)
        else:
            mission = rc_mission
            # Track user intent for auto-resume
            if rc_mission == Mission.FOLLOW:
                ctx.wants_follow = True
            elif rc_mission in (Mission.HOVER, Mission.RTL, Mission.LAND, Mission.KILL):
                ctx.wants_follow = False

    # ── Global failsafe (runs for airborne missions only) ──
    # TAKEOFF excluded — TakeoffManager has its own 120s timeout and
    # intermittent position data during pre-arm/arm causes false NO_OWN_GPS.
    failsafe_clear = True
    if mission in (Mission.HOVER, Mission.FOLLOW):
        # Use full leader checks if actively following OR hovering with intent
        # to resume (wants_follow). This prevents auto-resume from defeating
        # geofence/stale failsafes that caused the HOVER in the first place.
        check_leader = (mission == Mission.FOLLOW
                        or (mission == Mission.HOVER and ctx.wants_follow))
        if check_leader:
            fs = compute_failsafe(
                own_lat=status.pos['lat'] if status.pos else 0.0,
                own_lon=status.pos['lon'] if status.pos else 0.0,
                own_alt=status.pos['alt'] if status.pos else 0.0,
                own_gps_valid=status.gps_valid,
                peer_lat=status.leader['lat'] if status.leader else 0.0,
                peer_lon=status.leader['lon'] if status.leader else 0.0,
                peer_gps_valid=bool(status.leader and status.leader['lat'] != 0),
                leader_fresh=status.leader_fresh,
                in_catchup=(ctx.last_guidance_mode == 'CATCHUP'),
                cfg=ctx.fs_cfg,
                state=ctx.fs_state,
            )
        else:
            fs = compute_failsafe(
                own_lat=status.pos['lat'] if status.pos else 0.0,
                own_lon=status.pos['lon'] if status.pos else 0.0,
                own_alt=status.pos['alt'] if status.pos else 0.0,
                own_gps_valid=status.gps_valid,
                peer_lat=0.0,
                peer_lon=0.0,
                peer_gps_valid=False,
                leader_fresh=True,
                in_catchup=False,
                cfg=ctx.fs_cfg,
                state=ctx.fs_state,
            )

        if not fs['safe']:
            failsafe_clear = False
            log.warning("FAILSAFE: %s action=%s", fs['flags'], fs['action'])
            if fs['action'] == 'RTL':
                mission = Mission.RTL
            else:
                if mission not in (Mission.RTL, Mission.LAND, Mission.KILL):
                    mission = Mission.HOVER

    # ── Auto-promotions (only if RC/failsafe didn't override) ──
    # TAKEOFF requires explicit GCS CMD_TAKEOFF — no auto-promotion from IDLE.

    # Auto-resume FOLLOW only when failsafe is CLEAR this tick
    if (mission == Mission.HOVER and ctx.wants_follow
            and failsafe_clear
            and status.leader_fresh and status.gps_valid):
        log.info("Failsafe cleared — resuming FOLLOW")
        mission = Mission.FOLLOW

    if mission == Mission.TAKEOFF and ctx.takeoff and ctx.takeoff.complete:
        log.info("Airborne!")
        mission = Mission.HOVER
        ctx.is_airborne = True

    if mission == Mission.FOLLOW and ctx.no_leader_count > CONTROL_HZ * 30:
        log.warning("Leader lost for 30s — RTL")
        mission = Mission.RTL

    return mission


# ═══════════════════════════════════════════════════════════════
# 3. execute_mission — transition handling + dispatch
# ═══════════════════════════════════════════════════════════════

def execute_mission(mission: Mission, status: FlightStatus,
                    ctx: MissionContext) -> bool:
    """Handle mission transitions and dispatch. Returns False to stop loop."""
    old = ctx.current_mission

    # ── Transition ──
    if mission != old:
        _on_exit(old, ctx)
        _on_enter(mission, ctx)
        log.info("Mission: %s → %s", old.value, mission.value)
        ctx.current_mission = mission

    # ── Dispatch ──
    handlers = {
        Mission.IDLE:    do_idle,
        Mission.TAKEOFF: do_takeoff,
        Mission.HOVER:   do_hover,
        Mission.FOLLOW:  do_follow,
        Mission.RTL:     do_rtl,
        Mission.LAND:    do_land,
        Mission.KILL:    do_kill,
    }
    return handlers[mission](status, ctx)


def _on_exit(old: Mission, ctx: MissionContext):
    """Clean up when leaving a mission."""
    if old == Mission.FOLLOW:
        ctx.fs_state.stale_counter = 0
        ctx.fs_state.catchup_ticks = 0
        ctx.last_guidance_mode = ""
        if ctx.ctrl is not None:
            ctx.ctrl.reset()


def _on_enter(new: Mission, ctx: MissionContext):
    """Set up when entering a mission."""
    if new == Mission.TAKEOFF:
        ctx.takeoff = TakeoffManager(ctx.conn, TAKEOFF_ALT_M)
    elif new == Mission.FOLLOW:
        ctx.no_leader_count = 0
    elif new == Mission.RTL:
        if not ctx.rtl_sent:
            ctx.conn.set_mode("RTL")
            ctx.rtl_sent = True
    elif new == Mission.LAND:
        ctx.conn.set_mode("LAND")
    elif new == Mission.KILL:
        ctx.conn.force_disarm()


# ═══════════════════════════════════════════════════════════════
# Mission handlers — pure side-effects, never mutate current_mission
# ═══════════════════════════════════════════════════════════════

def do_idle(status: FlightStatus, ctx: MissionContext) -> bool:
    """Wait for GPS + leader detection. Sets flags for get_mission."""
    if status.gps_valid and not ctx.has_gps:
        log.info("GPS fix: (%.7f, %.7f)", status.pos['lat'], status.pos['lon'])
        ctx.has_gps = True

    if status.leader and not ctx.has_leader:
        log.info("Leader detected: (%.7f, %.7f)",
                 status.leader['lat'], status.leader['lon'])
        ctx.has_leader = True

    if ctx.has_gps and not ctx.has_leader:
        log.warning("No leader detected yet — proceeding anyway")

    return True


def do_takeoff(status: FlightStatus, ctx: MissionContext) -> bool:
    """Delegate to TakeoffManager. Sets takeoff.complete for get_mission."""
    if ctx.takeoff is None:
        ctx.takeoff = TakeoffManager(ctx.conn, TAKEOFF_ALT_M)

    ctx.takeoff.tick(
        has_gps=status.gps_valid,
        mode=status.heartbeat['mode'] if status.heartbeat else '',
        armed=status.heartbeat['armed'] if status.heartbeat else False,
        alt=status.pos['alt'] if status.pos else 0.0,
    )
    return True


def do_hover(status: FlightStatus, ctx: MissionContext) -> bool:
    """Hold position — zero velocity."""
    ctx.conn.send_velocity_ned(0, 0, 0)
    return True


def do_follow(status: FlightStatus, ctx: MissionContext) -> bool:
    """Full guidance loop with leader tracking. Sets flags for get_mission."""
    if status.leader_fresh:
        ctx.no_leader_count = 0
    else:
        ctx.no_leader_count += 1

    # Run guidance
    result = ctx.ctrl.tick(status.leader)

    if result is None:
        # Leader momentarily unavailable — hold position instead of drifting
        ctx.conn.send_velocity_ned(0, 0, 0)
        return True

    # Report guidance mode to bridge for telemetry
    _MODE_CODES = {'TRACKING': 1, 'CATCHUP': 2, 'EVASION': 3}
    ctx.bridge.set_guidance_mode(_MODE_CODES.get(result['mode'], 0))

    if result.get('emergency'):
        log.warning("GUIDANCE EMERGENCY: flags=%s mode=%s",
                    result['flags'], result['mode'])

    # Track guidance mode for next tick's failsafe
    ctx.last_guidance_mode = result['mode']

    return True


def do_rtl(status: FlightStatus, ctx: MissionContext) -> bool:
    """Wait for landing after RTL. Returns False when landed."""
    if status.pos and status.pos['alt'] < 1.0:
        log.info("Landed (RTL)")
        return False
    return True


def do_land(status: FlightStatus, ctx: MissionContext) -> bool:
    """Wait for landing. Returns False when landed."""
    if status.pos and status.pos['alt'] < 1.0:
        log.info("Landed (LAND)")
        return False
    return True


def do_kill(status: FlightStatus, ctx: MissionContext) -> bool:
    """Force disarm — return False immediately."""
    return False


# ═══════════════════════════════════════════════════════════════
# Tick sleep helper
# ═══════════════════════════════════════════════════════════════

def sleep_tick(tick_start: float):
    """Sleep for the remainder of a control tick."""
    elapsed = time.time() - tick_start
    sleep_time = (1.0 / CONTROL_HZ) - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)
