"""Unified drone entry point — role determined by hierarchy.

Phases (same for all drones):
    1. Launch SITL
    2. Connect pymavlink
    3. Start multi-peer bridge
    4. Wait for GPS
    5. Wait for GCS TAKEOFF command
    6. Execute takeoff
    7. Role-specific control loop:
        - Leader: GCS-controlled (WASD/HOVER)
        - Follower: Mission-dispatch (guidance loop)
    8. Shutdown
"""

import logging
import os
import signal
import sys
import time

from docker_sim.config import (
    DRONE_ID, DRONE_ROLE, CONTROL_HZ,
    PEER_LISTEN_PORT, PEER_TARGETS,
    GCS_HOST, GCS_TELEM_PORT, GCS_CMD_PORT,
    OUTPUT_DIR, PEER_STALE_TIMEOUT, LEADER_ID,
    TAKEOFF_ALT_M, HOME_LAT, HOME_LON,
    FOLLOW_OFFSET_N, FOLLOW_OFFSET_E, FOLLOW_OFFSET_D,
)
from docker_sim.sitl_launcher import SITLLauncher
from docker_sim.mavlink_conn import MavlinkConn
from docker_sim.takeoff import TakeoffManager
from docker_sim.mavlink_bridge import (
    MavlinkBridge, CMD_NAMES,
    CMD_RTL, CMD_LAND, CMD_KILL, CMD_HOVER,
    CMD_WASD, CMD_TAKEOFF, CMD_FOLLOW,
)

_TAG = f"DRONE-{DRONE_ID}-{DRONE_ROLE.upper()}"
logging.basicConfig(
    level=logging.INFO,
    format=f"%(asctime)s [{_TAG}] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(f"drone-{DRONE_ID}")

_running = True


def _signal_handler(sig, frame):
    global _running
    log.info("Interrupted -- shutting down...")
    _running = False


# ═══════════════════════════════════════════════════════════════
# Shared startup (all roles)
# ═══════════════════════════════════════════════════════════════

def _startup(launcher, bridge):
    """Phases 1-6: Launch SITL, connect, GPS, wait for takeoff, takeoff."""
    global _running

    # Phase 1: Launch SITL
    log.info("=" * 50)
    log.info("Phase 1: Launching SITL")
    log.info("=" * 50)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    launcher.launch()
    if not launcher.wait_ready():
        raise TimeoutError("SITL port not reachable")

    # Phase 2: Connect
    log.info("=" * 50)
    log.info("Phase 2: Connecting pymavlink")
    log.info("=" * 50)
    conn = MavlinkConn(timeout=60)

    # Phase 3: Bridge + GPS
    log.info("=" * 50)
    log.info("Phase 3: Waiting for GPS fix")
    log.info("=" * 50)
    bridge.start()

    gps_deadline = time.time() + 90.0
    while _running and time.time() < gps_deadline:
        data = conn.drain_latest()
        pos = data.get('position')
        if pos and pos['lat'] != 0:
            log.info("GPS fix: (%.7f, %.7f)", pos['lat'], pos['lon'])
            break
        time.sleep(0.5)
    else:
        if _running:
            raise TimeoutError("GPS fix timeout")

    # Phase 4: Wait for GCS TAKEOFF command
    log.info("=" * 50)
    log.info("Phase 4: Ready — waiting for GCS TAKEOFF command")
    log.info("=" * 50)

    takeoff_requested = False
    while _running and not takeoff_requested:
        data = conn.drain_latest()
        pos = data.get('position')
        if pos:
            bridge.update_own_state(pos)

        while True:
            gcs_cmd = bridge.get_pending_command()
            if gcs_cmd is None:
                break
            c = gcs_cmd['cmd']
            if c == CMD_TAKEOFF:
                log.info("GCS: TAKEOFF received")
                takeoff_requested = True
                break
            elif c == CMD_KILL:
                log.info("GCS: KILL (pre-takeoff)")
                conn.force_disarm()
                _running = False
                return conn
            else:
                log.info("GCS: Ignoring %s (waiting for TAKEOFF)",
                         CMD_NAMES.get(c, f"?{c}"))

        time.sleep(1.0 / CONTROL_HZ)

    # Phase 5: Execute takeoff
    if _running and takeoff_requested:
        log.info("Phase 5: Takeoff to %.0fm", TAKEOFF_ALT_M)
        takeoff = TakeoffManager(conn, TAKEOFF_ALT_M)

        while _running:
            data = conn.drain_latest()
            pos = data.get('position')
            hb = data.get('heartbeat')

            if pos:
                bridge.update_own_state(pos)

            done = takeoff.tick(
                has_gps=bool(pos and pos['lat'] != 0),
                mode=hb['mode'] if hb else '',
                armed=hb['armed'] if hb else False,
                alt=pos['alt'] if pos else 0.0,
            )
            if done:
                break
            time.sleep(1.0 / CONTROL_HZ)

        log.info("Airborne!")

    return conn


# ═══════════════════════════════════════════════════════════════
# Leader control loop
# ═══════════════════════════════════════════════════════════════

def _leader_loop(conn, bridge):
    """Leader: GCS-controlled WASD/HOVER."""
    global _running

    log.info("=" * 50)
    log.info("Leader control loop (HOVER)")
    log.info("=" * 50)

    mode = "HOVER"
    wasd_vn = 0.0
    wasd_ve = 0.0

    while _running:
        tick_start = time.time()

        data = conn.drain_latest()
        pos = data.get('position')
        if pos:
            bridge.update_own_state(pos)

        # Process GCS commands (drain all pending)
        while True:
            gcs_cmd = bridge.get_pending_command()
            if gcs_cmd is None:
                break
            c = gcs_cmd['cmd']

            if c == CMD_RTL:
                log.info("GCS: RTL")
                conn.set_mode("RTL")
                _running = False
                break
            elif c == CMD_LAND:
                log.info("GCS: LAND")
                conn.set_mode("LAND")
                _running = False
                break
            elif c == CMD_KILL:
                log.info("GCS: KILL")
                conn.force_disarm()
                _running = False
                break
            elif c == CMD_HOVER:
                if mode != "HOVER":
                    log.info("GCS: HOVER")
                mode = "HOVER"
                wasd_vn = 0.0
                wasd_ve = 0.0
            elif c == CMD_WASD:
                wasd_vn = gcs_cmd.get('vn', 0.0)
                wasd_ve = gcs_cmd.get('ve', 0.0)
                mode = "WASD" if (wasd_vn != 0 or wasd_ve != 0) else "HOVER"
            elif c == CMD_TAKEOFF:
                log.info("GCS: TAKEOFF ignored (already airborne)")
            else:
                log.warning("Ignoring unexpected command: %s",
                            CMD_NAMES.get(c, f"?{c}"))

        if not _running:
            break

        if mode == "HOVER":
            conn.send_velocity_ned(0, 0, 0)
        elif mode == "WASD":
            conn.send_velocity_ned(wasd_vn, wasd_ve, 0)

        _sleep_tick(tick_start)


# ═══════════════════════════════════════════════════════════════
# Follower control loop
# ═══════════════════════════════════════════════════════════════

def _follower_loop(conn, bridge):
    """Follower: mission-dispatch loop calling guidance_lib + failsafe_lib."""
    global _running

    from guidance_lib import GuidanceConfig
    from failsafe_lib import FailsafeState, load_config as load_failsafe_config
    from docker_sim.follower_controller import FollowerController
    from docker_sim.follower_missions import (
        MissionContext, check_status, get_mission, execute_mission, sleep_tick,
    )

    log.info("=" * 50)
    log.info("Follower control loop (mission-dispatch)")
    log.info("  leader_id=%s  offset=(%.1f, %.1f, %.1f)",
             LEADER_ID, FOLLOW_OFFSET_N, FOLLOW_OFFSET_E, FOLLOW_OFFSET_D)
    log.info("=" * 50)

    # Load failsafe config (single source of truth)
    fs_cfg = load_failsafe_config(home_lat=HOME_LAT, home_lon=HOME_LON)

    # Guidance config — max_altitude_m sourced from failsafe config
    cfg = GuidanceConfig(max_altitude_m=fs_cfg.max_altitude_m)

    ctrl = FollowerController(
        conn, cfg,
        offset_n=FOLLOW_OFFSET_N,
        offset_e=FOLLOW_OFFSET_E,
        offset_d=FOLLOW_OFFSET_D,
        control_hz=CONTROL_HZ,
        bridge=bridge,
        my_id=DRONE_ID,
    )

    ctx = MissionContext(
        conn=conn,
        bridge=bridge,
        cfg=cfg,
        fs_cfg=fs_cfg,
        fs_state=FailsafeState(),
        ctrl=ctrl,
        leader_id=LEADER_ID,
    )

    # The follower starts airborne (startup already did takeoff)
    ctx.is_airborne = True

    while _running:
        tick_start = time.time()
        status = check_status(ctx)
        mission = get_mission(status, ctx)
        _running = execute_mission(mission, status, ctx)
        sleep_tick(tick_start)

    if conn and not ctx.rtl_sent:
        conn.set_mode("RTL")


# ═══════════════════════════════════════════════════════════════
# Shutdown
# ═══════════════════════════════════════════════════════════════

def _shutdown(bridge, conn, launcher):
    """Phase 8: Cleanup."""
    log.info("=" * 50)
    log.info("Shutdown")
    log.info("=" * 50)

    bridge.stop()
    if conn:
        # Wait for landing
        land_deadline = time.time() + 60.0
        while time.time() < land_deadline:
            data = conn.drain_latest()
            pos = data.get('position')
            if pos and pos['alt'] < 1.0:
                log.info("Landed")
                break
            time.sleep(1.0)

        try:
            conn.set_mode("LAND")
        except Exception:
            pass

    time.sleep(2)
    launcher.kill()
    log.info("Cleanup complete")


def _sleep_tick(tick_start):
    elapsed = time.time() - tick_start
    sleep_time = (1.0 / CONTROL_HZ) - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    global _running
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    launcher = SITLLauncher()
    conn = None
    bridge = MavlinkBridge(
        drone_id=DRONE_ID,
        peer_targets=PEER_TARGETS,
        listen_port=PEER_LISTEN_PORT,
        gcs_host=GCS_HOST,
        gcs_telem_port=GCS_TELEM_PORT,
        gcs_cmd_port=GCS_CMD_PORT,
        stale_timeout=PEER_STALE_TIMEOUT,
    )

    try:
        # Phases 1-6: Launch, connect, GPS, takeoff (same for all roles)
        conn = _startup(launcher, bridge)
        if conn is None or not _running:
            return

        # Phase 7: Role-specific control loop
        if DRONE_ROLE == "leader":
            _leader_loop(conn, bridge)
        else:
            _follower_loop(conn, bridge)

    except Exception as e:
        log.error("Fatal: %s", e, exc_info=True)
    finally:
        _shutdown(bridge, conn, launcher)


if __name__ == "__main__":
    main()
