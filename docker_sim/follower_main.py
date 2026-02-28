"""Follower drone container entry point.

Launches ArduCopter SITL, connects, starts MAVLink bridge,
then enters the mission-dispatch loop:
    1. check_status()    — GPS, heartbeat, leader snapshot
    2. get_mission()     — RC commands decide mission, failsafe override
    3. execute_mission() — dispatch to per-mission handler

Missions: IDLE → TAKEOFF → HOVER ↔ FOLLOW → RTL/LAND/KILL

RC has topmost authority. Global failsafe (GPS, geofence, altitude)
runs every tick. Follow-specific failsafe (leader stale, leader
geofence, catchup timeout) only runs inside FOLLOW.
"""

import logging
import os
import signal
import sys
import time

# Add project root so guidance_lib is importable
_PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PROJECT_DIR)

from docker_sim.config import (
    DRONE_ID, CONTROL_HZ,
    PEER_HOST, BROADCAST_PORT, LISTEN_PORT,
    GCS_HOST, GCS_TELEM_PORT, GCS_CMD_PORT,
    FOLLOW_OFFSET_N, FOLLOW_OFFSET_E, FOLLOW_OFFSET_D,
    OUTPUT_DIR, HOME_LAT, HOME_LON,
    PEER_STALE_TIMEOUT,
)
from docker_sim.sitl_launcher import SITLLauncher
from docker_sim.mavlink_conn import MavlinkConn
from docker_sim.mavlink_bridge import MavlinkBridge
from docker_sim.follower_missions import (
    MissionContext, check_status, get_mission, execute_mission, sleep_tick,
)

from guidance_lib import (
    GuidanceConfig, GuidanceState, compute_guidance,
    CommandSmoother,
)
from failsafe_lib import FailsafeState, load_config as load_failsafe_config
from guidance_lib.target import TargetComputer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [FOLLOWER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("follower")

_running = True


def _signal_handler(sig, frame):
    global _running
    log.info("Interrupted -- shutting down...")
    _running = False


class FollowerController:
    """3-mode hard-switched guidance with modular safety.

    Key differences from old version:
      - Leader state passed DIRECTLY each tick (None = stale/missing)
      - Smoother resets on failsafe (no residual velocity)
      - TargetComputer handles offset + feedforward cleanly
    """

    def __init__(self, conn: MavlinkConn, cfg: GuidanceConfig):
        self.conn = conn
        self.cfg = cfg
        self.state = GuidanceState()
        self.smoother = CommandSmoother(dt=1.0 / CONTROL_HZ)
        self.target_computer = TargetComputer()

        # Own state
        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_alt = 0.0
        self.my_vn = 0.0
        self.my_ve = 0.0
        self.my_vd = 0.0

    def update_own_state(self, pos: dict):
        self.my_lat = pos['lat']
        self.my_lon = pos['lon']
        self.my_alt = pos['alt']
        self.my_vn = pos['vx']
        self.my_ve = pos['vy']
        self.my_vd = pos['vz']

    def tick(self, leader: dict | None) -> dict | None:
        """Run one guidance tick.

        Args:
            leader: Leader state dict {lat, lon, alt, vx, vy} or None if stale.

        Returns:
            Guidance result dict, or None if own state not ready.
        """
        if self.my_lat == 0.0 and self.my_lon == 0.0:
            return None

        # Use leader data directly -- no stale cache
        if leader is None or (leader['lat'] == 0.0 and leader['lon'] == 0.0):
            return None

        leader_lat = leader['lat']
        leader_lon = leader['lon']
        leader_alt = leader['alt']
        leader_vn = leader['vx']
        leader_ve = leader['vy']

        # Compute target = leader + offset + feedforward
        target_lat, target_lon, target_alt = self.target_computer.compute(
            leader_lat, leader_lon, leader_alt,
            leader_vn, leader_ve,
            FOLLOW_OFFSET_N, FOLLOW_OFFSET_E, FOLLOW_OFFSET_D,
            ff_gain=self.cfg.ff_gain, dt=1.0 / CONTROL_HZ)

        # 3-mode guidance
        result = compute_guidance(
            my_lat=self.my_lat, my_lon=self.my_lon, my_alt=self.my_alt,
            my_vn=self.my_vn, my_ve=self.my_ve, my_vd=self.my_vd,
            peer_lat=leader_lat, peer_lon=leader_lon, peer_alt=leader_alt,
            peer_vn=leader_vn, peer_ve=leader_ve,
            goal_lat=target_lat, goal_lon=target_lon, goal_alt=target_alt,
            cfg=self.cfg, state=self.state,
        )

        # Smooth & send
        sm_vn, sm_ve, sm_vd = self.smoother.filter(
            result['vn'], result['ve'], result['vd'])
        self.conn.send_velocity_ned(sm_vn, sm_ve, sm_vd)

        if result['mode'] != 'TRACKING':
            log.info("Mode: %s (w_e=%.2f w_t=%.2f w_c=%.2f)",
                     result['mode'], result['w_evasion'],
                     result['w_tracking'], result['w_catchup'])

        return result

    def reset(self):
        """Reset all internal state. Call on failsafe transitions."""
        self.smoother.reset()
        self.state = GuidanceState()


# ── Main ─────────────────────────────────────────────────────

def main():
    global _running
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    launcher = SITLLauncher()
    conn = None
    bridge = MavlinkBridge(
        PEER_HOST, BROADCAST_PORT, LISTEN_PORT,
        gcs_host=GCS_HOST, gcs_telem_port=GCS_TELEM_PORT,
        gcs_cmd_port=GCS_CMD_PORT,
        stale_timeout=PEER_STALE_TIMEOUT,
    )
    # Load failsafe config (single source of truth)
    fs_cfg = load_failsafe_config(home_lat=HOME_LAT, home_lon=HOME_LON)

    # Guidance config — max_altitude_m sourced from failsafe config
    cfg = GuidanceConfig(max_altitude_m=fs_cfg.max_altitude_m)

    try:
        # ── Phase 1: Launch SITL ─────────────────────────────
        log.info("=" * 50)
        log.info("Phase 1: Launching SITL")
        log.info("=" * 50)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        launcher.launch()
        if not launcher.wait_ready():
            raise TimeoutError("SITL port not reachable")

        # ── Phase 2: Connect ─────────────────────────────────
        log.info("=" * 50)
        log.info("Phase 2: Connecting pymavlink")
        log.info("=" * 50)
        conn = MavlinkConn(timeout=60)

        # ── Phase 3: Start bridge ────────────────────────────
        log.info("=" * 50)
        log.info("Phase 3: Starting bridge")
        log.info("=" * 50)
        bridge.start()

        # ── Phase 4: Mission-dispatch loop ───────────────────
        log.info("=" * 50)
        log.info("Phase 4: Mission loop (IDLE)")
        log.info("=" * 50)

        ctrl = FollowerController(conn, cfg)
        ctx = MissionContext(
            conn=conn,
            bridge=bridge,
            cfg=cfg,
            fs_cfg=fs_cfg,
            fs_state=FailsafeState(),
            ctrl=ctrl,
        )

        while _running:
            tick_start = time.time()

            # 1. Check status
            status = check_status(ctx)

            # 2. Get mission (RC commands + failsafe override)
            mission = get_mission(status, ctx)

            # 3. Execute mission
            _running = execute_mission(mission, status, ctx)

            # Sleep remainder of tick
            sleep_tick(tick_start)

        # ── Phase 5: Shutdown ────────────────────────────────
        log.info("=" * 50)
        log.info("Phase 5: Shutdown")
        log.info("=" * 50)
        if conn and not ctx.rtl_sent:
            conn.set_mode("RTL")

        land_deadline = time.time() + 60.0
        while time.time() < land_deadline:
            data = conn.drain_latest()
            pos = data.get('position')
            if pos:
                bridge.update_own_state(pos)
            if pos and pos['alt'] < 1.0:
                log.info("Landed")
                break
            time.sleep(1.0)

    except Exception as e:
        log.error("Fatal: %s", e, exc_info=True)
    finally:
        bridge.stop()
        if conn:
            try:
                conn.set_mode("LAND")
            except Exception:
                pass
        time.sleep(2)
        launcher.kill()
        log.info("Cleanup complete")


if __name__ == "__main__":
    main()
