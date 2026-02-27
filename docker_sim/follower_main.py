"""Follower drone container entry point.

Launches ArduCopter SITL, connects, starts MAVLink bridge,
waits for leader state, takes off, runs the follower guidance loop,
then shuts down when leader completes RTL.
"""

import logging
import math
import os
import signal
import sys
import time

# Add project root so follower_drone package is importable
_PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PROJECT_DIR, "follower_drone"))

from docker_sim.config import (
    DRONE_ID, TAKEOFF_ALT_M, CONTROL_HZ,
    PEER_HOST, BROADCAST_PORT, LISTEN_PORT,
    GCS_HOST, GCS_TELEM_PORT, GCS_CMD_PORT,
    FOLLOW_OFFSET_N, FOLLOW_OFFSET_E, FOLLOW_OFFSET_D,
    FEEDFORWARD_GAIN, METERS_PER_DEG_LAT, OUTPUT_DIR,
    HOME_LAT, HOME_LON,
)
from docker_sim.sitl_launcher import SITLLauncher
from docker_sim.mavlink_conn import MavlinkConn
from docker_sim.takeoff import TakeoffManager
from docker_sim.mavlink_bridge import MavlinkBridge, CMD_RTL, CMD_LAND, CMD_KILL

from follower_drone.guidance import GuidanceConfig, GuidanceState, compute_guidance
from follower_drone.geo_utils import ned_to_gps, gps_to_ned
from follower_drone.command_smoother import CommandSmoother

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [FOLLOWER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("follower")

_running = True


def _signal_handler(sig, frame):
    global _running
    log.info("Interrupted — shutting down...")
    _running = False


class FollowerController:
    """3-mode hard-switched guidance loop (same logic as sim/follower_loop.py)."""

    def __init__(self, conn: MavlinkConn, geofence_m: float = 200.0):
        self.conn = conn
        self.cfg = GuidanceConfig(
            home_lat=HOME_LAT,
            home_lon=HOME_LON,
            geofence_radius_m=geofence_m,
        )
        self.state = GuidanceState()
        self.smoother = CommandSmoother()

        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_alt = 0.0
        self.my_vn = 0.0
        self.my_ve = 0.0
        self.my_vd = 0.0

        self.leader_lat = 0.0
        self.leader_lon = 0.0
        self.leader_alt = 0.0
        self.leader_vn = 0.0
        self.leader_ve = 0.0

    def update_own_state(self, pos):
        self.my_lat = pos['lat']
        self.my_lon = pos['lon']
        self.my_alt = pos['alt']
        self.my_vn = pos['vx']
        self.my_ve = pos['vy']
        self.my_vd = pos['vz']

    def update_leader_state(self, pos):
        self.leader_lat = pos['lat']
        self.leader_lon = pos['lon']
        self.leader_alt = pos['alt']
        self.leader_vn = pos['vx']
        self.leader_ve = pos['vy']

    def tick(self) -> dict | None:
        if self.my_lat == 0.0 and self.my_lon == 0.0:
            return None
        if self.leader_lat == 0.0 and self.leader_lon == 0.0:
            return None

        # Compute target = leader + NED offset
        target_lat, target_lon = ned_to_gps(
            FOLLOW_OFFSET_N, FOLLOW_OFFSET_E,
            self.leader_lat, self.leader_lon)
        target_alt = self.leader_alt - FOLLOW_OFFSET_D

        # Feedforward shift
        ff = self.cfg.ff_gain
        if ff > 0.01 and abs(self.leader_lat) > 1e-6:
            cos_lat = math.cos(math.radians(self.leader_lat))
            target_lat += ff * self.leader_vn * 0.1 / METERS_PER_DEG_LAT
            target_lon += (ff * self.leader_ve * 0.1
                           / (METERS_PER_DEG_LAT * max(cos_lat, 1e-6)))

        # 3-mode guidance
        result = compute_guidance(
            my_lat=self.my_lat, my_lon=self.my_lon, my_alt=self.my_alt,
            my_vn=self.my_vn, my_ve=self.my_ve, my_vd=self.my_vd,
            peer_lat=self.leader_lat, peer_lon=self.leader_lon,
            peer_alt=self.leader_alt,
            peer_vn=self.leader_vn, peer_ve=self.leader_ve,
            goal_lat=target_lat, goal_lon=target_lon, goal_alt=target_alt,
            cfg=self.cfg, state=self.state,
        )

        # Handle failsafe — zero velocity already set by guidance
        if result['mode'] == 'FAILSAFE':
            self.conn.send_velocity_ned(0, 0, 0)
            log.warning("FAILSAFE: %s (peer_dist=%.1f)",
                        result['flags'], result['peer_dist'])
            return result

        # Smooth & send
        sm_vn, sm_ve, sm_vd = self.smoother.filter(
            result['vn'], result['ve'], result['vd'])
        self.conn.send_velocity_ned(sm_vn, sm_ve, sm_vd)

        if result['mode'] != 'TRACKING':
            log.info("Mode: %s (w_e=%.2f w_t=%.2f w_c=%.2f)",
                     result['mode'], result['w_evasion'],
                     result['w_tracking'], result['w_catchup'])

        return result


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
    )

    try:
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

        # Phase 3: Start bridge + wait GPS + wait for leader
        log.info("=" * 50)
        log.info("Phase 3: Waiting for GPS + leader")
        log.info("=" * 50)
        bridge.start()

        gps_deadline = time.time() + 90.0
        has_gps = False
        has_leader = False
        while _running and time.time() < gps_deadline:
            data = conn.drain_latest()
            pos = data.get('position')
            if pos and pos['lat'] != 0 and not has_gps:
                log.info("GPS fix: (%.7f, %.7f)", pos['lat'], pos['lon'])
                has_gps = True

            peer = bridge.get_peer_state()
            if peer and not has_leader:
                log.info("Leader detected: (%.7f, %.7f)", peer['lat'], peer['lon'])
                has_leader = True

            if has_gps and has_leader:
                break
            time.sleep(0.5)

        if not has_gps:
            raise TimeoutError("GPS fix timeout")
        if not has_leader:
            log.warning("No leader detected yet — proceeding anyway")

        # Phase 4: Takeoff
        log.info("=" * 50)
        log.info("Phase 4: Takeoff to %.0fm", TAKEOFF_ALT_M)
        log.info("=" * 50)
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

        # Phase 5: Follower guidance loop
        log.info("=" * 50)
        log.info("Phase 5: Following leader")
        log.info("=" * 50)
        ctrl = FollowerController(conn)
        no_leader_count = 0

        while _running:
            tick_start = time.time()

            # Own state
            data = conn.drain_latest()
            pos = data.get('position')
            if pos:
                bridge.update_own_state(pos)
                ctrl.update_own_state(pos)

            # Check for GCS commands
            gcs_cmd = bridge.get_pending_command()
            if gcs_cmd == CMD_RTL:
                log.info("GCS command: RTL")
                conn.set_mode("RTL")
                break
            elif gcs_cmd == CMD_LAND:
                log.info("GCS command: LAND")
                conn.set_mode("LAND")
                break
            elif gcs_cmd == CMD_KILL:
                log.info("GCS command: KILL (force disarm)")
                conn.force_disarm()
                break

            # Leader state from bridge
            peer = bridge.get_peer_state()
            if peer:
                ctrl.update_leader_state(peer)
                no_leader_count = 0
            else:
                no_leader_count += 1

            # Run guidance
            result = ctrl.tick()

            # Failsafe triggered — RTL
            if result and result['mode'] == 'FAILSAFE':
                log.warning("Guidance failsafe triggered: %s — RTL",
                            result['flags'])
                break

            # If leader lost for 60s, assume mission over → RTL
            if no_leader_count > CONTROL_HZ * 60:
                log.warning("Leader lost for 60s — RTL")
                break

            elapsed = time.time() - tick_start
            sleep_time = (1.0 / CONTROL_HZ) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Phase 6: Shutdown
        log.info("=" * 50)
        log.info("Phase 6: Shutdown (RTL)")
        log.info("=" * 50)
        if conn:
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
