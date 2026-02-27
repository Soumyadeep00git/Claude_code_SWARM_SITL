"""Leader drone container entry point.

Launches ArduCopter SITL, connects, starts MAVLink bridge,
takes off, runs the leader mission, then shuts down.
"""

import logging
import math
import os
import signal
import sys
import time
from enum import Enum

from docker_sim.config import (
    DRONE_ID, TAKEOFF_ALT_M, CONTROL_HZ,
    PEER_HOST, BROADCAST_PORT, LISTEN_PORT,
    GCS_HOST, GCS_TELEM_PORT, GCS_CMD_PORT,
    MY_HOME_LAT, MY_HOME_LON,
    HOVER_TIME_S, LEADER_SPEED_MS,
    MISSION_NORTH_M, MISSION_EAST_M,
    MOVE_ARRIVAL_M, MOVE_TIMEOUT_S,
    ERRATIC_SPRINT_SPEED, ERRATIC_ZIGZAG_PERIOD, ERRATIC_ZIGZAG_SPEED,
    ERRATIC_CHARGE_SPEED, ERRATIC_PHASE_TIME,
    FOLLOW_OFFSET_N, FOLLOW_OFFSET_E,
    METERS_PER_DEG_LAT, OUTPUT_DIR,
)
from docker_sim.sitl_launcher import SITLLauncher
from docker_sim.mavlink_conn import MavlinkConn
from docker_sim.takeoff import TakeoffManager
from docker_sim.mavlink_bridge import MavlinkBridge, CMD_RTL, CMD_LAND, CMD_KILL

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [LEADER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("leader")

_running = True


def _signal_handler(sig, frame):
    global _running
    log.info("Interrupted — shutting down...")
    _running = False


# ── Leader Mission (inlined to avoid sim.config imports) ─────

class Phase(Enum):
    HOVER_1 = "HOVER_1"
    MOVE_NORTH = "MOVE_NORTH"
    HOVER_2 = "HOVER_2"
    MOVE_EAST = "MOVE_EAST"
    HOVER_3 = "HOVER_3"
    CHARGE_SOUTH = "CHARGE_SOUTH"
    HOVER_4 = "HOVER_4"
    ZIGZAG = "ZIGZAG"
    HOVER_5 = "HOVER_5"
    SUDDEN_STOP = "SUDDEN_STOP"
    HOVER_6 = "HOVER_6"
    SPRINT = "SPRINT"
    HOVER_7 = "HOVER_7"
    RTL = "RTL"
    DONE = "DONE"


def _gps_dist(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * METERS_PER_DEG_LAT
    de = (lon2 - lon1) * METERS_PER_DEG_LAT * math.cos(math.radians(lat1))
    return math.sqrt(dn * dn + de * de)


def _ned_to_gps(n, e, ref_lat, ref_lon):
    lat = ref_lat + n / METERS_PER_DEG_LAT
    cos_lat = math.cos(math.radians(ref_lat))
    lon = ref_lon + e / (METERS_PER_DEG_LAT * max(cos_lat, 1e-10))
    return lat, lon


class LeaderMission:
    def __init__(self, conn, home_lat, home_lon):
        self.conn = conn
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.phase = Phase.HOVER_1
        self._phase_start = time.time()
        self._rtl_sent = False

        self.wp_north_lat, self.wp_north_lon = _ned_to_gps(
            MISSION_NORTH_M, 0.0, home_lat, home_lon)
        self.wp_east_lat, self.wp_east_lon = _ned_to_gps(
            MISSION_NORTH_M, MISSION_EAST_M, home_lat, home_lon)

        log.info("LeaderMission: home=(%.7f, %.7f)", home_lat, home_lon)

    def tick(self, lat, lon, alt):
        now = time.time()
        elapsed = now - self._phase_start

        if self.phase == Phase.HOVER_1:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= HOVER_TIME_S:
                self._transition(Phase.MOVE_NORTH)
        elif self.phase == Phase.MOVE_NORTH:
            self.conn.send_velocity_ned(LEADER_SPEED_MS, 0, 0)
            if _gps_dist(lat, lon, self.wp_north_lat, self.wp_north_lon) < MOVE_ARRIVAL_M or elapsed > MOVE_TIMEOUT_S:
                self._transition(Phase.HOVER_2)
        elif self.phase == Phase.HOVER_2:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= HOVER_TIME_S:
                self._transition(Phase.MOVE_EAST)
        elif self.phase == Phase.MOVE_EAST:
            self.conn.send_velocity_ned(0, LEADER_SPEED_MS, 0)
            if _gps_dist(lat, lon, self.wp_east_lat, self.wp_east_lon) < MOVE_ARRIVAL_M or elapsed > MOVE_TIMEOUT_S:
                self._transition(Phase.HOVER_3)
        elif self.phase == Phase.HOVER_3:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= HOVER_TIME_S:
                self._transition(Phase.CHARGE_SOUTH)
        elif self.phase == Phase.CHARGE_SOUTH:
            charge_n = FOLLOW_OFFSET_N / abs(FOLLOW_OFFSET_N) * ERRATIC_CHARGE_SPEED
            charge_e = FOLLOW_OFFSET_E / max(abs(FOLLOW_OFFSET_E), 0.1) * (ERRATIC_CHARGE_SPEED * 0.3)
            self.conn.send_velocity_ned(charge_n, charge_e, 0)
            if elapsed >= ERRATIC_PHASE_TIME:
                self._transition(Phase.HOVER_4)
        elif self.phase == Phase.HOVER_4:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= 10.0:
                self._transition(Phase.ZIGZAG)
        elif self.phase == Phase.ZIGZAG:
            half = ERRATIC_ZIGZAG_PERIOD / 2.0
            cycle = elapsed % ERRATIC_ZIGZAG_PERIOD
            ve = ERRATIC_ZIGZAG_SPEED if cycle < half else -ERRATIC_ZIGZAG_SPEED
            self.conn.send_velocity_ned(LEADER_SPEED_MS * 0.5, ve, 0)
            if elapsed >= ERRATIC_PHASE_TIME:
                self._transition(Phase.HOVER_5)
        elif self.phase == Phase.HOVER_5:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= 10.0:
                self._transition(Phase.SUDDEN_STOP)
        elif self.phase == Phase.SUDDEN_STOP:
            if elapsed < 5.0:
                self.conn.send_velocity_ned(ERRATIC_SPRINT_SPEED, 0, 0)
            else:
                self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= ERRATIC_PHASE_TIME:
                self._transition(Phase.HOVER_6)
        elif self.phase == Phase.HOVER_6:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= 10.0:
                self._transition(Phase.SPRINT)
        elif self.phase == Phase.SPRINT:
            self.conn.send_velocity_ned(0, -ERRATIC_SPRINT_SPEED, 0)
            if elapsed >= ERRATIC_PHASE_TIME:
                self._transition(Phase.HOVER_7)
        elif self.phase == Phase.HOVER_7:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= HOVER_TIME_S:
                self._transition(Phase.RTL)
        elif self.phase == Phase.RTL:
            if not self._rtl_sent:
                self.conn.set_mode("RTL")
                self._rtl_sent = True
            if elapsed > 30.0:
                self._transition(Phase.DONE)

        return self.phase

    def _transition(self, new_phase):
        log.info("Mission: %s -> %s", self.phase.value, new_phase.value)
        self.phase = new_phase
        self._phase_start = time.time()


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

        # Phase 3: Start bridge + wait GPS
        log.info("=" * 50)
        log.info("Phase 3: Waiting for GPS fix")
        log.info("=" * 50)
        bridge.start()

        gps_deadline = time.time() + 90.0
        has_gps = False
        while _running and time.time() < gps_deadline:
            data = conn.drain_latest()
            pos = data.get('position')
            if pos and pos['lat'] != 0:
                log.info("GPS fix: (%.7f, %.7f)", pos['lat'], pos['lon'])
                has_gps = True
                break
            time.sleep(0.5)
        if not has_gps:
            raise TimeoutError("GPS fix timeout")

        # Phase 4: Takeoff
        log.info("=" * 50)
        log.info("Phase 4: Takeoff to %.0fm", TAKEOFF_ALT_M)
        log.info("=" * 50)
        takeoff = TakeoffManager(conn, TAKEOFF_ALT_M)

        while _running:
            data = conn.drain_latest()
            pos = data.get('position')
            hb = data.get('heartbeat')

            # Broadcast own state for follower
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

        # Phase 5: Mission loop
        log.info("=" * 50)
        log.info("Phase 5: Running mission")
        log.info("=" * 50)

        data = conn.drain_latest()
        pos = data.get('position', {})
        home_lat = pos.get('lat', MY_HOME_LAT)
        home_lon = pos.get('lon', MY_HOME_LON)

        mission = LeaderMission(conn, home_lat, home_lon)

        while _running:
            tick_start = time.time()

            data = conn.drain_latest()
            pos = data.get('position')
            if pos:
                bridge.update_own_state(pos)

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

            phase = mission.tick(
                pos['lat'] if pos else 0,
                pos['lon'] if pos else 0,
                pos['alt'] if pos else 0,
            )

            if phase == Phase.DONE:
                log.info("Mission complete")
                break

            elapsed = time.time() - tick_start
            sleep_time = (1.0 / CONTROL_HZ) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Phase 6: Shutdown
        log.info("=" * 50)
        log.info("Phase 6: Shutdown")
        log.info("=" * 50)

        # Wait for landing
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
