"""Leader waypoint mission sequencer with erratic behavior phases.

Normal phases test steady following. Erratic phases test follower resilience:
  - CHARGE_SOUTH: leader flies directly toward follower's expected position
  - ZIGZAG: rapid east-west oscillations while moving
  - SUDDEN_STOP: full speed then instant zero velocity
  - SPRINT: high speed burst exceeding follower's max_speed
"""

import logging
import math
import time
from enum import Enum

from sim.config import (
    HOVER_TIME_S, LEADER_SPEED_MS,
    MISSION_NORTH_M, MISSION_EAST_M,
    MOVE_ARRIVAL_M, MOVE_TIMEOUT_S,
    ERRATIC_SPRINT_SPEED, ERRATIC_ZIGZAG_PERIOD, ERRATIC_ZIGZAG_SPEED,
    ERRATIC_CHARGE_SPEED, ERRATIC_PHASE_TIME,
    FOLLOW_OFFSET_N, FOLLOW_OFFSET_E,
    METERS_PER_DEG_LAT,
)
from sim.mavlink_conn import MavlinkConn

log = logging.getLogger(__name__)


class Phase(Enum):
    # ── Normal phases ─────────────────────────────────────
    HOVER_1 = "HOVER_1"
    MOVE_NORTH = "MOVE_NORTH"
    HOVER_2 = "HOVER_2"
    MOVE_EAST = "MOVE_EAST"
    HOVER_3 = "HOVER_3"
    # ── Erratic phases ────────────────────────────────────
    CHARGE_SOUTH = "CHARGE_SOUTH"     # Fly toward follower
    HOVER_4 = "HOVER_4"
    ZIGZAG = "ZIGZAG"                 # Rapid direction changes
    HOVER_5 = "HOVER_5"
    SUDDEN_STOP = "SUDDEN_STOP"       # Full speed → zero
    HOVER_6 = "HOVER_6"
    SPRINT = "SPRINT"                 # High speed burst
    HOVER_7 = "HOVER_7"
    # ── Finish ────────────────────────────────────────────
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
    """State-machine leader mission with erratic behavior phases.

    Call tick() at 10Hz.
    """

    def __init__(self, conn: MavlinkConn, home_lat: float, home_lon: float):
        self.conn = conn
        self.home_lat = home_lat
        self.home_lon = home_lon

        self.phase = Phase.HOVER_1
        self._phase_start = time.time()
        self._rtl_sent = False
        self._sudden_stop_sprinted = False

        # Pre-compute normal waypoints
        self.wp_north_lat, self.wp_north_lon = _ned_to_gps(
            MISSION_NORTH_M, 0.0, home_lat, home_lon)
        self.wp_east_lat, self.wp_east_lon = _ned_to_gps(
            MISSION_NORTH_M, MISSION_EAST_M, home_lat, home_lon)

        log.info("LeaderMission (with erratic phases): "
                 "home=(%.7f, %.7f)", home_lat, home_lon)

    def tick(self, lat: float, lon: float, alt: float) -> Phase:
        """Run one tick. Sends velocity commands, manages transitions."""
        now = time.time()
        elapsed = now - self._phase_start

        # ── Normal phases ─────────────────────────────────
        if self.phase == Phase.HOVER_1:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= HOVER_TIME_S:
                self._transition(Phase.MOVE_NORTH)

        elif self.phase == Phase.MOVE_NORTH:
            self.conn.send_velocity_ned(LEADER_SPEED_MS, 0, 0)
            dist = _gps_dist(lat, lon, self.wp_north_lat, self.wp_north_lon)
            if dist < MOVE_ARRIVAL_M or elapsed > MOVE_TIMEOUT_S:
                self._transition(Phase.HOVER_2)

        elif self.phase == Phase.HOVER_2:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= HOVER_TIME_S:
                self._transition(Phase.MOVE_EAST)

        elif self.phase == Phase.MOVE_EAST:
            self.conn.send_velocity_ned(0, LEADER_SPEED_MS, 0)
            dist = _gps_dist(lat, lon, self.wp_east_lat, self.wp_east_lon)
            if dist < MOVE_ARRIVAL_M or elapsed > MOVE_TIMEOUT_S:
                self._transition(Phase.HOVER_3)

        elif self.phase == Phase.HOVER_3:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= HOVER_TIME_S:
                self._transition(Phase.CHARGE_SOUTH)

        # ── Erratic: CHARGE toward follower ───────────────
        elif self.phase == Phase.CHARGE_SOUTH:
            # Follower is expected ~5m south, ~3m east of leader.
            # Leader flies south (negative north) to close that gap.
            charge_n = FOLLOW_OFFSET_N / abs(FOLLOW_OFFSET_N) * ERRATIC_CHARGE_SPEED
            charge_e = FOLLOW_OFFSET_E / max(abs(FOLLOW_OFFSET_E), 0.1) * (ERRATIC_CHARGE_SPEED * 0.3)
            self.conn.send_velocity_ned(charge_n, charge_e, 0)
            if elapsed >= ERRATIC_PHASE_TIME:
                self._transition(Phase.HOVER_4)

        elif self.phase == Phase.HOVER_4:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= 10.0:
                self._transition(Phase.ZIGZAG)

        # ── Erratic: ZIGZAG ──────────────────────────────
        elif self.phase == Phase.ZIGZAG:
            # Alternate east/west every ZIGZAG_PERIOD seconds while moving north
            half_period = ERRATIC_ZIGZAG_PERIOD / 2.0
            cycle = elapsed % ERRATIC_ZIGZAG_PERIOD
            if cycle < half_period:
                ve = ERRATIC_ZIGZAG_SPEED
            else:
                ve = -ERRATIC_ZIGZAG_SPEED
            self.conn.send_velocity_ned(LEADER_SPEED_MS * 0.5, ve, 0)
            if elapsed >= ERRATIC_PHASE_TIME:
                self._transition(Phase.HOVER_5)

        elif self.phase == Phase.HOVER_5:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= 10.0:
                self._transition(Phase.SUDDEN_STOP)

        # ── Erratic: SUDDEN STOP ─────────────────────────
        elif self.phase == Phase.SUDDEN_STOP:
            # Sprint north at high speed for 5s, then instant zero
            if elapsed < 5.0:
                self.conn.send_velocity_ned(ERRATIC_SPRINT_SPEED, 0, 0)
                self._sudden_stop_sprinted = True
            else:
                self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= ERRATIC_PHASE_TIME:
                self._transition(Phase.HOVER_6)

        elif self.phase == Phase.HOVER_6:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= 10.0:
                self._transition(Phase.SPRINT)

        # ── Erratic: SPRINT ──────────────────────────────
        elif self.phase == Phase.SPRINT:
            # High-speed burst that exceeds follower's max_speed (3 m/s)
            # Follower should fall behind but not crash
            self.conn.send_velocity_ned(0, -ERRATIC_SPRINT_SPEED, 0)
            if elapsed >= ERRATIC_PHASE_TIME:
                self._transition(Phase.HOVER_7)

        elif self.phase == Phase.HOVER_7:
            self.conn.send_velocity_ned(0, 0, 0)
            if elapsed >= HOVER_TIME_S:
                self._transition(Phase.RTL)

        # ── Finish ────────────────────────────────────────
        elif self.phase == Phase.RTL:
            if not self._rtl_sent:
                self.conn.set_mode("RTL")
                self._rtl_sent = True
            if elapsed > 30.0:
                self._transition(Phase.DONE)

        return self.phase

    def _transition(self, new_phase: Phase):
        log.info("Leader mission: %s -> %s", self.phase.value, new_phase.value)
        self.phase = new_phase
        self._phase_start = time.time()
