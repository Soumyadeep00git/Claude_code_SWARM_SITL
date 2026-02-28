"""Non-blocking takeoff state machine for dockerized drone."""

import logging
import time

from docker_sim.config import TAKEOFF_COMPLETE_FRAC
from docker_sim.mavlink_conn import MavlinkConn

log = logging.getLogger(__name__)


class TakeoffManager:
    """State machine: wait GPS -> GUIDED -> ARM -> TAKEOFF -> detect altitude."""

    def __init__(self, conn: MavlinkConn, target_alt: float,
                 max_time: float = 120.0):
        self.conn = conn
        self.target_alt = target_alt
        self._last_cmd_time = 0.0
        self._takeoff_sent = False
        self._start_time = time.time()
        self._max_time = max_time
        self.complete = False

    def tick(self, has_gps: bool, mode: str, armed: bool, alt: float) -> bool:
        if self.complete:
            return True

        now = time.time()

        # Overall timeout
        if now - self._start_time > self._max_time:
            log.error("Takeoff TIMEOUT after %.0fs — aborting",
                      self._max_time)
            self.complete = True
            return True

        if now - self._last_cmd_time < 2.0:
            if self._takeoff_sent and alt >= self.target_alt * TAKEOFF_COMPLETE_FRAC:
                self.complete = True
                log.info("drone_id=%d: takeoff COMPLETE at %.1fm",
                         self.conn.drone_id, alt)
                return True
            return False

        if not has_gps:
            log.info("drone_id=%d: waiting for GPS fix...", self.conn.drone_id)
            self._last_cmd_time = now
            return False

        if "GUIDED" not in mode:
            self.conn.set_mode("GUIDED")
            self._last_cmd_time = now
            return False

        if not armed:
            self.conn.arm()
            self._last_cmd_time = now
            return False

        if not self._takeoff_sent:
            self.conn.takeoff(self.target_alt)
            self._takeoff_sent = True
            self._last_cmd_time = now
            return False

        if alt >= self.target_alt * TAKEOFF_COMPLETE_FRAC:
            self.complete = True
            log.info("drone_id=%d: takeoff COMPLETE at %.1fm",
                     self.conn.drone_id, alt)
            return True

        if now - self._last_cmd_time >= 3.0:
            self.conn.takeoff(self.target_alt)
            self._last_cmd_time = now

        return False
