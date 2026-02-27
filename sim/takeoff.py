"""Non-blocking takeoff state machine."""

import logging
import time

from sim.config import TAKEOFF_COMPLETE_FRAC
from sim.mavlink_conn import MavlinkConn

log = logging.getLogger(__name__)


class TakeoffManager:
    """State machine: wait GPS → GUIDED → ARM → TAKEOFF → detect altitude.

    Call tick() at 10Hz with the latest telemetry. Returns True when complete.
    """

    def __init__(self, conn: MavlinkConn, target_alt: float):
        self.conn = conn
        self.target_alt = target_alt
        self._last_cmd_time = 0.0
        self._takeoff_sent = False
        self.complete = False

    def tick(self, has_gps: bool, mode: str, armed: bool, alt: float) -> bool:
        """Run one tick of the takeoff state machine.

        Args:
            has_gps: True if lat/lon are non-zero
            mode: Current flight mode string (e.g. "GUIDED")
            armed: True if drone is armed
            alt: Current relative altitude in meters

        Returns:
            True when takeoff is complete (alt >= 80% of target)
        """
        if self.complete:
            return True

        now = time.time()

        # Rate-limit commands to every 2 seconds
        if now - self._last_cmd_time < 2.0:
            # Still check altitude completion even between commands
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

        # Check altitude
        if alt >= self.target_alt * TAKEOFF_COMPLETE_FRAC:
            self.complete = True
            log.info("drone_id=%d: takeoff COMPLETE at %.1fm",
                     self.conn.drone_id, alt)
            return True

        # Re-send takeoff if altitude isn't rising
        if now - self._last_cmd_time >= 3.0:
            self.conn.takeoff(self.target_alt)
            self._last_cmd_time = now
            log.info("drone_id=%d: re-sending takeoff (alt=%.1fm)",
                     self.conn.drone_id, alt)

        return False
