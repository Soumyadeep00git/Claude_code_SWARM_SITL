"""
Local planner — executes immediate movement commands.
Translates high-level intents (waypoint, velocity, takeoff, land)
into MAVLink calls via DroneConnection.

All operations are non-blocking. The tick() method is called each loop
iteration and resends active commands.
"""

import time
import logging

from mavlink_layer.drone_connection import DroneConnection

log = logging.getLogger(__name__)


# Task states
IDLE = "IDLE"
TAKEOFF = "TAKEOFF"
WAYPOINT = "WAYPOINT"
VELOCITY = "VELOCITY"
LAND = "LAND"
HOLD = "HOLD"


class LocalPlanner:
    """Executes movement commands against one SITL instance."""

    def __init__(self, conn: DroneConnection):
        self.conn = conn
        self.task: str = IDLE

        # Waypoint target (global coords)
        self.wp_lat: float = 0.0
        self.wp_lon: float = 0.0
        self.wp_alt: float = 0.0

        # Velocity target (NED m/s)
        self.vel_n: float = 0.0
        self.vel_e: float = 0.0
        self.vel_d: float = 0.0

        # Takeoff state machine
        self._takeoff_alt: float = 0.0
        self._arm_sent_time: float = 0.0
        self._takeoff_sent: bool = False

    def do_takeoff(self, alt: float):
        """Initiate takeoff sequence (non-blocking). Handled in tick()."""
        log.info("Drone %d: takeoff requested to %.1f m", self.conn.drone_id, alt)
        self._takeoff_alt = alt
        self._takeoff_sent = False
        self._arm_sent_time = 0.0
        self.task = TAKEOFF

    def do_land(self):
        """Switch to LAND mode."""
        log.info("Drone %d: landing", self.conn.drone_id)
        self.conn.land()
        self.task = LAND

    def set_waypoint(self, lat: float, lon: float, alt: float):
        """Set a GPS waypoint target. Drone flies there in GUIDED mode."""
        self.wp_lat = lat
        self.wp_lon = lon
        self.wp_alt = alt
        self.task = WAYPOINT

    def set_velocity(self, vn: float, ve: float, vd: float):
        """Set a NED velocity setpoint."""
        self.vel_n = vn
        self.vel_e = ve
        self.vel_d = vd
        self.task = VELOCITY

    def hold(self):
        """Stop all motion — send zero velocity."""
        self.vel_n = 0.0
        self.vel_e = 0.0
        self.vel_d = 0.0
        self.task = HOLD

    def tick(self, armed: bool = False, mode: str = "", has_gps: bool = False):
        """
        Called every loop iteration. Handles state machine for takeoff
        and resends active commands.
        """
        if self.task == TAKEOFF:
            self._tick_takeoff(armed, mode, has_gps)

        elif self.task == WAYPOINT:
            self.conn.send_goto_global(self.wp_lat, self.wp_lon, self.wp_alt)

        elif self.task == VELOCITY:
            self.conn.send_velocity_ned(self.vel_n, self.vel_e, self.vel_d)

        elif self.task == HOLD:
            self.conn.send_velocity_ned(0, 0, 0)

        # LAND, IDLE: no resend needed

    def _tick_takeoff(self, armed: bool, mode: str, has_gps: bool):
        """Non-blocking takeoff state machine."""
        now = time.time()

        # Step 0: Wait for GPS fix before doing anything
        if not has_gps:
            if now - self._arm_sent_time > 5.0:
                log.info("Drone %d: waiting for GPS fix before takeoff...",
                         self.conn.drone_id)
                self._arm_sent_time = now
            return

        # Step 1: Set GUIDED mode (resend every 2s until confirmed)
        if "GUIDED" not in mode:
            if now - self._arm_sent_time > 2.0:
                self.conn.set_mode("GUIDED")
                self._arm_sent_time = now
            return

        # Step 2: Arm (resend every 2s until confirmed)
        if not armed:
            if now - self._arm_sent_time > 2.0:
                self.conn.arm()
                self._arm_sent_time = now
            return

        # Step 3: Send takeoff command (resend every 3s until airborne)
        if not self._takeoff_sent or (now - self._arm_sent_time > 3.0):
            self.conn.takeoff(self._takeoff_alt)
            self._takeoff_sent = True
            self._arm_sent_time = now
            log.info("Drone %d: takeoff command sent (%.1fm)",
                     self.conn.drone_id, self._takeoff_alt)
