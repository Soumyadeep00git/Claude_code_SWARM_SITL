"""
pymavlink wrapper for a single ArduCopter SITL instance.
Thin layer — translates high-level commands into MAVLink calls.
No business logic here.
"""

import time
import logging
from pymavlink import mavutil

from config import SITL_BASE_PORT, SITL_PORT_STEP

log = logging.getLogger(__name__)


class DroneConnection:
    """Manages the MAVLink connection to one ArduCopter SITL instance."""

    def __init__(self, drone_id: int, timeout: float = 60.0):
        self.drone_id = drone_id
        port = SITL_BASE_PORT + drone_id * SITL_PORT_STEP
        conn_str = f"tcp:127.0.0.1:{port}"
        log.info("Drone %d: connecting to %s", drone_id, conn_str)

        self.conn = mavutil.mavlink_connection(conn_str, source_system=255)
        self.conn.wait_heartbeat(timeout=timeout)

        self.sysid = self.conn.target_system
        self.compid = self.conn.target_component

        # If sysid is 0, set expected sysid from config (drone_id + 1)
        if self.sysid == 0:
            self.sysid = drone_id + 1
            log.info("Drone %d: heartbeat sysid=0, using expected sysid=%d",
                     drone_id, self.sysid)

        log.info("Drone %d: connected (sysid=%d)", drone_id, self.sysid)

        self._request_data_streams()

    # ── Data streams ───────────────────────────────────────

    def _request_data_streams(self):
        """Request position, attitude, and status streams at 10 Hz."""
        self.conn.mav.request_data_stream_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # Hz
            1,   # start
        )

    # ── Mode control ───────────────────────────────────────

    def set_mode(self, mode_name: str):
        """Set flight mode by name (e.g. 'GUIDED', 'LAND', 'RTL')."""
        mode_map = self.conn.mode_mapping()
        if mode_name not in mode_map:
            log.error("Drone %d: unknown mode '%s'", self.drone_id, mode_name)
            return
        mode_id = mode_map[mode_name]
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0,
        )
        log.info("Drone %d: mode -> %s", self.drone_id, mode_name)

    # ── Arm / Disarm ───────────────────────────────────────

    def arm(self):
        """Send arm command (non-blocking). Check armed state via heartbeat."""
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        log.info("Drone %d: arm command sent", self.drone_id)

    def disarm(self):
        """Disarm motors."""
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    # ── Takeoff / Land ─────────────────────────────────────

    def takeoff(self, alt_m: float):
        """Command takeoff to altitude (meters). Must be in GUIDED mode and armed."""
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt_m,
        )
        log.info("Drone %d: takeoff to %.1f m", self.drone_id, alt_m)

    def land(self):
        """Switch to LAND mode."""
        self.set_mode("LAND")

    def rtl(self):
        """Switch to RTL mode."""
        self.set_mode("RTL")

    # ── Movement commands ──────────────────────────────────

    def send_position_ned(self, n: float, e: float, d: float):
        """Send NED position setpoint relative to EKF origin."""
        self.conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (ignored)
            self.sysid, self.compid,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_1111_1111_1000,  # type_mask: position only
            n, e, d,      # x, y, z (NED)
            0, 0, 0,      # vx, vy, vz
            0, 0, 0,      # afx, afy, afz
            0, 0,          # yaw, yaw_rate
        )

    def send_velocity_ned(self, vn: float, ve: float, vd: float):
        """Send NED velocity setpoint. Must resend every ~200ms."""
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.sysid, self.compid,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_1111_1100_0111,  # type_mask: velocity only
            0, 0, 0,
            vn, ve, vd,
            0, 0, 0,
            0, 0,
        )

    def send_goto_global(self, lat: float, lon: float, alt: float):
        """Send global position setpoint (lat/lon/alt above home)."""
        self.conn.mav.set_position_target_global_int_send(
            0,
            self.sysid, self.compid,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000_1111_1111_1000,  # position only
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )

    # ── Telemetry reads ────────────────────────────────────

    def get_position(self) -> dict | None:
        """Read latest GLOBAL_POSITION_INT (non-blocking)."""
        msg = self.conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            return {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt": msg.relative_alt / 1000.0,
                "vx": msg.vx / 100.0,
                "vy": msg.vy / 100.0,
                "vz": msg.vz / 100.0,
                "heading": msg.hdg / 100.0,
            }
        return None

    def get_battery(self) -> int | None:
        """Read battery remaining % from SYS_STATUS."""
        msg = self.conn.recv_match(type="SYS_STATUS", blocking=False)
        if msg:
            return msg.battery_remaining
        return None

    def get_heartbeat(self) -> dict | None:
        """Read latest HEARTBEAT for mode and armed state."""
        msg = self.conn.recv_match(type="HEARTBEAT", blocking=False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            return {"mode": mode, "armed": armed}
        return None

    def get_local_position(self) -> dict | None:
        """Read latest LOCAL_POSITION_NED."""
        msg = self.conn.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg:
            return {
                "x": msg.x, "y": msg.y, "z": msg.z,
                "vx": msg.vx, "vy": msg.vy, "vz": msg.vz,
            }
        return None

    def drain(self):
        """Drain all pending MAVLink messages to keep buffer clean."""
        while self.conn.recv_match(blocking=False) is not None:
            pass
