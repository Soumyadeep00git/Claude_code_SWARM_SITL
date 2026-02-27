"""pymavlink connection wrapper for SITL test."""

import logging
import time

from pymavlink import mavutil

from sim.config import SITL_BASE_PORT, SITL_PORT_STEP

log = logging.getLogger(__name__)


class MavlinkConn:
    """Manages a single pymavlink TCP connection to a SITL instance."""

    def __init__(self, drone_id: int, timeout: float = 60.0):
        self.drone_id = drone_id
        port = SITL_BASE_PORT + drone_id * SITL_PORT_STEP
        conn_str = f"tcp:127.0.0.1:{port}"

        log.info("Connecting to drone_id=%d at %s ...", drone_id, conn_str)
        self.conn = mavutil.mavlink_connection(conn_str, source_system=255)
        self.conn.wait_heartbeat(timeout=timeout)

        # Handle sysid=0 quirk (some SITL instances report 0)
        self.sysid = self.conn.target_system
        if self.sysid == 0:
            self.sysid = drone_id + 1
        self.compid = self.conn.target_component or 1

        log.info("Connected drone_id=%d sysid=%d compid=%d",
                 drone_id, self.sysid, self.compid)

        # Request all streams at 10Hz
        self.conn.mav.request_data_stream_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1)

    def set_mode(self, mode_name: str):
        """Set flight mode by name (GUIDED, LAND, RTL, LOITER, etc.)."""
        mode_map = self.conn.mode_mapping()
        if mode_name not in mode_map:
            log.error("Unknown mode: %s (available: %s)", mode_name, list(mode_map.keys()))
            return
        mode_id = mode_map[mode_name]
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0)
        log.info("drone_id=%d: set_mode(%s)", self.drone_id, mode_name)

    def arm(self):
        """Send arm command."""
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        log.info("drone_id=%d: arm", self.drone_id)

    def disarm(self):
        """Send disarm command."""
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)

    def takeoff(self, alt_m: float):
        """Send takeoff command."""
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt_m)
        log.info("drone_id=%d: takeoff to %.1fm", self.drone_id, alt_m)

    def send_velocity_ned(self, vn: float, ve: float, vd: float):
        """Send NED velocity setpoint."""
        self.conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.sysid, self.compid,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_1111_1100_0111,  # type_mask: velocity only
            0, 0, 0,       # position (ignored)
            vn, ve, vd,    # velocity
            0, 0, 0,       # acceleration (ignored)
            0, 0)          # yaw, yaw_rate (ignored)

    def drain_latest(self) -> dict:
        """Drain entire MAVLink buffer, return latest of each message type.

        Returns dict with optional keys:
            'position': {lat, lon, alt, vx, vy, vz, heading}
            'heartbeat': {mode, armed}
            'local': {x, y, z, vx, vy, vz}
        """
        last_pos = None
        last_hb = None
        last_local = None

        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "GLOBAL_POSITION_INT":
                last_pos = msg
            elif msg_type == "HEARTBEAT":
                src = msg.get_srcSystem()
                if src > 0 and src != 255:
                    last_hb = msg
            elif msg_type == "LOCAL_POSITION_NED":
                last_local = msg

        result = {}

        if last_pos is not None:
            result['position'] = {
                'lat': last_pos.lat / 1e7,
                'lon': last_pos.lon / 1e7,
                'alt': last_pos.relative_alt / 1000.0,
                'vx': last_pos.vx / 100.0,
                'vy': last_pos.vy / 100.0,
                'vz': last_pos.vz / 100.0,
                'heading': last_pos.hdg / 100.0,
            }

        if last_hb is not None:
            mode = mavutil.mode_string_v10(last_hb)
            armed = bool(last_hb.base_mode
                         & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            result['heartbeat'] = {'mode': mode, 'armed': armed}

        if last_local is not None:
            result['local'] = {
                'x': last_local.x, 'y': last_local.y, 'z': last_local.z,
                'vx': last_local.vx, 'vy': last_local.vy, 'vz': last_local.vz,
            }

        return result
