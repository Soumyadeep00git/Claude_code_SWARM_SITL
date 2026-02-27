"""pymavlink connection wrapper for dockerized SITL."""

import logging
import time

from pymavlink import mavutil

from docker_sim.config import SITL_PORT, DRONE_ID

log = logging.getLogger(__name__)


class MavlinkConn:
    """Manages a single pymavlink TCP connection to a SITL instance."""

    def __init__(self, timeout: float = 60.0):
        self.drone_id = DRONE_ID
        conn_str = f"tcp:127.0.0.1:{SITL_PORT}"

        log.info("Connecting to SITL at %s ...", conn_str)
        self.conn = mavutil.mavlink_connection(conn_str, source_system=255)
        self.conn.wait_heartbeat(timeout=timeout)

        self.sysid = self.conn.target_system
        if self.sysid == 0:
            self.sysid = DRONE_ID + 1
        self.compid = self.conn.target_component or 1

        log.info("Connected drone_id=%d sysid=%d compid=%d",
                 DRONE_ID, self.sysid, self.compid)

        self.conn.mav.request_data_stream_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1)

    def set_mode(self, mode_name: str):
        mode_map = self.conn.mode_mapping()
        if mode_name not in mode_map:
            log.error("Unknown mode: %s", mode_name)
            return
        mode_id = mode_map[mode_name]
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0)
        log.info("drone_id=%d: set_mode(%s)", self.drone_id, mode_name)

    def arm(self):
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        log.info("drone_id=%d: arm", self.drone_id)

    def force_disarm(self):
        """Force disarm (KILL) — motors stop immediately regardless of state."""
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196, 0, 0, 0, 0, 0)  # param2=21196 = force
        log.info("drone_id=%d: FORCE DISARM (KILL)", self.drone_id)

    def takeoff(self, alt_m: float):
        self.conn.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt_m)
        log.info("drone_id=%d: takeoff to %.1fm", self.drone_id, alt_m)

    def send_velocity_ned(self, vn: float, ve: float, vd: float):
        self.conn.mav.set_position_target_local_ned_send(
            0, self.sysid, self.compid,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_1111_1100_0111,
            0, 0, 0,
            vn, ve, vd,
            0, 0, 0,
            0, 0)

    def drain_latest(self) -> dict:
        """Drain buffer, return latest position/heartbeat."""
        last_pos = None
        last_hb = None

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

        return result
