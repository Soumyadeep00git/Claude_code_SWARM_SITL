"""Swarm Bridge Node — peer-to-peer + WiFi GCS bridge.

Peer-to-peer link — two selectable transports (param peer_transport):
  "rfd"  — RFD900x serial, MAVLink2 (original)
           TX 10 Hz: GLOBAL_POSITION_INT + NAMED_VALUE_INT/gmode
           TX  1 Hz: HEARTBEAT + NAMED_VALUE_FLOAT (battery x3)
           RX: GLOBAL_POSITION_INT, NAMED_VALUE_INT/FLOAT, HEARTBEAT,
               RADIO_STATUS (ID 109)
  "wifi" — plain UDP socket (!i15d struct)
           TX 10 Hz: drone_id + 15 doubles (pos, vel, heading, FC, batt)
           RX: same packet from peer

GCS link (WiFi/Ethernet UDP — always active):
  TX at 10 Hz: !i21d extended telemetry (FC, battery, radio quality)
  RX: variable-length commands (RTL, LAND, KILL, WASD, WP, etc.)

MAVLink messages handled:
  ID  0  HEARTBEAT             TX 1 Hz  (FC state → peer)
  ID  1  SYS_STATUS            RX       (battery via /mavros/battery)
  ID 33  GLOBAL_POSITION_INT   TX 10 Hz (pos/vel → peer)
  ID 109 RADIO_STATUS          RX       (modem-injected, link quality)
  ID 252 NAMED_VALUE_FLOAT     TX 1 Hz  (battery → peer)
  ID 254 NAMED_VALUE_INT       TX 10 Hz (guidance_mode → peer)

Subscribes:
    /mavros/global_position/global       — NavSatFix          (GPS)
    /mavros/global_position/rel_alt      — Float64            (relative alt)
    /mavros/local_position/velocity_local — TwistStamped      (ENU → NED)
    /mavros/global_position/compass_hdg  — Float64            (heading)
    /mavros/state                        — State              (FC health)
    /mavros/battery                      — BatteryState       (SYS_STATUS)
    /swarm/guidance_mode                 — Float64            (from control node)

Publishes:
    /swarm/peer_states   — PeerStates   (peer telem + health + radio)
    /swarm/gcs_command   — SwarmCommand (decoded GCS commands)

Threads:
  1. Radio TX  or  Peer UDP TX — own state to peer @ 10 Hz
  2. Radio RX  or  Peer UDP RX — peer state reception
  3. GCS TX    — !i21d telemetry @ 10 Hz
  4. GCS CMD RX — UDP command listener
"""

import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State

from pymavlink import mavutil

from swarm_msgs.msg import PeerState, PeerStates, SwarmCommand

from .config_loader import load_hardware_config


# ═══════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════

# GCS UDP telemetry: int32 drone_id + 21 doubles
# [lat, lon, alt, vn, ve, vd, heading, timestamp, gmode,
#  fc_connected, fc_armed, fc_mode_code, fc_system_status,
#  batt_voltage, batt_current, batt_remaining,
#  radio_rssi, radio_remrssi, radio_txbuf, radio_noise, radio_remnoise]
_GCS_TELEM_FMT = "!i21d"

# GCS command wire formats
_CMD_WASD_FMT = "!Bff"
_CMD_WAYPOINT_FMT = "!Bdd"
_CMD_SPEED_FMT = "!Bf"
_CMD_ALT_FMT = "!Bf"

# Peer WiFi UDP: int32 drone_id + 15 doubles
# [lat, lon, alt, vn, ve, vd, heading, timestamp, gmode,
#  fc_connected, fc_armed, fc_mode_code,
#  batt_voltage, batt_current, batt_remaining]
_PEER_UDP_FMT = "!i15d"

CMD_RTL = 1
CMD_LAND = 2
CMD_KILL = 3
CMD_FOLLOW = 4
CMD_HOVER = 5
CMD_TAKEOFF = 6
CMD_WASD = 10
CMD_WAYPOINT = 11
CMD_SPEED = 12
CMD_ALTITUDE = 13

_SIMPLE_CMDS = {CMD_RTL, CMD_LAND, CMD_KILL, CMD_FOLLOW, CMD_HOVER, CMD_TAKEOFF}

# MAVLink NAMED_VALUE names (10-byte null-padded)
_GMODE_NAME = b"gmode\x00\x00\x00\x00\x00"
_BATT_V_NAME = b"batt_v\x00\x00\x00\x00"
_BATT_A_NAME = b"batt_a\x00\x00\x00\x00"
_BATT_R_NAME = b"batt_r\x00\x00\x00\x00"

# ArduPilot Copter mode name → mode number
_ARDU_MODE_CODES = {
    'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3,
    'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'CIRCLE': 7,
    'LAND': 9, 'DRIFT': 11, 'SPORT': 13, 'FLIP': 14,
    'AUTOTUNE': 15, 'POSHOLD': 16, 'BRAKE': 17,
    'THROW': 18, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21,
}

# Timeouts
_PEER_JETSON_TIMEOUT = 5.0    # seconds before peer Jetson considered dead
_RADIO_STATUS_TIMEOUT = 5.0   # seconds before radio link considered dead


# ═══════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════

def _safe_float(v, default=0.0):
    """Return float(v), replacing NaN/Inf/errors with *default*."""
    try:
        fv = float(v)
        if math.isnan(fv) or math.isinf(fv):
            return default
        return fv
    except (TypeError, ValueError):
        return default


def _clamp(v, lo, hi):
    """Clamp v to [lo, hi]."""
    return max(lo, min(hi, v))


# ═══════════════════════════════════════════════════════════════════
# Node
# ═══════════════════════════════════════════════════════════════════

class SwarmBridgeNode(Node):
    def __init__(self):
        super().__init__('swarm_bridge_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('hierarchy_path', '/home/orin/hierarchy.yaml')
        self.declare_parameter('gcs_host', '')
        self.declare_parameter('peer_stale_timeout', 5.0)
        self.declare_parameter('radio_device', '/dev/ttyUSB1')
        self.declare_parameter('radio_baud', 115200)
        self.declare_parameter('peer_transport', 'rfd')   # "rfd" or "wifi"
        self.declare_parameter('peer_host', '')            # peer Jetson IP
        self.declare_parameter('peer_udp_port', 14560)    # UDP port

        drone_id = self.get_parameter('drone_id').value
        hierarchy_path = self.get_parameter('hierarchy_path').value
        gcs_host = self.get_parameter('gcs_host').value
        stale_timeout = self.get_parameter('peer_stale_timeout').value
        radio_device = self.get_parameter('radio_device').value
        radio_baud = self.get_parameter('radio_baud').value
        self._peer_transport = self.get_parameter('peer_transport').value
        self._peer_host = self.get_parameter('peer_host').value
        self._peer_udp_port = self.get_parameter('peer_udp_port').value

        # ── Load config from hierarchy ────────────────────────────
        self._cfg = load_hardware_config(
            hierarchy_path=hierarchy_path,
            drone_id=drone_id,
            gcs_host=gcs_host,
            peer_stale_timeout=stale_timeout,
        )

        self.get_logger().info(
            f"SwarmBridge: drone_id={self._cfg.drone_id} "
            f"role={self._cfg.role} peers={self._cfg.peer_ids} "
            f"peer_transport={self._peer_transport} "
            f"gcs={self._cfg.gcs_host}:{self._cfg.gcs_telem_port}"
        )

        # ── Own state (updated by MAVROS2 subscriptions) ──────────
        self._own_lat = 0.0
        self._own_lon = 0.0
        self._own_alt = 0.0
        self._own_vn = 0.0
        self._own_ve = 0.0
        self._own_vd = 0.0
        self._own_heading = 0.0
        self._guidance_mode = 0
        self._fc_connected = False
        self._fc_armed = False
        self._fc_mode_str = ""
        self._fc_system_status = 0      # MAV_STATE enum (0=UNINIT)
        self._own_lock = threading.Lock()
        self._boot_time = time.time()

        # ── Battery state (from /mavros/battery — SYS_STATUS ID1) ─
        self._batt_voltage = 0.0        # Volts
        self._batt_current = 0.0        # Amps
        self._batt_remaining = -1.0     # 0.0–1.0 (-1 = unknown)

        # ── Radio link quality (from RADIO_STATUS ID109) ──────────
        self._radio_rssi = 0
        self._radio_remrssi = 0
        self._radio_txbuf = 0
        self._radio_noise = 0
        self._radio_remnoise = 0
        self._radio_rxerrors = 0
        self._radio_last_status = 0.0

        # ── Peer state storage ────────────────────────────────────
        self._peer_states: dict = {}          # drone_id -> state dict
        self._peer_recv_times: dict = {}      # drone_id -> timestamp
        self._peer_gmode: dict = {}           # drone_id -> guidance_mode
        self._peer_jetson_last_hb: dict = {}  # drone_id -> timestamp
        self._peer_fc_connected: dict = {}    # drone_id -> bool
        self._peer_fc_armed: dict = {}        # drone_id -> bool
        self._peer_fc_mode_code: dict = {}    # drone_id -> int
        self._peer_batt_voltage: dict = {}    # drone_id -> float
        self._peer_batt_current: dict = {}    # drone_id -> float
        self._peer_batt_remaining: dict = {}  # drone_id -> float
        self._peer_lock = threading.Lock()

        # ── Peer transport setup ──────────────────────────────────
        self._radio = None
        self._radio_write_lock = threading.Lock()
        if self._peer_transport == 'rfd':
            try:
                self.get_logger().info(
                    f"Opening RFD900x at {radio_device} @ {radio_baud} baud...")
                self._radio = mavutil.mavlink_connection(
                    radio_device,
                    baud=radio_baud,
                    source_system=self._cfg.drone_id,
                    source_component=1,
                )
                self.get_logger().info("RFD900x serial connection established")
            except Exception as e:
                self.get_logger().error(
                    f"FAILED to open RFD900x at {radio_device}: {e} "
                    f"— radio threads will not start")
        elif self._peer_transport == 'wifi':
            self.get_logger().info(
                f"Peer WiFi UDP -> {self._peer_host}:{self._peer_udp_port}")

        # ── ROS2 Subscribers (MAVROS2) ────────────────────────────
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self._on_global_pos, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            self._on_rel_alt, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_velocity, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg',
            self._on_heading, qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state', self._on_fc_state, 10)
        self.create_subscription(
            BatteryState, '/mavros/battery', self._on_battery, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/swarm/guidance_mode',
            self._on_guidance_mode, 10)

        # ── ROS2 Publishers ───────────────────────────────────────
        self._peer_pub = self.create_publisher(
            PeerStates, '/swarm/peer_states', 10)
        self._cmd_pub = self.create_publisher(
            SwarmCommand, '/swarm/gcs_command', 10)

        # ── Publish peer states at 10 Hz ──────────────────────────
        self._peer_publish_timer = self.create_timer(
            0.1, self._publish_peer_states)

        # ── Start threads ─────────────────────────────────────────
        self._running = True

        if self._peer_transport == 'wifi':
            if self._peer_host:
                threading.Thread(target=self._peer_udp_tx_loop, daemon=True,
                                 name='peer-udp-tx').start()
                threading.Thread(target=self._peer_udp_rx_loop, daemon=True,
                                 name='peer-udp-rx').start()
            else:
                self.get_logger().warning(
                    "Peer UDP threads NOT started — peer_host not set")
        else:  # rfd
            if self._radio is not None:
                threading.Thread(target=self._radio_tx_loop, daemon=True,
                                 name='radio-tx').start()
                threading.Thread(target=self._radio_rx_loop, daemon=True,
                                 name='radio-rx').start()
            else:
                self.get_logger().warning(
                    "Radio threads NOT started — no serial connection")

        threading.Thread(target=self._gcs_tx_loop, daemon=True,
                         name='gcs-tx').start()
        threading.Thread(target=self._gcs_cmd_rx_loop, daemon=True,
                         name='gcs-cmd-rx').start()

    # ═══════════════════════════════════════════════════════════════
    # MAVROS2 callbacks (all thread-safe via _own_lock)
    # ═══════════════════════════════════════════════════════════════

    def _on_global_pos(self, msg: NavSatFix):
        with self._own_lock:
            self._own_lat = _safe_float(msg.latitude)
            self._own_lon = _safe_float(msg.longitude)

    def _on_rel_alt(self, msg: Float64):
        with self._own_lock:
            self._own_alt = _safe_float(msg.data)

    def _on_velocity(self, msg: TwistStamped):
        # MAVROS2 velocity_local is ENU → convert to NED
        with self._own_lock:
            self._own_vn = _safe_float(msg.twist.linear.y)    # North = ENU.y
            self._own_ve = _safe_float(msg.twist.linear.x)    # East  = ENU.x
            self._own_vd = _safe_float(-msg.twist.linear.z)   # Down  = -ENU.z

    def _on_heading(self, msg: Float64):
        with self._own_lock:
            self._own_heading = _safe_float(msg.data)

    def _on_fc_state(self, msg: State):
        """Track FC connection, armed state, flight mode, and system_status."""
        with self._own_lock:
            prev_conn = self._fc_connected
            self._fc_connected = msg.connected
            self._fc_armed = msg.armed
            self._fc_mode_str = msg.mode
            self._fc_system_status = msg.system_status
        # Log transitions outside lock
        if msg.connected and not prev_conn:
            self.get_logger().info("MAVROS2 <-> CubeOrange+ connected")
        elif not msg.connected and prev_conn:
            self.get_logger().warning("MAVROS2 <-> CubeOrange+ DISCONNECTED")

    def _on_battery(self, msg: BatteryState):
        """Track battery voltage, current, and remaining from SYS_STATUS."""
        with self._own_lock:
            self._batt_voltage = _safe_float(msg.voltage, 0.0)
            self._batt_current = _safe_float(msg.current, 0.0)
            # percentage: MAVROS2 reports 0.0–1.0, some setups 0–100
            pct = _safe_float(msg.percentage, -1.0)
            if pct > 1.0:
                pct /= 100.0  # normalize to 0.0–1.0
            self._batt_remaining = _clamp(pct, -1.0, 1.0)

    def _on_guidance_mode(self, msg: Float64):
        with self._own_lock:
            self._guidance_mode = int(_safe_float(msg.data, 0.0))

    # ═══════════════════════════════════════════════════════════════
    # Peer state publishing (ROS2 PeerStates at 10 Hz)
    # ═══════════════════════════════════════════════════════════════

    def _publish_peer_states(self):
        """Publish non-stale peer states + own battery + radio quality."""
        try:
            now = time.time()
            msg = PeerStates()
            msg.header_stamp = self.get_clock().now().to_msg()

            with self._peer_lock:
                for pid, state in self._peer_states.items():
                    recv_time = self._peer_recv_times.get(pid, 0.0)
                    if now - recv_time > self._cfg.peer_stale_timeout:
                        continue
                    peer_msg = PeerState()
                    peer_msg.drone_id = state['drone_id']
                    peer_msg.latitude = state['lat']
                    peer_msg.longitude = state['lon']
                    peer_msg.altitude = state['alt']
                    peer_msg.vn = state['vn']
                    peer_msg.ve = state['ve']
                    peer_msg.vd = state['vd']
                    peer_msg.heading = state['heading']
                    peer_msg.stamp = recv_time
                    peer_msg.guidance_mode = self._peer_gmode.get(pid, 0)
                    # FC state from peer HEARTBEAT
                    peer_msg.fc_mode_code = self._peer_fc_mode_code.get(
                        pid, 0)
                    peer_msg.fc_armed = self._peer_fc_armed.get(pid, False)
                    peer_msg.fc_connected = self._peer_fc_connected.get(
                        pid, False)
                    # Battery from peer NAMED_VALUE_FLOAT
                    peer_msg.battery_voltage = self._peer_batt_voltage.get(
                        pid, 0.0)
                    peer_msg.battery_current = self._peer_batt_current.get(
                        pid, 0.0)
                    peer_msg.battery_remaining = self._peer_batt_remaining.get(
                        pid, -1.0)
                    msg.peers.append(peer_msg)

                # Peer Jetson liveness
                any_alive = False
                latest_hb = 0.0
                for pid, hb_time in self._peer_jetson_last_hb.items():
                    if hb_time > latest_hb:
                        latest_hb = hb_time
                    if now - hb_time < _PEER_JETSON_TIMEOUT:
                        any_alive = True

            msg.peer_jetson_alive = any_alive
            msg.peer_last_heartbeat = latest_hb

            # Own battery (thread-safe read)
            with self._own_lock:
                msg.own_battery_voltage = self._batt_voltage
                msg.own_battery_current = self._batt_current
                msg.own_battery_remaining = self._batt_remaining
                # Radio link quality
                msg.radio_rssi = self._radio_rssi
                msg.radio_remrssi = self._radio_remrssi
                msg.radio_txbuf = self._radio_txbuf
                msg.radio_noise = self._radio_noise
                msg.radio_remnoise = self._radio_remnoise
                msg.radio_rxerrors = self._radio_rxerrors
                msg.radio_link_alive = (
                    now - self._radio_last_status < _RADIO_STATUS_TIMEOUT)
                msg.radio_last_status = self._radio_last_status

            self._peer_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Peer state publish error: {e}")

    # ═══════════════════════════════════════════════════════════════
    # RADIO TX: own state → peer via RFD900x (10 Hz pos, 1 Hz health)
    # ═══════════════════════════════════════════════════════════════

    def _radio_tx_loop(self):
        """Send position @ 10 Hz + heartbeat/battery @ 1 Hz over RFD900x."""
        last_heartbeat = 0.0
        self.get_logger().info("Radio TX thread started")

        while self._running and rclpy.ok():
            try:
                now = time.time()
                time_boot_ms = (
                    int((now - self._boot_time) * 1000) & 0xFFFFFFFF)

                with self._own_lock:
                    lat = self._own_lat
                    lon = self._own_lon
                    alt = self._own_alt
                    vn = self._own_vn
                    ve = self._own_ve
                    vd = self._own_vd
                    heading = self._own_heading
                    gmode = self._guidance_mode

                # ── 1 Hz: HEARTBEAT + battery relay ───────────────
                if now - last_heartbeat >= 1.0:
                    self._radio_send_heartbeat()
                    self._radio_send_battery(time_boot_ms)
                    last_heartbeat = now

                # ── 10 Hz: position + guidance mode ───────────────
                if lat != 0.0 or lon != 0.0:
                    lat_e7 = int(lat * 1e7)
                    lon_e7 = int(lon * 1e7)
                    alt_mm = int(alt * 1000)
                    vx_cms = int(_clamp(vn * 100, -32767, 32767))
                    vy_cms = int(_clamp(ve * 100, -32767, 32767))
                    vz_cms = int(_clamp(vd * 100, -32767, 32767))
                    hdg_cdeg = int(heading * 100) % 36000

                    with self._radio_write_lock:
                        self._radio.mav.global_position_int_send(
                            time_boot_ms,
                            lat_e7, lon_e7,
                            alt_mm, alt_mm,   # AMSL ~ relative
                            vx_cms, vy_cms, vz_cms,
                            hdg_cdeg,
                        )
                        self._radio.mav.named_value_int_send(
                            time_boot_ms,
                            _GMODE_NAME,
                            gmode,
                        )

            except Exception as e:
                self.get_logger().warning(f"Radio TX error: {e}")

            time.sleep(0.1)  # 10 Hz

    def _radio_send_heartbeat(self):
        """Send 1 Hz HEARTBEAT with FC state encoded in MAVLink fields.

        Encoding:
          base_mode bit 7 (0x80) = armed
          custom_mode = ArduPilot mode number (0=STABILIZE, 4=GUIDED, etc.)
          system_status = FC system_status if connected, UNINIT if not
        """
        with self._own_lock:
            fc_conn = self._fc_connected
            fc_arm = self._fc_armed
            fc_mode = self._fc_mode_str
            sys_status = self._fc_system_status

        base_mode = 0x80 if fc_arm else 0
        custom_mode = _ARDU_MODE_CODES.get(fc_mode, 255)

        # Forward actual FC system_status when available
        if fc_conn and sys_status > 0:
            mav_status = sys_status
        elif fc_conn:
            mav_status = mavutil.mavlink.MAV_STATE_ACTIVE
        else:
            mav_status = mavutil.mavlink.MAV_STATE_UNINIT

        try:
            with self._radio_write_lock:
                self._radio.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    base_mode,
                    custom_mode,
                    mav_status,
                )
        except Exception as e:
            self.get_logger().warning(f"Heartbeat send failed: {e}")

    def _radio_send_battery(self, time_boot_ms: int):
        """Send battery as 3 NAMED_VALUE_FLOAT messages at 1 Hz.

        Messages: batt_v (Volts), batt_a (Amps), batt_r (0.0-1.0 remaining).
        ~66 bytes total — negligible on RFD900x bandwidth.
        """
        with self._own_lock:
            bv = _safe_float(self._batt_voltage, 0.0)
            ba = _safe_float(self._batt_current, 0.0)
            br = _safe_float(self._batt_remaining, -1.0)

        try:
            with self._radio_write_lock:
                self._radio.mav.named_value_float_send(
                    time_boot_ms, _BATT_V_NAME, bv)
                self._radio.mav.named_value_float_send(
                    time_boot_ms, _BATT_A_NAME, ba)
                self._radio.mav.named_value_float_send(
                    time_boot_ms, _BATT_R_NAME, br)
        except Exception as e:
            self.get_logger().warning(f"Battery relay send failed: {e}")

    # ═══════════════════════════════════════════════════════════════
    # RADIO RX: peer state + RADIO_STATUS from RFD900x serial
    # ═══════════════════════════════════════════════════════════════

    def _radio_rx_loop(self):
        """Read MAVLink from RFD900x — peer state + radio quality."""
        self.get_logger().info(
            "Radio RX thread started — listening for peer MAVLink")

        while self._running and rclpy.ok():
            try:
                msg = self._radio.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue

                mtype = msg.get_type()
                sender_id = msg.get_srcSystem()

                # ── RADIO_STATUS from local modem (before sender filter) ─
                # Modem injects this locally (sys_id ~51), not from peer
                if mtype == 'RADIO_STATUS':
                    self._handle_radio_status(msg)
                    continue

                # Skip our own messages (shouldn't happen point-to-point)
                if sender_id == self._cfg.drone_id:
                    continue

                if mtype == 'GLOBAL_POSITION_INT':
                    self._handle_peer_position(sender_id, msg)

                elif mtype == 'NAMED_VALUE_INT':
                    name = msg.name.rstrip('\x00')
                    if name == 'gmode':
                        with self._peer_lock:
                            self._peer_gmode[sender_id] = msg.value

                elif mtype == 'NAMED_VALUE_FLOAT':
                    self._handle_peer_battery(sender_id, msg)

                elif mtype == 'HEARTBEAT':
                    self._handle_peer_heartbeat(sender_id, msg)

            except Exception as e:
                if self._running:
                    self.get_logger().debug(f'Radio RX error: {e}')
                    time.sleep(0.1)

    def _handle_radio_status(self, msg):
        """Process RADIO_STATUS (ID 109) — RFD900x link quality.

        Fields from SiK firmware:
          rssi/remrssi  — signal strength 0-255 (higher = stronger)
          txbuf         — TX buffer free % 0-100 (low = congestion)
          noise/remnoise — background noise 0-255 (lower = better)
          rxerrors      — cumulative receive errors
        """
        with self._own_lock:
            was_alive = (
                time.time() - self._radio_last_status
                < _RADIO_STATUS_TIMEOUT)
            self._radio_rssi = _clamp(msg.rssi, 0, 255)
            self._radio_remrssi = _clamp(msg.remrssi, 0, 255)
            self._radio_txbuf = _clamp(msg.txbuf, 0, 100)
            self._radio_noise = _clamp(msg.noise, 0, 255)
            self._radio_remnoise = _clamp(msg.remnoise, 0, 255)
            self._radio_rxerrors = max(0, msg.rxerrors)
            self._radio_last_status = time.time()

        if not was_alive:
            self.get_logger().info(
                f"RFD900x link alive — RSSI={msg.rssi} "
                f"RemRSSI={msg.remrssi} TXbuf={msg.txbuf}%")

    def _handle_peer_position(self, sender_id: int, msg):
        """Process GLOBAL_POSITION_INT (ID 33) from peer."""
        peer = {
            'drone_id': sender_id,
            'lat': msg.lat / 1e7,
            'lon': msg.lon / 1e7,
            'alt': msg.relative_alt / 1000.0,   # mm → m
            'vn': msg.vx / 100.0,                # cm/s → m/s
            've': msg.vy / 100.0,
            'vd': msg.vz / 100.0,
            'heading': msg.hdg / 100.0,           # cdeg → deg
        }
        # Ignore zero-position (GPS not yet locked)
        if peer['lat'] != 0.0 or peer['lon'] != 0.0:
            with self._peer_lock:
                self._peer_states[sender_id] = peer
                self._peer_recv_times[sender_id] = time.time()

    def _handle_peer_battery(self, sender_id: int, msg):
        """Process NAMED_VALUE_FLOAT battery fields from peer."""
        name = msg.name.rstrip('\x00')
        val = _safe_float(msg.value, 0.0)
        with self._peer_lock:
            if name == 'batt_v':
                self._peer_batt_voltage[sender_id] = val
            elif name == 'batt_a':
                self._peer_batt_current[sender_id] = val
            elif name == 'batt_r':
                self._peer_batt_remaining[sender_id] = val

    def _handle_peer_heartbeat(self, sender_id: int, msg):
        """Process HEARTBEAT from peer — FC state + Jetson liveness."""
        with self._peer_lock:
            was_alive = (
                sender_id in self._peer_jetson_last_hb
                and time.time() - self._peer_jetson_last_hb[sender_id]
                < _PEER_JETSON_TIMEOUT)
            self._peer_jetson_last_hb[sender_id] = time.time()
            self._peer_fc_armed[sender_id] = bool(msg.base_mode & 0x80)
            self._peer_fc_mode_code[sender_id] = msg.custom_mode
            self._peer_fc_connected[sender_id] = (
                msg.system_status != mavutil.mavlink.MAV_STATE_UNINIT)

        if not was_alive:
            self.get_logger().info(
                f"Peer drone {sender_id} Jetson alive — "
                f"FC mode={msg.custom_mode} "
                f"armed={bool(msg.base_mode & 0x80)}")

    # ═══════════════════════════════════════════════════════════════
    # PEER UDP TX/RX: WiFi transport (alternative to RFD900x)
    # ═══════════════════════════════════════════════════════════════

    def _peer_udp_tx_loop(self):
        """Send own state to peer via UDP at 10 Hz.

        Packet: _PEER_UDP_FMT = !i15d  (124 bytes)
        Same data as radio TX but in one flat struct — no MAVLink overhead.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest = (self._peer_host, self._peer_udp_port)
        self.get_logger().info(f"Peer UDP TX thread started -> {dest}")

        while self._running and rclpy.ok():
            try:
                now = time.time()
                with self._own_lock:
                    lat = self._own_lat
                    lon = self._own_lon
                    alt = self._own_alt
                    vn = self._own_vn
                    ve = self._own_ve
                    vd = self._own_vd
                    heading = self._own_heading
                    gmode = self._guidance_mode
                    fc_conn = self._fc_connected
                    fc_arm = self._fc_armed
                    fc_mode = self._fc_mode_str
                    batt_v = self._batt_voltage
                    batt_a = self._batt_current
                    batt_r = self._batt_remaining

                if lat != 0.0 or lon != 0.0:
                    data = struct.pack(
                        _PEER_UDP_FMT,
                        self._cfg.drone_id,
                        lat, lon, alt, vn, ve, vd, heading,
                        now, float(gmode),
                        1.0 if fc_conn else 0.0,
                        1.0 if fc_arm else 0.0,
                        float(_ARDU_MODE_CODES.get(fc_mode, 255)),
                        _safe_float(batt_v),
                        _safe_float(batt_a),
                        _safe_float(batt_r, -1.0),
                    )
                    sock.sendto(data, dest)

            except (OSError, socket.gaierror) as e:
                self.get_logger().debug(f'Peer UDP TX error: {e}')

            time.sleep(0.1)  # 10 Hz

        sock.close()

    def _peer_udp_rx_loop(self):
        """Receive peer state via UDP, populate same dicts as RFD path."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', self._peer_udp_port))
        except OSError as e:
            self.get_logger().error(
                f"Failed to bind peer UDP port {self._peer_udp_port}: {e}")
            return
        sock.settimeout(1.0)

        pkt_size = struct.calcsize(_PEER_UDP_FMT)
        self.get_logger().info(
            f"Peer UDP RX thread started <- 0.0.0.0:{self._peer_udp_port} "
            f"({pkt_size} bytes/pkt)")

        while self._running and rclpy.ok():
            try:
                data, addr = sock.recvfrom(256)
                if len(data) != pkt_size:
                    continue

                vals = struct.unpack(_PEER_UDP_FMT, data)
                sender_id = vals[0]

                # Ignore own echo (shouldn't happen on different IPs)
                if sender_id == self._cfg.drone_id:
                    continue

                (_, lat, lon, alt, vn, ve, vd, heading,
                 _timestamp, gmode,
                 fc_conn, fc_arm, fc_mode_code,
                 batt_v, batt_a, batt_r) = vals

                if lat == 0.0 and lon == 0.0:
                    continue

                peer = {
                    'drone_id': sender_id,
                    'lat': lat,
                    'lon': lon,
                    'alt': alt,
                    'vn': vn,
                    've': ve,
                    'vd': vd,
                    'heading': heading,
                }
                now = time.time()
                with self._peer_lock:
                    self._peer_states[sender_id] = peer
                    self._peer_recv_times[sender_id] = now
                    self._peer_gmode[sender_id] = int(gmode)
                    self._peer_jetson_last_hb[sender_id] = now
                    self._peer_fc_connected[sender_id] = bool(fc_conn)
                    self._peer_fc_armed[sender_id] = bool(fc_arm)
                    self._peer_fc_mode_code[sender_id] = int(fc_mode_code)
                    self._peer_batt_voltage[sender_id] = batt_v
                    self._peer_batt_current[sender_id] = batt_a
                    self._peer_batt_remaining[sender_id] = batt_r

            except socket.timeout:
                continue
            except (struct.error, OSError) as e:
                if self._running:
                    self.get_logger().debug(f'Peer UDP RX error: {e}')
                    time.sleep(0.1)

        sock.close()

    # ═══════════════════════════════════════════════════════════════
    # GCS TX: extended telemetry → GCS via WiFi UDP (10 Hz)
    # ═══════════════════════════════════════════════════════════════

    def _gcs_tx_loop(self):
        """Send !i21d telemetry: position + FC + battery + radio quality."""
        if not self._cfg.gcs_host or not self._cfg.gcs_telem_port:
            self.get_logger().info("GCS host not configured — skipping GCS TX")
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pkt_size = struct.calcsize(_GCS_TELEM_FMT)
        self.get_logger().info(
            f"GCS telemetry -> {self._cfg.gcs_host}:{self._cfg.gcs_telem_port}"
            f" ({pkt_size} bytes/pkt)")

        while self._running and rclpy.ok():
            try:
                with self._own_lock:
                    lat = _safe_float(self._own_lat)
                    lon = _safe_float(self._own_lon)
                    alt = _safe_float(self._own_alt)
                    vn = _safe_float(self._own_vn)
                    ve = _safe_float(self._own_ve)
                    vd = _safe_float(self._own_vd)
                    heading = _safe_float(self._own_heading)
                    gmode = float(self._guidance_mode)
                    fc_conn = self._fc_connected
                    fc_arm = self._fc_armed
                    fc_mode = self._fc_mode_str
                    fc_sys_status = self._fc_system_status
                    batt_v = _safe_float(self._batt_voltage)
                    batt_a = _safe_float(self._batt_current)
                    batt_r = _safe_float(self._batt_remaining, -1.0)
                    r_rssi = float(self._radio_rssi)
                    r_remrssi = float(self._radio_remrssi)
                    r_txbuf = float(self._radio_txbuf)
                    r_noise = float(self._radio_noise)
                    r_remnoise = float(self._radio_remnoise)

                if lat != 0.0 or lon != 0.0:
                    data = struct.pack(
                        _GCS_TELEM_FMT,
                        self._cfg.drone_id,
                        lat, lon, alt,
                        vn, ve, vd,
                        heading, time.time(), gmode,
                        1.0 if fc_conn else 0.0,
                        1.0 if fc_arm else 0.0,
                        float(_ARDU_MODE_CODES.get(fc_mode, 255)),
                        float(fc_sys_status),
                        batt_v, batt_a, batt_r,
                        r_rssi, r_remrssi, r_txbuf,
                        r_noise, r_remnoise,
                    )
                    sock.sendto(
                        data,
                        (self._cfg.gcs_host, self._cfg.gcs_telem_port))

            except struct.error as e:
                self.get_logger().error(f"GCS TX pack error: {e}")
            except (OSError, socket.gaierror) as e:
                self.get_logger().debug(f'GCS TX failed: {e}')

            time.sleep(0.1)  # 10 Hz

        sock.close()

    # ═══════════════════════════════════════════════════════════════
    # GCS CMD RX: receive commands from GCS via WiFi UDP
    # ═══════════════════════════════════════════════════════════════

    def _gcs_cmd_rx_loop(self):
        """Receive GCS commands via UDP, publish as SwarmCommand."""
        if not self._cfg.gcs_cmd_port:
            self.get_logger().info(
                "GCS cmd port not configured — skipping CMD RX")
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', self._cfg.gcs_cmd_port))
        except OSError as e:
            self.get_logger().error(
                f"Failed to bind GCS cmd port {self._cfg.gcs_cmd_port}: {e}")
            return
        sock.settimeout(1.0)

        self.get_logger().info(
            f'GCS commands <- UDP:{self._cfg.gcs_cmd_port}')

        while self._running and rclpy.ok():
            try:
                data, addr = sock.recvfrom(256)
                if len(data) < 1:
                    continue

                cmd_msg = self._parse_and_publish_command(data)
                if cmd_msg is not None:
                    self.get_logger().info(
                        f'GCS command: cmd={cmd_msg.cmd} from {addr}')

            except socket.timeout:
                continue
            except OSError as e:
                if self._running:
                    self.get_logger().warning(f'GCS CMD RX error: {e}')
                    time.sleep(0.5)

        sock.close()

    def _parse_and_publish_command(self, data: bytes):
        """Parse UDP command packet and publish as SwarmCommand."""
        if len(data) < 1:
            return None

        cmd = data[0]
        n = len(data)
        msg = SwarmCommand()

        try:
            # Simple 1-byte commands
            if n == 1 and cmd in _SIMPLE_CMDS:
                msg.cmd = cmd
                self._cmd_pub.publish(msg)
                return msg

            # WASD: "!Bff" -> 9 bytes
            wasd_size = struct.calcsize(_CMD_WASD_FMT)
            if n == wasd_size and cmd == CMD_WASD:
                _, vn, ve = struct.unpack(_CMD_WASD_FMT, data)
                msg.cmd = CMD_WASD
                msg.vn = _safe_float(vn)
                msg.ve = _safe_float(ve)
                self._cmd_pub.publish(msg)
                return msg

            # SPEED: "!Bf" -> 5 bytes
            speed_size = struct.calcsize(_CMD_SPEED_FMT)
            if n == speed_size and cmd == CMD_SPEED:
                _, speed = struct.unpack(_CMD_SPEED_FMT, data)
                msg.cmd = CMD_SPEED
                msg.speed = _safe_float(speed)
                self._cmd_pub.publish(msg)
                return msg

            # ALTITUDE: "!Bf" -> 5 bytes
            alt_size = struct.calcsize(_CMD_ALT_FMT)
            if n == alt_size and cmd == CMD_ALTITUDE:
                _, vd = struct.unpack(_CMD_ALT_FMT, data)
                msg.cmd = CMD_ALTITUDE
                msg.vd = _safe_float(vd)
                self._cmd_pub.publish(msg)
                return msg

            # WAYPOINT: "!Bdd" -> 17 bytes
            wp_size = struct.calcsize(_CMD_WAYPOINT_FMT)
            if n == wp_size and cmd == CMD_WAYPOINT:
                _, lat, lon = struct.unpack(_CMD_WAYPOINT_FMT, data)
                msg.cmd = CMD_WAYPOINT
                msg.lat = _safe_float(lat)
                msg.lon = _safe_float(lon)
                self._cmd_pub.publish(msg)
                return msg

        except struct.error as e:
            self.get_logger().warning(f'Command parse error: {e}')
            return None

        self.get_logger().warning(f'Unknown command: cmd={cmd} len={n}')
        return None

    # ═══════════════════════════════════════════════════════════════
    # Shutdown
    # ═══════════════════════════════════════════════════════════════

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
