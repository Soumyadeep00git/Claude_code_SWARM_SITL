#!/usr/bin/env python3
"""Follower Event Logger — captures ALL MAVROS + swarm data at native rates.

EVENT-DRIVEN: one CSV row per ROS2 message received. No fixed timer.
Each row is a full state snapshot (self-contained for pandas analysis).

╔══════════════════════════════════════════════════════════════════════╗
║  TOPICS SUBSCRIBED  (with expected rates on ArduCopter V4.6)        ║
╠══════════════════════════════════════════════════════════════════════╣
║  /mavros/global_position/global        NavSatFix       ~5-10 Hz     ║
║  /mavros/global_position/rel_alt       Float64         ~5-10 Hz     ║
║  /mavros/local_position/velocity_local TwistStamped    ~4-10 Hz     ║
║  /mavros/global_position/compass_hdg   Float64         ~5-10 Hz     ║
║  /mavros/local_position/pose           PoseStamped     ~4-10 Hz     ║
║  /mavros/state                         State           ~1    Hz     ║
║  /mavros/battery                       BatteryState    ~1    Hz     ║
║  /swarm/peer_states                    PeerStates      ~10   Hz     ║
║  /swarm/gcs_command                    SwarmCommand    event        ║
║  /mavros/setpoint_raw/local            PositionTarget  ~10   Hz     ║
║  /swarm/guidance_mode                  Float64         ~10   Hz     ║
║  /swarm/mission_state                  String          event        ║
╚══════════════════════════════════════════════════════════════════════╝

Actual rates depend on ArduPilot SR2_* stream rate parameters.
Run `ros2 topic hz /mavros/global_position/global` to measure live.

CSV COLUMNS (every row is a complete state snapshot):
─────────────────────────────────────────────────────
Timestamps:
  gcs_ts          GCS-corrected unix time (for cross-Jetson alignment)
  local_ts        Raw local Jetson unix time
  sync_off_ms     Clock offset: gcs = local + offset (ms)

Event metadata:
  event           What topic triggered this row:
                    GPS, ALT, VEL, HDG, POSE, STATE, BATT — own FC data
                    PEER — leader data via RFD900x radio
                    CMD — velocity command sent TO own FC by control node
                    GCS_CMD — high-level GCS command (FOLLOW, RTL, etc.)
                    GUID — guidance mode change
                    MISSION — mission state change
  seq             Per-topic sequence number (for rate calculation)

Own FC position (from MAVROS2 ← own CubeOrange+ EKF3-fused):
  lat             Latitude (deg, 8 decimal places ≈ 1mm)
  lon             Longitude (deg)
  rel_alt         Relative altitude above home (m)

Own FC velocity (EKF3-fused, NED frame):
  vn              North velocity (m/s)
  ve              East velocity (m/s)
  vd              Down velocity (m/s, positive = descending)

Own heading:
  heading         Compass heading (0–360°)

Own EKF local position (from /mavros/local_position/pose):
  local_x         EKF local East (m, from EKF origin) — ENU frame
  local_y         EKF local North (m)
  local_z         EKF local Up (m)
                  USEFUL FOR: high-resolution relative motion analysis.
                  GPS lat/lon has ~1cm quantization. Local position is
                  continuous and integrates IMU between GPS updates.

Own FC state:
  fc_mode         Flight mode string (GUIDED, STABILIZE, LOITER, RTL...)
  fc_armed        Armed flag (True/False)
  fc_connected    MAVROS↔FC serial link status
  fc_sys_status   MAV_STATE enum (0=UNINIT, 3=STANDBY, 4=ACTIVE)

Own battery:
  batt_v          Battery voltage (V)
  batt_a          Battery current (A, negative = discharging on some FCs)
  batt_pct        Remaining capacity (0.0–1.0, -1 = unknown)

Leader data (from /swarm/peer_states — via RFD900x radio):
  ldr_sysid       Leader's MAVLink System ID (SHOULD always be 1)
                  *** BUG DETECTOR: if this shows 2, the follower is
                  receiving its own radio echo back as "leader" data ***
  ldr_lat         Leader's EKF3 latitude (deg)
  ldr_lon         Leader's EKF3 longitude (deg)
  ldr_alt         Leader's relative altitude (m)
  ldr_vn/ve/vd    Leader's NED velocity (m/s)
  ldr_hdg         Leader's heading (deg)
  ldr_fc_mode     Leader's ArduPilot mode code:
                    0=STABILIZE, 2=ALT_HOLD, 4=GUIDED, 5=LOITER, 6=RTL, 9=LAND
  ldr_armed       Leader's armed flag
  ldr_fc_conn     Leader's MAVROS↔FC connection alive
  ldr_batt_v      Leader's battery voltage (V)
  ldr_guid        Leader's guidance mode (not used for leader, always 0)

Leader Jetson heartbeat:
  ldr_alive       Leader Jetson sending heartbeats over RFD (True/False)
                  Goes False if leader Jetson crashes or radio dies.
  ldr_hb_ts       Unix time of last leader Jetson heartbeat received
                  USEFUL FOR: detecting leader Jetson crash/reboot

Leader data timing:
  ldr_stamp       Unix time when OUR swarm_bridge received the leader's
                  position from the radio. This is NOT the leader's clock —
                  it's our local receive time.
  ldr_data_age_s  How stale: time.time() - ldr_stamp (seconds).
                  Healthy: ~0.007s (7ms). Stale: >2s.
                  USEFUL FOR: measuring radio latency + processing delay.
                  Does NOT include GPS sensor lag (~200ms) or EKF lag (~50ms).

Radio link quality (from RADIO_STATUS injected by RFD900x modem):
  rssi            Local received signal strength (0–255, higher=better)
                  ≈220-250 at <10m, drops with distance
  remrssi         Remote's signal strength (0–255)
  noise           Local background noise (0–255, lower=better)
  remnoise        Remote background noise
  txbuf           TX buffer free (0–100%). Low = radio congested
  rxerrors        Cumulative RX error count since radio power-on
  radio_alive     RADIO_STATUS received within last 5s (True/False)
                  USEFUL FOR: characterizing RSSI vs distance for your
                  link budget analysis.

Velocity commands TO own FC (from /mavros/setpoint_raw/local):
  cmd_vn          Commanded North velocity to FC (m/s, NED)
  cmd_ve          Commanded East velocity (m/s)
  cmd_vd          Commanded Down velocity (m/s)
  cmd_yaw_rate    Commanded yaw rate (rad/s, usually 0)
  cmd_type_mask   PositionTarget type_mask (bitmask of which fields active)
                  USEFUL FOR: seeing exactly what the guidance algorithm
                  is telling the FC to do, at native 10Hz rate.

GCS commands (from /swarm/gcs_command, event-driven):
  gcs_cmd         Command type: 1=RTL, 2=LAND, 3=KILL, 4=FOLLOW,
                  5=HOVER, 6=TAKEOFF, 10=WASD, 11=WAYPOINT
  gcs_lat         Waypoint lat (CMD_WAYPOINT only)
  gcs_lon         Waypoint lon (CMD_WAYPOINT only)

Mission + guidance state:
  mission         Mission state: IDLE, TAKEOFF, HOVER, FOLLOW,
                  GEOFENCE_RETURN, RTL, LAND, KILL
                  USEFUL FOR: correlating with GPS/velocity data to see
                  algorithm behavior in each mission phase.
  guid_mode       Guidance mode numeric: 0=NONE, 1=TRACKING, 2=CATCHUP,
                  3=EVASION
                  USEFUL FOR: analyzing which guidance mode was active
                  and how frequently it switches.

Derived:
  dist_m          3D distance between own GPS and leader GPS (m)
                  Computed locally from both positions.
                  USEFUL FOR: ground truth distance vs. what guidance sees.
                  NOTE: both GPSs have ~1-3m error. For distances <5m,
                  this is very noisy.

Rate counters (updated every 1s):
  hz_gps          Measured GPS topic rate (should be ~5-10 Hz)
  hz_vel          Measured velocity topic rate
  hz_peer         Measured peer_states rate (should be ~10 Hz)
  hz_cmd          Measured velocity command rate (should be ~10 Hz)

Usage:
  python3 follower_logger.py --gcs 172.16.0.26
  python3 follower_logger.py --no-sync              # skip time sync
"""

import argparse
import csv
import math
import os
import socket
import struct
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Float64, String
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State, PositionTarget

try:
    from swarm_msgs.msg import PeerStates, SwarmCommand
    HAS_SWARM = True
except ImportError:
    HAS_SWARM = False
    print("[WARN] swarm_msgs not found — swarm topics disabled")

METERS_PER_DEG = 111_320.0


# ═══════════════════════════════════════════════════════════════
# GCS Time Sync Client (NTP-like, embedded for standalone deploy)
# ═══════════════════════════════════════════════════════════════

class TimeSyncClient:
    """NTP-like clock offset estimator against GCS master clock.

    offset = gcs_time - local_time
    corrected_time = local_time + offset
    """

    SYNC_PORT = 14590
    TAG_BEACON = 0x01
    TAG_SYNC_REQ = 0x02
    TAG_SYNC_RESP = 0x03

    def __init__(self, gcs_host: str):
        self._gcs_addr = (gcs_host, self.SYNC_PORT)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.5)
        self._sock.bind(('0.0.0.0', self.SYNC_PORT))
        self._offset = 0.0
        self._samples = []
        self._max_samples = 20
        self._lock = threading.Lock()
        self._synced = False
        self._running = True
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._req_loop, daemon=True).start()

    @property
    def synced(self): return self._synced

    @property
    def offset_ms(self):
        with self._lock: return self._offset * 1000.0

    def corrected_time(self):
        with self._lock: return time.time() + self._offset

    def _update(self, offset):
        with self._lock:
            self._samples.append(offset)
            if len(self._samples) > self._max_samples:
                self._samples.pop(0)
            s = sorted(self._samples)
            self._offset = s[len(s) // 2]
            self._synced = len(self._samples) >= 3

    def _rx_loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(64)
            except (socket.timeout, OSError):
                continue
            now = time.time()
            if len(data) < 1: continue
            tag = data[0]
            if tag == self.TAG_BEACON and len(data) >= 9:
                _, gcs_t = struct.unpack('!Bd', data[:9])
                if not self._synced:
                    self._update(gcs_t - now)
            elif tag == self.TAG_SYNC_RESP and len(data) >= 25:
                _, t1, t2, t3 = struct.unpack('!Bddd', data[:25])
                self._update(((t2 - t1) + (t3 - now)) / 2.0)

    def _req_loop(self):
        while self._running:
            time.sleep(5.0)
            try:
                t1 = time.time()
                self._sock.sendto(struct.pack('!Bd', self.TAG_SYNC_REQ, t1),
                                  self._gcs_addr)
            except OSError:
                pass


# ═══════════════════════════════════════════════════════════════
# Follower Logger Node
# ═══════════════════════════════════════════════════════════════

class FollowerLogger(Node):
    def __init__(self, gcs_host: str):
        super().__init__('follower_logger')

        # ── Time sync ────────────────────────────────────────
        self._sync = None
        if gcs_host:
            try:
                self._sync = TimeSyncClient(gcs_host)
                self.get_logger().info(f"Time sync → {gcs_host}:14590")
            except Exception as e:
                self.get_logger().warn(f"Sync failed: {e}, using local clock")

        # ── Own FC state ─────────────────────────────────────
        self._lat = 0.0;   self._lon = 0.0;   self._rel_alt = 0.0
        self._vn = 0.0;    self._ve = 0.0;     self._vd = 0.0
        self._heading = 0.0
        self._local_x = 0.0; self._local_y = 0.0; self._local_z = 0.0
        self._fc_mode = '?'; self._fc_armed = False
        self._fc_connected = False; self._fc_sys_status = 0
        self._batt_v = 0.0; self._batt_a = 0.0; self._batt_pct = -1.0

        # ── Leader state (from peer_states via RFD) ──────────
        self._ldr_sysid = 0
        self._ldr_lat = 0.0; self._ldr_lon = 0.0; self._ldr_alt = 0.0
        self._ldr_vn = 0.0;  self._ldr_ve = 0.0;  self._ldr_vd = 0.0
        self._ldr_hdg = 0.0
        self._ldr_fc_mode = 0; self._ldr_armed = False
        self._ldr_fc_conn = False
        self._ldr_batt_v = 0.0; self._ldr_guid = 0
        self._ldr_alive = False; self._ldr_hb_ts = 0.0
        self._ldr_stamp = 0.0

        # ── Radio ────────────────────────────────────────────
        self._rssi = 0; self._remrssi = 0; self._noise = 0
        self._remnoise = 0; self._txbuf = 0; self._rxerrors = 0
        self._radio_alive = False

        # ── Velocity commands to FC ──────────────────────────
        self._cmd_vn = 0.0; self._cmd_ve = 0.0; self._cmd_vd = 0.0
        self._cmd_yaw_rate = 0.0; self._cmd_type_mask = 0

        # ── GCS command ──────────────────────────────────────
        self._gcs_cmd = 0; self._gcs_lat = 0.0; self._gcs_lon = 0.0

        # ── Mission / guidance ───────────────────────────────
        self._mission = '?'
        self._guid_mode = 0

        # ── Rate counters ────────────────────────────────────
        self._cnt = {'GPS': 0, 'ALT': 0, 'VEL': 0, 'HDG': 0,
                     'POSE': 0, 'STATE': 0, 'BATT': 0, 'PEER': 0,
                     'CMD': 0, 'GCS_CMD': 0, 'GUID': 0, 'MISSION': 0}
        self._hz = {'GPS': 0.0, 'VEL': 0.0, 'PEER': 0.0, 'CMD': 0.0}
        self._rate_t = time.time()
        self._event_total = 0
        self._start_t = time.time()

        # ── CSV ──────────────────────────────────────────────
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = f'follower_events_{ts}.csv'
        self._fh = open(self._csv_path, 'w', newline='')
        self._wr = csv.writer(self._fh)
        self._wr.writerow([
            # Timestamps
            'gcs_ts', 'local_ts', 'sync_off_ms',
            # Event
            'event', 'seq',
            # Own FC
            'lat', 'lon', 'rel_alt',
            'vn', 've', 'vd', 'heading',
            'local_x', 'local_y', 'local_z',
            'fc_mode', 'fc_armed', 'fc_connected', 'fc_sys_status',
            'batt_v', 'batt_a', 'batt_pct',
            # Leader via RFD
            'ldr_sysid', 'ldr_lat', 'ldr_lon', 'ldr_alt',
            'ldr_vn', 'ldr_ve', 'ldr_vd', 'ldr_hdg',
            'ldr_fc_mode', 'ldr_armed', 'ldr_fc_conn',
            'ldr_batt_v', 'ldr_guid',
            'ldr_alive', 'ldr_hb_ts', 'ldr_stamp', 'ldr_data_age_s',
            # Radio
            'rssi', 'remrssi', 'noise', 'remnoise',
            'txbuf', 'rxerrors', 'radio_alive',
            # Velocity commands to FC
            'cmd_vn', 'cmd_ve', 'cmd_vd', 'cmd_yaw_rate', 'cmd_type_mask',
            # GCS commands
            'gcs_cmd', 'gcs_lat', 'gcs_lon',
            # Mission + guidance
            'mission', 'guid_mode',
            # Derived
            'dist_m',
            # Rates
            'hz_gps', 'hz_vel', 'hz_peer', 'hz_cmd',
        ])

        # ═════════════════════════════════════════════════════
        # Topic subscriptions — 12 topics
        # ═════════════════════════════════════════════════════

        # Own FC data (via MAVROS2)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self._on_gps, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self._on_alt, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',
                                 self._on_vel, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg',
                                 self._on_hdg, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state',
                                 self._on_state, 10)
        self.create_subscription(BatteryState, '/mavros/battery',
                                 self._on_batt, qos_profile_sensor_data)

        # Velocity commands being sent TO FC by follower_control_node
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local',
                                 self._on_cmd, 10)

        # Swarm topics
        if HAS_SWARM:
            self.create_subscription(PeerStates, '/swarm/peer_states',
                                     self._on_peers, 10)
            self.create_subscription(SwarmCommand, '/swarm/gcs_command',
                                     self._on_gcs_cmd, 10)

        # Mission + guidance (from follower_control_node)
        self.create_subscription(Float64, '/swarm/guidance_mode',
                                 self._on_guid, 10)
        self.create_subscription(String, '/swarm/mission_state',
                                 self._on_mission, 10)

        # Rate + terminal
        self.create_timer(1.0, self._compute_rates)
        self.create_timer(0.5, self._print_status)

        self.get_logger().info(f"Follower logger → {self._csv_path}")

    # ── Timestamp helpers ────────────────────────────────────
    def _now(self):
        local = time.time()
        if self._sync and self._sync.synced:
            return self._sync.corrected_time(), local, self._sync.offset_ms
        return local, local, 0.0

    # ── Write one CSV row (full state snapshot) ──────────────
    def _write(self, event: str):
        gcs, local, off = self._now()
        self._cnt[event] = self._cnt.get(event, 0) + 1
        self._event_total += 1

        ldr_age = (time.time() - self._ldr_stamp
                   if self._ldr_stamp > 0 else -1.0)
        dist = self._dist3d()

        self._wr.writerow([
            f'{gcs:.6f}', f'{local:.6f}', f'{off:.2f}',
            event, self._cnt[event],
            # Own FC
            f'{self._lat:.8f}', f'{self._lon:.8f}', f'{self._rel_alt:.3f}',
            f'{self._vn:.4f}', f'{self._ve:.4f}', f'{self._vd:.4f}',
            f'{self._heading:.1f}',
            f'{self._local_x:.4f}', f'{self._local_y:.4f}', f'{self._local_z:.4f}',
            self._fc_mode, self._fc_armed, self._fc_connected,
            self._fc_sys_status,
            f'{self._batt_v:.2f}', f'{self._batt_a:.2f}', f'{self._batt_pct:.3f}',
            # Leader
            self._ldr_sysid,
            f'{self._ldr_lat:.8f}', f'{self._ldr_lon:.8f}', f'{self._ldr_alt:.3f}',
            f'{self._ldr_vn:.4f}', f'{self._ldr_ve:.4f}', f'{self._ldr_vd:.4f}',
            f'{self._ldr_hdg:.1f}',
            self._ldr_fc_mode, self._ldr_armed, self._ldr_fc_conn,
            f'{self._ldr_batt_v:.2f}', self._ldr_guid,
            self._ldr_alive, f'{self._ldr_hb_ts:.3f}',
            f'{self._ldr_stamp:.3f}',
            f'{ldr_age:.4f}' if ldr_age >= 0 else '-1',
            # Radio
            self._rssi, self._remrssi, self._noise, self._remnoise,
            self._txbuf, self._rxerrors, self._radio_alive,
            # CMD to FC
            f'{self._cmd_vn:.4f}', f'{self._cmd_ve:.4f}', f'{self._cmd_vd:.4f}',
            f'{self._cmd_yaw_rate:.4f}', self._cmd_type_mask,
            # GCS
            self._gcs_cmd, f'{self._gcs_lat:.7f}', f'{self._gcs_lon:.7f}',
            # Mission + guidance
            self._mission, self._guid_mode,
            # Derived
            f'{dist:.3f}' if dist >= 0 else '-1',
            # Rates
            f'{self._hz["GPS"]:.1f}', f'{self._hz["VEL"]:.1f}',
            f'{self._hz["PEER"]:.1f}', f'{self._hz["CMD"]:.1f}',
        ])
        self._fh.flush()

    # ── Distance ─────────────────────────────────────────────
    def _dist3d(self):
        if (self._lat == 0 and self._lon == 0): return -1.0
        if (self._ldr_lat == 0 and self._ldr_lon == 0): return -1.0
        dn = (self._ldr_lat - self._lat) * METERS_PER_DEG
        de = (self._ldr_lon - self._lon) * METERS_PER_DEG * math.cos(
            math.radians(self._lat))
        dd = self._ldr_alt - self._rel_alt
        return math.sqrt(dn*dn + de*de + dd*dd)

    # ═════════════════════════════════════════════════════════
    # Topic callbacks — each updates state + writes row
    # ═════════════════════════════════════════════════════════

    def _on_gps(self, msg):
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._write('GPS')

    def _on_alt(self, msg):
        self._rel_alt = msg.data
        self._write('ALT')

    def _on_vel(self, msg):
        self._vn = msg.twist.linear.y     # ENU.y → North
        self._ve = msg.twist.linear.x     # ENU.x → East
        self._vd = -msg.twist.linear.z    # -ENU.z → Down
        self._write('VEL')

    def _on_hdg(self, msg):
        self._heading = msg.data
        self._write('HDG')

    def _on_pose(self, msg):
        self._local_x = msg.pose.position.x   # ENU East
        self._local_y = msg.pose.position.y   # ENU North
        self._local_z = msg.pose.position.z   # ENU Up
        self._write('POSE')

    def _on_state(self, msg):
        self._fc_mode = msg.mode
        self._fc_armed = msg.armed
        self._fc_connected = msg.connected
        self._fc_sys_status = msg.system_status
        self._write('STATE')

    def _on_batt(self, msg):
        self._batt_v = msg.voltage
        self._batt_a = msg.current
        pct = msg.percentage
        if pct > 1.0: pct /= 100.0
        self._batt_pct = pct
        self._write('BATT')

    def _on_peers(self, msg):
        self._ldr_alive = msg.peer_jetson_alive
        self._ldr_hb_ts = msg.peer_last_heartbeat
        self._rssi = msg.radio_rssi
        self._remrssi = msg.radio_remrssi
        self._noise = msg.radio_noise
        self._remnoise = msg.radio_remnoise
        self._txbuf = msg.radio_txbuf
        self._rxerrors = msg.radio_rxerrors
        self._radio_alive = msg.radio_link_alive
        if msg.peers:
            p = msg.peers[0]
            self._ldr_sysid = p.drone_id
            self._ldr_lat = p.latitude
            self._ldr_lon = p.longitude
            self._ldr_alt = p.altitude
            self._ldr_vn = p.vn
            self._ldr_ve = p.ve
            self._ldr_vd = p.vd
            self._ldr_hdg = p.heading
            self._ldr_fc_mode = p.fc_mode_code
            self._ldr_armed = p.fc_armed
            self._ldr_fc_conn = p.fc_connected
            self._ldr_batt_v = p.battery_voltage
            self._ldr_guid = p.guidance_mode
            self._ldr_stamp = p.stamp
        self._write('PEER')

    def _on_cmd(self, msg):
        """Velocity command flowing from follower_control_node → FC.

        PositionTarget in FRAME_LOCAL_NED: velocity.x=North, y=East, z=Down.
        """
        self._cmd_vn = msg.velocity.x      # North (NED)
        self._cmd_ve = msg.velocity.y      # East  (NED)
        self._cmd_vd = msg.velocity.z      # Down  (NED)
        self._cmd_yaw_rate = msg.yaw_rate
        self._cmd_type_mask = msg.type_mask
        self._write('CMD')

    def _on_gcs_cmd(self, msg):
        self._gcs_cmd = msg.cmd
        self._gcs_lat = msg.lat
        self._gcs_lon = msg.lon
        self._write('GCS_CMD')

    def _on_guid(self, msg):
        self._guid_mode = int(msg.data)
        self._write('GUID')

    def _on_mission(self, msg):
        self._mission = msg.data
        self._write('MISSION')

    # ── Rate computation ─────────────────────────────────────
    def _compute_rates(self):
        now = time.time()
        dt = now - self._rate_t
        if dt < 0.5: return
        self._hz['GPS'] = self._cnt.get('GPS', 0) / dt
        self._hz['VEL'] = self._cnt.get('VEL', 0) / dt
        self._hz['PEER'] = self._cnt.get('PEER', 0) / dt
        self._hz['CMD'] = self._cnt.get('CMD', 0) / dt
        for k in self._cnt:
            self._cnt[k] = 0
        self._rate_t = now

    # ── Terminal status ──────────────────────────────────────
    def _print_status(self):
        dt = time.time() - self._start_t
        d = self._dist3d()
        ds = f'{d:.1f}m' if d >= 0 else '---'
        ss = (f'sync={self._sync.offset_ms:+.1f}ms'
              if self._sync and self._sync.synced else 'NO_SYNC')
        la = ((time.time() - self._ldr_stamp)
              if self._ldr_stamp > 0 else -1)
        las = f'{la:.3f}s' if la >= 0 else '---'
        cmd_spd = math.sqrt(self._cmd_vn**2 + self._cmd_ve**2 + self._cmd_vd**2)
        line = (
            f'\r[FOLLOWER] {dt:.0f}s | events={self._event_total} | '
            f'GPS={self._hz["GPS"]:.0f}Hz VEL={self._hz["VEL"]:.0f}Hz '
            f'PEER={self._hz["PEER"]:.0f}Hz CMD={self._hz["CMD"]:.0f}Hz | '
            f'dist={ds} ldr_age={las} ldr_id={self._ldr_sysid} | '
            f'cmd={cmd_spd:.2f}m/s | '
            f'mode={self._fc_mode} mission={self._mission} '
            f'guid={self._guid_mode} | '
            f'rssi={self._rssi}/{self._remrssi} | {ss}  '
        )
        sys.stdout.write(line[:240])
        sys.stdout.flush()

    def destroy_node(self):
        if self._fh:
            self._fh.close()
            print(f'\n[DONE] {self._csv_path} ({self._event_total} events)')
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Follower event logger')
    parser.add_argument('--gcs', default='172.16.0.26',
                        help='GCS IP for time sync (default: 172.16.0.26)')
    parser.add_argument('--no-sync', action='store_true',
                        help='Disable GCS time sync')
    args = parser.parse_args()

    rclpy.init()
    node = FollowerLogger(gcs_host='' if args.no_sync else args.gcs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
