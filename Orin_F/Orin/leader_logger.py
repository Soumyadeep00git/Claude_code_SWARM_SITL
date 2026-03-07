#!/usr/bin/env python3
"""Leader Event Logger — captures ALL MAVROS data at native topic rates.

EVENT-DRIVEN: one CSV row per ROS2 message received. No fixed timer.
Each row is a full state snapshot so every row is self-contained for
analysis (no need to forward-fill missing columns in pandas).

╔══════════════════════════════════════════════════════════════════╗
║  TOPICS SUBSCRIBED  (with expected rates on ArduCopter V4.6)    ║
╠══════════════════════════════════════════════════════════════════╣
║  /mavros/global_position/global        NavSatFix      ~5-10 Hz  ║
║  /mavros/global_position/rel_alt       Float64        ~5-10 Hz  ║
║  /mavros/local_position/velocity_local TwistStamped   ~4-10 Hz  ║
║  /mavros/global_position/compass_hdg   Float64        ~5-10 Hz  ║
║  /mavros/local_position/pose           PoseStamped    ~4-10 Hz  ║
║  /mavros/state                         State          ~1    Hz  ║
║  /mavros/battery                       BatteryState   ~1    Hz  ║
║  /swarm/peer_states                    PeerStates     ~10   Hz  ║
║  /swarm/gcs_command                    SwarmCommand   event     ║
╚══════════════════════════════════════════════════════════════════╝

Actual rates depend on ArduPilot SR2_* stream rate parameters.
Run `ros2 topic hz /mavros/global_position/global` to measure live.

CSV COLUMNS (every row is a complete state snapshot):
─────────────────────────────────────────────────────
Timestamps:
  gcs_ts          GCS-corrected unix time (for cross-Jetson alignment)
  local_ts        Raw local Jetson unix time
  sync_off_ms     Clock offset: gcs = local + offset (ms)

Event metadata:
  event           Topic that triggered this row:
                    GPS, ALT, VEL, HDG, POSE, STATE, BATT, PEER, GCS_CMD
  seq             Per-topic sequence number (for rate calculation)

Own FC data (from MAVROS2 ← CubeOrange+ EKF3-fused):
  lat             Latitude (deg, 8 dp ≈ 1mm resolution)
  lon             Longitude (deg)
  rel_alt         Relative altitude above home (m)
  vn              North velocity (m/s, NED)
  ve              East velocity (m/s)
  vd              Down velocity (m/s, positive=descending)
  heading         Compass heading (0-360°)
  local_x         EKF local East  (m, from EKF origin, ENU frame)
  local_y         EKF local North (m)
  local_z         EKF local Up    (m)
  fc_mode         Flight mode string (GUIDED, STABILIZE, LOITER, etc.)
  fc_armed        Armed flag (True/False)
  fc_connected    MAVROS↔FC serial link status
  fc_sys_status   MAV_STATE enum (0=UNINIT, 3=STANDBY, 4=ACTIVE, 8=FLIGHT_TERMINATION)
  batt_v          Battery voltage (V)
  batt_a          Battery current (A)
  batt_pct        Battery remaining (0.0-1.0, -1=unknown)

Peer data (from /swarm/peer_states — follower via RFD900x):
  peer_id         Peer's MAVLink SYSID (should be 2 for follower)
                  NOTE: If this shows your OWN id, it's a radio echo bug
  peer_lat        Peer's EKF3 latitude (deg)
  peer_lon        Peer's EKF3 longitude (deg)
  peer_alt        Peer's relative altitude (m)
  peer_vn/ve/vd   Peer's NED velocity (m/s)
  peer_hdg        Peer's heading (deg)
  peer_fc_mode    Peer's ArduPilot mode code (0=STABILIZE, 4=GUIDED, 5=LOITER, 6=RTL)
  peer_armed      Peer's armed flag
  peer_fc_conn    Peer's MAVROS↔FC connection status
  peer_batt_v     Peer's battery voltage (V)
  peer_guid       Peer's guidance mode (0=NONE, 1=TRACK, 2=CATCHUP, 3=EVASION)
  peer_alive      Peer Jetson sending heartbeats (True/False)
  peer_hb_ts      Unix time of last peer Jetson heartbeat
  peer_stamp      Unix time when THIS Jetson's swarm_bridge received the last
                  peer position from the radio (NOT the peer's original timestamp)
  peer_age_s      Data staleness: now - peer_stamp (seconds). Typically 0.007s
                  if radio is working. >2s means stale data.

Radio link (from RADIO_STATUS injected by RFD900x modem):
  rssi            Local received signal strength (0-255, higher=better)
  remrssi         Remote's received signal strength (0-255)
  noise           Local background noise (0-255, lower=better)
  remnoise        Remote background noise (0-255)
  txbuf           TX buffer free space (0-100%, low=congestion)
  rxerrors        Cumulative RX error count
  radio_alive     Radio link alive (RADIO_STATUS within last 5s)

GCS commands (from /swarm/gcs_command, event-driven):
  gcs_cmd         Command type (1=RTL, 2=LAND, 3=KILL, 4=FOLLOW, 5=HOVER, 6=TAKEOFF)
  gcs_lat         Waypoint lat (CMD_WAYPOINT only)
  gcs_lon         Waypoint lon (CMD_WAYPOINT only)

Derived:
  dist_m          3D distance to peer (m, computed locally from both GPS positions)

Rate counters (updated every 1s, for diagnosing stream rate issues):
  hz_gps          Measured GPS topic rate
  hz_vel          Measured velocity topic rate
  hz_peer         Measured peer_states topic rate

Usage:
  python3 leader_logger.py --gcs 172.16.0.26
  python3 leader_logger.py --no-sync          # skip time sync
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
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State

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

    Protocol:
      - Receives 1Hz beacons from GCS (coarse sync)
      - Sends SYNC_REQ every 5s, gets SYNC_RESP with round-trip measurement
      - Maintains median-filtered offset for robustness

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
# Leader Logger Node
# ═══════════════════════════════════════════════════════════════

class LeaderLogger(Node):
    def __init__(self, gcs_host: str):
        super().__init__('leader_logger')

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

        # ── Peer (follower) state ────────────────────────────
        self._peer_id = 0
        self._peer_lat = 0.0; self._peer_lon = 0.0; self._peer_alt = 0.0
        self._peer_vn = 0.0;  self._peer_ve = 0.0;  self._peer_vd = 0.0
        self._peer_hdg = 0.0
        self._peer_fc_mode = 0; self._peer_armed = False
        self._peer_fc_conn = False
        self._peer_batt_v = 0.0; self._peer_guid = 0
        self._peer_alive = False; self._peer_hb_ts = 0.0
        self._peer_stamp = 0.0
        # Radio
        self._rssi = 0; self._remrssi = 0; self._noise = 0
        self._remnoise = 0; self._txbuf = 0; self._rxerrors = 0
        self._radio_alive = False

        # ── GCS command ──────────────────────────────────────
        self._gcs_cmd = 0; self._gcs_lat = 0.0; self._gcs_lon = 0.0

        # ── Rate counters ────────────────────────────────────
        self._cnt = {'GPS': 0, 'ALT': 0, 'VEL': 0, 'HDG': 0,
                     'POSE': 0, 'STATE': 0, 'BATT': 0, 'PEER': 0}
        self._hz = {'GPS': 0.0, 'VEL': 0.0, 'PEER': 0.0}
        self._rate_t = time.time()
        self._event_total = 0
        self._start_t = time.time()

        # ── CSV ──────────────────────────────────────────────
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = f'leader_events_{ts}.csv'
        self._fh = open(self._csv_path, 'w', newline='')
        self._wr = csv.writer(self._fh)
        self._wr.writerow([
            'gcs_ts', 'local_ts', 'sync_off_ms', 'event', 'seq',
            'lat', 'lon', 'rel_alt', 'vn', 've', 'vd', 'heading',
            'local_x', 'local_y', 'local_z',
            'fc_mode', 'fc_armed', 'fc_connected', 'fc_sys_status',
            'batt_v', 'batt_a', 'batt_pct',
            'peer_id', 'peer_lat', 'peer_lon', 'peer_alt',
            'peer_vn', 'peer_ve', 'peer_vd', 'peer_hdg',
            'peer_fc_mode', 'peer_armed', 'peer_fc_conn',
            'peer_batt_v', 'peer_guid',
            'peer_alive', 'peer_hb_ts', 'peer_stamp', 'peer_age_s',
            'rssi', 'remrssi', 'noise', 'remnoise', 'txbuf', 'rxerrors',
            'radio_alive',
            'gcs_cmd', 'gcs_lat', 'gcs_lon',
            'dist_m',
            'hz_gps', 'hz_vel', 'hz_peer',
        ])

        # ── Subscriptions ────────────────────────────────────
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
        if HAS_SWARM:
            self.create_subscription(PeerStates, '/swarm/peer_states',
                                     self._on_peers, 10)
            self.create_subscription(SwarmCommand, '/swarm/gcs_command',
                                     self._on_gcs_cmd, 10)

        # Rate + terminal display
        self.create_timer(1.0, self._compute_rates)
        self.create_timer(0.5, self._print_status)

        self.get_logger().info(f"Leader logger → {self._csv_path}")

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

        peer_age = (time.time() - self._peer_stamp
                    if self._peer_stamp > 0 else -1.0)
        dist = self._dist3d()

        self._wr.writerow([
            f'{gcs:.6f}', f'{local:.6f}', f'{off:.2f}',
            event, self._cnt[event],
            f'{self._lat:.8f}', f'{self._lon:.8f}', f'{self._rel_alt:.3f}',
            f'{self._vn:.4f}', f'{self._ve:.4f}', f'{self._vd:.4f}',
            f'{self._heading:.1f}',
            f'{self._local_x:.4f}', f'{self._local_y:.4f}', f'{self._local_z:.4f}',
            self._fc_mode, self._fc_armed, self._fc_connected,
            self._fc_sys_status,
            f'{self._batt_v:.2f}', f'{self._batt_a:.2f}', f'{self._batt_pct:.3f}',
            self._peer_id,
            f'{self._peer_lat:.8f}', f'{self._peer_lon:.8f}', f'{self._peer_alt:.3f}',
            f'{self._peer_vn:.4f}', f'{self._peer_ve:.4f}', f'{self._peer_vd:.4f}',
            f'{self._peer_hdg:.1f}',
            self._peer_fc_mode, self._peer_armed, self._peer_fc_conn,
            f'{self._peer_batt_v:.2f}', self._peer_guid,
            self._peer_alive, f'{self._peer_hb_ts:.3f}',
            f'{self._peer_stamp:.3f}',
            f'{peer_age:.4f}' if peer_age >= 0 else '-1',
            self._rssi, self._remrssi, self._noise, self._remnoise,
            self._txbuf, self._rxerrors, self._radio_alive,
            self._gcs_cmd, f'{self._gcs_lat:.7f}', f'{self._gcs_lon:.7f}',
            f'{dist:.3f}' if dist >= 0 else '-1',
            f'{self._hz["GPS"]:.1f}', f'{self._hz["VEL"]:.1f}',
            f'{self._hz["PEER"]:.1f}',
        ])
        self._fh.flush()

    # ── Distance ─────────────────────────────────────────────
    def _dist3d(self):
        if (self._lat == 0 and self._lon == 0): return -1.0
        if (self._peer_lat == 0 and self._peer_lon == 0): return -1.0
        dn = (self._peer_lat - self._lat) * METERS_PER_DEG
        de = (self._peer_lon - self._lon) * METERS_PER_DEG * math.cos(
            math.radians(self._lat))
        dd = self._peer_alt - self._rel_alt
        return math.sqrt(dn*dn + de*de + dd*dd)

    # ── Topic callbacks (each updates state + writes row) ────

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
        self._peer_alive = msg.peer_jetson_alive
        self._peer_hb_ts = msg.peer_last_heartbeat
        self._rssi = msg.radio_rssi
        self._remrssi = msg.radio_remrssi
        self._noise = msg.radio_noise
        self._remnoise = msg.radio_remnoise
        self._txbuf = msg.radio_txbuf
        self._rxerrors = msg.radio_rxerrors
        self._radio_alive = msg.radio_link_alive
        if msg.peers:
            p = msg.peers[0]
            self._peer_id = p.drone_id
            self._peer_lat = p.latitude
            self._peer_lon = p.longitude
            self._peer_alt = p.altitude
            self._peer_vn = p.vn
            self._peer_ve = p.ve
            self._peer_vd = p.vd
            self._peer_hdg = p.heading
            self._peer_fc_mode = p.fc_mode_code
            self._peer_armed = p.fc_armed
            self._peer_fc_conn = p.fc_connected
            self._peer_batt_v = p.battery_voltage
            self._peer_guid = p.guidance_mode
            self._peer_stamp = p.stamp
        self._write('PEER')

    def _on_gcs_cmd(self, msg):
        self._gcs_cmd = msg.cmd
        self._gcs_lat = msg.lat
        self._gcs_lon = msg.lon
        self._write('GCS_CMD')

    # ── Rate computation ─────────────────────────────────────
    def _compute_rates(self):
        now = time.time()
        dt = now - self._rate_t
        if dt < 0.5: return
        self._hz['GPS'] = self._cnt.get('GPS', 0) / dt
        self._hz['VEL'] = self._cnt.get('VEL', 0) / dt
        self._hz['PEER'] = self._cnt.get('PEER', 0) / dt
        # Reset counters  (keep seq in separate dict if needed)
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
        line = (
            f'\r[LEADER] {dt:.0f}s | events={self._event_total} | '
            f'GPS={self._hz["GPS"]:.0f}Hz VEL={self._hz["VEL"]:.0f}Hz '
            f'PEER={self._hz["PEER"]:.0f}Hz | '
            f'pos=({self._lat:.6f},{self._lon:.6f}) alt={self._rel_alt:.1f} | '
            f'mode={self._fc_mode} arm={self._fc_armed} | '
            f'peer={self._peer_id} dist={ds} rssi={self._rssi}/{self._remrssi} | '
            f'{ss}  '
        )
        sys.stdout.write(line[:220])
        sys.stdout.flush()

    def destroy_node(self):
        if self._fh:
            self._fh.close()
            print(f'\n[DONE] {self._csv_path} ({self._event_total} events)')
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Leader event logger')
    parser.add_argument('--gcs', default='172.16.0.26',
                        help='GCS IP for time sync (default: 172.16.0.26)')
    parser.add_argument('--no-sync', action='store_true',
                        help='Disable GCS time sync')
    args = parser.parse_args()

    rclpy.init()
    node = LeaderLogger(gcs_host='' if args.no_sync else args.gcs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
