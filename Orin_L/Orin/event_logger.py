#!/usr/bin/env python3
"""Event-Driven Synchronized Logger — runs on each Jetson.

Logs data ONLY when new data arrives (no timer polling). Each ROS2
callback writes a CSV row with GCS-synchronized timestamps.

Features:
  - Event-driven: logs on every topic callback (GPS ~30Hz, peer ~10Hz, etc.)
  - GCS time sync: NTP-like offset correction for cross-Jetson alignment
  - Separate raw log + merged state snapshot log
  - Works for both leader and follower (pass --role leader or --role follower)

Output files (created in current directory):
  <role>_events_<timestamp>.csv   — every topic callback, one row per event
  <role>_state_<timestamp>.csv    — complete state snapshot on each event

Usage on Jetson:
  # Follower:
  python3 event_logger.py --role follower --drone-id 2 --gcs 172.16.0.26

  # Leader:
  python3 event_logger.py --role leader --drone-id 1 --gcs 172.16.0.26
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
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State

try:
    from swarm_msgs.msg import PeerStates, SwarmCommand
    HAS_SWARM_MSGS = True
except ImportError:
    HAS_SWARM_MSGS = False
    print("[WARN] swarm_msgs not found — swarm topics disabled")

# ── Constants ─────────────────────────────────────────────────
METERS_PER_DEG = 111_320.0
SYNC_PORT = 14590
TAG_BEACON = 0x01
TAG_SYNC_REQ = 0x02
TAG_SYNC_RESP = 0x03


class TimeSyncClient:
    """NTP-like time sync with the GCS master clock.

    Receives beacons for coarse offset, sends SYNC_REQ for precise
    round-trip offset calculation. Maintains a running average.
    """

    def __init__(self, gcs_host: str, gcs_port: int = SYNC_PORT):
        self._gcs_addr = (gcs_host, gcs_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.5)
        self._sock.bind(('0.0.0.0', gcs_port))

        self._offset = 0.0           # gcs_time = local_time + offset
        self._offset_samples = []    # last N offset measurements
        self._max_samples = 20
        self._lock = threading.Lock()
        self._running = True
        self._synced = False

        threading.Thread(target=self._rx_loop, daemon=True, name='sync-rx').start()
        threading.Thread(target=self._req_loop, daemon=True, name='sync-req').start()

    @property
    def synced(self) -> bool:
        return self._synced

    @property
    def offset_ms(self) -> float:
        """Current offset in milliseconds."""
        with self._lock:
            return self._offset * 1000.0

    def corrected_time(self) -> float:
        """Return current time corrected to GCS clock."""
        with self._lock:
            return time.time() + self._offset

    def _update_offset(self, offset: float):
        with self._lock:
            self._offset_samples.append(offset)
            if len(self._offset_samples) > self._max_samples:
                self._offset_samples.pop(0)
            # Median filter (robust to outliers)
            sorted_s = sorted(self._offset_samples)
            mid = len(sorted_s) // 2
            self._offset = sorted_s[mid]
            self._synced = len(self._offset_samples) >= 3

    def _rx_loop(self):
        """Receive beacons and sync responses."""
        while self._running:
            try:
                data, addr = self._sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < 1:
                continue

            local_now = time.time()
            tag = data[0]

            if tag == TAG_BEACON and len(data) >= 9:
                # Coarse sync from beacon
                _, gcs_t = struct.unpack('!Bd', data[:9])
                coarse_offset = gcs_t - local_now
                if not self._synced:
                    self._update_offset(coarse_offset)

            elif tag == TAG_SYNC_RESP and len(data) >= 25:
                # Precise round-trip sync
                _, jetson_t1, gcs_t2, gcs_t3 = struct.unpack('!Bddd', data[:25])
                jetson_t4 = local_now
                # NTP-style offset
                offset = ((gcs_t2 - jetson_t1) + (gcs_t3 - jetson_t4)) / 2.0
                self._update_offset(offset)

    def _req_loop(self):
        """Send SYNC_REQ every 5 seconds for precise measurement."""
        while self._running:
            time.sleep(5.0)
            try:
                t1 = time.time()
                pkt = struct.pack('!Bd', TAG_SYNC_REQ, t1)
                self._sock.sendto(pkt, self._gcs_addr)
            except OSError:
                pass


class EventLogger(Node):
    """Event-driven logger — writes a row on every topic callback."""

    def __init__(self, role: str, drone_id: int, gcs_host: str):
        super().__init__('event_logger')
        self._role = role
        self._drone_id = drone_id

        # ── Time sync ────────────────────────────────────────
        self._sync = None
        if gcs_host:
            try:
                self._sync = TimeSyncClient(gcs_host)
                self.get_logger().info(f"Time sync → {gcs_host}:{SYNC_PORT}")
            except Exception as e:
                self.get_logger().warn(f"Time sync init failed: {e}")

        # ── State variables ──────────────────────────────────
        self._own_lat = 0.0
        self._own_lon = 0.0
        self._own_alt = 0.0
        self._own_vn = 0.0
        self._own_ve = 0.0
        self._own_vd = 0.0
        self._own_heading = 0.0
        self._own_mode = '?'
        self._own_armed = False
        self._own_connected = False
        self._own_batt_v = 0.0
        self._own_batt_a = 0.0
        self._own_batt_pct = -1.0

        # Peer (leader for follower, follower for leader)
        self._peer_id = 0
        self._peer_lat = 0.0
        self._peer_lon = 0.0
        self._peer_alt = 0.0
        self._peer_vn = 0.0
        self._peer_ve = 0.0
        self._peer_vd = 0.0
        self._peer_heading = 0.0
        self._peer_fc_mode = 0
        self._peer_armed = False
        self._peer_batt_v = 0.0
        self._peer_alive = False
        self._peer_stamp = 0.0

        # Radio
        self._radio_rssi = 0
        self._radio_remrssi = 0
        self._radio_noise = 0
        self._radio_txbuf = 0

        # Commands (follower only)
        self._cmd_vn = 0.0
        self._cmd_ve = 0.0
        self._cmd_vd = 0.0
        self._mission_state = '?'
        self._guid_mode = 0

        # Counters
        self._event_count = 0
        self._gps_count = 0
        self._vel_count = 0
        self._peer_count = 0
        self._cmd_count = 0

        # ── CSV files ────────────────────────────────────────
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._event_file = f'{role}_events_{ts}.csv'
        self._state_file = f'{role}_state_{ts}.csv'

        # Event log: lightweight, one row per ROS callback
        self._evt_fh = open(self._event_file, 'w', newline='')
        self._evt_csv = csv.writer(self._evt_fh)
        self._evt_csv.writerow([
            'gcs_time', 'local_time', 'offset_ms', 'event_id', 'source',
            # GPS fields (filled on GPS events, blank otherwise)
            'lat', 'lon', 'alt',
            # Velocity fields
            'vn', 've', 'vd',
            # Peer fields (filled on peer events)
            'peer_id', 'peer_lat', 'peer_lon', 'peer_alt',
            'peer_vn', 'peer_ve', 'peer_vd',
            # Command fields (filled on cmd events)
            'cmd_vn', 'cmd_ve', 'cmd_vd',
            # State fields
            'fc_mode', 'armed',
            # Radio
            'rssi', 'remrssi',
        ])

        # State snapshot log: complete state on EVERY event (for analysis)
        self._st_fh = open(self._state_file, 'w', newline='')
        self._st_csv = csv.writer(self._st_fh)
        self._st_csv.writerow([
            'gcs_time', 'local_time', 'offset_ms', 'event_id', 'source',
            'own_lat', 'own_lon', 'own_alt',
            'own_vn', 'own_ve', 'own_vd', 'own_heading',
            'own_mode', 'own_armed', 'own_batt_v',
            'peer_id', 'peer_lat', 'peer_lon', 'peer_alt',
            'peer_vn', 'peer_ve', 'peer_vd', 'peer_heading',
            'peer_fc_mode', 'peer_armed', 'peer_batt_v',
            'peer_alive', 'peer_age_s',
            'radio_rssi', 'radio_remrssi', 'radio_noise', 'radio_txbuf',
            'cmd_vn', 'cmd_ve', 'cmd_vd', 'cmd_speed',
            'mission_state', 'guid_mode',
            'dist_m',
            'gps_hz', 'vel_hz', 'peer_hz', 'cmd_hz',
        ])

        self._lock = threading.Lock()
        self._start_time = time.time()
        self._last_rate_time = time.time()
        self._gps_rate = 0.0
        self._vel_rate = 0.0
        self._peer_rate = 0.0
        self._cmd_rate = 0.0

        # ── ROS2 Subscriptions ────────────────────────────────
        # MAVROS topics (all drones)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self._on_gps, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            self._on_alt, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_vel, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg',
            self._on_hdg, qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state', self._on_state, 10)
        self.create_subscription(
            BatteryState, '/mavros/battery', self._on_batt, qos_profile_sensor_data)

        # Swarm topics
        if HAS_SWARM_MSGS:
            self.create_subscription(
                PeerStates, '/swarm/peer_states',
                self._on_peers, qos_profile_sensor_data)
            self.create_subscription(
                SwarmCommand, '/swarm/gcs_command',
                self._on_cmd, 10)

        # Guidance/mission (follower)
        self.create_subscription(
            Float64, '/swarm/guidance_mode', self._on_guid, 10)
        # Try mission_state topic if available
        self.create_subscription(
            String, '/swarm/mission_state', self._on_mission, 10)

        # Rate computation timer (1 Hz)
        self.create_timer(1.0, self._compute_rates)

        # Status print (5 Hz)
        self.create_timer(0.2, self._print_status)

        self.get_logger().info(
            f"EventLogger: role={role} drone_id={drone_id} "
            f"events→{self._event_file} state→{self._state_file}")

    # ═══════════════════════════════════════════════════════════
    # Timestamp helpers
    # ═══════════════════════════════════════════════════════════

    def _now(self) -> tuple:
        """Return (gcs_corrected_time, local_time, offset_ms)."""
        local = time.time()
        if self._sync and self._sync.synced:
            gcs = self._sync.corrected_time()
            offset_ms = self._sync.offset_ms
        else:
            gcs = local
            offset_ms = 0.0
        return gcs, local, offset_ms

    def _fmt_time(self, t: float) -> str:
        """Format Unix timestamp as HH:MM:SS.mmm."""
        dt = datetime.fromtimestamp(t)
        return dt.strftime('%H:%M:%S') + f'.{int(dt.microsecond/1000):03d}'

    # ═══════════════════════════════════════════════════════════
    # Event logging
    # ═══════════════════════════════════════════════════════════

    def _log_event(self, source: str, **fields):
        """Write event row (lightweight) + state snapshot row (complete)."""
        gcs_t, local_t, offset = self._now()
        self._event_count += 1
        eid = self._event_count

        # Event row (sparse — only fields relevant to this event)
        self._evt_csv.writerow([
            f'{gcs_t:.6f}', f'{local_t:.6f}', f'{offset:.3f}', eid, source,
            fields.get('lat', ''), fields.get('lon', ''), fields.get('alt', ''),
            fields.get('vn', ''), fields.get('ve', ''), fields.get('vd', ''),
            fields.get('peer_id', ''), fields.get('peer_lat', ''),
            fields.get('peer_lon', ''), fields.get('peer_alt', ''),
            fields.get('peer_vn', ''), fields.get('peer_ve', ''),
            fields.get('peer_vd', ''),
            fields.get('cmd_vn', ''), fields.get('cmd_ve', ''),
            fields.get('cmd_vd', ''),
            fields.get('fc_mode', ''), fields.get('armed', ''),
            fields.get('rssi', ''), fields.get('remrssi', ''),
        ])
        self._evt_fh.flush()

        # Full state snapshot
        dist = self._dist3d()
        peer_age = (time.time() - self._peer_stamp) if self._peer_stamp > 0 else -1.0
        cmd_speed = math.sqrt(
            self._cmd_vn**2 + self._cmd_ve**2 + self._cmd_vd**2)

        self._st_csv.writerow([
            f'{gcs_t:.6f}', f'{local_t:.6f}', f'{offset:.3f}', eid, source,
            f'{self._own_lat:.8f}', f'{self._own_lon:.8f}', f'{self._own_alt:.3f}',
            f'{self._own_vn:.4f}', f'{self._own_ve:.4f}', f'{self._own_vd:.4f}',
            f'{self._own_heading:.1f}',
            self._own_mode, self._own_armed, f'{self._own_batt_v:.2f}',
            self._peer_id,
            f'{self._peer_lat:.8f}', f'{self._peer_lon:.8f}', f'{self._peer_alt:.3f}',
            f'{self._peer_vn:.4f}', f'{self._peer_ve:.4f}', f'{self._peer_vd:.4f}',
            f'{self._peer_heading:.1f}',
            self._peer_fc_mode, self._peer_armed, f'{self._peer_batt_v:.2f}',
            self._peer_alive, f'{peer_age:.4f}' if peer_age >= 0 else '-1',
            self._radio_rssi, self._radio_remrssi,
            self._radio_noise, self._radio_txbuf,
            f'{self._cmd_vn:.4f}', f'{self._cmd_ve:.4f}', f'{self._cmd_vd:.4f}',
            f'{cmd_speed:.4f}',
            self._mission_state, self._guid_mode,
            f'{dist:.3f}' if dist >= 0 else '-1',
            f'{self._gps_rate:.1f}', f'{self._vel_rate:.1f}',
            f'{self._peer_rate:.1f}', f'{self._cmd_rate:.1f}',
        ])
        self._st_fh.flush()

    # ═══════════════════════════════════════════════════════════
    # MAVROS callbacks — each fires _log_event
    # ═══════════════════════════════════════════════════════════

    def _on_gps(self, msg):
        with self._lock:
            self._own_lat = msg.latitude
            self._own_lon = msg.longitude
            self._gps_count += 1
        self._log_event('GPS',
                        lat=f'{msg.latitude:.8f}',
                        lon=f'{msg.longitude:.8f}')

    def _on_alt(self, msg):
        with self._lock:
            self._own_alt = msg.data
        self._log_event('ALT', alt=f'{msg.data:.3f}')

    def _on_vel(self, msg):
        vn = msg.twist.linear.y    # ENU.y → North
        ve = msg.twist.linear.x    # ENU.x → East
        vd = -msg.twist.linear.z   # -ENU.z → Down
        with self._lock:
            self._own_vn = vn
            self._own_ve = ve
            self._own_vd = vd
            self._vel_count += 1
        self._log_event('VEL', vn=f'{vn:.4f}', ve=f'{ve:.4f}', vd=f'{vd:.4f}')

    def _on_hdg(self, msg):
        with self._lock:
            self._own_heading = msg.data
        self._log_event('HDG')

    def _on_state(self, msg):
        with self._lock:
            self._own_mode = msg.mode
            self._own_armed = msg.armed
            self._own_connected = msg.connected
        self._log_event('STATE', fc_mode=msg.mode, armed=msg.armed)

    def _on_batt(self, msg):
        with self._lock:
            self._own_batt_v = msg.voltage
            pct = msg.percentage
            if pct > 1.0:
                pct /= 100.0
            self._own_batt_pct = pct
        self._log_event('BATT')

    def _on_peers(self, msg):
        with self._lock:
            self._peer_alive = msg.peer_jetson_alive
            self._radio_rssi = msg.radio_rssi
            self._radio_remrssi = msg.radio_remrssi
            self._radio_noise = msg.radio_noise
            self._radio_txbuf = msg.radio_txbuf
            self._peer_count += 1

            if msg.peers:
                p = msg.peers[0]
                self._peer_id = p.drone_id
                self._peer_lat = p.latitude
                self._peer_lon = p.longitude
                self._peer_alt = p.altitude
                self._peer_vn = p.vn
                self._peer_ve = p.ve
                self._peer_vd = p.vd
                self._peer_heading = p.heading
                self._peer_fc_mode = p.fc_mode_code
                self._peer_armed = p.fc_armed
                self._peer_batt_v = p.battery_voltage
                self._peer_stamp = p.stamp

        if msg.peers:
            p = msg.peers[0]
            self._log_event('PEER',
                            peer_id=p.drone_id,
                            peer_lat=f'{p.latitude:.8f}',
                            peer_lon=f'{p.longitude:.8f}',
                            peer_alt=f'{p.altitude:.3f}',
                            peer_vn=f'{p.vn:.4f}',
                            peer_ve=f'{p.ve:.4f}',
                            peer_vd=f'{p.vd:.4f}',
                            rssi=msg.radio_rssi,
                            remrssi=msg.radio_remrssi)
        else:
            self._log_event('PEER_EMPTY', rssi=msg.radio_rssi,
                            remrssi=msg.radio_remrssi)

    def _on_cmd(self, msg):
        with self._lock:
            self._cmd_vn = msg.velocity.x
            self._cmd_ve = msg.velocity.y
            self._cmd_vd = msg.velocity.z
            self._cmd_count += 1
        self._log_event('CMD',
                        cmd_vn=f'{msg.velocity.x:.4f}',
                        cmd_ve=f'{msg.velocity.y:.4f}',
                        cmd_vd=f'{msg.velocity.z:.4f}')

    def _on_guid(self, msg):
        with self._lock:
            self._guid_mode = int(msg.data)
        self._log_event('GUID')

    def _on_mission(self, msg):
        with self._lock:
            self._mission_state = msg.data
        self._log_event('MISSION')

    # ═══════════════════════════════════════════════════════════
    # Derived
    # ═══════════════════════════════════════════════════════════

    def _dist3d(self):
        if self._peer_lat == 0 and self._peer_lon == 0:
            return -1.0
        if self._own_lat == 0 and self._own_lon == 0:
            return -1.0
        dn = (self._peer_lat - self._own_lat) * METERS_PER_DEG
        de = (self._peer_lon - self._own_lon) * METERS_PER_DEG * math.cos(
            math.radians(self._own_lat))
        dd = self._peer_alt - self._own_alt
        return math.sqrt(dn*dn + de*de + dd*dd)

    def _compute_rates(self):
        """Compute actual data rates from counters (called every 1s)."""
        now = time.time()
        dt = now - self._last_rate_time
        if dt < 0.5:
            return
        with self._lock:
            self._gps_rate = self._gps_count / dt
            self._vel_rate = self._vel_count / dt
            self._peer_rate = self._peer_count / dt
            self._cmd_rate = self._cmd_count / dt
            self._gps_count = 0
            self._vel_count = 0
            self._peer_count = 0
            self._cmd_count = 0
        self._last_rate_time = now

    def _print_status(self):
        """Print compact status line to terminal (5 Hz)."""
        dist = self._dist3d()
        dist_s = f'{dist:.1f}m' if dist >= 0 else '---'
        sync_s = (f'sync={self._sync.offset_ms:.1f}ms'
                  if self._sync and self._sync.synced else 'NO_SYNC')

        elapsed = time.time() - self._start_time
        line = (
            f'\r[{self._role.upper()}] '
            f't={elapsed:.0f}s  events={self._event_count}  '
            f'GPS={self._gps_rate:.0f}Hz  VEL={self._vel_rate:.0f}Hz  '
            f'PEER={self._peer_rate:.0f}Hz  CMD={self._cmd_rate:.0f}Hz  '
            f'dist={dist_s}  rssi={self._radio_rssi}/{self._radio_remrssi}  '
            f'{sync_s}  '
            f'own=({self._own_lat:.6f},{self._own_lon:.6f})  '
            f'peer=({self._peer_lat:.6f},{self._peer_lon:.6f})  '
            f'pid={self._peer_id}  mode={self._own_mode}  '
        )
        sys.stdout.write(line[:200])
        sys.stdout.flush()

    def destroy_node(self):
        if self._evt_fh:
            self._evt_fh.close()
        if self._st_fh:
            self._st_fh.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Event-driven synchronized logger')
    parser.add_argument('--role', choices=['leader', 'follower'],
                        required=True, help='Drone role')
    parser.add_argument('--drone-id', type=int, required=True,
                        help='Own drone ID (1=leader, 2=follower)')
    parser.add_argument('--gcs', type=str, default='172.16.0.26',
                        help='GCS host IP for time sync (default: 172.16.0.26)')
    parser.add_argument('--no-sync', action='store_true',
                        help='Disable time sync (use local clock only)')
    args = parser.parse_args()

    rclpy.init()
    node = EventLogger(
        role=args.role,
        drone_id=args.drone_id,
        gcs_host='' if args.no_sync else args.gcs,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f'\n[EventLogger] Files saved: {node._event_file}, {node._state_file}')


if __name__ == '__main__':
    main()
