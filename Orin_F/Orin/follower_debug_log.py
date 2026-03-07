#!/usr/bin/env python3
"""
follower_debug_log.py — Bench-test debug logger for follower drone.

Run on the FOLLOWER JETSON alongside the full ROS2 stack.
Shows a live dashboard with status, blockers, and all data flows:

  [STATUS]   What's working, what's blocking (GPS, GUIDED, leader, etc.)
  [LEADER]   Data received from leader via RFD900x radio
  [OWN]      Data from follower's own CubeOrange via MAVROS2
  [CMD]      Velocity commands being sent TO CubeOrange
  [CALC]     Independent guidance calc — shows what WOULD happen even if
             not in GUIDED or FOLLOW mode yet

Logs everything to timestamped CSV for post-test analysis.

Usage:
    python3 follower_debug_log.py              # 2 Hz print + CSV log
    python3 follower_debug_log.py --hz 5       # faster refresh
    python3 follower_debug_log.py --no-file    # no CSV, print only
"""

import os
import sys
import csv
import time
import math
import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Float64, String
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, PositionTarget
from swarm_msgs.msg import PeerStates

# Try importing guidance_lib for independent guidance calculation
try:
    from guidance_lib import GuidanceConfig, GuidanceState, compute_guidance
    from guidance_lib.target import TargetComputer
    HAS_GUIDANCE = True
except ImportError:
    HAS_GUIDANCE = False

# ═══════════════════════════════════════════════════════════════
# Colors
# ═══════════════════════════════════════════════════════════════
G = "\033[92m"   # green
R = "\033[91m"   # red
Y = "\033[93m"   # yellow
C = "\033[96m"   # cyan
M = "\033[95m"   # magenta
B = "\033[1m"    # bold
D = "\033[2m"    # dim
X = "\033[0m"    # reset

ARDUPILOT_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
    4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
    9: "LAND", 16: "POSHOLD", 17: "BRAKE", 21: "SMART_RTL",
}
GUID_MODES = {0: "NONE", 1: "TRACKING", 2: "CATCHUP", 3: "EVASION", 4: "FAILSAFE"}
METERS_PER_DEG = 111320.0


class FollowerDebugLogger(Node):
    def __init__(self, print_hz, log_file, offset_n, offset_e, offset_d):
        super().__init__('follower_debug_logger')

        self._hz = print_hz
        self._log_file = log_file
        self._csv_writer = None
        self._csv_fh = None
        self._tick = 0

        # Formation offset for independent calc
        self._offset_n = offset_n
        self._offset_e = offset_e
        self._offset_d = offset_d

        # ── Own CubeOrange data ─────────────────────────────────
        self._own_lat = 0.0
        self._own_lon = 0.0
        self._own_alt = 0.0
        self._own_vn = 0.0
        self._own_ve = 0.0
        self._own_vd = 0.0
        self._own_heading = 0.0
        self._own_mode = "?"
        self._own_armed = False
        self._own_connected = False
        self._own_batt_v = 0.0
        self._own_batt_pct = -1.0
        self._own_gps_t = 0.0
        self._own_vel_t = 0.0

        # ── Leader data (from RFD900x) ──────────────────────────
        self._ldr_id = 0
        self._ldr_lat = 0.0
        self._ldr_lon = 0.0
        self._ldr_alt = 0.0
        self._ldr_vn = 0.0
        self._ldr_ve = 0.0
        self._ldr_vd = 0.0
        self._ldr_heading = 0.0
        self._ldr_fc_mode = 0
        self._ldr_armed = False
        self._ldr_batt_v = 0.0
        self._ldr_guidance = 0
        self._ldr_stamp = 0.0
        self._peer_alive = False
        self._radio_rssi = 0
        self._radio_remrssi = 0
        self._radio_noise = 0
        self._radio_txbuf = 0
        self._peer_t = 0.0
        self._peer_count = 0

        # ── CMD to CubeOrange ───────────────────────────────────
        self._cmd_vn = 0.0
        self._cmd_ve = 0.0
        self._cmd_vd = 0.0
        self._cmd_t = 0.0
        self._cmd_count = 0

        # ── Mission state (from control node) ───────────────────
        self._mission_state = "?"
        self._mission_t = 0.0

        # ── Guidance mode ───────────────────────────────────────
        self._guid_mode_val = 0.0

        # ── Independent guidance calc ───────────────────────────
        self._calc_vn = 0.0
        self._calc_ve = 0.0
        self._calc_vd = 0.0
        self._calc_mode = "N/A"
        if HAS_GUIDANCE:
            self._guid_cfg = GuidanceConfig()
            self._guid_state = GuidanceState()
            self._target_comp = TargetComputer()

        # ═══════════════════════════════════════════════════════
        # Subscriptions
        # ═══════════════════════════════════════════════════════
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self._cb_gps, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self._cb_alt, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self._cb_vel, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self._cb_hdg, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self._cb_state, 10)
        self.create_subscription(BatteryState, '/mavros/battery', self._cb_batt, qos_profile_sensor_data)
        self.create_subscription(PeerStates, '/swarm/peer_states', self._cb_peers, 10)
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', self._cb_cmd, 10)
        self.create_subscription(Float64, '/swarm/guidance_mode', self._cb_guid, 10)
        self.create_subscription(String, '/swarm/mission_state', self._cb_mission, 10)

        # CSV
        if self._log_file:
            self._csv_fh = open(self._log_file, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_fh)
            self._csv_writer.writerow([
                'time', 'tick',
                'own_lat', 'own_lon', 'own_alt', 'own_vn', 'own_ve', 'own_vd',
                'own_heading', 'own_mode', 'own_armed', 'own_batt_v',
                'ldr_id', 'ldr_lat', 'ldr_lon', 'ldr_alt',
                'ldr_vn', 'ldr_ve', 'ldr_vd', 'ldr_heading',
                'ldr_fc_mode', 'ldr_armed', 'ldr_batt_v', 'ldr_age_s',
                'peer_alive', 'radio_rssi', 'radio_remrssi',
                'cmd_vn', 'cmd_ve', 'cmd_vd', 'cmd_speed',
                'mission_state', 'guidance_mode',
                'calc_vn', 'calc_ve', 'calc_vd', 'calc_mode', 'dist_m',
            ])

        # Timer
        self.create_timer(1.0 / print_hz, self._tick_fn)
        self.get_logger().info(f"Debug logger: {print_hz} Hz, log={log_file or 'disabled'}")

    # ─── Callbacks ────────────────────────────────────────────
    def _cb_gps(self, m):
        self._own_lat = m.latitude
        self._own_lon = m.longitude
        self._own_gps_t = time.time()

    def _cb_alt(self, m):
        self._own_alt = m.data

    def _cb_vel(self, m):
        self._own_vn = m.twist.linear.y    # ENU.y → North
        self._own_ve = m.twist.linear.x    # ENU.x → East
        self._own_vd = -m.twist.linear.z   # -ENU.z → Down
        self._own_vel_t = time.time()

    def _cb_hdg(self, m):
        self._own_heading = m.data

    def _cb_state(self, m):
        self._own_mode = m.mode
        self._own_armed = m.armed
        self._own_connected = m.connected

    def _cb_batt(self, m):
        self._own_batt_v = m.voltage
        if m.percentage >= 0:
            self._own_batt_pct = m.percentage * 100.0

    def _cb_peers(self, m):
        self._peer_t = time.time()
        self._peer_alive = m.peer_jetson_alive
        self._radio_rssi = m.radio_rssi
        self._radio_remrssi = m.radio_remrssi
        self._radio_noise = m.radio_noise
        self._radio_txbuf = m.radio_txbuf
        self._peer_count += 1
        if m.peers:
            p = m.peers[0]
            self._ldr_id = p.drone_id
            self._ldr_lat = p.latitude
            self._ldr_lon = p.longitude
            self._ldr_alt = p.altitude
            self._ldr_vn = p.vn
            self._ldr_ve = p.ve
            self._ldr_vd = p.vd
            self._ldr_heading = p.heading
            self._ldr_fc_mode = p.fc_mode_code
            self._ldr_armed = p.fc_armed
            self._ldr_batt_v = p.battery_voltage
            self._ldr_guidance = p.guidance_mode
            self._ldr_stamp = p.stamp

    def _cb_cmd(self, m):
        self._cmd_vn = m.velocity.x
        self._cmd_ve = m.velocity.y
        self._cmd_vd = m.velocity.z
        self._cmd_t = time.time()
        self._cmd_count += 1

    def _cb_guid(self, m):
        self._guid_mode_val = m.data

    def _cb_mission(self, m):
        self._mission_state = m.data
        self._mission_t = time.time()

    # ─── Derived ──────────────────────────────────────────────
    def _dist3d(self):
        if self._ldr_lat == 0 and self._ldr_lon == 0:
            return -1.0
        dn = (self._ldr_lat - self._own_lat) * METERS_PER_DEG
        de = (self._ldr_lon - self._own_lon) * METERS_PER_DEG * math.cos(math.radians(self._own_lat))
        dd = self._ldr_alt - self._own_alt
        return math.sqrt(dn*dn + de*de + dd*dd)

    def _ldr_age(self):
        return time.time() - self._peer_t if self._peer_t > 0 else -1.0

    def _cmd_spd(self):
        return math.sqrt(self._cmd_vn**2 + self._cmd_ve**2 + self._cmd_vd**2)

    def _run_calc(self):
        """Run guidance independently — shows what follower WOULD do."""
        if not HAS_GUIDANCE:
            self._calc_mode = "NO_LIB"
            return
        if self._own_lat == 0 or self._ldr_lat == 0:
            self._calc_mode = "NO_GPS"
            return
        if self._ldr_age() > 2.0 or self._ldr_age() < 0:
            self._calc_mode = "STALE"
            return

        leader = {
            'lat': self._ldr_lat, 'lon': self._ldr_lon, 'alt': self._ldr_alt,
            'vn': self._ldr_vn, 've': self._ldr_ve, 'vd': self._ldr_vd,
            'heading': self._ldr_heading, 'stamp': self._ldr_stamp,
        }
        follower = {
            'lat': self._own_lat, 'lon': self._own_lon, 'alt': self._own_alt,
            'vn': self._own_vn, 've': self._own_ve, 'vd': self._own_vd,
            'heading': self._own_heading,
        }
        try:
            target = self._target_comp.compute(
                leader, self._offset_n, self._offset_e, self._offset_d)
            result = compute_guidance(
                self._guid_cfg, self._guid_state,
                follower, target, [leader],
            )
            self._calc_vn = result.get('vn', 0.0)
            self._calc_ve = result.get('ve', 0.0)
            self._calc_vd = result.get('vd', 0.0)
            self._calc_mode = result.get('mode', '?')
        except Exception as e:
            self._calc_mode = "ERR"
            self.get_logger().debug(f"Calc: {e}")

    # ─── Display tick ─────────────────────────────────────────
    def _tick_fn(self):
        self._tick += 1
        now = time.time()
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-4]

        self._run_calc()

        ldr_age = self._ldr_age()
        dist = self._dist3d()
        cmd_spd = self._cmd_spd()
        cmd_active = (now - self._cmd_t < 1.0) if self._cmd_t > 0 else False
        guid_str = GUID_MODES.get(int(self._guid_mode_val), "?")
        ldr_mode = ARDUPILOT_MODES.get(self._ldr_fc_mode, f"?({self._ldr_fc_mode})")
        own_spd = math.sqrt(self._own_vn**2 + self._own_ve**2)
        calc_spd = math.sqrt(self._calc_vn**2 + self._calc_ve**2 + self._calc_vd**2)

        # Status flags
        has_mavros = self._own_connected
        has_gps = self._own_lat != 0.0 or self._own_lon != 0.0
        is_guided = "GUIDED" in self._own_mode
        has_leader = 0 <= ldr_age < 2.0
        is_follow = "FOLLOW" in self._mission_state.upper() if self._mission_state else False

        def chk(ok, ok_t, fail_t):
            return f"{G}{ok_t}{X}" if ok else f"{R}{fail_t}{X}"

        # ═══ Clear + render ═══
        print("\033[2J\033[H", end="")

        W = 72
        print(f"{B}╔{'═'*W}╗")
        print(f"║  FOLLOWER BENCH TEST — {ts}  #{self._tick:<6}   NO PROPS TEST        ║")
        print(f"╚{'═'*W}╝{X}")

        # ── STATUS ──
        print(f"\n  {B}STATUS:{X}  "
              f"MAVROS:{chk(has_mavros,'OK','NO')}  "
              f"GPS:{chk(has_gps,'OK','NO')}  "
              f"MODE:{chk(is_guided,'GUIDED',self._own_mode)}  "
              f"Leader:{chk(has_leader,'OK','NO')}  "
              f"Mission:{Y}{B}{self._mission_state}{X}")

        blockers = []
        if not has_mavros: blockers.append("MAVROS2 not connected — is ros2 launch running?")
        if not has_gps:    blockers.append("No GPS fix — go outdoors or wait for sat lock")
        if not is_guided:  blockers.append(f"FC not in GUIDED (is {self._own_mode}) — switch via RC or GCS")
        if not has_leader:
            if ldr_age < 0: blockers.append("No leader data — is leader Jetson + RFD900x on?")
            else:           blockers.append(f"Leader data stale ({ldr_age:.1f}s old)")
        if has_leader and is_guided and not is_follow and self._mission_state not in ("?",""):
            blockers.append(f"Mission is {self._mission_state} — send FOLLOW from GCS")

        if blockers:
            print(f"\n  {R}{B}BLOCKERS (fix these to see CMD output):{X}")
            for bl in blockers:
                print(f"    {R}► {bl}{X}")
        elif cmd_active:
            print(f"\n  {G}{B}  ✓ ALL SYSTEMS GO — commands flowing to CubeOrange!{X}")
        else:
            print(f"\n  {Y}{B}  Status looks OK but no commands yet — wait a moment...{X}")

        # ── FROM LEADER ──
        print(f"\n{C}{B}  ◄◄ FROM LEADER (RFD900x)  [{self._peer_count} msgs received]{X}")
        print(f"  ┌{'─'*W}┐")
        if self._peer_t > 0:
            ac = G if (0 <= ldr_age < 1) else (Y if ldr_age < 2 else R)
            rc = G if self._radio_rssi > 100 else (Y if self._radio_rssi > 50 else R)
            print(f"  │  ID: {self._ldr_id:<3}  FC: {ldr_mode:<12}  Armed: {'YES' if self._ldr_armed else 'no ':3}  Batt: {self._ldr_batt_v:5.1f}V             │")
            print(f"  │  GPS:  {self._ldr_lat:12.7f}°  {self._ldr_lon:12.7f}°  alt {self._ldr_alt:6.1f}m              │")
            print(f"  │  Vel:  vn={self._ldr_vn:+6.2f}  ve={self._ldr_ve:+6.2f}  vd={self._ldr_vd:+6.2f} m/s   hdg {self._ldr_heading:5.1f}°       │")
            print(f"  │  Age:  {ac}{ldr_age:5.2f}s{X}   Alive: {'YES' if self._peer_alive else 'NO ':3}   "
                  f"RSSI: {rc}{self._radio_rssi:3}{X}/{self._radio_remrssi:3}  TxBuf: {self._radio_txbuf:3}%         │")
        else:
            print(f"  │  {D}Waiting for RFD900x link... (is leader Jetson running?){X}              │")
        print(f"  └{'─'*W}┘")

        # ── FROM OWN CUBE ──
        mc = G if is_guided else Y
        print(f"\n{G}{B}  ■■ FROM OWN CUBE (MAVROS2){X}")
        print(f"  ┌{'─'*W}┐")
        print(f"  │  FC: {mc}{self._own_mode:<12}{X}  Armed: {'YES' if self._own_armed else 'no ':3}  Conn: {'YES' if self._own_connected else 'NO ':3}  Batt: {self._own_batt_v:5.1f}V          │")
        print(f"  │  GPS:  {self._own_lat:12.7f}°  {self._own_lon:12.7f}°  alt {self._own_alt:6.1f}m              │")
        print(f"  │  Vel:  vn={self._own_vn:+6.2f}  ve={self._own_ve:+6.2f}  vd={self._own_vd:+6.2f} m/s   hdg {self._own_heading:5.1f}°       │")
        print(f"  │  Speed: {own_spd:5.2f} m/s  (drone is {'moving' if own_spd > 0.1 else 'stationary'})                            │")
        print(f"  └{'─'*W}┘")

        # ── CMD TO CUBE (actual from control node) ──
        print(f"\n{M}{B}  ►► COMMAND TO CUBE  [{self._cmd_count} cmds sent]{X}")
        print(f"  ┌{'─'*W}┐")
        if cmd_active:
            sc = R if cmd_spd > 5 else (Y if cmd_spd > 3 else G)
            print(f"  │  {B}vn={sc}{self._cmd_vn:+7.3f}{X}  {B}ve={sc}{self._cmd_ve:+7.3f}{X}  {B}vd={sc}{self._cmd_vd:+7.3f}{X} m/s               │")
            print(f"  │  Speed: {sc}{B}{cmd_spd:5.2f} m/s{X}  Guidance: {B}{guid_str:<10}{X}  Mission: {B}{self._mission_state}{X}     │")
        else:
            reason = "waiting for: " + " + ".join(
                [x for x in [
                    "" if has_mavros else "MAVROS",
                    "" if has_gps else "GPS",
                    "" if is_guided else "GUIDED",
                    "" if has_leader else "LEADER",
                    "" if is_follow else "FOLLOW cmd",
                ] if x]) or "starting up..."
            print(f"  │  {D}No commands — {reason}{X}{'':>{W-17-len(reason)}}│")
            print(f"  │  {D}Mission: {self._mission_state}   Guidance: {guid_str}{X}                                    │")
        print(f"  └{'─'*W}┘")

        # ── INDEPENDENT CALC (what WOULD happen) ──
        print(f"\n{Y}{B}  ⚡ CALC (independent guidance — works without GUIDED/FOLLOW){X}")
        print(f"  ┌{'─'*W}┐")
        if self._calc_mode not in ("NO_GPS", "STALE", "N/A", "NO_LIB", "ERR"):
            cc = R if calc_spd > 5 else (Y if calc_spd > 3 else G)
            print(f"  │  {B}vn={cc}{self._calc_vn:+7.3f}{X}  {B}ve={cc}{self._calc_ve:+7.3f}{X}  {B}vd={cc}{self._calc_vd:+7.3f}{X} m/s               │")
            print(f"  │  Speed: {cc}{B}{calc_spd:5.2f} m/s{X}  Mode: {B}{self._calc_mode:<10}{X}  "
                  f"Dist: {dist:5.1f}m                │")
            if cmd_active:
                dvn = abs(self._cmd_vn - self._calc_vn)
                dve = abs(self._cmd_ve - self._calc_ve)
                match = dvn < 0.1 and dve < 0.1
                print(f"  │  vs CMD: Δvn={dvn:.3f} Δve={dve:.3f}  "
                      f"{'MATCH ✓' if match else 'MISMATCH (smoother/timing)':30}        │")
            else:
                print(f"  │  {D}(This is what control node WOULD send if in GUIDED+FOLLOW){X}       │")
        else:
            reasons = {"NO_GPS": "Need GPS on both drones", "STALE": "Leader data too old (>2s)",
                       "N/A": "Waiting for data", "NO_LIB": "guidance_lib not on PYTHONPATH",
                       "ERR": "Guidance compute error"}
            print(f"  │  {D}Cannot compute: {reasons.get(self._calc_mode, self._calc_mode)}{X}                              │")
            print(f"  │  {D}(Move leader so both have different GPS positions){X}                    │")
        print(f"  └{'─'*W}┘")

        # ── Distance ──
        dn = (self._ldr_lat - self._own_lat) * METERS_PER_DEG if has_gps and self._ldr_lat != 0 else 0
        de = (self._ldr_lon - self._own_lon) * METERS_PER_DEG * math.cos(math.radians(self._own_lat)) if has_gps and self._ldr_lat != 0 else 0
        dd = self._ldr_alt - self._own_alt if has_gps and self._ldr_lat != 0 else 0

        # ── KEY DATA: Position & Velocity Comparison ──
        print(f"\n{B}  ═══ POSITION & VELOCITY COMPARISON ═══{X}")
        print(f"  ┌─────────────────┬──────────────────────────┬──────────────────────────┐")
        print(f"  │                 │  {C}{B}LEADER{X}                   │  {G}{B}FOLLOWER{X}                 │")
        print(f"  ├─────────────────┼──────────────────────────┼──────────────────────────┤")
        print(f"  │  Latitude       │  {self._ldr_lat:13.7f}°          │  {self._own_lat:13.7f}°          │")
        print(f"  │  Longitude      │  {self._ldr_lon:13.7f}°          │  {self._own_lon:13.7f}°          │")
        print(f"  │  Altitude       │  {self._ldr_alt:8.2f} m            │  {self._own_alt:8.2f} m            │")
        print(f"  │  Vel North      │  {self._ldr_vn:+8.3f} m/s          │  {self._own_vn:+8.3f} m/s          │")
        print(f"  │  Vel East       │  {self._ldr_ve:+8.3f} m/s          │  {self._own_ve:+8.3f} m/s          │")
        print(f"  │  Vel Down       │  {self._ldr_vd:+8.3f} m/s          │  {self._own_vd:+8.3f} m/s          │")
        print(f"  │  Heading        │  {self._ldr_heading:7.1f}°             │  {self._own_heading:7.1f}°             │")
        print(f"  ├─────────────────┼──────────────────────────┴──────────────────────────┤")
        dc = G if dist >= 0 and dist < 20 else (Y if dist >= 0 and dist < 50 else R)
        print(f"  │  {B}Δ North{X}        │  {dc}{B}{dn:+8.2f} m{X}                                              │")
        print(f"  │  {B}Δ East{X}         │  {dc}{B}{de:+8.2f} m{X}                                              │")
        print(f"  │  {B}Δ Alt{X}          │  {dc}{B}{dd:+8.2f} m{X}                                              │")
        print(f"  │  {B}Distance{X}       │  {dc}{B}{dist:8.2f} m{X}                                              │")
        print(f"  ├─────────────────┼─────────────────────────────────────────────────────┤")
        if cmd_active:
            print(f"  │  {M}{B}CMD velocity{X}   │  vn={self._cmd_vn:+7.3f}  ve={self._cmd_ve:+7.3f}  vd={self._cmd_vd:+7.3f}  spd={cmd_spd:.2f}   │")
        else:
            print(f"  │  CMD velocity   │  {D}(no commands yet){X}                                    │")
        print(f"  │  Guidance       │  {guid_str:<10}   Mission: {self._mission_state:<15}              │")
        print(f"  └─────────────────┴─────────────────────────────────────────────────────┘")

        print(f"\n  Offset: [{self._offset_n},{self._offset_e},{self._offset_d}] NED  |  RSSI: {self._radio_rssi}/{self._radio_remrssi}  |  Peers: {self._peer_count}")
        print(f"  {D}Ctrl+C to stop | Log: {self._log_file or 'disabled'} | Hz: {self._hz}{X}")

        # ── CSV ──
        if self._csv_writer:
            self._csv_writer.writerow([
                ts, self._tick,
                f"{self._own_lat:.7f}", f"{self._own_lon:.7f}", f"{self._own_alt:.2f}",
                f"{self._own_vn:.3f}", f"{self._own_ve:.3f}", f"{self._own_vd:.3f}",
                f"{self._own_heading:.1f}", self._own_mode, self._own_armed, f"{self._own_batt_v:.2f}",
                self._ldr_id, f"{self._ldr_lat:.7f}", f"{self._ldr_lon:.7f}", f"{self._ldr_alt:.2f}",
                f"{self._ldr_vn:.3f}", f"{self._ldr_ve:.3f}", f"{self._ldr_vd:.3f}",
                f"{self._ldr_heading:.1f}", self._ldr_fc_mode, self._ldr_armed, f"{self._ldr_batt_v:.2f}",
                f"{ldr_age:.3f}" if ldr_age >= 0 else "-1",
                self._peer_alive, self._radio_rssi, self._radio_remrssi,
                f"{self._cmd_vn:.3f}", f"{self._cmd_ve:.3f}", f"{self._cmd_vd:.3f}", f"{cmd_spd:.3f}",
                self._mission_state, guid_str,
                f"{self._calc_vn:.3f}", f"{self._calc_ve:.3f}", f"{self._calc_vd:.3f}",
                self._calc_mode, f"{dist:.2f}" if dist >= 0 else "-1",
            ])
            self._csv_fh.flush()

    def destroy_node(self):
        if self._csv_fh:
            self._csv_fh.close()
        super().destroy_node()


def _preflight(node, timeout=60):
    """
    Interactive preflight check — spins the node until all data
    links are confirmed or the user hits Ctrl+C.  Returns True
    when all checks pass.
    """
    CHECKS = [
        ("MAVROS Connected",  lambda: node._own_connected),
        ("GPS Fix",           lambda: node._own_lat != 0.0 or node._own_lon != 0.0),
        ("Leader Link (RFD)", lambda: 0 <= node._ldr_age() < 3.0),
        ("Leader GPS",        lambda: node._ldr_lat != 0.0 or node._ldr_lon != 0.0),
    ]

    TIPS = {
        "MAVROS Connected":  "Is  ros2 launch follower_pkg follower_bringup.launch.py  running?",
        "GPS Fix":           "Go outdoors & wait for sat lock.  Takes 30-90s cold start.",
        "Leader Link (RFD)": "Is leader Jetson running its launch?  Check RFD900x LED & USB.",
        "Leader GPS":        "Move leader outdoors until its GPS locks.",
    }

    start = time.time()
    all_pass = False

    while True:
        # spin a few times to get fresh callbacks
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)

        elapsed = time.time() - start
        print("\033[2J\033[H", end="")
        print(f"{B}╔══════════════════════════════════════════════════════════════════════╗")
        print(f"║           PREFLIGHT DATA CHECK          {elapsed:5.0f}s elapsed              ║")
        print(f"╚══════════════════════════════════════════════════════════════════════╝{X}\n")

        results = []
        all_ok = True
        for label, fn in CHECKS:
            ok = fn()
            results.append((label, ok))
            if not ok:
                all_ok = False

        for label, ok in results:
            icon = f"{G}✓ PASS{X}" if ok else f"{R}✗ WAIT{X}"
            print(f"    {icon}   {label}")
            if not ok:
                tip = TIPS.get(label, "")
                if tip:
                    print(f"           {D}→ {tip}{X}")

        # Extra info once MAVROS is up
        if node._own_connected:
            own_lat_s = f"{node._own_lat:.7f}" if node._own_lat != 0 else "---"
            own_lon_s = f"{node._own_lon:.7f}" if node._own_lon != 0 else "---"
            print(f"\n    {D}Follower FC:  mode={node._own_mode}  armed={'Y' if node._own_armed else 'N'}  "
                  f"batt={node._own_batt_v:.1f}V{X}")
            print(f"    {D}Follower GPS: {own_lat_s}° , {own_lon_s}°{X}")

        if node._ldr_lat != 0:
            print(f"    {D}Leader GPS:   {node._ldr_lat:.7f}° , {node._ldr_lon:.7f}°{X}")
            age = node._ldr_age()
            print(f"    {D}Leader age:   {age:.1f}s  RSSI: {node._radio_rssi}/{node._radio_remrssi}{X}")

        if all_ok:
            print(f"\n  {G}{B}  ✓ ALL CHECKS PASSED{X}")
            print(f"\n  Press {B}ENTER{X} to start dashboard + logging, or {B}Ctrl+C{X} to abort.")
            all_pass = True
            break
        else:
            print(f"\n  {Y}Waiting for all checks to pass... (Ctrl+C to abort){X}")

    return all_pass


def main():
    parser = argparse.ArgumentParser(description="Follower bench-test debug logger")
    parser.add_argument('--hz', type=float, default=2.0, help='Refresh rate (default: 2)')
    parser.add_argument('--no-file', action='store_true', help='Disable CSV logging')
    parser.add_argument('--skip-check', action='store_true', help='Skip preflight data check')
    parser.add_argument('--offset-n', type=float, default=-4.0, help='Formation offset N (default: -4.0)')
    parser.add_argument('--offset-e', type=float, default=3.0, help='Formation offset E (default: 3.0)')
    parser.add_argument('--offset-d', type=float, default=0.0, help='Formation offset D (default: 0.0)')
    args = parser.parse_args()

    log_file = None
    if not args.no_file:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = os.path.expanduser(f'~/follower_log_{stamp}.csv')

    rclpy.init()
    node = FollowerDebugLogger(args.hz, log_file, args.offset_n, args.offset_e, args.offset_d)
    try:
        # ── Preflight data check ──
        if not args.skip_check:
            try:
                passed = _preflight(node)
            except KeyboardInterrupt:
                print(f"\n\n  {R}Preflight aborted.{X}\n")
                node.destroy_node()
                rclpy.shutdown()
                return
            if passed:
                input()  # wait for ENTER
                print(f"\n  {G}Starting dashboard...{X}\n")
                time.sleep(0.5)
        else:
            print(f"  {Y}Preflight check skipped (--skip-check){X}")
            time.sleep(1)

        # ── Main dashboard loop ──
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n\n  Stopped.")
        if log_file:
            print(f"  Log saved: {log_file}")
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
