#!/usr/bin/env python3
"""Pre-flight check script for Orin Swarm Leader-Follower system.

Run on each Jetson AFTER launching MAVROS2 only (not the full stack):
    ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyUSB0:115200

Then in another terminal:
    python3 preflight_check.py

This script checks:
  1. MAVROS2 connectivity — is the FC talking?
  2. GPS lock — do we have a valid fix?
  3. Battery — is it reading voltage?
  4. ENU→NED velocity mapping — shows live velocity so you can physically
     push/tilt the drone and verify which axis responds.
  5. Serial port detection — are ttyUSB0 and ttyUSB1 present?
  6. RFD900x radio — can we open the serial port?
  7. hierarchy.yaml — does it exist and parse correctly?

Usage:
    python3 preflight_check.py                  # run all checks
    python3 preflight_check.py --velocity-only  # just the live velocity monitor
"""

import sys
import os
import time
import math
import argparse
import subprocess

# ═══════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

def ok(msg):
    print(f"  {GREEN}[OK]{RESET}   {msg}")

def fail(msg):
    print(f"  {RED}[FAIL]{RESET} {msg}")

def warn(msg):
    print(f"  {YELLOW}[WARN]{RESET} {msg}")

def info(msg):
    print(f"  {CYAN}[INFO]{RESET} {msg}")

def header(title):
    print(f"\n{BOLD}{'═' * 60}")
    print(f"  {title}")
    print(f"{'═' * 60}{RESET}")


# ═══════════════════════════════════════════════════════════════
# Check 1: Serial ports
# ═══════════════════════════════════════════════════════════════

def check_serial_ports():
    header("CHECK 1: Serial Ports")
    import glob
    usb_ports = sorted(glob.glob('/dev/ttyUSB*'))
    ths_ports = sorted(glob.glob('/dev/ttyTHS*'))
    all_ports = usb_ports + ths_ports

    if not all_ports:
        fail("No serial ports found (/dev/ttyUSB* or /dev/ttyTHS*)")
        info("Check USB-TTL adapter and RFD900x connections")
        return False

    for p in all_ports:
        info(f"Found: {p}")

    if '/dev/ttyUSB0' in usb_ports:
        ok("ttyUSB0 present (expected: CubeOrange+ TELEM2 via USB-TTL)")
    else:
        warn("ttyUSB0 not found — CubeOrange+ may be on a different port")

    if '/dev/ttyUSB1' in usb_ports:
        ok("ttyUSB1 present (expected: RFD900x radio)")
    else:
        warn("ttyUSB1 not found — RFD900x may be on a different port or unplugged")

    return True


# ═══════════════════════════════════════════════════════════════
# Check 2: hierarchy.yaml
# ═══════════════════════════════════════════════════════════════

def check_hierarchy():
    header("CHECK 2: hierarchy.yaml")
    import yaml

    # Check common paths
    paths = [
        os.path.expanduser('~/hierarchy.yaml'),
        os.path.join(os.path.dirname(__file__), 'hierarchy.yaml'),
    ]

    found_path = None
    for p in paths:
        if os.path.isfile(p):
            found_path = p
            break

    if not found_path:
        fail(f"hierarchy.yaml not found at: {paths}")
        return False

    ok(f"Found: {found_path}")

    with open(found_path) as f:
        data = yaml.safe_load(f)

    lat = data.get('home', {}).get('lat', 0)
    lon = data.get('home', {}).get('lon', 0)

    if lat == 0 and lon == 0:
        fail("Home lat/lon are both 0.0 — set your field GPS coordinates!")
        return False

    # Sanity check — is it a real GPS coordinate?
    if abs(lat) < 1 or abs(lon) < 1:
        warn(f"Home coords look suspicious: lat={lat}, lon={lon}")
    else:
        ok(f"Home: lat={lat:.7f}, lon={lon:.7f}")

    drones = data.get('drones', {})
    for did, d in drones.items():
        role = d.get('role', 'unknown')
        info(f"Drone {did}: role={role}, offset={d.get('offset', 'N/A')}")

    return True


# ═══════════════════════════════════════════════════════════════
# Check 3: RFD900x serial
# ═══════════════════════════════════════════════════════════════

def check_rfd900x(device='/dev/ttyUSB1', baud=115200):
    header("CHECK 3: RFD900x Radio Serial")

    if not os.path.exists(device):
        fail(f"Device {device} does not exist")
        return False

    try:
        import serial
        ser = serial.Serial(device, baud, timeout=2)
        ok(f"Opened {device} @ {baud} baud")
        ser.close()
        return True
    except ImportError:
        # Try with pymavlink instead
        try:
            from pymavlink import mavutil
            conn = mavutil.mavlink_connection(device, baud=baud)
            ok(f"Opened {device} @ {baud} baud (via pymavlink)")
            conn.close()
            return True
        except Exception as e:
            fail(f"Cannot open {device}: {e}")
            return False
    except Exception as e:
        fail(f"Cannot open {device}: {e}")
        info("Check permissions: sudo chmod 666 /dev/ttyUSB1")
        info("Or add user to dialout group: sudo usermod -aG dialout $USER")
        return False


# ═══════════════════════════════════════════════════════════════
# Check 4-7: ROS2 / MAVROS2 checks (requires MAVROS2 running)
# ═══════════════════════════════════════════════════════════════

def check_ros2_topics():
    """Check MAVROS2 connectivity, GPS, battery using ros2 topic echo."""
    header("CHECK 4: MAVROS2 Connectivity")

    # Check if MAVROS2 is running
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=5)
        topics = result.stdout.strip().split('\n')
    except (FileNotFoundError, subprocess.TimeoutExpired):
        fail("ros2 command not found or timed out — is ROS2 sourced?")
        info("Run: source /opt/ros/humble/setup.bash")
        return False

    mavros_topics = [t for t in topics if t.startswith('/mavros')]
    if not mavros_topics:
        fail("No /mavros topics found — is MAVROS2 running?")
        info("Start with: ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyUSB0:115200")
        return False

    ok(f"Found {len(mavros_topics)} MAVROS2 topics")

    # ── FC State ──
    header("CHECK 5: FC Connection (HEARTBEAT)")
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/state', '--once'],
            capture_output=True, text=True, timeout=10)
        output = result.stdout
        if 'connected: true' in output.lower() or 'connected: True' in output:
            ok("CubeOrange+ FC connected!")
            if 'armed: true' in output.lower() or 'armed: True' in output:
                warn("FC is ARMED — be careful!")
            else:
                info("FC is disarmed (expected on ground)")
            # Extract mode
            for line in output.split('\n'):
                if 'mode:' in line:
                    info(f"FC {line.strip()}")
        else:
            fail("FC not connected — check USB-TTL wiring (TELEM2)")
            return False
    except subprocess.TimeoutExpired:
        fail("Timed out waiting for /mavros/state — FC not responding")
        return False

    # ── GPS ──
    header("CHECK 6: GPS Fix")
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/global_position/global', '--once'],
            capture_output=True, text=True, timeout=15)
        output = result.stdout

        lat = lon = 0.0
        for line in output.split('\n'):
            if 'latitude:' in line:
                lat = float(line.split(':')[1].strip())
            if 'longitude:' in line:
                lon = float(line.split(':')[1].strip())

        if lat != 0.0 or lon != 0.0:
            ok(f"GPS fix: lat={lat:.7f}, lon={lon:.7f}")
        else:
            warn("GPS reads 0.0, 0.0 — waiting for satellite lock (go outdoors)")
    except subprocess.TimeoutExpired:
        warn("Timed out waiting for GPS — may not have lock yet")

    # ── Battery ──
    header("CHECK 7: Battery (SYS_STATUS)")
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/battery', '--once'],
            capture_output=True, text=True, timeout=10)
        output = result.stdout

        voltage = 0.0
        for line in output.split('\n'):
            if 'voltage:' in line:
                try:
                    voltage = float(line.split(':')[1].strip())
                except ValueError:
                    pass

        if voltage > 5.0:
            ok(f"Battery voltage: {voltage:.1f}V")
            if voltage < 14.0:
                warn(f"Battery low! {voltage:.1f}V — charge before flight")
        elif voltage > 0:
            info(f"Battery voltage: {voltage:.1f}V (USB powered?)")
        else:
            warn("Battery voltage not available")
    except subprocess.TimeoutExpired:
        warn("Timed out waiting for battery data")

    return True


# ═══════════════════════════════════════════════════════════════
# Check 9-12: Extended preflight (EKF, mode change, RC, swarm)
# ═══════════════════════════════════════════════════════════════

def check_ekf_status():
    """Check EKF health via /mavros/estimator_status or /diagnostics."""
    header("CHECK 9: EKF Health")

    # Try /mavros/estimator_status first
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/estimator_status', '--once'],
            capture_output=True, text=True, timeout=8)
        output = result.stdout
        if output.strip():
            ok("Estimator status topic is publishing")
            # Look for position/velocity flags
            for line in output.split('\n'):
                line_lower = line.strip().lower()
                if 'pos_horiz_abs' in line_lower or 'pred_pos_horiz_abs' in line_lower:
                    if 'true' in line_lower:
                        ok(f"  {line.strip()}")
                    else:
                        warn(f"  {line.strip()}")
                elif 'velocity_horiz' in line_lower:
                    if 'true' in line_lower:
                        ok(f"  {line.strip()}")
                    else:
                        warn(f"  {line.strip()}")
            return True
        else:
            warn("No estimator status data received")
    except subprocess.TimeoutExpired:
        pass

    # Fallback: check /diagnostics for EKF
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/diagnostics', '--once'],
            capture_output=True, text=True, timeout=8)
        output = result.stdout
        if 'ekf' in output.lower() or 'EKF' in output:
            info("EKF data found in /diagnostics (check manually)")
            return True
    except subprocess.TimeoutExpired:
        pass

    warn("Cannot verify EKF health — check manually in Mission Planner")
    info("Look for 'EKF' status on HUD — should show green")
    return True  # Non-fatal


def check_mode_change():
    """Test GUIDED mode switch: current → GUIDED → restore original.

    This is CRITICAL for the follower — it MUST accept GUIDED mode to fly autonomously.
    The leader does not need this (RC pilot flies it), but it's good to verify.
    """
    header("CHECK 10: GUIDED Mode Switch Test")

    warn("This test will briefly switch FC to GUIDED mode, then switch back.")
    info("Props should NOT be attached for this test!")
    try:
        resp = input(f"  {BOLD}Run mode switch test? (y/n): {RESET}").strip().lower()
    except EOFError:
        resp = 'n'
    if resp != 'y':
        info("Skipped (user declined)")
        return True

    # Get current mode
    original_mode = None
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/state', '--once'],
            capture_output=True, text=True, timeout=10)
        for line in result.stdout.split('\n'):
            if 'mode:' in line:
                # mode: "STABILIZE" or mode: "LOITER" etc.
                original_mode = line.split(':')[1].strip().strip('"').strip("'")
                break
    except subprocess.TimeoutExpired:
        fail("Cannot read current FC mode — skipping test")
        return False

    if original_mode:
        info(f"Current mode: {original_mode}")
    else:
        fail("Could not determine current mode")
        return False

    # Switch to GUIDED
    info("Switching to GUIDED...")
    try:
        result = subprocess.run(
            ['ros2', 'service', 'call', '/mavros/set_mode',
             'mavros_msgs/srv/SetMode', '{custom_mode: "GUIDED"}'],
            capture_output=True, text=True, timeout=10)
        output = result.stdout + result.stderr

        if 'true' in output.lower() or 'mode_sent: true' in output.lower():
            ok("GUIDED mode accepted!")
        else:
            fail(f"GUIDED mode REJECTED — check pre-arm conditions")
            info("Common fixes: need GPS lock, EKF healthy, safety switch pressed")
            info(f"Service response: {output.strip()[:200]}")
            return False
    except subprocess.TimeoutExpired:
        fail("Set mode service timed out")
        return False

    time.sleep(1)

    # Verify mode actually changed
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/state', '--once'],
            capture_output=True, text=True, timeout=10)
        for line in result.stdout.split('\n'):
            if 'mode:' in line:
                new_mode = line.split(':')[1].strip().strip('"').strip("'")
                if new_mode == "GUIDED":
                    ok(f"Confirmed: FC is now in GUIDED mode")
                else:
                    warn(f"Mode is '{new_mode}' (expected GUIDED) — FC may have rejected it")
                break
    except subprocess.TimeoutExpired:
        warn("Could not verify mode change")

    # Restore original mode
    info(f"Restoring original mode: {original_mode}")
    try:
        subprocess.run(
            ['ros2', 'service', 'call', '/mavros/set_mode',
             'mavros_msgs/srv/SetMode', f'{{custom_mode: "{original_mode}"}}'],
            capture_output=True, text=True, timeout=10)
        ok(f"Restored to {original_mode}")
    except subprocess.TimeoutExpired:
        warn(f"Could not restore mode — manually set to {original_mode} via RC")

    return True


def check_rc_link():
    """Check if RC transmitter is connected to FC."""
    header("CHECK 11: RC Transmitter Link")

    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/rc/in', '--once'],
            capture_output=True, text=True, timeout=8)
        output = result.stdout

        if not output.strip():
            warn("No RC data — transmitter may be off or out of range")
            info("Leader needs RC! Follower can fly without RC (autonomous).")
            return True  # Non-fatal for follower

        # Parse channel values
        channels = []
        in_channels = False
        for line in output.split('\n'):
            if 'channels:' in line:
                in_channels = True
                continue
            if in_channels and line.strip().startswith('-'):
                try:
                    val = int(line.strip().strip('-').strip())
                    channels.append(val)
                except ValueError:
                    pass
            elif in_channels and not line.strip().startswith('-'):
                in_channels = False

        if channels:
            # RC channels should read ~1000-2000 when transmitter is on
            # 0 or 65535 = no signal
            active = [ch for ch in channels if 800 < ch < 2200]
            if len(active) >= 4:
                ok(f"RC transmitter linked! {len(active)} active channels")
                info(f"  Ch1-4 (Roll/Pitch/Throt/Yaw): {channels[:4]}")
            else:
                warn(f"RC channels look wrong: {channels[:8]}")
                info("  Expected values: 1000-2000 per channel when TX is on")
        else:
            warn("Could not parse RC channel data")

    except subprocess.TimeoutExpired:
        warn("Timed out reading RC — /mavros/rc/in may not be publishing")
        info("Set SR2_RC_CHAN=2 in ArduPilot if you want RC data on TELEM2")

    return True


def check_swarm_peer():
    """Check if swarm bridge is running and receiving peer data (follower only)."""
    header("CHECK 12: Swarm Peer Link (RFD900x)")

    # Check if swarm bridge topics exist
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=5)
        topics = result.stdout.strip().split('\n')
    except (FileNotFoundError, subprocess.TimeoutExpired):
        warn("Cannot list topics")
        return True

    swarm_topics = [t for t in topics if '/swarm/' in t]
    if not swarm_topics:
        info("No /swarm/ topics — swarm_bridge_node not running yet")
        info("This is expected if you only launched MAVROS2 so far.")
        info("After full launch ('ros2 launch ...'), re-run this check.")
        return True

    ok(f"Found swarm topics: {', '.join(swarm_topics)}")

    # Try to read peer states
    if '/swarm/peer_states' in swarm_topics:
        info("Reading /swarm/peer_states (waiting up to 10s for radio data)...")
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'echo', '/swarm/peer_states', '--once'],
                capture_output=True, text=True, timeout=12)
            output = result.stdout

            if 'peers:' in output:
                # Check if there are actual peers
                if 'lat:' in output:
                    ok("Peer data received! Leader is transmitting over RFD900x.")
                    for line in output.split('\n'):
                        if 'lat:' in line or 'lon:' in line or 'alt_m:' in line:
                            info(f"  {line.strip()}")
                        if 'radio_rssi:' in line:
                            info(f"  {line.strip()}")
                else:
                    warn("PeerStates published but no peer data inside (leader radio off?)")
            elif 'peers: []' in output or "peers:\n- {}" not in output:
                warn("PeerStates is empty — no peers detected")
                info("  Check: Is the leader Jetson running? Is RFD900x powered?")
                info("  Check: Do both radios have the same Net ID?")
            else:
                info(f"PeerStates output (raw):\n{output[:300]}")
        except subprocess.TimeoutExpired:
            warn("Timed out waiting for peer data — radio link may be down")
            info("  Power-cycle the RFD900x and try again")

    # Check radio RSSI from the bridge
    radio_status_topic = '/swarm/radio_status'
    if radio_status_topic in swarm_topics:
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'echo', radio_status_topic, '--once'],
                capture_output=True, text=True, timeout=8)
            if result.stdout.strip():
                ok("Radio status (RSSI) available")
                for line in result.stdout.split('\n'):
                    if 'rssi' in line.lower() or 'noise' in line.lower():
                        info(f"  {line.strip()}")
        except subprocess.TimeoutExpired:
            pass

    return True


# ═══════════════════════════════════════════════════════════════
# Live velocity monitor (ENU→NED verification)
# ═══════════════════════════════════════════════════════════════

def velocity_monitor():
    """Live display of velocity_local for ENU→NED axis verification.

    Instructions:
      1. Make sure MAVROS2 is running
      2. Place drone on a table, power it with battery (need IMU active)
      3. Watch this display while pushing/tilting the drone:

         Push NORTH → vn (=linear.y) should go POSITIVE
         Push EAST  → ve (=linear.x) should go POSITIVE
         Push DOWN  → vd (=-linear.z) should go POSITIVE

      4. If vn responds to East push and ve to North push → axes are swapped
         → you need to swap x/y in the code

      5. Press Ctrl+C when done
    """
    header("LIVE VELOCITY MONITOR (ENU→NED Verification)")
    print(f"""
  {BOLD}Instructions:{RESET}
    Push/tilt the drone in a known direction and watch which value changes.

    Expected mapping (MAVROS2 ENU → our NED):
      {GREEN}Push NORTH{RESET} → vn (from linear.y) goes {GREEN}POSITIVE{RESET}
      {GREEN}Push EAST{RESET}  → ve (from linear.x) goes {GREEN}POSITIVE{RESET}
      {GREEN}Lift UP{RESET}    → vd (from -linear.z) goes {GREEN}NEGATIVE{RESET} (up = negative down)

    If the wrong axis responds, the ENU→NED mapping needs adjustment.

    Press {BOLD}Ctrl+C{RESET} to stop.
""")

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
    except ImportError:
        fail("rclpy not available — is ROS2 sourced?")
        info("Falling back to ros2 topic echo...")
        os.system('ros2 topic echo /mavros/local_position/velocity_local')
        return

    rclpy.init()

    class VelMonitor(Node):
        def __init__(self):
            super().__init__('preflight_vel_monitor')
            self.sub = self.create_subscription(
                TwistStamped,
                '/mavros/local_position/velocity_local',
                self.on_vel, 10)
            self.count = 0
            self.last_print = 0

        def on_vel(self, msg):
            now = time.time()
            if now - self.last_print < 0.2:  # 5 Hz display
                return
            self.last_print = now

            # Raw ENU from MAVROS2
            enu_x = msg.twist.linear.x  # East
            enu_y = msg.twist.linear.y  # North
            enu_z = msg.twist.linear.z  # Up

            # Our NED conversion
            vn = enu_y       # North = ENU.y
            ve = enu_x       # East  = ENU.x
            vd = -enu_z      # Down  = -ENU.z

            speed = math.sqrt(vn*vn + ve*ve)

            # Color code: highlight any axis > 0.3 m/s
            def col(v, label):
                if abs(v) > 0.3:
                    return f"{GREEN}{BOLD}{label}={v:+6.2f}{RESET}"
                return f"{label}={v:+6.2f}"

            raw = (f"  RAW ENU:  x(E)={enu_x:+6.2f}  y(N)={enu_y:+6.2f}  "
                   f"z(U)={enu_z:+6.2f}")
            ned = (f"  OUR NED:  {col(vn,'vn')}  {col(ve,'ve')}  "
                   f"{col(vd,'vd')}  |  speed={speed:.2f} m/s")

            # Overwrite lines
            print(f"\r{raw}          ")
            print(f"\r{ned}          ", end='')
            sys.stdout.write(f"\033[1A")  # cursor up one line

    node = VelMonitor()
    try:
        print()  # blank line for overwrite area
        print()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n\n  {CYAN}Velocity monitor stopped.{RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ═══════════════════════════════════════════════════════════════
# Check 8: ROS2 topic Hz (are messages flowing at expected rate?)
# ═══════════════════════════════════════════════════════════════

def check_topic_rates():
    header("CHECK 8: Topic Publish Rates (5-second sample)")
    topics_to_check = [
        ('/mavros/global_position/global', 10.0, 'GPS position'),
        ('/mavros/local_position/velocity_local', 10.0, 'Velocity'),
        ('/mavros/state', 1.0, 'FC state'),
        ('/mavros/battery', 1.0, 'Battery'),
    ]

    for topic, expected_hz, desc in topics_to_check:
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'hz', topic],
                capture_output=True, text=True, timeout=7)
            output = result.stdout + result.stderr
            # Parse "average rate: X.XX"
            for line in output.split('\n'):
                if 'average rate:' in line:
                    rate = float(line.split(':')[1].strip())
                    if rate >= expected_hz * 0.5:
                        ok(f"{desc}: {rate:.1f} Hz (expected ~{expected_hz})")
                    else:
                        warn(f"{desc}: {rate:.1f} Hz (expected ~{expected_hz} — low!)")
                    break
            else:
                warn(f"{desc}: no rate measured (topic may be inactive)")
        except subprocess.TimeoutExpired:
            warn(f"{desc}: timed out (topic may not be publishing)")


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='Orin Swarm Pre-Flight Check')
    parser.add_argument('--velocity-only', action='store_true',
                        help='Only run the live velocity ENU→NED monitor')
    parser.add_argument('--radio-device', default='/dev/ttyUSB1',
                        help='RFD900x serial device (default: /dev/ttyUSB1)')
    parser.add_argument('--radio-baud', type=int, default=115200,
                        help='RFD900x baud rate (default: 115200)')
    args = parser.parse_args()

    if args.velocity_only:
        velocity_monitor()
        return

    print(f"\n{BOLD}╔══════════════════════════════════════════════════════════╗")
    print(f"║        ORIN SWARM — PRE-FLIGHT CHECK                     ║")
    print(f"╚══════════════════════════════════════════════════════════╝{RESET}")
    print(f"  Run this AFTER 'ros2 launch ...' is started (full stack).")
    print(f"  Checks: serial, config, FC, GPS, battery, EKF, mode,")
    print(f"          RC link, swarm peer, topic rates.\n")

    results = {}

    # Non-ROS checks (always run)
    results['serial'] = check_serial_ports()
    results['hierarchy'] = check_hierarchy()
    results['rfd900x'] = check_rfd900x(args.radio_device, args.radio_baud)

    # ROS2 checks (need MAVROS2 running)
    results['ros2'] = check_ros2_topics()

    # Extended checks
    results['ekf'] = check_ekf_status()
    results['mode_switch'] = check_mode_change()
    results['rc_link'] = check_rc_link()
    results['swarm_peer'] = check_swarm_peer()

    # Topic rate check
    check_topic_rates()

    # ── Summary ───────────────────────────────────────────────
    header("SUMMARY")
    all_ok = all(results.values())
    for name, passed in results.items():
        status = f"{GREEN}PASS{RESET}" if passed else f"{RED}FAIL{RESET}"
        print(f"  {status}  {name}")

    if all_ok:
        print(f"\n  {GREEN}{BOLD}All checks passed!{RESET}")
        print(f"  Ready to proceed to velocity verification.\n")
    else:
        print(f"\n  {YELLOW}Some checks failed — fix before flight.{RESET}\n")

    # Offer velocity monitor
    print(f"  Next steps:")
    print(f"    1. Verify ENU→NED mapping:")
    print(f"       python3 preflight_check.py --velocity-only")
    print(f"    2. If all green, open flight console from laptop:")
    print(f"       python3 field_startup.py cmd\n")

    # ── Final interactive mode change test ────────────────────
    interactive_mode_test()


def interactive_mode_test():
    """Interactive GUIDED mode test at the very end.

    Prints every step clearly so the operator can see exactly what happened.
    Waits for YES before exiting.
    """
    header("FINAL CHECK: Interactive GUIDED Mode Test")
    print(f"""
  {BOLD}This test will:{RESET}
    1. Read current FC mode
    2. Switch to GUIDED (needed for autonomous flight)
    3. Verify GUIDED was accepted
    4. Switch back to original mode

  {YELLOW}Make sure props are OFF for this test!{RESET}
""")

    try:
        resp = input(f"  Run interactive mode test? (y/n): ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        resp = 'n'
    if resp != 'y':
        info("Skipped. Preflight complete.\n")
        return

    # ── Step 1: Read current mode ──
    print(f"\n  {BOLD}[Step 1/4] Reading current FC mode...{RESET}")
    original_mode = None
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/state', '--once'],
            capture_output=True, text=True, timeout=10)
        for line in result.stdout.split('\n'):
            if 'mode:' in line:
                original_mode = line.split(':')[1].strip().strip('"').strip("'")
            if 'armed:' in line:
                armed_str = line.split(':')[1].strip()
                print(f"             Armed:  {armed_str}")
        if original_mode:
            print(f"             Mode:   {GREEN}{BOLD}{original_mode}{RESET}")
        else:
            fail("Could not read FC mode.")
            return
    except subprocess.TimeoutExpired:
        fail("Timed out reading /mavros/state")
        return

    # ── Step 2: Switch to GUIDED ──
    print(f"\n  {BOLD}[Step 2/4] Switching to GUIDED...{RESET}")
    try:
        result = subprocess.run(
            ['ros2', 'service', 'call', '/mavros/set_mode',
             'mavros_msgs/srv/SetMode', '{custom_mode: "GUIDED"}'],
            capture_output=True, text=True, timeout=10)
        output = result.stdout + result.stderr
        if 'true' in output.lower() or 'mode_sent: true' in output.lower():
            print(f"             {GREEN}GUIDED mode command sent successfully{RESET}")
        else:
            print(f"             {RED}GUIDED mode REJECTED by FC{RESET}")
            print(f"             Response: {output.strip()[:200]}")
            info("Common reasons: no GPS lock, EKF not healthy, safety switch not pressed")
            print(f"\n  {YELLOW}Fix the issue and re-run this test.{RESET}")
            _wait_for_yes()
            return
    except subprocess.TimeoutExpired:
        fail("Set mode service call timed out")
        _wait_for_yes()
        return

    time.sleep(1.5)

    # ── Step 3: Verify GUIDED is active ──
    print(f"\n  {BOLD}[Step 3/4] Verifying FC is now in GUIDED...{RESET}")
    guided_confirmed = False
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/mavros/state', '--once'],
            capture_output=True, text=True, timeout=10)
        for line in result.stdout.split('\n'):
            if 'mode:' in line:
                current = line.split(':')[1].strip().strip('"').strip("'")
                if current == "GUIDED":
                    print(f"             {GREEN}{BOLD}CONFIRMED: FC is in GUIDED mode ✓{RESET}")
                    guided_confirmed = True
                else:
                    print(f"             {RED}FC mode is '{current}' — NOT GUIDED{RESET}")
                    print(f"             The FC rejected the mode change.")
                    info("Possible causes:")
                    info("  - No GPS 3D fix yet (go outdoors)")
                    info("  - EKF not converged (wait 30s after GPS lock)")
                    info("  - Pre-arm checks failing (check Mission Planner)")
    except subprocess.TimeoutExpired:
        warn("Timed out verifying mode")

    # ── Step 4: Restore original mode ──
    print(f"\n  {BOLD}[Step 4/4] Restoring original mode: {original_mode}...{RESET}")
    try:
        result = subprocess.run(
            ['ros2', 'service', 'call', '/mavros/set_mode',
             'mavros_msgs/srv/SetMode', f'{{custom_mode: "{original_mode}"}}'],
            capture_output=True, text=True, timeout=10)
        output = result.stdout + result.stderr
        if 'true' in output.lower():
            print(f"             {GREEN}Restored to {original_mode} ✓{RESET}")
        else:
            print(f"             {YELLOW}Could not restore — set {original_mode} via RC switch{RESET}")
    except subprocess.TimeoutExpired:
        warn(f"Timed out — manually switch back to {original_mode} via RC")

    # ── Result ──
    print()
    print(f"  {'═' * 50}")
    if guided_confirmed:
        print(f"  {GREEN}{BOLD}  MODE TEST PASSED — FC accepts GUIDED mode ✓{RESET}")
        print(f"  {GREEN}{BOLD}  This drone is ready for autonomous flight.{RESET}")
    else:
        print(f"  {RED}{BOLD}  MODE TEST FAILED — FC did NOT enter GUIDED{RESET}")
        print(f"  {RED}{BOLD}  DO NOT attempt autonomous flight until this passes.{RESET}")
    print(f"  {'═' * 50}")

    _wait_for_yes()


def _wait_for_yes():
    """Block until user types YES."""
    print()
    while True:
        try:
            resp = input(f"  {BOLD}Type YES to acknowledge and exit: {RESET}").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if resp.upper() == "YES":
            print(f"\n  {CYAN}Preflight complete. Good flight!{RESET}\n")
            break
        else:
            print(f"  Type YES (all caps) to continue...")


if __name__ == '__main__':
    main()
