#!/usr/bin/env python3
"""
field_startup.py — One-stop field operations script for Orin swarm.

Run on your LAPTOP. Handles:
  1. Discover Jetson IPs on your hotspot
  2. Update params.yaml on both Jetsons (gcs_host = your laptop IP)
  3. Launch ROS2 nodes on both Jetsons via SSH
  4. Send flight commands (takeoff, follow, hover, land, etc.)

Prerequisites:
  - Laptop mobile hotspot ON
  - Both Jetsons connected to your hotspot WiFi
  - SSH keys set up (ssh-copy-id) OR you'll be prompted for passwords
  - install.sh already run on both Jetsons (ROS2 workspace built)

Usage:
    python3 field_startup.py                     # Full interactive startup
    python3 field_startup.py discover             # Just find Jetson IPs
    python3 field_startup.py launch               # Just launch ROS2 on both
    python3 field_startup.py cmd                   # Just open command console

Network topology:
    Laptop (hotspot gateway)  ←WiFi→  Leader Jetson  ←RFD900x→  Follower Jetson
                              ←WiFi→  Follower Jetson
    (Drone-to-drone comms use RFD900x radio, NOT WiFi)
"""

import argparse
import json
import os
import platform
import re
import socket
import struct
import subprocess
import sys
import time


# ═══════════════════════════════════════════════════════════════
# CONFIGURATION — Edit these to match your setup
# ═══════════════════════════════════════════════════════════════

# Jetson SSH credentials
LEADER_USER   = "iris1"
FOLLOWER_USER = "gaurav"

# Static IPs (if you set them on the Jetsons — recommended)
# Leave as None to auto-discover via ARP scan
LEADER_STATIC_IP   = None   # e.g., "192.168.137.10"
FOLLOWER_STATIC_IP = None   # e.g., "192.168.137.20"

# ROS2 launch commands (run on the Jetsons)
LEADER_LAUNCH   = "source /opt/ros/humble/setup.bash && source ~/swarm_ws/install/setup.bash && ros2 launch leader_pkg leader_bringup.launch.py"
FOLLOWER_LAUNCH = "source /opt/ros/humble/setup.bash && source ~/swarm_ws/install/setup.bash && ros2 launch follower_pkg follower_bringup.launch.py"

# GCS command ports (14580 + drone_id)
LEADER_CMD_PORT   = 14581
FOLLOWER_CMD_PORT = 14582

# Command codes (must match SwarmCommand.msg)
COMMANDS = {
    "rtl": 1, "land": 2, "kill": 3,
    "follow": 4, "hover": 5, "takeoff": 6,
}
COMMAND_NAMES = {v: k.upper() for k, v in COMMANDS.items()}


# ═══════════════════════════════════════════════════════════════
# STEP 1: Network Discovery
# ═══════════════════════════════════════════════════════════════

def get_laptop_hotspot_ip():
    """Get this laptop's IP on the mobile hotspot interface."""
    try:
        # On Windows, mobile hotspot typically uses 192.168.137.1
        if platform.system() == "Windows":
            result = subprocess.run(
                ["powershell", "-Command",
                 "Get-NetIPAddress -InterfaceAlias 'Wi-Fi*','Local Area Connection*','Ethernet*' "
                 "-AddressFamily IPv4 | Select-Object IPAddress,InterfaceAlias | ConvertTo-Json"],
                capture_output=True, text=True, timeout=10
            )
            # Also try mobile hotspot adapter
            result2 = subprocess.run(
                ["powershell", "-Command",
                 "Get-NetIPAddress -AddressFamily IPv4 | "
                 "Where-Object { $_.IPAddress -like '192.168.137.*' -or $_.IPAddress -like '192.168.*' } | "
                 "Select-Object IPAddress,InterfaceAlias | ConvertTo-Json"],
                capture_output=True, text=True, timeout=10
            )
            ips = []
            for r in [result, result2]:
                if r.returncode == 0 and r.stdout.strip():
                    try:
                        data = json.loads(r.stdout)
                        if isinstance(data, dict):
                            data = [data]
                        for entry in data:
                            ip = entry.get('IPAddress', '')
                            alias = entry.get('InterfaceAlias', '')
                            if ip and not ip.startswith('127.'):
                                ips.append((ip, alias))
                    except json.JSONDecodeError:
                        pass

            # Prefer 192.168.137.1 (Windows mobile hotspot default)
            for ip, alias in ips:
                if ip == "192.168.137.1":
                    return ip
            # Otherwise return first non-loopback
            if ips:
                return ips[0][0]
        else:
            # Linux/Mac
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
    except Exception:
        pass
    return None


def discover_jetsons_arp():
    """Try to find Jetsons via ARP table (they must have connected recently)."""
    found = {}
    try:
        if platform.system() == "Windows":
            result = subprocess.run(["arp", "-a"], capture_output=True, text=True, timeout=10)
        else:
            result = subprocess.run(["arp", "-a"], capture_output=True, text=True, timeout=10)

        if result.returncode == 0:
            # Find all IPs in the hotspot subnet
            for line in result.stdout.splitlines():
                match = re.search(r'(192\.168\.\d+\.\d+)', line)
                if match:
                    ip = match.group(1)
                    if not ip.endswith('.1') and not ip.endswith('.255'):
                        found[ip] = True
    except Exception:
        pass
    return list(found.keys())


def check_ssh(user, ip, timeout=5):
    """Check if SSH to a Jetson works."""
    try:
        result = subprocess.run(
            ["ssh", "-o", "ConnectTimeout=3", "-o", "StrictHostKeyChecking=no",
             "-o", "BatchMode=yes", f"{user}@{ip}", "hostname"],
            capture_output=True, text=True, timeout=timeout
        )
        return result.returncode == 0, result.stdout.strip()
    except Exception:
        return False, ""


def check_ping(ip, timeout=3):
    """Ping an IP to check reachability."""
    try:
        if platform.system() == "Windows":
            result = subprocess.run(
                ["ping", "-n", "1", "-w", str(timeout * 1000), ip],
                capture_output=True, text=True, timeout=timeout + 2
            )
        else:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", str(timeout), ip],
                capture_output=True, text=True, timeout=timeout + 2
            )
        return result.returncode == 0
    except Exception:
        return False


def discover_step(leader_ip=None, follower_ip=None):
    """Discover or verify Jetson IPs."""
    print("\n" + "=" * 56)
    print("  STEP 1: NETWORK DISCOVERY")
    print("=" * 56)

    laptop_ip = get_laptop_hotspot_ip()
    if laptop_ip:
        print(f"\n  Your laptop IP: {laptop_ip}")
    else:
        laptop_ip = input("  Could not detect laptop IP. Enter it manually: ").strip()

    # Try static IPs first
    if leader_ip and follower_ip:
        print(f"\n  Using configured IPs:")
        print(f"    Leader:   {leader_ip}")
        print(f"    Follower: {follower_ip}")
    else:
        print(f"\n  Scanning for Jetsons on network...")
        arp_ips = discover_jetsons_arp()

        if arp_ips:
            print(f"  Found {len(arp_ips)} device(s): {', '.join(arp_ips)}")
            # Try SSH to identify which is which
            for ip in arp_ips:
                if not leader_ip:
                    ok, hostname = check_ssh(LEADER_USER, ip)
                    if ok:
                        leader_ip = ip
                        print(f"    {ip} → Leader  (SSH as {LEADER_USER} OK, hostname={hostname})")
                        continue
                if not follower_ip:
                    ok, hostname = check_ssh(FOLLOWER_USER, ip)
                    if ok:
                        follower_ip = ip
                        print(f"    {ip} → Follower (SSH as {FOLLOWER_USER} OK, hostname={hostname})")
                        continue

        if not leader_ip:
            leader_ip = input(f"  Enter Leader Jetson IP ({LEADER_USER}@?): ").strip()
        if not follower_ip:
            follower_ip = input(f"  Enter Follower Jetson IP ({FOLLOWER_USER}@?): ").strip()

    # Verify connectivity
    print(f"\n  Verifying connectivity...")
    for name, user, ip in [("Leader", LEADER_USER, leader_ip), ("Follower", FOLLOWER_USER, follower_ip)]:
        if check_ping(ip):
            print(f"    {name} ({ip}): PING OK", end="")
            ok, hostname = check_ssh(user, ip)
            if ok:
                print(f" | SSH OK (hostname={hostname})")
            else:
                print(f" | SSH FAILED (run: ssh-copy-id {user}@{ip})")
        else:
            print(f"    {name} ({ip}): NOT REACHABLE")
            print(f"      → Is it connected to your hotspot?")
            print(f"      → Try: ssh {user}@{ip}")

    return laptop_ip, leader_ip, follower_ip


# ═══════════════════════════════════════════════════════════════
# STEP 2: Update GCS host IP on Jetsons
# ═══════════════════════════════════════════════════════════════

def update_gcs_host_on_jetson(user, ip, laptop_ip):
    """SSH into Jetson and update gcs_host in params.yaml to the laptop's IP."""
    ws = "swarm_ws"
    # Determine which params.yaml to edit
    if user == LEADER_USER:
        params_file = f"/home/{user}/{ws}/src/Leader/config/params.yaml"
        pkg = "Leader"
    else:
        params_file = f"/home/{user}/{ws}/src/Follower/config/params.yaml"
        pkg = "Follower"

    cmd = (f"ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no {user}@{ip} "
           f"\"sed -i 's|gcs_host: \\\".*\\\"|gcs_host: \\\"{laptop_ip}\\\"|g' {params_file} && "
           f"echo UPDATED\"")

    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
        if "UPDATED" in result.stdout:
            print(f"    {pkg} params.yaml: gcs_host → {laptop_ip}  ✓")
            return True
        else:
            print(f"    {pkg} params.yaml: FAILED to update")
            print(f"      stderr: {result.stderr.strip()}")
            return False
    except Exception as e:
        print(f"    {pkg} params.yaml: ERROR {e}")
        return False


def update_step(laptop_ip, leader_ip, follower_ip):
    """Update gcs_host on both Jetsons."""
    print("\n" + "=" * 56)
    print("  STEP 2: UPDATE GCS HOST IP ON JETSONS")
    print("=" * 56)
    print(f"\n  Setting gcs_host = {laptop_ip} on both Jetsons...")

    update_gcs_host_on_jetson(LEADER_USER, leader_ip, laptop_ip)
    update_gcs_host_on_jetson(FOLLOWER_USER, follower_ip, laptop_ip)


# ═══════════════════════════════════════════════════════════════
# STEP 3: Update hierarchy.yaml GPS coordinates
# ═══════════════════════════════════════════════════════════════

def update_gps_step(leader_ip, follower_ip):
    """Prompt for field GPS coords and update hierarchy.yaml on both Jetsons."""
    print("\n" + "=" * 56)
    print("  STEP 3: SET FIELD GPS COORDINATES")
    print("=" * 56)

    print("\n  Get your current GPS coordinates from your phone.")
    print("  (Google Maps → long-press → copy coordinates)")
    print()
    lat_str = input("  Enter field latitude  (e.g., 27.123456): ").strip()
    lon_str = input("  Enter field longitude (e.g., 77.654321): ").strip()

    if not lat_str or not lon_str:
        print("  Skipping GPS update (no coordinates entered).")
        return

    try:
        lat = float(lat_str)
        lon = float(lon_str)
    except ValueError:
        print("  Invalid coordinates. Skipping.")
        return

    for user, ip, name in [(LEADER_USER, leader_ip, "Leader"), (FOLLOWER_USER, follower_ip, "Follower")]:
        hier_file = f"/home/{user}/hierarchy.yaml"
        cmd = (f"ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no {user}@{ip} "
               f"\"sed -i 's|lat:.*|lat: {lat}|;s|lon:.*|lon: {lon}|' {hier_file} && echo UPDATED\"")
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
            if "UPDATED" in result.stdout:
                print(f"    {name} hierarchy.yaml: home → {lat}, {lon}  ✓")
            else:
                print(f"    {name} hierarchy.yaml: FAILED")
        except Exception as e:
            print(f"    {name} hierarchy.yaml: ERROR {e}")


# ═══════════════════════════════════════════════════════════════
# STEP 4: Launch ROS2 Nodes
# ═══════════════════════════════════════════════════════════════

def launch_step(leader_ip, follower_ip):
    """Launch ROS2 on both Jetsons via SSH (background, with screen)."""
    print("\n" + "=" * 56)
    print("  STEP 4: LAUNCH ROS2 NODES")
    print("=" * 56)

    print("\n  This will start ROS2 nodes on both Jetsons using 'screen'.")
    print("  To view logs later:  ssh <user>@<ip> 'screen -r ros2'")
    print("  To detach from screen: Ctrl+A, D")
    print()

    for name, user, ip, launch_cmd in [
        ("Leader",   LEADER_USER,   leader_ip,   LEADER_LAUNCH),
        ("Follower", FOLLOWER_USER, follower_ip, FOLLOWER_LAUNCH),
    ]:
        # Kill any existing screen session first
        kill_cmd = f"ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no {user}@{ip} 'screen -S ros2 -X quit 2>/dev/null; sleep 1'"
        subprocess.run(kill_cmd, shell=True, capture_output=True, timeout=15)

        # Launch in a detached screen session
        ssh_cmd = (
            f"ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no {user}@{ip} "
            f"\"screen -dmS ros2 bash -c '{launch_cmd}'\""
        )

        try:
            result = subprocess.run(ssh_cmd, shell=True, capture_output=True, text=True, timeout=15)
            if result.returncode == 0:
                print(f"    {name} ({user}@{ip}): ROS2 launched in screen session  ✓")
                print(f"      View logs: ssh {user}@{ip} 'screen -r ros2'")
            else:
                print(f"    {name}: LAUNCH FAILED")
                print(f"      stderr: {result.stderr.strip()}")
        except Exception as e:
            print(f"    {name}: ERROR {e}")

    print("\n  Waiting 5 seconds for nodes to initialize...")
    time.sleep(5)
    print("  Nodes should be running. Verify with preflight checks.")


# ═══════════════════════════════════════════════════════════════
# STEP 5: Preflight Check (remote)
# ═══════════════════════════════════════════════════════════════

def preflight_step(leader_ip, follower_ip):
    """Run preflight_check.py on both Jetsons."""
    print("\n" + "=" * 56)
    print("  STEP 5: PREFLIGHT CHECKS")
    print("=" * 56)

    for name, user, ip in [("Leader", LEADER_USER, leader_ip), ("Follower", FOLLOWER_USER, follower_ip)]:
        print(f"\n  ── {name} ({user}@{ip}) ──")
        cmd = (
            f"ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no {user}@{ip} "
            f"\"source /opt/ros/humble/setup.bash && source ~/orin_ws/install/setup.bash && "
            f"python3 ~/preflight_check.py 2>&1 | head -60\""
        )
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30)
            if result.stdout:
                for line in result.stdout.splitlines():
                    print(f"    {line}")
            if result.returncode != 0 and result.stderr:
                print(f"    stderr: {result.stderr.strip()[:200]}")
        except subprocess.TimeoutExpired:
            print(f"    Timed out (30s). SSH into {user}@{ip} and run manually.")
        except Exception as e:
            print(f"    ERROR: {e}")


# ═══════════════════════════════════════════════════════════════
# STEP 6: Flight Commands
# ═══════════════════════════════════════════════════════════════

def send_command(target_name, cmd_name, leader_ip, follower_ip, confirm=True):
    """Send a UDP command to target drone(s)."""
    cmd_code = COMMANDS.get(cmd_name.lower())
    if cmd_code is None:
        print(f"  [ERROR] Unknown command: {cmd_name}")
        return

    targets = {
        "leader":   [(leader_ip,   LEADER_CMD_PORT)],
        "follower": [(follower_ip, FOLLOWER_CMD_PORT)],
        "both":     [(leader_ip,   LEADER_CMD_PORT),
                     (follower_ip, FOLLOWER_CMD_PORT)],
    }
    endpoints = targets.get(target_name.lower(), [])

    if confirm and cmd_name.lower() == "kill":
        resp = input("  *** KILL cuts ALL motors mid-air! Type YES to confirm: ")
        if resp.strip() != "YES":
            print("  Cancelled.")
            return

    packet = struct.pack('B', cmd_code)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    for ip, port in endpoints:
        try:
            sock.sendto(packet, (ip, port))
            drone = "Leader" if port == LEADER_CMD_PORT else "Follower"
            print(f"  [{drone}] {COMMAND_NAMES[cmd_code]} (cmd={cmd_code}) → {ip}:{port}")
        except OSError as e:
            print(f"  [ERROR] {ip}:{port}: {e}")

    sock.close()


def command_console(leader_ip, follower_ip):
    """Interactive flight command console."""
    print("\n" + "=" * 56)
    print("       ORIN SWARM — FLIGHT COMMAND CONSOLE")
    print("=" * 56)
    print(f"\n  Leader:   {leader_ip}:{LEADER_CMD_PORT}")
    print(f"  Follower: {follower_ip}:{FOLLOWER_CMD_PORT}")
    print()
    print("  ┌─────────────────────────────────────────┐")
    print("  │  FLIGHT SEQUENCE:                        │")
    print("  │   1. takeoff    (auto-sends to follower) │")
    print("  │   2. wait ~10s until stable hover        │")
    print("  │   3. follow     (start formation)        │")
    print("  │   4. fly leader with RC transmitter      │")
    print("  │   5. hover      (stop following)         │")
    print("  │   6. land       (land both)              │")
    print("  │                                          │")
    print("  │  EMERGENCY: kill, rtl                    │")
    print("  └─────────────────────────────────────────┘")
    print()

    while True:
        print("─" * 56)
        print("  Commands: takeoff | follow | hover | land | rtl | kill")
        print("  Targets:  leader (L) | follower (F) | both (B)")
        print("  Type 'q' to quit")
        print()

        try:
            raw = input("  > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n  Bye.")
            break

        if raw in ('q', 'quit', 'exit'):
            break
        if not raw:
            continue

        parts = raw.split()
        cmd_name = parts[0]

        if cmd_name not in COMMANDS:
            print(f"  Unknown command: {cmd_name}")
            continue

        if len(parts) >= 2:
            target_raw = parts[1]
        else:
            if cmd_name in ("kill", "land", "rtl"):
                target_raw = "both"
            else:
                target_raw = "follower"

        target_map = {
            "l": "leader", "leader": "leader",
            "f": "follower", "follower": "follower",
            "b": "both", "both": "both",
        }
        target = target_map.get(target_raw)
        if not target:
            print(f"  Unknown target: {target_raw}")
            continue

        print()
        send_command(target, cmd_name, leader_ip, follower_ip)
        print()


# ═══════════════════════════════════════════════════════════════
# STOP: Kill ROS2 on Jetsons
# ═══════════════════════════════════════════════════════════════

def stop_step(leader_ip, follower_ip):
    """Stop ROS2 screen sessions on both Jetsons."""
    print("\n  Stopping ROS2 on both Jetsons...")
    for name, user, ip in [("Leader", LEADER_USER, leader_ip), ("Follower", FOLLOWER_USER, follower_ip)]:
        cmd = f"ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no {user}@{ip} 'screen -S ros2 -X quit 2>/dev/null; pkill -f ros2 2>/dev/null; echo STOPPED'"
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
            if "STOPPED" in result.stdout:
                print(f"    {name}: stopped  ✓")
            else:
                print(f"    {name}: may still be running")
        except Exception as e:
            print(f"    {name}: {e}")


# ═══════════════════════════════════════════════════════════════
# MAIN MENU
# ═══════════════════════════════════════════════════════════════

def full_startup():
    """Full interactive startup sequence."""
    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║         ORIN SWARM — FIELD STARTUP WIZARD           ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Run this on your LAPTOP with hotspot enabled.      ║")
    print("║  Both Jetsons must be connected to your hotspot.    ║")
    print("║  No internet required.                              ║")
    print("╚══════════════════════════════════════════════════════╝")

    # Step 1: Discover
    laptop_ip, leader_ip, follower_ip = discover_step(
        LEADER_STATIC_IP, FOLLOWER_STATIC_IP)

    while True:
        print("\n" + "=" * 56)
        print("  MAIN MENU")
        print("=" * 56)
        print(f"  Laptop:   {laptop_ip}")
        print(f"  Leader:   {LEADER_USER}@{leader_ip}")
        print(f"  Follower: {FOLLOWER_USER}@{follower_ip}")
        print()
        print("  [1] Re-discover IPs")
        print("  [2] Update gcs_host on Jetsons (set laptop IP)")
        print("  [3] Set field GPS coordinates")
        print("  [4] Launch ROS2 nodes")
        print("  [5] Run preflight checks")
        print("  [6] Flight command console")
        print("  [7] Stop ROS2 on both Jetsons")
        print("  [A] Full sequence (2 → 3 → 4 → 5 → 6)")
        print("  [Q] Quit")
        print()

        try:
            choice = input("  Select: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n  Bye.")
            break

        if choice == '1':
            laptop_ip, leader_ip, follower_ip = discover_step(None, None)
        elif choice == '2':
            update_step(laptop_ip, leader_ip, follower_ip)
        elif choice == '3':
            update_gps_step(leader_ip, follower_ip)
        elif choice == '4':
            launch_step(leader_ip, follower_ip)
        elif choice == '5':
            preflight_step(leader_ip, follower_ip)
        elif choice == '6':
            command_console(leader_ip, follower_ip)
        elif choice == '7':
            stop_step(leader_ip, follower_ip)
        elif choice == 'a':
            update_step(laptop_ip, leader_ip, follower_ip)
            update_gps_step(leader_ip, follower_ip)
            launch_step(leader_ip, follower_ip)
            preflight_step(leader_ip, follower_ip)
            input("\n  Press Enter to open flight command console...")
            command_console(leader_ip, follower_ip)
        elif choice in ('q', 'quit', 'exit'):
            print("  Bye.")
            break
        else:
            print("  Invalid choice.")


def main():
    parser = argparse.ArgumentParser(description="Orin Swarm field startup helper")
    parser.add_argument("action", nargs="?", default="full",
                        choices=["full", "discover", "launch", "preflight", "cmd", "stop"],
                        help="Action to perform (default: full interactive)")
    parser.add_argument("--leader-ip", help="Leader Jetson IP")
    parser.add_argument("--follower-ip", help="Follower Jetson IP")

    args = parser.parse_args()

    leader_ip = args.leader_ip or LEADER_STATIC_IP
    follower_ip = args.follower_ip or FOLLOWER_STATIC_IP

    if args.action == "full":
        full_startup()
    elif args.action == "discover":
        discover_step(leader_ip, follower_ip)
    elif args.action == "launch":
        if not leader_ip or not follower_ip:
            _, leader_ip, follower_ip = discover_step(leader_ip, follower_ip)
        launch_step(leader_ip, follower_ip)
    elif args.action == "preflight":
        if not leader_ip or not follower_ip:
            _, leader_ip, follower_ip = discover_step(leader_ip, follower_ip)
        preflight_step(leader_ip, follower_ip)
    elif args.action == "cmd":
        if not leader_ip or not follower_ip:
            _, leader_ip, follower_ip = discover_step(leader_ip, follower_ip)
        command_console(leader_ip, follower_ip)
    elif args.action == "stop":
        if not leader_ip or not follower_ip:
            _, leader_ip, follower_ip = discover_step(leader_ip, follower_ip)
        stop_step(leader_ip, follower_ip)


if __name__ == "__main__":
    main()
