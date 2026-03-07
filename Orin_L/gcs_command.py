#!/usr/bin/env python3
"""
GCS Command Sender — send commands to Leader and Follower drones.

Usage (interactive menu):
    python3 gcs_command.py

Usage (direct command):
    python3 gcs_command.py --target follower --cmd follow
    python3 gcs_command.py --target leader --cmd rtl
    python3 gcs_command.py --target both --cmd land

IP configuration (pick one):
    1. Edit LEADER_IP / FOLLOWER_IP below
    2. Set env vars:  LEADER_IP=x.x.x.x  FOLLOWER_IP=y.y.y.y
    3. Use flags:     --leader-ip x.x.x.x --follower-ip y.y.y.y

Commands:
    rtl      (1)  — Return to launch
    land     (2)  — Land in place
    kill     (3)  — Emergency motor kill  *** DANGEROUS ***
    follow   (4)  — Start formation follow (follower only)
    hover    (5)  — Hold position
    takeoff  (6)  — Arm + GUIDED + takeoff

Run from your laptop on the same WiFi/hotspot network as the Jetsons.
No internet required — laptop mobile hotspot works perfectly.
"""

import argparse
import os
import socket
import struct
import sys
import time

# ═══════════════════════════════════════════════════════════════
# EDIT THESE to match your Jetson IPs on your hotspot/WiFi:
# (or use env vars LEADER_IP / FOLLOWER_IP, or --leader-ip / --follower-ip flags)
# ═══════════════════════════════════════════════════════════════
DEFAULT_LEADER_IP   = "172.16.0.159"      # Leader Jetson IP on current network
DEFAULT_FOLLOWER_IP = "172.16.0.199"      # Follower Jetson IP on current network

# GCS command ports: 14580 + drone_id   (auto-computed, don't change)
LEADER_CMD_PORT   = 14581   # drone_id = 1
FOLLOWER_CMD_PORT = 14582   # drone_id = 2

# ── Command codes (must match SwarmCommand.msg) ─────────────
COMMANDS = {
    "rtl":     1,
    "land":    2,
    "kill":    3,
    "follow":  4,
    "hover":   5,
    "takeoff": 6,
}

COMMAND_NAMES = {v: k.upper() for k, v in COMMANDS.items()}


def build_targets(leader_ip: str, follower_ip: str) -> dict:
    return {
        "leader":   [(leader_ip,   LEADER_CMD_PORT)],
        "follower": [(follower_ip, FOLLOWER_CMD_PORT)],
        "both":     [(leader_ip,   LEADER_CMD_PORT),
                     (follower_ip, FOLLOWER_CMD_PORT)],
    }


def resolve_ips(args) -> tuple:
    """Resolve IPs from flags > env vars > defaults."""
    leader_ip = (
        getattr(args, 'leader_ip', None)
        or os.environ.get('LEADER_IP')
        or DEFAULT_LEADER_IP
    )
    follower_ip = (
        getattr(args, 'follower_ip', None)
        or os.environ.get('FOLLOWER_IP')
        or DEFAULT_FOLLOWER_IP
    )
    return leader_ip, follower_ip


def send_command(target_name: str, cmd_name: str, targets: dict, confirm: bool = True):
    """Send a 1-byte command to the target drone(s)."""
    cmd_code = COMMANDS.get(cmd_name.lower())
    if cmd_code is None:
        print(f"[ERROR] Unknown command: {cmd_name}")
        print(f"        Valid: {', '.join(COMMANDS.keys())}")
        return False

    endpoints = targets.get(target_name.lower())
    if endpoints is None:
        print(f"[ERROR] Unknown target: {target_name}")
        print(f"        Valid: leader, follower, both")
        return False

    # Safety confirmation for dangerous commands
    if confirm and cmd_name.lower() == "kill":
        resp = input(f"  *** KILL will cut ALL motors mid-air! Type YES to confirm: ")
        if resp.strip() != "YES":
            print("  Cancelled.")
            return False

    packet = struct.pack('B', cmd_code)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    for ip, port in endpoints:
        try:
            sock.sendto(packet, (ip, port))
            drone = "Leader" if port == LEADER_CMD_PORT else "Follower"
            print(f"  [{drone}] {COMMAND_NAMES[cmd_code]} (cmd={cmd_code}) → {ip}:{port}")
        except OSError as e:
            print(f"  [ERROR] Failed to send to {ip}:{port}: {e}")

    sock.close()
    return True


def interactive_menu(leader_ip: str, follower_ip: str, targets: dict):
    """Interactive command menu for field use."""
    print()
    print("=" * 56)
    print("       ORIN SWARM — GCS COMMAND CONSOLE")
    print("=" * 56)
    print()
    print(f"  Leader:   {leader_ip}:{LEADER_CMD_PORT}")
    print(f"  Follower: {follower_ip}:{FOLLOWER_CMD_PORT}")
    print()
    print("  ┌─────────────────────────────────────────┐")
    print("  │  TYPICAL FLIGHT SEQUENCE:                │")
    print("  │   1. takeoff (follower)                  │")
    print("  │   2. wait ~10s until stable hover        │")
    print("  │   3. follow  (follower)                  │")
    print("  │   4. fly leader with RC                  │")
    print("  │   5. hover   (follower) to stop          │")
    print("  │   6. land    (follower) to land          │")
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
            print("  Bye.")
            break

        if not raw:
            continue

        # Parse: "follow follower" or "follow f" or just "follow"
        parts = raw.split()
        cmd_name = parts[0]

        if cmd_name not in COMMANDS:
            print(f"  Unknown command: {cmd_name}")
            continue

        # Default target
        if len(parts) >= 2:
            target_raw = parts[1]
        else:
            # Default: follower for flight cmds, both for emergency
            if cmd_name in ("kill", "land", "rtl"):
                target_raw = "both"
            else:
                target_raw = "follower"

        # Shorthand expansion
        target_map = {
            "l": "leader", "leader": "leader",
            "f": "follower", "follower": "follower",
            "b": "both", "both": "both",
        }
        target = target_map.get(target_raw)
        if target is None:
            print(f"  Unknown target: {target_raw}")
            continue

        print()
        send_command(target, cmd_name, targets, confirm=True)
        print()


def main():
    parser = argparse.ArgumentParser(
        description="Send commands to Orin swarm drones")
    parser.add_argument("--target", "-t",
                        choices=["leader", "follower", "both"],
                        help="Target drone(s)")
    parser.add_argument("--cmd", "-c",
                        choices=list(COMMANDS.keys()),
                        help="Command to send")
    parser.add_argument("--no-confirm", action="store_true",
                        help="Skip confirmation for dangerous commands")
    parser.add_argument("--leader-ip",
                        help="Leader Jetson IP (default: env LEADER_IP or config)")
    parser.add_argument("--follower-ip",
                        help="Follower Jetson IP (default: env FOLLOWER_IP or config)")

    args = parser.parse_args()
    leader_ip, follower_ip = resolve_ips(args)
    targets = build_targets(leader_ip, follower_ip)

    if args.cmd and args.target:
        # Direct mode: send and exit
        send_command(args.target, args.cmd, targets, confirm=not args.no_confirm)
    elif args.cmd or args.target:
        print("Both --target and --cmd are required for direct mode.")
        print("Or run without arguments for interactive menu.")
        sys.exit(1)
    else:
        # Interactive mode
        interactive_menu(leader_ip, follower_ip, targets)


if __name__ == "__main__":
    main()
