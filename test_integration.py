#!/usr/bin/env python3
"""Automated integration test for N-drone swarm system.

Reads hierarchy.yaml to discover topology, then:
  1. Takeoff all leaders, then all followers
  2. Wait for ALL to be airborne (alt > 8m)
  3. Start pursuit on all followers
  4. Move leader(s) north ~24m, wait for all followers to converge
  5. Move leader(s) east ~24m, wait for all followers to converge
  6. Push leader(s) outside geofence, observe failsafe
  7. Bring leader(s) back, observe recovery
  8. All RTL

Usage:
    python test_integration.py [hierarchy.yaml] [gcs_url]
"""

import os
import sys
import socketio
import time
import threading

# Add project root to path
_PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _PROJECT_DIR)

from swarm_lib.hierarchy import SwarmTopology, DroneRole

# ── Configuration ─────────────────────────────────────────────
HIERARCHY_PATH = sys.argv[1] if len(sys.argv) > 1 else "hierarchy.yaml"
GCS_URL = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:5000"
WASD_SPEED = 3.0       # m/s for normal movement
FAST_SPEED = 5.0       # m/s for geofence breach

# ── Load topology ─────────────────────────────────────────────
topo = SwarmTopology.from_yaml(HIERARCHY_PATH)
ALL_IDS = topo.all_drone_ids()
LEADER_IDS = topo.leader_ids()
FOLLOWER_IDS = topo.follower_ids()

# ── Shared state updated by SocketIO events ───────────────────
# drones: {drone_id: {lat, lon, alt, speed, home_dist, guidance_mode, ...}}
# pairs:  [{leader_id, follower_id, dist}, ...]
# modes:  {drone_id_str: mode_str}
state = {
    "drones": {},
    "pairs": [],
    "modes": {},
}
_lock = threading.Lock()


# ── SocketIO client ──────────────────────────────────────────
sio = socketio.Client(logger=False, engineio_logger=False)

@sio.on("state_update")
def on_state(data):
    with _lock:
        if "drones" in data:
            state["drones"] = data["drones"]
        if "pairs" in data:
            state["pairs"] = data["pairs"]

@sio.on("mode_update")
def on_mode(data):
    with _lock:
        if "modes" in data:
            state["modes"] = data["modes"]

@sio.on("log")
def on_log(data):
    msg = data.get("msg", "")
    log(f"  [GCS] {msg}")


# ── Helpers ───────────────────────────────────────────────────
def log(msg):
    t = time.strftime("%H:%M:%S")
    print(f"[{t}] {msg}", flush=True)

def drone_state(did):
    """Get state dict for a drone by ID."""
    with _lock:
        # Keys in drones dict may be int or str depending on JSON serialization
        return state["drones"].get(did, state["drones"].get(str(did), {}))

def drone_alt(did):
    return drone_state(did).get("alt", 0)

def drone_mode(did):
    with _lock:
        return state["modes"].get(str(did), "IDLE")

def pair_dist(leader_id, follower_id):
    """Get distance between a specific leader-follower pair."""
    with _lock:
        for p in state["pairs"]:
            if p["leader_id"] == leader_id and p["follower_id"] == follower_id:
                return p["dist"]
    return 999.0

def all_pair_dists():
    """Get all pair distances as dict: (leader_id, follower_id) -> dist."""
    with _lock:
        return {
            (p["leader_id"], p["follower_id"]): p["dist"]
            for p in state["pairs"]
        }

def max_pair_dist():
    """Get the maximum pair distance across all leader-follower pairs."""
    dists = all_pair_dists()
    return max(dists.values()) if dists else 999.0

def snap():
    """Return a snapshot string of current swarm state."""
    parts = []
    for did in ALL_IDS:
        ds = drone_state(did)
        role_char = "L" if did in LEADER_IDS else "F"
        parts.append(
            f"D{did}({role_char}): alt={ds.get('alt',0):5.1f}m "
            f"home={ds.get('home_dist',0):4.0f}m "
            f"spd={ds.get('speed',0):4.1f}"
        )

    pair_strs = []
    for lid in LEADER_IDS:
        for fid in FOLLOWER_IDS:
            node = topo.get_node(fid)
            if node.leader_id == lid:
                d = pair_dist(lid, fid)
                guid = drone_state(fid).get("guidance_mode", "?")
                pair_strs.append(f"{lid}->{fid}: {d:5.1f}m/{guid}")

    return " | ".join(parts) + " || " + "  ".join(pair_strs)

def wait(seconds, label=""):
    """Wait while printing status every 2s."""
    start = time.time()
    while time.time() - start < seconds:
        remaining = seconds - (time.time() - start)
        time.sleep(min(2.0, max(0.1, remaining)))
        log(f"  [{label:^16s}] {snap()}")

def wait_for(condition, label, timeout=60):
    """Wait until condition() returns True, logging every 2s."""
    start = time.time()
    while time.time() - start < timeout:
        if condition():
            log(f"  [{label:^16s}] CONDITION MET")
            return True
        time.sleep(2.0)
        log(f"  [{label:^16s}] {snap()}")
    log(f"  [{label:^16s}] TIMEOUT after {timeout}s!")
    return False

def section(title):
    log("")
    log("=" * 70)
    log(f"  {title}")
    log("=" * 70)

# ── Command helpers (use generic events) ──────────────────────
def cmd(drone_id, cmd_name):
    """Send a command to a specific drone."""
    sio.emit("drone_cmd", {"drone_id": drone_id, "cmd": cmd_name})

def wasd(drone_id, vn, ve):
    """Send WASD velocity to a specific drone."""
    sio.emit("drone_wasd", {"drone_id": drone_id, "vn": vn, "ve": ve})

def cmd_all(cmd_name):
    """Send a command to all drones."""
    sio.emit("all_cmd", {"cmd": cmd_name})


# ── Test sequence ─────────────────────────────────────────────
def main():
    log(f"Topology: {len(ALL_IDS)} drones "
        f"({len(LEADER_IDS)} leaders, {len(FOLLOWER_IDS)} followers)")
    for did in ALL_IDS:
        node = topo.get_node(did)
        log(f"  drone-{did}: {node.role.value}"
            + (f"  leader={node.leader_id}" if node.leader_id else "")
            + (f"  followers={list(node.follower_ids)}" if node.follower_ids else ""))

    log(f"Connecting to GCS at {GCS_URL} ...")
    sio.connect(GCS_URL, wait_timeout=10)
    log("Connected!")
    time.sleep(2)  # let initial state arrive

    try:
        # ── 1. Takeoff leaders ────────────────────────────────
        section(f"STEP 1: TAKEOFF LEADERS {LEADER_IDS}")
        for lid in LEADER_IDS:
            cmd(lid, "TAKEOFF")
            time.sleep(0.5)

        time.sleep(1)

        # ── 2. Takeoff followers ──────────────────────────────
        section(f"STEP 2: TAKEOFF FOLLOWERS {FOLLOWER_IDS}")
        for fid in FOLLOWER_IDS:
            cmd(fid, "TAKEOFF")
            time.sleep(0.5)

        # Wait until ALL are airborne (alt > 8m)
        section(f"WAITING FOR ALL {len(ALL_IDS)} DRONES AIRBORNE")
        ok = wait_for(
            lambda: all(drone_alt(did) > 8.0 for did in ALL_IDS),
            "TAKEOFF", timeout=90)
        if not ok:
            log("TAKEOFF FAILED -- aborting test")
            for did in ALL_IDS:
                log(f"  drone-{did} alt: {drone_alt(did):.1f}m")
            return

        wait(3, "STABILIZE")

        # ── 3. Start pursuit on all followers ─────────────────
        section(f"STEP 3: ALL FOLLOWERS PURSUIT")
        for fid in FOLLOWER_IDS:
            cmd(fid, "FOLLOW")
            time.sleep(0.5)

        # Wait for all followers to lock on
        wait(10, "PURSUIT LOCK")

        # ── 4. Move leaders north (waypoint 1) ────────────────
        section("STEP 4: LEADERS NORTH ~24m (waypoint 1)")
        for lid in LEADER_IDS:
            wasd(lid, WASD_SPEED, 0.0)
        wait(8, "MOVING NORTH")
        for lid in LEADER_IDS:
            wasd(lid, 0.0, 0.0)
        log(">> Leaders stopped at WP1")

        # Wait for ALL follower pairs to converge (max pair_dist < 15m)
        wait_for(lambda: max_pair_dist() < 15.0, "WP1 CONVERGE", timeout=30)
        wait(5, "WP1 HOLD")

        # ── 5. Move leaders east (waypoint 2) ─────────────────
        section("STEP 5: LEADERS EAST ~24m (waypoint 2)")
        for lid in LEADER_IDS:
            wasd(lid, 0.0, WASD_SPEED)
        wait(8, "MOVING EAST")
        for lid in LEADER_IDS:
            wasd(lid, 0.0, 0.0)
        log(">> Leaders stopped at WP2")

        wait_for(lambda: max_pair_dist() < 15.0, "WP2 CONVERGE", timeout=30)
        wait(5, "WP2 HOLD")

        # ── 6. Push leaders outside geofence ──────────────────
        section("STEP 6: LEADERS OUTSIDE GEOFENCE")
        log(f">> Moving north at {FAST_SPEED:.0f} m/s")
        for lid in LEADER_IDS:
            wasd(lid, FAST_SPEED, 0.0)
        wait(12, "EXITING FENCE")
        for lid in LEADER_IDS:
            wasd(lid, 0.0, 0.0)
        log(">> Leaders stopped outside fence")
        wait(10, "OUTSIDE FENCE")

        # ── 7. Bring leaders back inside ──────────────────────
        section("STEP 7: LEADERS RETURN INSIDE GEOFENCE")
        for lid in LEADER_IDS:
            wasd(lid, -FAST_SPEED, 0.0)
        wait(12, "RETURNING")
        for lid in LEADER_IDS:
            wasd(lid, 0.0, 0.0)
        log(">> Leaders stopped inside fence")
        wait(15, "RECOVERY")

        # ── 8. All RTL ───────────────────────────────────────
        section("STEP 8: ALL RTL")
        cmd_all("RTL")
        wait(30, "RTL")

        section("TEST COMPLETE")
        log("Final state: %s" % snap())

    except KeyboardInterrupt:
        log("Interrupted -- sending emergency RTL")
        cmd_all("RTL")
    finally:
        sio.disconnect()
        log("Disconnected from GCS")


if __name__ == "__main__":
    main()
