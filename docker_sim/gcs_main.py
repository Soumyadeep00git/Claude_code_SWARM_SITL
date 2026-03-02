"""GCS (Ground Control Station) — N-drone swarm support.

Loads hierarchy.yaml to discover all drones, then:
  1. Creates one TelemReceiver per drone (listening on gcs_telem_port(drone_id))
  2. Serves a Flask + SocketIO web UI with dynamic per-drone panels
  3. Forwards commands to drones via UDP (generic drone_cmd event)

Backward-compatible: also supports legacy per-name events (leader_takeoff, etc.)
for existing integration tests.
"""

import logging
import math
import os
import signal
import struct
import socket
import threading
import time

from flask import Flask, render_template
from flask_socketio import SocketIO

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [GCS] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("gcs")

# ── Load hierarchy ──────────────────────────────────────────────
import sys
_PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PROJECT_DIR)

from swarm_lib.hierarchy import SwarmTopology, DroneRole
from swarm_lib.swarm_config import (
    container_name, gcs_telem_port, gcs_cmd_port,
)

HIERARCHY_PATH = os.environ.get("HIERARCHY_PATH", "/app/hierarchy.yaml")
WEB_PORT = int(os.environ.get("WEB_PORT", "5000"))
GEOFENCE_RADIUS_M = float(os.environ.get("GEOFENCE_RADIUS_M", "200.0"))
SAFETY_DIST_M = float(os.environ.get("SAFETY_DIST_M", "1.0"))
CATCHUP_DIST_M = float(os.environ.get("CATCHUP_DIST_M", "8.0"))

METERS_PER_DEG_LAT = 111320.0

# ── Wire format (must match mavlink_bridge.py) ──────────────────
# New format: drone_id (int32) + lat, lon, alt, vx, vy, vz, heading, timestamp, guidance_mode
_TELEM_FMT = "!i9d"
_TELEM_SIZE = struct.calcsize(_TELEM_FMT)

# Command codes (must match mavlink_bridge.py)
CMD_RTL = 1
CMD_LAND = 2
CMD_KILL = 3
CMD_FOLLOW = 4
CMD_HOVER = 5
CMD_TAKEOFF = 6
CMD_WASD = 10
CMD_ALTITUDE = 13

_CMD_SIMPLE_FMT = "!B"
_CMD_WASD_FMT = "!Bff"
_CMD_ALT_FMT = "!Bf"

_CMD_NAME_MAP = {
    "TAKEOFF": CMD_TAKEOFF, "RTL": CMD_RTL, "LAND": CMD_LAND,
    "KILL": CMD_KILL, "FOLLOW": CMD_FOLLOW, "HOVER": CMD_HOVER,
}


# ═══════════════════════════════════════════════════════════════
# Telemetry Receiver (updated for new wire format)
# ═══════════════════════════════════════════════════════════════

class TelemReceiver:
    """Listens for drone telemetry on a UDP port."""

    _GUIDANCE_MODE_NAMES = {0: 'NONE', 1: 'TRACKING', 2: 'CATCHUP',
                            3: 'EVASION', 4: 'FAILSAFE'}

    def __init__(self, port: int, label: str):
        self.port = port
        self.label = label
        self._state = None
        self._lock = threading.Lock()
        self._running = False
        self._pkt_count = 0
        self._last_count = 0
        self._last_rate_time = 0.0
        self._rate = 0.0
        self._stats_lock = threading.Lock()

    def start(self):
        self._running = True
        threading.Thread(
            target=self._listen, daemon=True, name=f"telem-{self.label}"
        ).start()

    def stop(self):
        self._running = False

    def get_state(self):
        with self._lock:
            return self._state

    def get_stats(self):
        now = time.time()
        with self._stats_lock:
            dt = max(now - self._last_rate_time, 0.1)
            if dt >= 1.0:
                self._rate = (self._pkt_count - self._last_count) / dt
                self._last_count = self._pkt_count
                self._last_rate_time = now
            return {'count': self._pkt_count, 'rate': round(self._rate, 1)}

    def _listen(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(1.0)
        log.info("Listening for %s telemetry on UDP:%d", self.label, self.port)

        while self._running:
            try:
                data, _ = sock.recvfrom(4096)
                if len(data) == _TELEM_SIZE:
                    vals = struct.unpack(_TELEM_FMT, data)
                    # vals[0] = drone_id (int32), vals[1..9] = position data
                    state = {
                        'drone_id': vals[0],
                        'lat': vals[1],
                        'lon': vals[2],
                        'alt': vals[3],
                        'vx': vals[4],
                        'vy': vals[5],
                        'vz': vals[6],
                        'heading': vals[7],
                        'timestamp': vals[8],
                        'guidance_mode': self._GUIDANCE_MODE_NAMES.get(int(vals[9]), 'NONE'),
                    }
                    if state['lat'] != 0.0 or state['lon'] != 0.0:
                        with self._lock:
                            self._state = state
                        with self._stats_lock:
                            self._pkt_count += 1
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.5)
        sock.close()


# ═══════════════════════════════════════════════════════════════
# Command Sender
# ═══════════════════════════════════════════════════════════════

class CommandSender:
    """Sends commands to drones via UDP."""

    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_counts = {}  # drone_id -> count

    def send_simple(self, host: str, port: int, cmd_code: int, drone_id: int = 0):
        data = struct.pack(_CMD_SIMPLE_FMT, cmd_code)
        try:
            self._sock.sendto(data, (host, port))
            self.cmd_counts[drone_id] = self.cmd_counts.get(drone_id, 0) + 1
        except (OSError, socket.gaierror) as e:
            log.error("Failed to send to %s:%d: %s", host, port, e)

    def send_wasd(self, host: str, port: int, vn: float, ve: float, drone_id: int = 0):
        data = struct.pack(_CMD_WASD_FMT, CMD_WASD, vn, ve)
        try:
            self._sock.sendto(data, (host, port))
            self.cmd_counts[drone_id] = self.cmd_counts.get(drone_id, 0) + 1
        except (OSError, socket.gaierror) as e:
            log.error("Failed to send to %s:%d: %s", host, port, e)

    def send_altitude(self, host: str, port: int, vd: float, drone_id: int = 0):
        data = struct.pack(_CMD_ALT_FMT, CMD_ALTITUDE, vd)
        try:
            self._sock.sendto(data, (host, port))
            self.cmd_counts[drone_id] = self.cmd_counts.get(drone_id, 0) + 1
        except (OSError, socket.gaierror) as e:
            log.error("Failed to send to %s:%d: %s", host, port, e)


# ═══════════════════════════════════════════════════════════════
# Swarm GCS (dynamic N-drone)
# ═══════════════════════════════════════════════════════════════

class SwarmGCS:
    """Web GCS for N-drone swarm — topology-driven."""

    def __init__(self, topology: SwarmTopology):
        self.topology = topology
        self._running = True

        # One TelemReceiver per drone
        self.telem_receivers = {}
        for did in topology.all_drone_ids():
            node = topology.get_node(did)
            port = gcs_telem_port(did)
            label = f"drone-{did}-{node.role.value}"
            self.telem_receivers[did] = TelemReceiver(port, label)

        # Command sender
        self.cmd_sender = CommandSender()

        # Tracked drone modes
        self.drone_modes = {did: "IDLE" for did in topology.all_drone_ids()}

        # Flask + SocketIO
        template_dir = os.path.join(os.path.dirname(__file__), "templates")
        self.app = Flask(__name__, template_folder=template_dir)
        self.app.config["SECRET_KEY"] = "swarm-gcs"
        self.sio = SocketIO(self.app, async_mode="threading", cors_allowed_origins="*")

        self._register_routes()
        self._register_events()

    def _register_routes(self):
        @self.app.route("/")
        def index():
            drones = []
            for did in self.topology.all_drone_ids():
                node = self.topology.get_node(did)
                drones.append({
                    "id": did,
                    "role": node.role.value,
                    "leader_id": node.leader_id,
                    "follower_ids": list(node.follower_ids),
                })
            return render_template("index.html",
                                   home_lat=self.topology.base_home_lat,
                                   home_lon=self.topology.base_home_lon,
                                   geofence_radius_m=GEOFENCE_RADIUS_M,
                                   safety_dist_m=SAFETY_DIST_M,
                                   catchup_dist_m=CATCHUP_DIST_M,
                                   drones=drones)

    def _send_cmd_to_drone(self, drone_id, cmd_name):
        """Send a simple command to a drone by ID."""
        if drone_id not in self.topology.nodes:
            return
        cmd_code = _CMD_NAME_MAP.get(cmd_name)
        if cmd_code is None:
            return
        host = container_name(drone_id)
        port = gcs_cmd_port(drone_id)
        self.cmd_sender.send_simple(host, port, cmd_code, drone_id)
        self.drone_modes[drone_id] = cmd_name
        self.sio.emit("log", {"msg": f"Drone {drone_id}: {cmd_name}"})
        self.sio.emit("drone_event", {"drone_id": drone_id, "type": "cmd", "msg": cmd_name})
        self._emit_mode_update()

    def _send_wasd_to_drone(self, drone_id, vn, ve):
        """Send WASD velocity to a drone by ID."""
        if drone_id not in self.topology.nodes:
            return
        host = container_name(drone_id)
        port = gcs_cmd_port(drone_id)
        self.cmd_sender.send_wasd(host, port, vn, ve, drone_id)
        self.drone_modes[drone_id] = "WASD" if (vn != 0 or ve != 0) else "HOVER"
        self._emit_mode_update()

    def _send_altitude_to_drone(self, drone_id, vd):
        """Send vertical velocity to a drone by ID."""
        if drone_id not in self.topology.nodes:
            return
        host = container_name(drone_id)
        port = gcs_cmd_port(drone_id)
        self.cmd_sender.send_altitude(host, port, vd, drone_id)

    def _register_events(self):
        sio = self.sio

        @sio.on("connect")
        def on_connect():
            log.info("Browser connected")
            self._emit_mode_update()

        # ── Generic drone commands ──
        @sio.on("drone_cmd")
        def on_drone_cmd(data):
            did = data.get("drone_id")
            cmd_name = data.get("cmd")
            if did is not None:
                self._send_cmd_to_drone(int(did), cmd_name)

        @sio.on("drone_wasd")
        def on_drone_wasd(data):
            did = int(data.get("drone_id", 0))
            vn = float(data.get("vn", 0))
            ve = float(data.get("ve", 0))
            self._send_wasd_to_drone(did, vn, ve)

        @sio.on("drone_altitude")
        def on_drone_altitude(data):
            did = int(data.get("drone_id", 0))
            vd = float(data.get("vd", 0))
            self._send_altitude_to_drone(did, vd)

        @sio.on("all_cmd")
        def on_all_cmd(data):
            cmd_name = data.get("cmd")
            for did in self.topology.all_drone_ids():
                self._send_cmd_to_drone(did, cmd_name)

        # ── Legacy per-name events (backward compat for integration tests) ──
        # Find leader and follower IDs for legacy mapping
        leaders = self.topology.leader_ids()
        followers = self.topology.follower_ids()
        leader_id = leaders[0] if leaders else 1
        follower_id = followers[0] if followers else 2

        @sio.on("leader_takeoff")
        def on_leader_takeoff(data=None):
            self._send_cmd_to_drone(leader_id, "TAKEOFF")

        @sio.on("leader_rtl")
        def on_leader_rtl(data=None):
            self._send_cmd_to_drone(leader_id, "RTL")

        @sio.on("leader_land")
        def on_leader_land(data=None):
            self._send_cmd_to_drone(leader_id, "LAND")

        @sio.on("leader_wasd")
        def on_leader_wasd(data=None):
            vn = float((data or {}).get("vn", 0))
            ve = float((data or {}).get("ve", 0))
            self._send_wasd_to_drone(leader_id, vn, ve)

        @sio.on("follower_takeoff")
        def on_follower_takeoff(data=None):
            self._send_cmd_to_drone(follower_id, "TAKEOFF")

        @sio.on("follower_rtl")
        def on_follower_rtl(data=None):
            self._send_cmd_to_drone(follower_id, "RTL")

        @sio.on("follower_land")
        def on_follower_land(data=None):
            self._send_cmd_to_drone(follower_id, "LAND")

        @sio.on("follower_follow")
        def on_follower_follow(data=None):
            self._send_cmd_to_drone(follower_id, "FOLLOW")

        @sio.on("cmd_rtl")
        def on_rtl(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self._send_cmd_to_drone(leader_id, "RTL")
            if target in ("follower", "both"):
                self._send_cmd_to_drone(follower_id, "RTL")

        @sio.on("cmd_land")
        def on_land(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self._send_cmd_to_drone(leader_id, "LAND")
            if target in ("follower", "both"):
                self._send_cmd_to_drone(follower_id, "LAND")

        @sio.on("cmd_kill")
        def on_kill(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self._send_cmd_to_drone(leader_id, "KILL")
            if target in ("follower", "both"):
                self._send_cmd_to_drone(follower_id, "KILL")

    def _emit_mode_update(self):
        """Push mode state to all connected browsers."""
        # New generic format
        modes = {str(did): mode for did, mode in self.drone_modes.items()}

        # Also emit legacy format for backward compat
        leaders = self.topology.leader_ids()
        followers = self.topology.follower_ids()
        leader_mode = self.drone_modes.get(leaders[0], "IDLE") if leaders else "IDLE"
        follower_mode = self.drone_modes.get(followers[0], "IDLE") if followers else "IDLE"

        self.sio.emit("mode_update", {
            "modes": modes,
            "leader_mode": leader_mode,
            "follower_mode": follower_mode,
        })

    def _broadcast_loop(self):
        """Emit telemetry to all connected browsers at 4 Hz."""
        log.info("Broadcast loop started")
        while self._running:
            try:
                self._broadcast_tick()
            except Exception as e:
                log.error("Broadcast error: %s", e)
            time.sleep(0.25)

    def _broadcast_tick(self):
        home_lat = self.topology.base_home_lat
        home_lon = self.topology.base_home_lon

        # Build per-drone state
        drones_data = {}
        for did, receiver in self.telem_receivers.items():
            state = receiver.get_state() or {}
            node = self.topology.get_node(did)
            stats = receiver.get_stats()

            lat = state.get("lat", 0)
            lon = state.get("lon", 0)
            alt = state.get("alt", 0)
            vx = state.get("vx", 0)
            vy = state.get("vy", 0)

            speed = math.sqrt(vx ** 2 + vy ** 2) if vx is not None else 0.0
            home_dist = 0.0
            if lat != 0:
                dn = (lat - home_lat) * METERS_PER_DEG_LAT
                de = (lon - home_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(home_lat))
                home_dist = math.sqrt(dn * dn + de * de)

            drones_data[did] = {
                "id": did,
                "role": node.role.value,
                "lat": lat, "lon": lon, "alt": alt,
                "vn": vx, "ve": vy,
                "heading": state.get("heading", 0),
                "speed": round(speed, 2),
                "home_dist": round(home_dist, 1),
                "guidance_mode": state.get("guidance_mode", "NONE"),
                "mode": self.drone_modes.get(did, "IDLE"),
                "connected": lat != 0,
                "rx_count": stats['count'],
                "rx_rate": stats['rate'],
                "cmd_count": self.cmd_sender.cmd_counts.get(did, 0),
            }

        # Compute pairwise distances for leader-follower pairs
        pairs = []
        for did, node in self.topology.nodes.items():
            if node.role == DroneRole.FOLLOWER and node.leader_id in drones_data:
                f = drones_data[did]
                l = drones_data[node.leader_id]
                if f["lat"] != 0 and l["lat"] != 0:
                    dn = (l["lat"] - f["lat"]) * METERS_PER_DEG_LAT
                    de = ((l["lon"] - f["lon"]) * METERS_PER_DEG_LAT
                          * math.cos(math.radians(f["lat"])))
                    dist = math.sqrt(dn * dn + de * de)
                else:
                    dist = 0.0
                pairs.append({
                    "leader_id": node.leader_id,
                    "follower_id": did,
                    "dist": round(dist, 2),
                })

        # Emit new generic format
        payload = {"drones": drones_data, "pairs": pairs}

        # Also emit legacy fields for backward compat (test_integration.py)
        leaders = self.topology.leader_ids()
        followers = self.topology.follower_ids()
        if leaders and leaders[0] in drones_data:
            payload["leader"] = drones_data[leaders[0]]
        else:
            payload["leader"] = {"lat": 0, "lon": 0, "alt": 0, "speed": 0, "home_dist": 0}
        if followers and followers[0] in drones_data:
            payload["follower"] = drones_data[followers[0]]
        else:
            payload["follower"] = {"lat": 0, "lon": 0, "alt": 0, "speed": 0, "home_dist": 0}

        # Legacy peer_dist (first leader-follower pair)
        payload["peer_dist"] = pairs[0]["dist"] if pairs else 0
        payload["follower_guidance_mode"] = (
            drones_data[followers[0]]["guidance_mode"] if followers and followers[0] in drones_data else "NONE"
        )
        payload["leader_connected"] = payload["leader"].get("connected", False)
        payload["follower_connected"] = payload["follower"].get("connected", False)
        payload["leader_mode"] = self.drone_modes.get(leaders[0], "IDLE") if leaders else "IDLE"
        payload["follower_mode"] = self.drone_modes.get(followers[0], "IDLE") if followers else "IDLE"

        self.sio.emit("state_update", payload)

    def run(self):
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, '_running', False))
        signal.signal(signal.SIGTERM, lambda s, f: setattr(self, '_running', False))

        for receiver in self.telem_receivers.values():
            receiver.start()

        threading.Thread(target=self._broadcast_loop, daemon=True).start()

        all_ids = self.topology.all_drone_ids()
        log.info("=" * 50)
        log.info("Swarm GCS — %d drones — http://0.0.0.0:%d", len(all_ids), WEB_PORT)
        for did in all_ids:
            node = self.topology.get_node(did)
            port = gcs_telem_port(did)
            log.info("  drone-%d (%s) telem<-:%d cmd->%s:%d",
                     did, node.role.value, port,
                     container_name(did), gcs_cmd_port(did))
        log.info("=" * 50)

        try:
            self.sio.run(self.app, host="0.0.0.0", port=WEB_PORT,
                         allow_unsafe_werkzeug=True)
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            for receiver in self.telem_receivers.values():
                receiver.stop()
            log.info("GCS shutdown complete")


if __name__ == "__main__":
    if os.path.exists(HIERARCHY_PATH):
        topo = SwarmTopology.from_yaml(HIERARCHY_PATH)
        errors = topo.validate()
        if errors:
            log.error("Hierarchy validation failed: %s", errors)
            sys.exit(1)
    else:
        log.error("Hierarchy file not found: %s", HIERARCHY_PATH)
        sys.exit(1)

    gcs = SwarmGCS(topo)
    gcs.run()
