"""GCS (Ground Control Station) container entry point.

Does NOT run SITL or guidance -- only:
  1. Receives telemetry from leader and follower via UDP
  2. Serves a Flask + SocketIO web UI
  3. Forwards commands to drones via UDP:
     - Leader: WASD velocity, waypoint, speed, hover, RTL/LAND/KILL
     - Follower: FOLLOW, HOVER, RTL/LAND/KILL (RC mock)
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

# ── Configuration (from env) ──────────────────────────────────
LEADER_HOST = os.environ.get("LEADER_HOST", "leader-drone")
FOLLOWER_HOST = os.environ.get("FOLLOWER_HOST", "follower-drone")

# Port on which drones broadcast their state (GCS listens)
LEADER_TELEM_PORT = int(os.environ.get("LEADER_TELEM_PORT", "14570"))
FOLLOWER_TELEM_PORT = int(os.environ.get("FOLLOWER_TELEM_PORT", "14571"))

# Port on which drones listen for GCS commands
LEADER_CMD_PORT = int(os.environ.get("LEADER_CMD_PORT", "14580"))
FOLLOWER_CMD_PORT = int(os.environ.get("FOLLOWER_CMD_PORT", "14581"))

HOME_LAT = float(os.environ.get("HOME_LAT", "-35.3632620"))
HOME_LON = float(os.environ.get("HOME_LON", "149.1652370"))
WEB_PORT = int(os.environ.get("WEB_PORT", "5000"))
GEOFENCE_RADIUS_M = float(os.environ.get("GEOFENCE_RADIUS_M", "200.0"))
SAFETY_DIST_M = float(os.environ.get("SAFETY_DIST_M", "1.0"))
CATCHUP_DIST_M = float(os.environ.get("CATCHUP_DIST_M", "8.0"))

METERS_PER_DEG_LAT = 111320.0

# ── Wire format (must match mavlink_bridge.py) ────────────────
# Telemetry: lat, lon, alt, vx, vy, vz, heading, timestamp, guidance_mode
_TELEM_FMT = "!9d"
_TELEM_SIZE = struct.calcsize(_TELEM_FMT)

# Command codes (must match mavlink_bridge.py)
CMD_RTL = 1
CMD_LAND = 2
CMD_KILL = 3
CMD_FOLLOW = 4
CMD_HOVER = 5
CMD_WASD = 10
CMD_WAYPOINT = 11
CMD_SPEED = 12

# Command wire formats
_CMD_SIMPLE_FMT = "!B"
_CMD_WASD_FMT = "!Bff"
_CMD_WAYPOINT_FMT = "!Bdd"
_CMD_SPEED_FMT = "!Bf"


# ═══════════════════════════════════════════════════════════════
# Telemetry Receiver
# ═══════════════════════════════════════════════════════════════

class TelemReceiver:
    """Listens for drone telemetry on a UDP port."""

    def __init__(self, port: int, label: str):
        self.port = port
        self.label = label
        self._state = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        # Packet stats
        self._pkt_count: int = 0
        self._last_count: int = 0
        self._last_rate_time: float = 0.0
        self._rate: float = 0.0
        self._stats_lock = threading.Lock()

    def start(self):
        self._running = True
        self._thread = threading.Thread(
            target=self._listen, daemon=True, name=f"telem-{self.label}")
        self._thread.start()

    def stop(self):
        self._running = False

    def get_state(self) -> dict | None:
        with self._lock:
            return self._state

    def get_stats(self) -> dict:
        """Return packet count and current receive rate."""
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
                    _GUIDANCE_MODE_NAMES = {0: 'NONE', 1: 'TRACKING', 2: 'CATCHUP',
                                            3: 'EVASION', 4: 'FAILSAFE'}
                    state = {
                        'lat': vals[0],
                        'lon': vals[1],
                        'alt': vals[2],
                        'vx': vals[3],
                        'vy': vals[4],
                        'vz': vals[5],
                        'heading': vals[6],
                        'timestamp': vals[7],
                        'guidance_mode': _GUIDANCE_MODE_NAMES.get(int(vals[8]), 'NONE'),
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
# Command Sender (extended for variable-length commands)
# ═══════════════════════════════════════════════════════════════

class CommandSender:
    """Sends commands to drones via UDP (variable-length protocol)."""

    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.leader_cmd_count: int = 0
        self.follower_cmd_count: int = 0

    def _send_raw(self, host: str, port: int, data: bytes):
        try:
            self._sock.sendto(data, (host, port))
            if port == LEADER_CMD_PORT:
                self.leader_cmd_count += 1
            elif port == FOLLOWER_CMD_PORT:
                self.follower_cmd_count += 1
        except (OSError, socket.gaierror) as e:
            log.error("Failed to send to %s:%d: %s", host, port, e)

    def send_simple(self, host: str, port: int, cmd_code: int):
        """Send a 1-byte command (RTL, LAND, KILL, FOLLOW, HOVER)."""
        data = struct.pack(_CMD_SIMPLE_FMT, cmd_code)
        self._send_raw(host, port, data)
        log.info("Sent %s to %s:%d", {
            CMD_RTL: "RTL", CMD_LAND: "LAND", CMD_KILL: "KILL",
            CMD_FOLLOW: "FOLLOW", CMD_HOVER: "HOVER",
        }.get(cmd_code, f"cmd={cmd_code}"), host, port)

    def send_wasd(self, host: str, port: int, vn: float, ve: float):
        """Send WASD velocity command to leader."""
        data = struct.pack(_CMD_WASD_FMT, CMD_WASD, vn, ve)
        self._send_raw(host, port, data)

    def send_waypoint(self, host: str, port: int, lat: float, lon: float):
        """Send waypoint command to leader."""
        data = struct.pack(_CMD_WAYPOINT_FMT, CMD_WAYPOINT, lat, lon)
        self._send_raw(host, port, data)
        log.info("Sent WAYPOINT (%.7f, %.7f) to %s:%d", lat, lon, host, port)

    def send_speed(self, host: str, port: int, speed: float):
        """Send operating speed command to leader."""
        data = struct.pack(_CMD_SPEED_FMT, CMD_SPEED, speed)
        self._send_raw(host, port, data)
        log.info("Sent SPEED %.1f to %s:%d", speed, host, port)


# ═══════════════════════════════════════════════════════════════
# Web GCS
# ═══════════════════════════════════════════════════════════════

class DockerGCS:
    """Web GCS for dockerized leader-follower system."""

    def __init__(self):
        self._running = True

        # Telemetry receivers
        self.leader_telem = TelemReceiver(LEADER_TELEM_PORT, "leader")
        self.follower_telem = TelemReceiver(FOLLOWER_TELEM_PORT, "follower")

        # Command sender
        self.cmd_sender = CommandSender()

        # Tracked drone modes (updated when GCS sends commands)
        self.leader_mode = "HOVER"    # HOVER / WASD / GOTO
        self.follower_mode = "HOVER"  # HOVER / FOLLOW

        # Leader waypoint target (for display)
        self.leader_wp_lat = 0.0
        self.leader_wp_lon = 0.0
        self.leader_speed = 1.5

        # Flask + SocketIO
        template_dir = os.path.join(os.path.dirname(__file__), "templates")
        self.app = Flask(__name__, template_folder=template_dir)
        self.app.config["SECRET_KEY"] = "leader-follower-gcs"
        self.sio = SocketIO(self.app, async_mode="threading", cors_allowed_origins="*")

        self._register_routes()
        self._register_events()

    def _register_routes(self):
        @self.app.route("/")
        def index():
            return render_template("index.html",
                                   home_lat=HOME_LAT, home_lon=HOME_LON,
                                   geofence_radius_m=GEOFENCE_RADIUS_M,
                                   safety_dist_m=SAFETY_DIST_M,
                                   catchup_dist_m=CATCHUP_DIST_M)

    def _register_events(self):
        sio = self.sio

        @sio.on("connect")
        def on_connect():
            log.info("Browser connected")
            # Send current state to newly connected client
            sio.emit("mode_update", {
                "leader_mode": self.leader_mode,
                "follower_mode": self.follower_mode,
                "leader_speed": self.leader_speed,
                "leader_wp_lat": self.leader_wp_lat,
                "leader_wp_lon": self.leader_wp_lon,
            })

        # ── Safety commands (both/leader/follower) ──

        @sio.on("cmd_rtl")
        def on_rtl(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self.cmd_sender.send_simple(LEADER_HOST, LEADER_CMD_PORT, CMD_RTL)
                self.leader_mode = "RTL"
            if target in ("follower", "both"):
                self.cmd_sender.send_simple(FOLLOWER_HOST, FOLLOWER_CMD_PORT, CMD_RTL)
                self.follower_mode = "RTL"
            sio.emit("log", {"msg": f"RTL sent to {target}"})
            self._emit_mode_update()

        @sio.on("cmd_land")
        def on_land(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self.cmd_sender.send_simple(LEADER_HOST, LEADER_CMD_PORT, CMD_LAND)
                self.leader_mode = "LAND"
            if target in ("follower", "both"):
                self.cmd_sender.send_simple(FOLLOWER_HOST, FOLLOWER_CMD_PORT, CMD_LAND)
                self.follower_mode = "LAND"
            sio.emit("log", {"msg": f"LAND sent to {target}"})
            self._emit_mode_update()

        @sio.on("cmd_kill")
        def on_kill(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self.cmd_sender.send_simple(LEADER_HOST, LEADER_CMD_PORT, CMD_KILL)
                self.leader_mode = "KILLED"
            if target in ("follower", "both"):
                self.cmd_sender.send_simple(FOLLOWER_HOST, FOLLOWER_CMD_PORT, CMD_KILL)
                self.follower_mode = "KILLED"
            sio.emit("log", {"msg": f"KILL sent to {target}"})
            self._emit_mode_update()

        # ── Leader control ──

        @sio.on("leader_hover")
        def on_leader_hover(data=None):
            self.cmd_sender.send_simple(LEADER_HOST, LEADER_CMD_PORT, CMD_HOVER)
            self.leader_mode = "HOVER"
            self.leader_wp_lat = 0.0
            self.leader_wp_lon = 0.0
            sio.emit("log", {"msg": "Leader: HOVER"})
            self._emit_mode_update()

        @sio.on("leader_wasd")
        def on_leader_wasd(data=None):
            vn = float((data or {}).get("vn", 0))
            ve = float((data or {}).get("ve", 0))
            self.cmd_sender.send_wasd(LEADER_HOST, LEADER_CMD_PORT, vn, ve)
            self.leader_mode = "WASD"
            self.leader_wp_lat = 0.0
            self.leader_wp_lon = 0.0
            self._emit_mode_update()

        @sio.on("leader_waypoint")
        def on_leader_waypoint(data=None):
            lat = float((data or {}).get("lat", 0))
            lon = float((data or {}).get("lon", 0))
            if lat != 0 and lon != 0:
                self.cmd_sender.send_waypoint(LEADER_HOST, LEADER_CMD_PORT, lat, lon)
                self.leader_mode = "GOTO"
                self.leader_wp_lat = lat
                self.leader_wp_lon = lon
                sio.emit("log", {"msg": f"Leader: GOTO ({lat:.6f}, {lon:.6f})"})
                self._emit_mode_update()

        @sio.on("leader_speed")
        def on_leader_speed(data=None):
            speed = float((data or {}).get("speed", 1.5))
            speed = max(0.5, min(speed, 10.0))
            self.cmd_sender.send_speed(LEADER_HOST, LEADER_CMD_PORT, speed)
            self.leader_speed = speed
            sio.emit("log", {"msg": f"Leader speed: {speed:.1f} m/s"})
            self._emit_mode_update()

        # ── Follower RC mock ──

        @sio.on("follower_follow")
        def on_follower_follow(data=None):
            self.cmd_sender.send_simple(FOLLOWER_HOST, FOLLOWER_CMD_PORT, CMD_FOLLOW)
            self.follower_mode = "FOLLOW"
            sio.emit("log", {"msg": "Follower: FOLLOW"})
            self._emit_mode_update()

        @sio.on("follower_hover")
        def on_follower_hover(data=None):
            self.cmd_sender.send_simple(FOLLOWER_HOST, FOLLOWER_CMD_PORT, CMD_HOVER)
            self.follower_mode = "HOVER"
            sio.emit("log", {"msg": "Follower: HOVER"})
            self._emit_mode_update()

    def _emit_mode_update(self):
        """Push mode state to all connected browsers."""
        self.sio.emit("mode_update", {
            "leader_mode": self.leader_mode,
            "follower_mode": self.follower_mode,
            "leader_speed": self.leader_speed,
            "leader_wp_lat": self.leader_wp_lat,
            "leader_wp_lon": self.leader_wp_lon,
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
        lp = self.leader_telem.get_state() or {}
        fp = self.follower_telem.get_state() or {}

        # Compute peer distance
        peer_dist = 0.0
        if lp.get("lat") and fp.get("lat"):
            dn = (lp["lat"] - fp["lat"]) * METERS_PER_DEG_LAT
            de = ((lp["lon"] - fp["lon"]) * METERS_PER_DEG_LAT
                  * math.cos(math.radians(fp["lat"])))
            peer_dist = math.sqrt(dn * dn + de * de)

        l_speed = 0.0
        if lp.get("vx") is not None:
            l_speed = math.sqrt(lp["vx"] ** 2 + lp["vy"] ** 2)

        f_speed = 0.0
        if fp.get("vx") is not None:
            f_speed = math.sqrt(fp["vx"] ** 2 + fp["vy"] ** 2)

        # Home distances
        l_home_dist = 0.0
        if lp.get("lat"):
            dn = (lp["lat"] - HOME_LAT) * METERS_PER_DEG_LAT
            de = (lp["lon"] - HOME_LON) * METERS_PER_DEG_LAT * math.cos(math.radians(HOME_LAT))
            l_home_dist = math.sqrt(dn * dn + de * de)

        f_home_dist = 0.0
        if fp.get("lat"):
            dn = (fp["lat"] - HOME_LAT) * METERS_PER_DEG_LAT
            de = (fp["lon"] - HOME_LON) * METERS_PER_DEG_LAT * math.cos(math.radians(HOME_LAT))
            f_home_dist = math.sqrt(dn * dn + de * de)

        # Auto-detect leader GOTO arrival
        if self.leader_mode == "GOTO" and self.leader_wp_lat != 0 and lp.get("lat"):
            wp_dist = math.sqrt(
                ((lp["lat"] - self.leader_wp_lat) * METERS_PER_DEG_LAT) ** 2 +
                ((lp["lon"] - self.leader_wp_lon) * METERS_PER_DEG_LAT
                 * math.cos(math.radians(lp["lat"]))) ** 2)
            if wp_dist < 2.0:
                self.leader_mode = "HOVER"
                self.leader_wp_lat = 0.0
                self.leader_wp_lon = 0.0
                self._emit_mode_update()

        # Network stats
        l_stats = self.leader_telem.get_stats()
        f_stats = self.follower_telem.get_stats()

        payload = {
            "leader": {
                "lat": lp.get("lat", 0),
                "lon": lp.get("lon", 0),
                "alt": lp.get("alt", 0),
                "vn": lp.get("vx", 0),
                "ve": lp.get("vy", 0),
                "heading": lp.get("heading", 0),
                "speed": round(l_speed, 2),
                "home_dist": round(l_home_dist, 1),
            },
            "follower": {
                "lat": fp.get("lat", 0),
                "lon": fp.get("lon", 0),
                "alt": fp.get("alt", 0),
                "vn": fp.get("vx", 0),
                "ve": fp.get("vy", 0),
                "heading": fp.get("heading", 0),
                "speed": round(f_speed, 2),
                "home_dist": round(f_home_dist, 1),
            },
            "peer_dist": round(peer_dist, 2),
            "follower_guidance_mode": fp.get("guidance_mode", "NONE"),
            "leader_connected": lp.get("lat", 0) != 0,
            "follower_connected": fp.get("lat", 0) != 0,
            "leader_mode": self.leader_mode,
            "follower_mode": self.follower_mode,
            "leader_speed": self.leader_speed,
            "leader_wp_lat": self.leader_wp_lat,
            "leader_wp_lon": self.leader_wp_lon,
            "network": {
                "leader_rx_count": l_stats['count'],
                "leader_rx_rate": l_stats['rate'],
                "follower_rx_count": f_stats['count'],
                "follower_rx_rate": f_stats['rate'],
                "leader_cmd_count": self.cmd_sender.leader_cmd_count,
                "follower_cmd_count": self.cmd_sender.follower_cmd_count,
            },
        }

        self.sio.emit("state_update", payload)

    def run(self):
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, '_running', False))
        signal.signal(signal.SIGTERM, lambda s, f: setattr(self, '_running', False))

        self.leader_telem.start()
        self.follower_telem.start()

        threading.Thread(target=self._broadcast_loop, daemon=True).start()

        log.info("=" * 50)
        log.info("Docker GCS -- http://0.0.0.0:%d", WEB_PORT)
        log.info("=" * 50)

        try:
            self.sio.run(self.app, host="0.0.0.0", port=WEB_PORT,
                         allow_unsafe_werkzeug=True)
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            self.leader_telem.stop()
            self.follower_telem.stop()
            log.info("GCS shutdown complete")


if __name__ == "__main__":
    gcs = DockerGCS()
    gcs.run()
