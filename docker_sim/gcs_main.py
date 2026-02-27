"""GCS (Ground Control Station) container entry point.

Does NOT run SITL or guidance — only:
  1. Receives telemetry from leader and follower via UDP
  2. Serves a Flask + SocketIO web UI
  3. Forwards keyboard commands (KILL, LAND, RTL) to drones via UDP
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

METERS_PER_DEG_LAT = 111320.0

# ── Wire format (must match mavlink_bridge.py) ────────────────
# lat, lon, alt, vx, vy, vz, heading, timestamp
_TELEM_FMT = "!8d"
_TELEM_SIZE = struct.calcsize(_TELEM_FMT)

# Command wire format: 1 byte command code
# 1=RTL, 2=LAND, 3=KILL
_CMD_FMT = "!B"

CMD_RTL = 1
CMD_LAND = 2
CMD_KILL = 3


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
                    state = {
                        'lat': vals[0],
                        'lon': vals[1],
                        'alt': vals[2],
                        'vx': vals[3],
                        'vy': vals[4],
                        'vz': vals[5],
                        'heading': vals[6],
                        'timestamp': vals[7],
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
    """Sends commands to a drone via UDP."""

    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.leader_cmd_count: int = 0
        self.follower_cmd_count: int = 0

    def send(self, host: str, port: int, cmd_code: int):
        try:
            data = struct.pack(_CMD_FMT, cmd_code)
            self._sock.sendto(data, (host, port))
            if port == LEADER_CMD_PORT:
                self.leader_cmd_count += 1
            elif port == FOLLOWER_CMD_PORT:
                self.follower_cmd_count += 1
            log.info("Sent command %d to %s:%d", cmd_code, host, port)
        except (OSError, socket.gaierror) as e:
            log.error("Failed to send command to %s:%d: %s", host, port, e)


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
                                   home_lat=HOME_LAT, home_lon=HOME_LON)

    def _register_events(self):
        sio = self.sio

        @sio.on("connect")
        def on_connect():
            log.info("Browser connected")

        @sio.on("cmd_rtl")
        def on_rtl(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self.cmd_sender.send(LEADER_HOST, LEADER_CMD_PORT, CMD_RTL)
            if target in ("follower", "both"):
                self.cmd_sender.send(FOLLOWER_HOST, FOLLOWER_CMD_PORT, CMD_RTL)
            sio.emit("log", {"msg": f"RTL sent to {target}"})

        @sio.on("cmd_land")
        def on_land(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self.cmd_sender.send(LEADER_HOST, LEADER_CMD_PORT, CMD_LAND)
            if target in ("follower", "both"):
                self.cmd_sender.send(FOLLOWER_HOST, FOLLOWER_CMD_PORT, CMD_LAND)
            sio.emit("log", {"msg": f"LAND sent to {target}"})

        @sio.on("cmd_kill")
        def on_kill(data=None):
            target = (data or {}).get("target", "both")
            if target in ("leader", "both"):
                self.cmd_sender.send(LEADER_HOST, LEADER_CMD_PORT, CMD_KILL)
            if target in ("follower", "both"):
                self.cmd_sender.send(FOLLOWER_HOST, FOLLOWER_CMD_PORT, CMD_KILL)
            sio.emit("log", {"msg": f"KILL sent to {target}"})

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
                "speed": round(l_speed, 2),
            },
            "follower": {
                "lat": fp.get("lat", 0),
                "lon": fp.get("lon", 0),
                "alt": fp.get("alt", 0),
                "vn": fp.get("vx", 0),
                "ve": fp.get("vy", 0),
                "speed": round(f_speed, 2),
            },
            "peer_dist": round(peer_dist, 2),
            "leader_connected": lp.get("lat", 0) != 0,
            "follower_connected": fp.get("lat", 0) != 0,
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
        log.info("Docker GCS — http://0.0.0.0:%d", WEB_PORT)
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
