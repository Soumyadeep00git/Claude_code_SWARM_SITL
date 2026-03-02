"""MAVLink UDP bridge — multi-peer telemetry relay for swarm drones.

Each drone runs threads:
  1. Broadcaster: sends own state to ALL peers + GCS via UDP
  2. Listener: receives peer state on a single UDP port (sender ID in packet)
  3. Command listener: receives GCS commands (RTL, LAND, KILL, etc.)

Wire format includes sender drone_id so receivers can demux packets
from multiple peers on a single listen port.
"""

import logging
import socket
import struct
import threading
import time

log = logging.getLogger(__name__)

# ── Telemetry wire format ────────────────────────────────────────
# drone_id (int32) + lat, lon, alt, vx, vy, vz, heading, timestamp, guidance_mode (9 doubles)
_PACK_FMT = "!i9d"
_PACK_SIZE = struct.calcsize(_PACK_FMT)

# ── GCS command format (unchanged — variable length) ─────────────
# Simple commands (1 byte):  RTL, LAND, KILL, FOLLOW, HOVER, TAKEOFF
# Extended commands:
#   CMD_WASD:     "!Bff"  (cmd, vn, ve)     → 9 bytes
#   CMD_WAYPOINT: "!Bdd"  (cmd, lat, lon)   → 17 bytes
#   CMD_SPEED:    "!Bf"   (cmd, speed)       → 5 bytes

CMD_RTL = 1
CMD_LAND = 2
CMD_KILL = 3
CMD_FOLLOW = 4   # follower: enter FOLLOW (guidance) mode
CMD_HOVER = 5    # enter HOVER (hold position) mode
CMD_TAKEOFF = 6  # explicit takeoff command from GCS
CMD_WASD = 10    # leader: velocity NED (payload: vn, ve floats)
CMD_WAYPOINT = 11  # leader: fly to waypoint (payload: lat, lon doubles)
CMD_SPEED = 12   # leader: set operating speed (payload: speed float)

_CMD_SIMPLE_SIZE = 1
_CMD_WASD_FMT = "!Bff"
_CMD_WAYPOINT_FMT = "!Bdd"
_CMD_SPEED_FMT = "!Bf"

CMD_NAMES = {
    CMD_RTL: "RTL", CMD_LAND: "LAND", CMD_KILL: "KILL",
    CMD_FOLLOW: "FOLLOW", CMD_HOVER: "HOVER", CMD_TAKEOFF: "TAKEOFF",
    CMD_WASD: "WASD", CMD_WAYPOINT: "WAYPOINT", CMD_SPEED: "SPEED",
}


class MavlinkBridge:
    """Multi-peer UDP bridge for swarm telemetry."""

    def __init__(self, drone_id: int,
                 peer_targets: dict,
                 listen_port: int,
                 gcs_host: str = "", gcs_telem_port: int = 0,
                 gcs_cmd_port: int = 0, stale_timeout: float = 5.0):
        """
        Args:
            drone_id: This drone's ID (included in broadcast packets).
            peer_targets: {peer_id: (hostname, port)} — where to send telemetry.
            listen_port: UDP port to listen on for incoming peer telemetry.
            gcs_host: GCS hostname for telemetry forwarding.
            gcs_telem_port: GCS port for telemetry forwarding.
            gcs_cmd_port: UDP port to listen on for GCS commands.
            stale_timeout: Seconds before a peer's state is considered stale.
        """
        self.drone_id = drone_id
        self.peer_targets = peer_targets
        self.listen_port = listen_port

        # GCS communication
        self.gcs_host = gcs_host
        self.gcs_telem_port = gcs_telem_port
        self.gcs_cmd_port = gcs_cmd_port

        # Multi-peer state storage: drone_id -> state dict
        self._peer_states: dict = {}
        self._peer_recv_times: dict = {}
        self._peer_stale_timeout: float = stale_timeout
        self._peer_lock = threading.Lock()

        # Guidance mode code broadcast with telemetry
        self._guidance_mode: float = 0.0
        self._guidance_mode_lock = threading.Lock()

        # Own state to broadcast
        self._own_state: dict = None
        self._own_lock = threading.Lock()

        # GCS command queue — stores dicts: {'cmd': int, ...payload}
        self._cmd_queue: list = []
        self._cmd_lock = threading.Lock()

        # Packet counters
        self._tx_peer: int = 0
        self._tx_gcs: int = 0
        self._rx_peer: int = 0
        self._rx_gcs_cmd: int = 0
        self._stats_lock = threading.Lock()
        self._last_stats_time: float = 0.0
        self._last_tx_peer: int = 0
        self._last_tx_gcs: int = 0
        self._last_rx_peer: int = 0
        self._last_rx_gcs_cmd: int = 0

        self._running = False

    def start(self):
        """Start all bridge threads."""
        self._running = True
        self._last_stats_time = time.time()

        threading.Thread(
            target=self._listen_loop, daemon=True, name="bridge-listen"
        ).start()

        threading.Thread(
            target=self._broadcast_loop, daemon=True, name="bridge-broadcast"
        ).start()

        if self.gcs_cmd_port:
            threading.Thread(
                target=self._cmd_listen_loop, daemon=True, name="bridge-cmd"
            ).start()

        threading.Thread(
            target=self._stats_loop, daemon=True, name="bridge-stats"
        ).start()

        peer_desc = ", ".join(f"{pid}@{h}:{p}" for pid, (h, p) in self.peer_targets.items())
        log.info("MAVLink bridge started: drone_id=%d peers=[%s] listen<-:%d gcs->%s:%d cmd<-:%d",
                 self.drone_id, peer_desc, self.listen_port,
                 self.gcs_host or "none", self.gcs_telem_port, self.gcs_cmd_port)

    def stop(self):
        self._running = False

    def update_own_state(self, pos_data: dict):
        """Called by main loop with latest own GLOBAL_POSITION_INT data."""
        with self._own_lock:
            self._own_state = pos_data

    def set_guidance_mode(self, code: int):
        """Set guidance mode code for broadcast (0=NONE,1=TRACKING,2=CATCHUP,3=EVASION,4=FAILSAFE)."""
        with self._guidance_mode_lock:
            self._guidance_mode = float(code)

    def get_peer_state(self, peer_id: int = None) -> dict:
        """Get latest state for a specific peer, or None if stale/missing.

        Args:
            peer_id: Drone ID of the peer to query. If None and only one peer
                     exists, returns that peer's state (backward compat).
        """
        with self._peer_lock:
            if peer_id is None:
                # Backward compat: if only one peer, return it
                if len(self._peer_states) == 1:
                    peer_id = next(iter(self._peer_states))
                else:
                    return None

            state = self._peer_states.get(peer_id)
            if state is None:
                return None
            recv_time = self._peer_recv_times.get(peer_id, 0.0)
            if time.time() - recv_time > self._peer_stale_timeout:
                return None
            return state

    def get_all_peer_states(self) -> dict:
        """Get all non-stale peer states: {drone_id: state_dict}."""
        now = time.time()
        result = {}
        with self._peer_lock:
            for pid, state in self._peer_states.items():
                recv_time = self._peer_recv_times.get(pid, 0.0)
                if now - recv_time <= self._peer_stale_timeout:
                    result[pid] = state
        return result

    def get_pending_command(self) -> dict:
        """Get and consume the next GCS command dict, or None.

        Returns dict like {'cmd': CMD_RTL} or {'cmd': CMD_WASD, 'vn': 1.0, 've': 0.0}.
        """
        with self._cmd_lock:
            if self._cmd_queue:
                return self._cmd_queue.pop(0)
            return None

    def get_stats(self) -> dict:
        """Return packet counts and rates for monitoring."""
        now = time.time()
        with self._stats_lock:
            dt = max(now - self._last_stats_time, 0.1)
            stats = {
                'tx_peer': self._tx_peer,
                'tx_gcs': self._tx_gcs,
                'rx_peer': self._rx_peer,
                'rx_gcs_cmd': self._rx_gcs_cmd,
                'tx_peer_rate': (self._tx_peer - self._last_tx_peer) / dt,
                'tx_gcs_rate': (self._tx_gcs - self._last_tx_gcs) / dt,
                'rx_peer_rate': (self._rx_peer - self._last_rx_peer) / dt,
                'rx_gcs_cmd_rate': (self._rx_gcs_cmd - self._last_rx_gcs_cmd) / dt,
            }
        return stats

    def _stats_loop(self):
        """Log bridge stats every 10 seconds."""
        while self._running:
            time.sleep(10.0)
            now = time.time()
            with self._stats_lock:
                dt = max(now - self._last_stats_time, 0.1)
                tx_p_rate = (self._tx_peer - self._last_tx_peer) / dt
                tx_g_rate = (self._tx_gcs - self._last_tx_gcs) / dt
                rx_p_rate = (self._rx_peer - self._last_rx_peer) / dt
                rx_c_rate = (self._rx_gcs_cmd - self._last_rx_gcs_cmd) / dt
                self._last_tx_peer = self._tx_peer
                self._last_tx_gcs = self._tx_gcs
                self._last_rx_peer = self._rx_peer
                self._last_rx_gcs_cmd = self._rx_gcs_cmd
                self._last_stats_time = now
            peers_str = ", ".join(f"{pid}" for pid in self._peer_states.keys())
            log.info("BRIDGE STATS | TX->peer: %d (%.1f/s) | TX->GCS: %d (%.1f/s) | "
                     "RX<-peer: %d (%.1f/s) | RX<-GCS cmd: %d (%.1f/s) | peers=[%s]",
                     self._tx_peer, tx_p_rate, self._tx_gcs, tx_g_rate,
                     self._rx_peer, rx_p_rate, self._rx_gcs_cmd, rx_c_rate, peers_str)

    def _broadcast_loop(self):
        """Send own state to ALL peers + GCS at ~10Hz."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while self._running:
            with self._own_lock:
                state = self._own_state

            if state is not None:
                with self._guidance_mode_lock:
                    gmode = self._guidance_mode
                data = struct.pack(
                    _PACK_FMT,
                    self.drone_id,
                    state.get('lat', 0.0),
                    state.get('lon', 0.0),
                    state.get('alt', 0.0),
                    state.get('vx', 0.0),
                    state.get('vy', 0.0),
                    state.get('vz', 0.0),
                    state.get('heading', 0.0),
                    time.time(),
                    gmode,
                )

                # Send to ALL peers
                for pid, (host, port) in self.peer_targets.items():
                    try:
                        sock.sendto(data, (host, port))
                        with self._stats_lock:
                            self._tx_peer += 1
                    except (OSError, socket.gaierror) as e:
                        log.debug("Peer broadcast to %s:%d failed: %s", host, port, e)

                # Send to GCS
                if self.gcs_host and self.gcs_telem_port:
                    try:
                        sock.sendto(data, (self.gcs_host, self.gcs_telem_port))
                        with self._stats_lock:
                            self._tx_gcs += 1
                    except (OSError, socket.gaierror) as e:
                        log.debug("GCS broadcast failed: %s", e)

            time.sleep(0.1)  # 10Hz
        sock.close()

    def _listen_loop(self):
        """Receive peer state via UDP — demux by sender drone_id."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.listen_port))
        sock.settimeout(1.0)

        log.info("Listening for peer state on UDP:%d", self.listen_port)

        while self._running:
            try:
                data, _ = sock.recvfrom(4096)
                if len(data) == _PACK_SIZE:
                    vals = struct.unpack(_PACK_FMT, data)
                    sender_id = vals[0]
                    peer = {
                        'drone_id': sender_id,
                        'lat': vals[1],
                        'lon': vals[2],
                        'alt': vals[3],
                        'vx': vals[4],
                        'vy': vals[5],
                        'vz': vals[6],
                        'heading': vals[7],
                        'guidance_mode': int(vals[9]),
                    }
                    if peer['lat'] != 0.0 or peer['lon'] != 0.0:
                        with self._peer_lock:
                            self._peer_states[sender_id] = peer
                            self._peer_recv_times[sender_id] = time.time()
                        with self._stats_lock:
                            self._rx_peer += 1
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.5)
        sock.close()

    def _cmd_listen_loop(self):
        """Receive GCS commands via UDP (variable-length protocol)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.gcs_cmd_port))
        sock.settimeout(1.0)

        log.info("Listening for GCS commands on UDP:%d", self.gcs_cmd_port)

        while self._running:
            try:
                data, addr = sock.recvfrom(256)
                if len(data) < 1:
                    continue
                cmd_msg = self._parse_command(data)
                if cmd_msg is not None:
                    log.info("GCS command received: %s from %s",
                             CMD_NAMES.get(cmd_msg['cmd'], f"?{cmd_msg['cmd']}"), addr)
                    with self._cmd_lock:
                        self._cmd_queue.append(cmd_msg)
                    with self._stats_lock:
                        self._rx_gcs_cmd += 1
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.5)
        sock.close()

    @staticmethod
    def _parse_command(data: bytes) -> dict:
        """Parse a variable-length command packet."""
        if len(data) < 1:
            return None
        cmd = data[0]
        n = len(data)

        # Simple 1-byte commands
        if n == 1 and cmd in (CMD_RTL, CMD_LAND, CMD_KILL, CMD_FOLLOW, CMD_HOVER, CMD_TAKEOFF):
            return {'cmd': cmd}

        # WASD: "!Bff" -> 9 bytes
        wasd_size = struct.calcsize(_CMD_WASD_FMT)
        if n == wasd_size and cmd == CMD_WASD:
            _, vn, ve = struct.unpack(_CMD_WASD_FMT, data)
            return {'cmd': CMD_WASD, 'vn': vn, 've': ve}

        # SPEED: "!Bf" -> 5 bytes
        speed_size = struct.calcsize(_CMD_SPEED_FMT)
        if n == speed_size and cmd == CMD_SPEED:
            _, speed = struct.unpack(_CMD_SPEED_FMT, data)
            return {'cmd': CMD_SPEED, 'speed': speed}

        # WAYPOINT: "!Bdd" -> 17 bytes
        wp_size = struct.calcsize(_CMD_WAYPOINT_FMT)
        if n == wp_size and cmd == CMD_WAYPOINT:
            _, lat, lon = struct.unpack(_CMD_WAYPOINT_FMT, data)
            return {'cmd': CMD_WAYPOINT, 'lat': lat, 'lon': lon}

        log.warning("Unknown command packet: cmd=%d len=%d", cmd, n)
        return None
