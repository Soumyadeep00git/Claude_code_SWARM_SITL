"""MAVLink UDP bridge — replaces the RFD9000 radio link between containers.

Each drone runs threads:
  1. Broadcaster: sends own state to peer + GCS via UDP
  2. Listener: receives peer's state on a UDP port
  3. Command listener: receives GCS commands (RTL, LAND, KILL)

This gives each drone visibility of the other's state, just like the
real radio mesh does on hardware.
"""

import logging
import socket
import struct
import threading
import time

log = logging.getLogger(__name__)

# Simple binary wire format for position state:
# lat, lon, alt, vx, vy, vz, heading, timestamp (all doubles)
_PACK_FMT = "!8d"
_PACK_SIZE = struct.calcsize(_PACK_FMT)

# GCS command format: 1 byte command code
# 1=RTL, 2=LAND, 3=KILL
_CMD_FMT = "!B"
_CMD_SIZE = struct.calcsize(_CMD_FMT)

CMD_RTL = 1
CMD_LAND = 2
CMD_KILL = 3


class MavlinkBridge:
    """Bridges MAVLink state between Docker containers via UDP."""

    def __init__(self, peer_host: str, broadcast_port: int, listen_port: int,
                 gcs_host: str = "", gcs_telem_port: int = 0,
                 gcs_cmd_port: int = 0):
        self.peer_host = peer_host
        self.broadcast_port = broadcast_port
        self.listen_port = listen_port

        # GCS communication
        self.gcs_host = gcs_host
        self.gcs_telem_port = gcs_telem_port
        self.gcs_cmd_port = gcs_cmd_port

        # Latest peer state + receive timestamp for staleness detection
        self.peer_state: dict | None = None
        self._peer_recv_time: float = 0.0
        self._peer_stale_timeout: float = 5.0  # seconds
        self._peer_lock = threading.Lock()

        # Own state to broadcast
        self._own_state: dict | None = None
        self._own_lock = threading.Lock()

        # Latest GCS command (consumed by main loop)
        self._pending_cmd: int | None = None
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

        log.info("MAVLink bridge started: broadcast->%s:%d, listen<-:%d, gcs_telem->%s:%d, gcs_cmd<-:%d",
                 self.peer_host, self.broadcast_port, self.listen_port,
                 self.gcs_host or "none", self.gcs_telem_port, self.gcs_cmd_port)

    def stop(self):
        self._running = False

    def update_own_state(self, pos_data: dict):
        """Called by main loop with latest own GLOBAL_POSITION_INT data."""
        with self._own_lock:
            self._own_state = pos_data

    def get_peer_state(self) -> dict | None:
        """Get latest peer position data, or None if stale/no data."""
        with self._peer_lock:
            if self.peer_state is None:
                return None
            if time.time() - self._peer_recv_time > self._peer_stale_timeout:
                return None
            return self.peer_state

    def get_pending_command(self) -> int | None:
        """Get and consume the latest GCS command, or None."""
        with self._cmd_lock:
            cmd = self._pending_cmd
            self._pending_cmd = None
            return cmd

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
                # Snapshot for next interval
                self._last_tx_peer = self._tx_peer
                self._last_tx_gcs = self._tx_gcs
                self._last_rx_peer = self._rx_peer
                self._last_rx_gcs_cmd = self._rx_gcs_cmd
                self._last_stats_time = now
            log.info("BRIDGE STATS | TX→peer: %d (%.1f/s) | TX→GCS: %d (%.1f/s) | "
                     "RX←peer: %d (%.1f/s) | RX←GCS cmd: %d (%.1f/s)",
                     self._tx_peer, tx_p_rate, self._tx_gcs, tx_g_rate,
                     self._rx_peer, rx_p_rate, self._rx_gcs_cmd, rx_c_rate)

    def _broadcast_loop(self):
        """Send own state to peer + GCS at ~10Hz."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while self._running:
            with self._own_lock:
                state = self._own_state

            if state is not None:
                data = struct.pack(
                    _PACK_FMT,
                    state.get('lat', 0.0),
                    state.get('lon', 0.0),
                    state.get('alt', 0.0),
                    state.get('vx', 0.0),
                    state.get('vy', 0.0),
                    state.get('vz', 0.0),
                    state.get('heading', 0.0),
                    time.time(),
                )

                # Send to peer
                try:
                    sock.sendto(data, (self.peer_host, self.broadcast_port))
                    with self._stats_lock:
                        self._tx_peer += 1
                except (OSError, socket.gaierror) as e:
                    log.debug("Peer broadcast failed: %s", e)

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
        """Receive peer state via UDP."""
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
                    peer = {
                        'lat': vals[0],
                        'lon': vals[1],
                        'alt': vals[2],
                        'vx': vals[3],
                        'vy': vals[4],
                        'vz': vals[5],
                        'heading': vals[6],
                    }
                    if peer['lat'] != 0.0 or peer['lon'] != 0.0:
                        with self._peer_lock:
                            self.peer_state = peer
                            self._peer_recv_time = time.time()
                        with self._stats_lock:
                            self._rx_peer += 1
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.5)
        sock.close()

    def _cmd_listen_loop(self):
        """Receive GCS commands via UDP."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.gcs_cmd_port))
        sock.settimeout(1.0)

        log.info("Listening for GCS commands on UDP:%d", self.gcs_cmd_port)

        while self._running:
            try:
                data, addr = sock.recvfrom(64)
                if len(data) == _CMD_SIZE:
                    cmd = struct.unpack(_CMD_FMT, data)[0]
                    cmd_names = {CMD_RTL: "RTL", CMD_LAND: "LAND", CMD_KILL: "KILL"}
                    log.info("GCS command received: %s from %s",
                             cmd_names.get(cmd, f"?{cmd}"), addr)
                    with self._cmd_lock:
                        self._pending_cmd = cmd
                    with self._stats_lock:
                        self._rx_gcs_cmd += 1
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.5)
        sock.close()
