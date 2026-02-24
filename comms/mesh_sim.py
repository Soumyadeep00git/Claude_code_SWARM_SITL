"""
Simulated UDP node with distance-based network effects.
Drop-in replacement for UDPNode — same interface (send, send_to_multiple, recv_all, close).

Adds:
  - Distance-based packet loss (quadratic falloff)
  - Latency simulation (base + jitter)
  - Per-link statistics tracking
  - Live link quality reporting

Only used by drone agents. GCS uses regular UDPNode.
"""

import time
import threading
import logging

from comms.udp_node import UDPNode
from comms.link_model import (
    haversine_m, link_quality, should_drop, compute_latency_ms,
)
from config import GCS_PORT, AGENT_BASE_PORT, AGENT_PORT_STEP

log = logging.getLogger(__name__)


class SimulatedUDPNode:
    """UDPNode wrapper with distance-based network simulation."""

    def __init__(self, listen_port: int, drone_id: int,
                 position_getter, max_range_m: float = 100.0,
                 buf_size: int = 4096):
        """
        Args:
            listen_port: UDP port to bind.
            drone_id: This drone's ID (1-based).
            position_getter: Callable returning {drone_id: (lat, lon)} for
                             all known drones (including self).
            max_range_m: Simulated radio range in meters.
            buf_size: UDP buffer size.
        """
        self._real = UDPNode(listen_port, buf_size)
        self.port = listen_port
        self.drone_id = drone_id
        self._pos_getter = position_getter
        self.max_range_m = max_range_m

        # Per-link stats: {peer_id: {sent, delivered, dropped, bytes, latency_sum}}
        self._link_stats: dict[int, dict] = {}
        self._stats_lock = threading.Lock()

        # Delay queue for latency simulation
        self._delay_queue: list[tuple[float, bytes, str, int]] = []
        self._queue_lock = threading.Lock()

        self._running = True
        self._delivery_thread = threading.Thread(
            target=self._deliver_loop, daemon=True, name=f"mesh-deliver-{drone_id}")
        self._delivery_thread.start()

        log.info("SimulatedUDPNode D%d: range=%dm", drone_id, max_range_m)

    # ── Public interface (matches UDPNode) ──────────────────

    def send(self, data: bytes, host: str, port: int):
        """Send with simulated network effects."""
        target_id = self._port_to_drone_id(port)

        # GCS links and unknown targets bypass simulation
        if target_id <= 0:
            self._real.send(data, host, port)
            return

        positions = self._pos_getter()
        my_pos = positions.get(self.drone_id)
        target_pos = positions.get(target_id)

        # No position info yet — send without simulation
        if not my_pos or not target_pos:
            self._real.send(data, host, port)
            return

        dist = haversine_m(my_pos[0], my_pos[1], target_pos[0], target_pos[1])
        quality = link_quality(dist, self.max_range_m)

        with self._stats_lock:
            if target_id not in self._link_stats:
                self._link_stats[target_id] = {
                    "sent": 0, "delivered": 0, "dropped": 0,
                    "bytes": 0, "latency_sum": 0.0,
                }
            stats = self._link_stats[target_id]
            stats["sent"] += 1

        if should_drop(quality):
            with self._stats_lock:
                stats["dropped"] += 1
            return

        latency_ms = compute_latency_ms(dist, quality)

        with self._stats_lock:
            stats["delivered"] += 1
            stats["bytes"] += len(data)
            stats["latency_sum"] += latency_ms

        # Low latency: send immediately to avoid thread overhead
        if latency_ms < 3.0:
            self._real.send(data, host, port)
        else:
            deliver_at = time.time() + latency_ms / 1000.0
            with self._queue_lock:
                self._delay_queue.append((deliver_at, data, host, port))

    def send_to_multiple(self, data: bytes, targets: list[tuple[str, int]]):
        """Send the same datagram to multiple targets with simulation."""
        for host, port in targets:
            self.send(data, host, port)

    def recv_all(self) -> list[tuple[bytes, tuple[str, int]]]:
        """Drain all pending datagrams (passthrough to real socket)."""
        return self._real.recv_all()

    def close(self):
        """Shut down simulation and close socket."""
        self._running = False
        self._real.close()

    # ── Stats API ───────────────────────────────────────────

    def get_link_stats(self) -> dict[int, dict]:
        """Per-peer packet statistics."""
        with self._stats_lock:
            return {pid: dict(s) for pid, s in self._link_stats.items()}

    def get_link_quality_map(self) -> dict[int, dict]:
        """Current link quality to all known peer positions."""
        positions = self._pos_getter()
        my_pos = positions.get(self.drone_id)
        if not my_pos:
            return {}
        result = {}
        for pid, pos in positions.items():
            if pid == self.drone_id:
                continue
            dist = haversine_m(my_pos[0], my_pos[1], pos[0], pos[1])
            result[pid] = {
                "quality": round(link_quality(dist, self.max_range_m), 3),
                "distance_m": round(dist, 1),
            }
        return result

    def set_range(self, range_m: float):
        """Live-update the simulated radio range."""
        self.max_range_m = max(1.0, range_m)
        log.info("SimulatedUDPNode D%d: range updated to %dm",
                 self.drone_id, self.max_range_m)

    # ── Internal ────────────────────────────────────────────

    @staticmethod
    def _port_to_drone_id(port: int) -> int:
        """Map UDP port to drone ID. Returns 0 for GCS, -1 for unknown."""
        if port == GCS_PORT:
            return 0
        offset = port - AGENT_BASE_PORT
        if offset > 0 and offset % AGENT_PORT_STEP == 0:
            return offset // AGENT_PORT_STEP
        return -1

    def _deliver_loop(self):
        """Background thread: deliver delayed packets."""
        while self._running:
            now = time.time()
            to_send = []
            with self._queue_lock:
                remaining = []
                for item in self._delay_queue:
                    if item[0] <= now:
                        to_send.append(item)
                    else:
                        remaining.append(item)
                self._delay_queue = remaining
            for _, data, host, port in to_send:
                try:
                    self._real.send(data, host, port)
                except Exception:
                    pass
            time.sleep(0.001)
