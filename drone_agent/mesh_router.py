"""
OLSR-lite mesh router for multi-hop packet forwarding.

Each drone agent runs one MeshRouter instance. It:
  1. Maintains a neighbor table with link qualities (from SimulatedUDPNode)
  2. Broadcasts NEIGHBOR_AD every 2s so peers learn the topology
  3. Builds a routing table via Dijkstra on the known topology
  4. Provides next-hop lookup and MESH_FORWARD message wrapping

Protocol messages used:
  - NEIGHBOR_AD:  {neighbors: {peer_id: quality, ...}}
  - MESH_FORWARD: {origin, dest, ttl, msg_id, payload}
"""

import time
import heapq
import logging
from collections import OrderedDict

log = logging.getLogger(__name__)

AD_INTERVAL_S = 2.0
MAX_TTL = 5
MAX_SEEN = 200


class MeshRouter:
    """Lightweight mesh routing with neighbor discovery and Dijkstra forwarding."""

    def __init__(self, drone_id: int, num_drones: int):
        self.drone_id = drone_id
        self.num_drones = num_drones

        # Neighbor table: {peer_id: quality} — updated from link quality map
        self.neighbors: dict[int, float] = {}

        # 2-hop topology from NEIGHBOR_ADs: {peer_id: {their_neighbor: quality}}
        self._topology: dict[int, dict[int, float]] = {}

        # Routing table: {dest_id: {"next_hop": int, "hops": int, "metric": float}}
        self._routes: dict[int, dict] = {}

        # Advertisement timing
        self._last_ad = 0.0

        # Forwarding dedup: msg_id -> timestamp (ordered for cleanup)
        self._seen: OrderedDict[str, float] = OrderedDict()

        # Stats
        self.forwarded_count = 0

    # ── Neighbor management ─────────────────────────────────

    def update_neighbors(self, link_quality_map: dict[int, dict]):
        """Update neighbor table from SimulatedUDPNode link quality map.
        link_quality_map: {peer_id: {"quality": float, "distance_m": float}}
        """
        self.neighbors = {
            pid: info["quality"]
            for pid, info in link_quality_map.items()
            if info["quality"] > 0.0
        }

    # ── NEIGHBOR_AD ─────────────────────────────────────────

    def should_advertise(self) -> bool:
        """True if it's time to broadcast our neighbor table."""
        now = time.time()
        if now - self._last_ad >= AD_INTERVAL_S:
            self._last_ad = now
            return True
        return False

    def get_advertisement_data(self) -> dict:
        """Payload for NEIGHBOR_AD message."""
        return {"neighbors": {str(k): round(v, 3) for k, v in self.neighbors.items()}}

    def handle_neighbor_ad(self, peer_id: int, data: dict):
        """Process a NEIGHBOR_AD from a peer — update topology and rebuild routes."""
        raw = data.get("neighbors", {})
        self._topology[peer_id] = {int(k): float(v) for k, v in raw.items()}
        self._rebuild_routes()

    # ── Routing ─────────────────────────────────────────────

    def get_next_hop(self, dest_id: int) -> int | None:
        """Next-hop drone ID to reach dest_id, or None if unreachable."""
        route = self._routes.get(dest_id)
        return route["next_hop"] if route else None

    def get_routing_table(self) -> dict[int, dict]:
        """Full routing table: {dest: {next_hop, hops, metric}}."""
        return dict(self._routes)

    def get_reachable_peers(self) -> set[int]:
        """Set of drone IDs reachable via routing."""
        return set(self._routes.keys())

    # ── MESH_FORWARD ────────────────────────────────────────

    def wrap_forward(self, dest_id: int, payload: dict) -> dict | None:
        """Create a MESH_FORWARD wrapper. Returns None if no route."""
        next_hop = self.get_next_hop(dest_id)
        if next_hop is None:
            return None
        msg_id = f"{self.drone_id}-{dest_id}-{int(time.time()*1000) % 100000}"
        self._mark_seen(msg_id)
        return {
            "origin": self.drone_id,
            "dest": dest_id,
            "next_hop": next_hop,
            "ttl": MAX_TTL,
            "msg_id": msg_id,
            "payload": payload,
        }

    def handle_mesh_forward(self, data: dict) -> dict | None:
        """Process a MESH_FORWARD message.
        Returns re-wrapped data for next hop, or None if:
          - This is the destination (caller should process payload)
          - Already seen (dedup)
          - TTL expired
          - No route to dest
        Caller checks data["dest"] == self.drone_id to know if it's for us.
        """
        msg_id = data.get("msg_id", "")
        if not self._check_and_mark_seen(msg_id):
            return None  # duplicate

        ttl = data.get("ttl", 0) - 1
        if ttl <= 0:
            return None  # expired

        dest = data.get("dest")
        if dest == self.drone_id:
            return None  # we're the destination — caller processes payload

        next_hop = self.get_next_hop(dest)
        if next_hop is None:
            return None

        self.forwarded_count += 1
        return {
            "origin": data.get("origin"),
            "dest": dest,
            "next_hop": next_hop,
            "ttl": ttl,
            "msg_id": msg_id,
            "payload": data.get("payload"),
        }

    # ── Internal ────────────────────────────────────────────

    def _rebuild_routes(self):
        """Dijkstra on known topology to build routing table."""
        # Build weighted graph: {node: {neighbor: cost}}
        graph: dict[int, dict[int, float]] = {}

        # Own neighbors
        graph[self.drone_id] = {
            pid: 1.0 / max(q, 0.01) for pid, q in self.neighbors.items()
        }

        # Topology from peer advertisements
        for pid, neighbors in self._topology.items():
            if pid not in graph:
                graph[pid] = {}
            for npid, q in neighbors.items():
                cost = 1.0 / max(q, 0.01)
                graph[pid][npid] = cost
                # Assume symmetric links
                if npid not in graph:
                    graph[npid] = {}
                graph[npid][pid] = cost

        # Dijkstra from self
        dist = {self.drone_id: 0.0}
        prev: dict[int, int] = {}
        visited: set[int] = set()
        heap = [(0.0, self.drone_id)]

        while heap:
            d, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)
            for v, w in graph.get(u, {}).items():
                nd = d + w
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        # Build routing table by tracing back to find first hop
        routes: dict[int, dict] = {}
        for dest in visited:
            if dest == self.drone_id:
                continue
            node = dest
            hops = 0
            while prev.get(node) is not None and prev[node] != self.drone_id:
                node = prev[node]
                hops += 1
            if prev.get(node) == self.drone_id:
                routes[dest] = {
                    "next_hop": node,
                    "hops": hops + 1,
                    "metric": round(dist[dest], 2),
                }

        self._routes = routes

    def _mark_seen(self, msg_id: str):
        self._seen[msg_id] = time.time()
        if len(self._seen) > MAX_SEEN:
            # Remove oldest entries
            while len(self._seen) > MAX_SEEN // 2:
                self._seen.popitem(last=False)

    def _check_and_mark_seen(self, msg_id: str) -> bool:
        """Returns True if msg_id is new (not seen before)."""
        if msg_id in self._seen:
            return False
        self._mark_seen(msg_id)
        return True
