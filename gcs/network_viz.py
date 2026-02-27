"""
Network topology aggregator for the Web GCS.

Collects mesh_stats from drone STATE_REPORTs and builds a topology
payload for the frontend (link qualities, packet stats, routing tables).
"""

import logging

log = logging.getLogger(__name__)


class NetworkAggregator:
    """Aggregates mesh network data from drone state reports."""

    def __init__(self):
        # {drone_id: mesh_stats dict from STATE_REPORT}
        self._mesh_data: dict[int, dict] = {}

    def update(self, drone_id: int, mesh_stats: dict | None):
        """Update mesh stats for a drone. Called from UDP loop."""
        if mesh_stats:
            self._mesh_data[drone_id] = mesh_stats

    def remove_drone(self, drone_id: int):
        """Remove a drone from mesh data."""
        self._mesh_data.pop(drone_id, None)

    def prune(self, active_ids: set[int]):
        """Remove drones not in active_ids."""
        to_remove = [did for did in self._mesh_data if did not in active_ids]
        for did in to_remove:
            del self._mesh_data[did]

    def get_topology_payload(self, stale_ids: list[int] | None = None) -> dict:
        """Build frontend-ready network topology payload.
        Excludes stale drones from link/routing data (they have no comms).

        Returns:
            {
                "links": [{src, dst, quality, distance_m}, ...],
                "nodes": {drone_id: {sent, delivered, dropped, forwarded, stale}, ...},
                "routing_tables": {drone_id: {dest: {next_hop, hops, metric}}, ...},
            }
        """
        stale_set = set(stale_ids) if stale_ids else set()
        links = []
        seen_links: set[tuple[int, int]] = set()
        nodes: dict[int, dict] = {}
        routing_tables: dict[int, dict] = {}

        for did, stats in self._mesh_data.items():
            is_stale = did in stale_set

            # Link qualities — skip links involving stale drones
            if not is_stale:
                link_qualities = stats.get("link_qualities", {})
                for peer_id_str, lq in link_qualities.items():
                    pid = int(peer_id_str)
                    if pid in stale_set:
                        continue
                    key = (min(did, pid), max(did, pid))
                    if key not in seen_links:
                        seen_links.add(key)
                        links.append({
                            "src": key[0],
                            "dst": key[1],
                            "quality": lq.get("quality", 0),
                            "distance_m": lq.get("distance_m", 0),
                        })

            # Per-node packet stats (always include, mark stale)
            link_stats = stats.get("link_stats", {})
            node_stats = {"sent": 0, "delivered": 0, "dropped": 0,
                          "forwarded": 0, "stale": is_stale}
            for _pid_str, st in link_stats.items():
                node_stats["sent"] += st.get("sent", 0)
                node_stats["delivered"] += st.get("delivered", 0)
                node_stats["dropped"] += st.get("dropped", 0)
            node_stats["forwarded"] = stats.get("forwarded", 0)
            nodes[did] = node_stats

            # Routing table — skip for stale drones
            if not is_stale:
                rt = stats.get("routing_table", {})
                if rt:
                    routing_tables[did] = rt

        return {
            "links": links,
            "nodes": nodes,
            "routing_tables": routing_tables,
        }
