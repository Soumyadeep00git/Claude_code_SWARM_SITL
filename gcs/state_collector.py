"""
State collector — aggregates latest state from all drones.
"""

import time
import threading
import logging

log = logging.getLogger(__name__)


class StateCollector:
    """Receives and stores the latest state from each drone."""

    def __init__(self, num_drones: int):
        self.num_drones = num_drones
        self.states: dict[int, dict] = {}
        self.timestamps: dict[int, float] = {}
        self._lock = threading.Lock()

    def update(self, drone_id: int, data: dict, ts: float):
        """Update state for a drone from a STATE_REPORT."""
        with self._lock:
            self.states[drone_id] = data
            self.timestamps[drone_id] = ts

    def get_state(self, drone_id: int) -> dict | None:
        with self._lock:
            return self.states.get(drone_id)

    def get_all_states(self) -> dict[int, dict]:
        with self._lock:
            return dict(self.states)

    def get_stale_drones(self, timeout_s: float) -> list[int]:
        """Return IDs of drones that haven't reported within timeout_s."""
        now = time.time()
        with self._lock:
            stale = []
            for did, ts in self.timestamps.items():
                if now - ts > timeout_s:
                    stale.append(did)
            return stale

    def get_centroid(self) -> tuple[float, float, float] | None:
        """Compute GPS centroid of all drones with valid positions."""
        with self._lock:
            valid = [(s["lat"], s["lon"], s.get("alt", 10.0))
                     for s in self.states.values()
                     if s.get("lat", 0) != 0.0 or s.get("lon", 0) != 0.0]
        if not valid:
            return None
        avg_lat = sum(v[0] for v in valid) / len(valid)
        avg_lon = sum(v[1] for v in valid) / len(valid)
        avg_alt = sum(v[2] for v in valid) / len(valid)
        return (avg_lat, avg_lon, avg_alt)

    def get_valid_drone_ids(self) -> list[int]:
        """Return drone IDs that have valid (non-zero) GPS."""
        with self._lock:
            return [did for did, s in self.states.items()
                    if s.get("lat", 0) != 0.0 or s.get("lon", 0) != 0.0]

    def remove_drone(self, drone_id: int):
        """Remove a drone from state tracking."""
        with self._lock:
            self.states.pop(drone_id, None)
            self.timestamps.pop(drone_id, None)

    def prune_stale(self, timeout_s: float) -> list[int]:
        """Remove drones stale beyond timeout_s. Returns list of pruned IDs."""
        now = time.time()
        with self._lock:
            pruned = [did for did, ts in self.timestamps.items()
                      if now - ts > timeout_s]
            for did in pruned:
                self.states.pop(did, None)
                self.timestamps.pop(did, None)
        return pruned

    def print_summary(self):
        """Print a one-line summary for each drone."""
        with self._lock:
            snapshot = {did: (dict(s), self.timestamps.get(did, 0))
                        for did, s in self.states.items()}
        now = time.time()
        for did in sorted(snapshot.keys()):
            s, ts = snapshot[did]
            age = now - ts
            print(
                f"  D{did}: mode={s.get('mode', '?'):>10s} "
                f"alt={s.get('alt', 0):5.1f}m "
                f"armed={'Y' if s.get('armed') else 'N'} "
                f"fs={'!' if s.get('failsafe_active') else '.'} "
                f"age={age:.1f}s"
            )
