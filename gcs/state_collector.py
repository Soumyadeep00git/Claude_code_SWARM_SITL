"""
State collector — aggregates latest state from all drones.
"""

import time
import logging

log = logging.getLogger(__name__)


class StateCollector:
    """Receives and stores the latest state from each drone."""

    def __init__(self, num_drones: int):
        self.num_drones = num_drones
        self.states: dict[int, dict] = {}
        self.timestamps: dict[int, float] = {}

    def update(self, drone_id: int, data: dict, ts: float):
        """Update state for a drone from a STATE_REPORT."""
        self.states[drone_id] = data
        self.timestamps[drone_id] = ts

    def get_state(self, drone_id: int) -> dict | None:
        return self.states.get(drone_id)

    def get_all_states(self) -> dict[int, dict]:
        return dict(self.states)

    def get_stale_drones(self, timeout_s: float) -> list[int]:
        """Return IDs of drones that haven't reported within timeout_s."""
        now = time.time()
        stale = []
        for did, ts in self.timestamps.items():
            if now - ts > timeout_s:
                stale.append(did)
        return stale

    def get_leader_position(self, leader_id: int = 1) -> tuple[float, float, float] | None:
        """Get leader's current position for formation reference."""
        s = self.states.get(leader_id)
        if s:
            return (s["lat"], s["lon"], s["alt"])
        return None

    def print_summary(self):
        """Print a one-line summary for each drone."""
        for did in sorted(self.states.keys()):
            s = self.states[did]
            age = time.time() - self.timestamps.get(did, 0)
            print(
                f"  D{did}: mode={s.get('mode', '?'):>10s} "
                f"alt={s.get('alt', 0):5.1f}m "
                f"armed={'Y' if s.get('armed') else 'N'} "
                f"fs={'!' if s.get('failsafe_active') else '.'} "
                f"age={age:.1f}s"
            )
