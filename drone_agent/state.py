"""
Drone state representation and peer tracking.
"""

import time
from dataclasses import dataclass, field


@dataclass
class DroneState:
    """Current state of this drone, updated from MAVLink telemetry."""
    drone_id: int
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    heading: float = 0.0
    battery_pct: int = 100
    mode: str = "STABILIZE"
    armed: bool = False
    formation_slot: int = -1
    failsafe_active: bool = False
    swarm_state: str = "NOMINAL"
    leader_id: int = 0  # 0 = unset; derived from failsafe election
    alive_count: int = 0
    rl_mode: bool = False

    # Optional mesh network stats (set by agent when mesh sim is active)
    mesh_stats: dict | None = None

    def to_dict(self) -> dict:
        """Serialize for STATE_REPORT message."""
        d = {
            "drone_id": self.drone_id,
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "vx": self.vx,
            "vy": self.vy,
            "vz": self.vz,
            "heading": self.heading,
            "battery_pct": self.battery_pct,
            "mode": self.mode,
            "armed": self.armed,
            "formation_slot": self.formation_slot,
            "failsafe_active": self.failsafe_active,
            "swarm_state": self.swarm_state,
            "leader_id": self.leader_id,
            "alive_count": self.alive_count,
            "rl_mode": self.rl_mode,
        }
        if self.mesh_stats is not None:
            d["mesh_stats"] = self.mesh_stats
        return d

    def update_from_position(self, pos: dict):
        """Update from DroneConnection.get_position() result."""
        self.lat = pos["lat"]
        self.lon = pos["lon"]
        self.alt = pos["alt"]
        self.vx = pos["vx"]
        self.vy = pos["vy"]
        self.vz = pos["vz"]
        self.heading = pos["heading"]

    def update_from_heartbeat(self, hb: dict):
        """Update from DroneConnection.get_heartbeat() result."""
        self.mode = hb["mode"]
        self.armed = hb["armed"]


class PeerTable:
    """
    Tracks known positions of all other drones.
    Updated from STATE_REPORT messages relayed by GCS.
    """

    def __init__(self):
        self.peers: dict[int, DroneState] = {}
        self.last_update: dict[int, float] = {}

    def update_peer(self, drone_id: int, state_dict: dict, timestamp: float):
        """Update or create a peer entry from a STATE_REPORT data dict."""
        if drone_id not in self.peers:
            self.peers[drone_id] = DroneState(drone_id=drone_id)
        peer = self.peers[drone_id]
        for key in ("lat", "lon", "alt", "vx", "vy", "vz",
                     "heading", "mode", "armed", "failsafe_active",
                     "swarm_state"):
            if key in state_dict:
                setattr(peer, key, state_dict[key])
        self.last_update[drone_id] = timestamp

    def get_peer(self, drone_id: int) -> DroneState | None:
        return self.peers.get(drone_id)

    def get_all_positions(self) -> list[tuple[int, float, float, float]]:
        """Returns list of (drone_id, lat, lon, alt) for all known peers."""
        return [
            (did, p.lat, p.lon, p.alt)
            for did, p in self.peers.items()
        ]

    def get_all_states(self) -> list[tuple[int, float, float, float, float, float, float]]:
        """Returns list of (drone_id, lat, lon, alt, vx, vy, vz) for all known peers."""
        return [
            (did, p.lat, p.lon, p.alt, p.vx, p.vy, p.vz)
            for did, p in self.peers.items()
        ]

    def is_stale(self, drone_id: int, timeout_s: float) -> bool:
        """Check if a peer's data is older than timeout_s."""
        last = self.last_update.get(drone_id)
        if last is None:
            return True
        return (time.time() - last) > timeout_s

    def get_alive_ids(self, timeout_s: float) -> set[int]:
        """Return set of peer IDs whose data is not stale."""
        now = time.time()
        return {
            did for did, ts in self.last_update.items()
            if (now - ts) <= timeout_s
        }

    def get_stale_ids(self, timeout_s: float) -> set[int]:
        """Return set of peer IDs whose data is stale."""
        now = time.time()
        return {
            did for did, ts in self.last_update.items()
            if (now - ts) > timeout_s
        }
