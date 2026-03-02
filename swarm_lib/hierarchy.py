"""Swarm hierarchy — defines topology for leader-follower formations.

Single source of truth: who is a leader, who is a follower, who follows whom.
Supports: 1 leader + N followers, M leaders + N followers (tree topology).

Each follower has exactly ONE leader (1:1 relationship preserved for guidance_lib).
Leaders can have zero or more followers.

Usage:
    topology = SwarmTopology.from_yaml("hierarchy.yaml")
    my_node = topology.get_node(drone_id=3)
    my_leader_id = my_node.leader_id          # None if I'm a leader
    my_follower_ids = my_node.follower_ids     # () if I'm a follower
    my_peers = topology.get_peers(drone_id=3)  # all drones I communicate with
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import yaml


class DroneRole(Enum):
    LEADER = "leader"
    FOLLOWER = "follower"


@dataclass(frozen=True)
class DroneNode:
    """One drone's identity and relationships."""
    drone_id: int
    role: DroneRole
    leader_id: Optional[int]        # None for leaders
    follower_ids: tuple             # () for followers, tuple of IDs for leaders
    offset_n: float = 0.0          # NED offset from leader (meters)
    offset_e: float = 0.0
    offset_d: float = 0.0
    home_spacing_m: float = 0.0    # east offset from base home (meters)


class SwarmTopology:
    """Complete swarm topology — immutable after construction."""

    def __init__(self, nodes: dict, base_home_lat: float, base_home_lon: float):
        self.nodes: dict[int, DroneNode] = nodes
        self.base_home_lat = base_home_lat
        self.base_home_lon = base_home_lon

    @staticmethod
    def from_yaml(path: str) -> "SwarmTopology":
        """Load topology from YAML hierarchy file.

        YAML format:
            home:
              lat: -35.3632620
              lon: 149.1652370
            drones:
              1:
                role: leader
                home_spacing_m: 0
              2:
                role: follower
                leader_id: 1
                offset: [-5.0, 3.0, 0.0]
                home_spacing_m: 20
        """
        with open(path) as f:
            raw = yaml.safe_load(f)

        home = raw["home"]
        nodes = {}

        # First pass: create all nodes (follower_ids filled in second pass)
        for did_str, cfg in raw["drones"].items():
            did = int(did_str)
            role = DroneRole(cfg["role"])
            leader_id = cfg.get("leader_id")
            offset = cfg.get("offset", [0.0, 0.0, 0.0])
            spacing = cfg.get("home_spacing_m", 0.0)

            nodes[did] = DroneNode(
                drone_id=did,
                role=role,
                leader_id=leader_id if role == DroneRole.FOLLOWER else None,
                follower_ids=(),
                offset_n=offset[0] if role == DroneRole.FOLLOWER else 0.0,
                offset_e=offset[1] if role == DroneRole.FOLLOWER else 0.0,
                offset_d=offset[2] if len(offset) > 2 and role == DroneRole.FOLLOWER else 0.0,
                home_spacing_m=spacing,
            )

        # Second pass: fill in follower_ids for leaders
        for did, node in list(nodes.items()):
            if node.role == DroneRole.LEADER:
                fids = tuple(
                    n.drone_id for n in sorted(nodes.values(), key=lambda x: x.drone_id)
                    if n.leader_id == did
                )
                # Rebuild frozen dataclass with follower_ids
                nodes[did] = DroneNode(
                    drone_id=node.drone_id,
                    role=node.role,
                    leader_id=None,
                    follower_ids=fids,
                    offset_n=0.0,
                    offset_e=0.0,
                    offset_d=0.0,
                    home_spacing_m=node.home_spacing_m,
                )

        return SwarmTopology(nodes=nodes,
                             base_home_lat=home["lat"],
                             base_home_lon=home["lon"])

    def get_node(self, drone_id: int) -> DroneNode:
        """Get a drone's node. Raises KeyError if not found."""
        return self.nodes[drone_id]

    def get_peers(self, drone_id: int) -> list:
        """Get all drone IDs this drone needs to communicate with.

        Leaders: all their followers.
        Followers: their leader + sibling followers under same leader.
        """
        node = self.nodes[drone_id]
        peers = set()
        if node.role == DroneRole.LEADER:
            peers.update(node.follower_ids)
        else:
            if node.leader_id is not None:
                peers.add(node.leader_id)
                leader_node = self.nodes[node.leader_id]
                peers.update(leader_node.follower_ids)
            peers.discard(drone_id)
        return sorted(peers)

    def all_drone_ids(self) -> list:
        return sorted(self.nodes.keys())

    def leader_ids(self) -> list:
        return sorted(d for d, n in self.nodes.items() if n.role == DroneRole.LEADER)

    def follower_ids(self) -> list:
        return sorted(d for d, n in self.nodes.items() if n.role == DroneRole.FOLLOWER)

    def validate(self) -> list:
        """Return list of validation errors (empty = valid)."""
        errors = []
        for did, node in self.nodes.items():
            if node.role == DroneRole.FOLLOWER:
                if node.leader_id is None:
                    errors.append(f"Drone {did}: follower has no leader_id")
                elif node.leader_id not in self.nodes:
                    errors.append(f"Drone {did}: leader_id={node.leader_id} not in topology")
                elif self.nodes[node.leader_id].role != DroneRole.LEADER:
                    errors.append(f"Drone {did}: leader_id={node.leader_id} is not a leader")
        return errors
