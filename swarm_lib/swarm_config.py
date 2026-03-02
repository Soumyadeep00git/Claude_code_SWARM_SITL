"""Per-drone configuration derived from swarm hierarchy.

Each drone container receives DRONE_ID via env. This module loads the
hierarchy YAML and extracts that drone's role, leader, peers, offsets,
and networking ports.

Port assignment scheme (deterministic from drone_id):
    SITL port:       5760 + drone_id * 10
    Mission Planner: SITL port + 3
    Peer listen:     14560 + drone_id      (drone listens for peer telemetry)
    GCS telem:       14570 + drone_id      (GCS listens for this drone)
    GCS cmd:         14580 + drone_id      (drone listens for GCS commands)
    Container name:  drone-{drone_id}
"""

import math
import os
from dataclasses import dataclass, field
from typing import Optional

from .hierarchy import SwarmTopology, DroneRole


# ── Port scheme ─────────────────────────────────────────────────

def peer_listen_port(drone_id: int) -> int:
    """Port this drone listens on for peer telemetry."""
    return 14560 + drone_id


def gcs_telem_port(drone_id: int) -> int:
    """Port GCS listens on for this drone's telemetry."""
    return 14570 + drone_id


def gcs_cmd_port(drone_id: int) -> int:
    """Port this drone listens on for GCS commands."""
    return 14580 + drone_id


def container_name(drone_id: int) -> str:
    """Docker container hostname for a drone."""
    return f"drone-{drone_id}"


def sitl_port(drone_id: int) -> int:
    return 5760 + drone_id * 10


def mp_port(drone_id: int) -> int:
    """Mission Planner TCP port (uartD offset from uartA)."""
    return sitl_port(drone_id) + 3


# ── DroneConfig ─────────────────────────────────────────────────

@dataclass
class DroneConfig:
    """Complete runtime configuration for one drone."""
    # Identity
    drone_id: int = 1
    role: DroneRole = DroneRole.LEADER

    # Hierarchy relationships
    leader_id: Optional[int] = None
    follower_ids: tuple = ()
    peer_ids: list = field(default_factory=list)

    # Formation offset (NED meters from leader, only for followers)
    offset_n: float = 0.0
    offset_e: float = 0.0
    offset_d: float = 0.0

    # Home position (unique per drone)
    home_lat: float = -35.3632620
    home_lon: float = 149.1652370
    home_alt: int = 584

    # Networking
    sitl_port: int = 5770
    mp_port: int = 5773
    sysid: int = 2
    peer_listen_port: int = 14561
    gcs_telem_port: int = 14571
    gcs_cmd_port: int = 14581
    peer_targets: dict = field(default_factory=dict)  # peer_id -> (hostname, port)
    gcs_host: str = "gcs"

    # Flight parameters
    takeoff_alt_m: float = 10.0
    control_hz: int = 10
    peer_stale_timeout: float = 5.0

    # Base home (for GCS geofence reference)
    base_home_lat: float = -35.3632620
    base_home_lon: float = 149.1652370


def load_drone_config(
    hierarchy_path: str = None,
    drone_id: int = None,
) -> DroneConfig:
    """Load config for the current drone from hierarchy + environment.

    Reads DRONE_ID from env if not provided.
    Reads HIERARCHY_PATH from env if not provided (default: /app/hierarchy.yaml).
    """
    if drone_id is None:
        drone_id = int(os.environ.get("DRONE_ID", "1"))
    if hierarchy_path is None:
        hierarchy_path = os.environ.get("HIERARCHY_PATH", "/app/hierarchy.yaml")

    # If hierarchy file doesn't exist, fall back to env-based config
    # (backward compat for running without hierarchy.yaml)
    if not os.path.exists(hierarchy_path):
        return _fallback_config(drone_id)

    topo = SwarmTopology.from_yaml(hierarchy_path)
    errors = topo.validate()
    if errors:
        raise ValueError(f"Hierarchy validation failed: {errors}")

    node = topo.get_node(drone_id)
    peers = topo.get_peers(drone_id)

    # Compute per-drone home position (offset east from base)
    METERS_PER_DEG_LAT = 111320.0
    cos_lat = math.cos(math.radians(topo.base_home_lat))
    home_lon_offset = node.home_spacing_m / (METERS_PER_DEG_LAT * cos_lat)
    home_lat = topo.base_home_lat
    home_lon = topo.base_home_lon + home_lon_offset
    home_alt = int(os.environ.get("HOME_ALT", "584"))

    # Build peer target map: for each peer, where do I send my telemetry?
    peer_targets = {}
    for pid in peers:
        hostname = container_name(pid)
        port = peer_listen_port(pid)
        peer_targets[pid] = (hostname, port)

    gcs_host_env = os.environ.get("GCS_HOST", "gcs")

    return DroneConfig(
        drone_id=drone_id,
        role=node.role,
        leader_id=node.leader_id,
        follower_ids=node.follower_ids,
        peer_ids=peers,
        offset_n=node.offset_n,
        offset_e=node.offset_e,
        offset_d=node.offset_d,
        home_lat=home_lat,
        home_lon=home_lon,
        home_alt=home_alt,
        sitl_port=sitl_port(drone_id),
        mp_port=mp_port(drone_id),
        sysid=drone_id + 1,
        peer_listen_port=peer_listen_port(drone_id),
        gcs_telem_port=gcs_telem_port(drone_id),
        gcs_cmd_port=gcs_cmd_port(drone_id),
        peer_targets=peer_targets,
        gcs_host=gcs_host_env,
        takeoff_alt_m=float(os.environ.get("TAKEOFF_ALT_M", "10.0")),
        control_hz=int(os.environ.get("CONTROL_HZ", "10")),
        peer_stale_timeout=float(os.environ.get("PEER_STALE_TIMEOUT", "5.0")),
        base_home_lat=topo.base_home_lat,
        base_home_lon=topo.base_home_lon,
    )


def _fallback_config(drone_id: int) -> DroneConfig:
    """Backward-compat config from environment variables (no hierarchy file)."""
    role_str = os.environ.get("DRONE_ROLE", "leader")
    role = DroneRole(role_str)

    METERS_PER_DEG_LAT = 111320.0
    base_lat = float(os.environ.get("HOME_LAT", "-35.3632620"))
    base_lon = float(os.environ.get("HOME_LON", "149.1652370"))
    spacing = float(os.environ.get("HOME_SPACING_M", "1.0"))
    cos_lat = math.cos(math.radians(base_lat))
    home_lon_offset = (drone_id - 1) * spacing / (METERS_PER_DEG_LAT * cos_lat)

    # For fallback, reconstruct the old peer-to-peer setup
    peer_host = os.environ.get("PEER_HOST", "localhost")
    broadcast_port = int(os.environ.get("BROADCAST_PORT", "14560"))
    listen_port = int(os.environ.get("LISTEN_PORT", "14560"))

    # Guess peer drone_id (old setup: leader=1, follower=2)
    peer_id = 2 if drone_id == 1 else 1
    peer_targets = {peer_id: (peer_host, broadcast_port)}

    return DroneConfig(
        drone_id=drone_id,
        role=role,
        leader_id=None if role == DroneRole.LEADER else 1,
        follower_ids=(2,) if role == DroneRole.LEADER else (),
        peer_ids=[peer_id],
        offset_n=-5.0 if role == DroneRole.FOLLOWER else 0.0,
        offset_e=3.0 if role == DroneRole.FOLLOWER else 0.0,
        offset_d=0.0,
        home_lat=base_lat,
        home_lon=base_lon + home_lon_offset,
        home_alt=int(os.environ.get("HOME_ALT", "584")),
        sitl_port=sitl_port(drone_id),
        mp_port=mp_port(drone_id),
        sysid=drone_id + 1,
        peer_listen_port=listen_port,
        gcs_telem_port=int(os.environ.get("GCS_TELEM_PORT", "0")),
        gcs_cmd_port=int(os.environ.get("GCS_CMD_PORT", "0")),
        peer_targets=peer_targets,
        gcs_host=os.environ.get("GCS_HOST", "gcs"),
        takeoff_alt_m=float(os.environ.get("TAKEOFF_ALT_M", "10.0")),
        control_hz=int(os.environ.get("CONTROL_HZ", "10")),
        peer_stale_timeout=float(os.environ.get("PEER_STALE_TIMEOUT", "5.0")),
        base_home_lat=base_lat,
        base_home_lon=base_lon,
    )
