"""Load drone config from hierarchy.yaml for Orin hardware deployment.

Reads hierarchy.yaml for topology (roles, offsets, peer IDs).
Networking is handled differently than Docker sim:
  - Peer telemetry: RFD900x serial (no IP addresses needed)
  - GCS communication: WiFi UDP (host/port from ROS2 params)
  - FCU connection: MAVROS2 serial (handled by launch file)
"""

import math
import yaml
from dataclasses import dataclass, field
from typing import Optional


def gcs_telem_port(drone_id: int) -> int:
    """Port GCS listens on for this drone's telemetry (same scheme as Docker sim)."""
    return 14570 + drone_id


def gcs_cmd_port(drone_id: int) -> int:
    """Port this drone listens on for GCS commands (same scheme as Docker sim)."""
    return 14580 + drone_id


@dataclass
class HardwareConfig:
    """Runtime configuration for one drone on Orin hardware."""
    drone_id: int = 1
    role: str = "leader"

    # Hierarchy relationships
    leader_id: Optional[int] = None
    follower_ids: tuple = ()
    peer_ids: list = field(default_factory=list)

    # Formation offset (NED meters from leader, only for followers)
    offset_n: float = 0.0
    offset_e: float = 0.0
    offset_d: float = 0.0

    # Home position
    home_lat: float = 0.0
    home_lon: float = 0.0
    base_home_lat: float = 0.0
    base_home_lon: float = 0.0

    # GCS networking (WiFi UDP)
    gcs_host: str = ""
    gcs_telem_port: int = 0
    gcs_cmd_port: int = 0

    # Flight parameters
    takeoff_alt_m: float = 10.0
    control_hz: int = 10
    peer_stale_timeout: float = 5.0


def load_hardware_config(
    hierarchy_path: str,
    drone_id: int,
    gcs_host: str = "",
    takeoff_alt_m: float = 10.0,
    control_hz: int = 10,
    peer_stale_timeout: float = 5.0,
) -> HardwareConfig:
    """Load config for this drone from hierarchy.yaml + ROS2 params.

    Args:
        hierarchy_path: Path to hierarchy.yaml.
        drone_id: This drone's ID.
        gcs_host: GCS computer IP address (WiFi).
        takeoff_alt_m: Target takeoff altitude.
        control_hz: Control loop frequency.
        peer_stale_timeout: Seconds before peer state is considered stale.
    """
    with open(hierarchy_path, 'r') as f:
        data = yaml.safe_load(f)

    base_lat = data['home']['lat']
    base_lon = data['home']['lon']
    drones = data['drones']

    if drone_id not in drones:
        raise ValueError(f"Drone {drone_id} not found in hierarchy.yaml")

    node = drones[drone_id]
    role = node['role']

    # Extract relationships
    leader_id = node.get('leader_id')
    offset = node.get('offset', [0.0, 0.0, 0.0])
    home_spacing_m = node.get('home_spacing_m', 0.0)

    # Find followers (drones that list this drone as their leader)
    follower_ids = tuple(
        did for did, d in drones.items()
        if d.get('leader_id') == drone_id
    )

    # Compute peers: leader sees followers, follower sees leader + siblings
    peer_ids = []
    if role == 'leader':
        peer_ids = list(follower_ids)
    else:
        if leader_id is not None:
            peer_ids.append(leader_id)
        for did, d in drones.items():
            if did != drone_id and d.get('leader_id') == leader_id:
                peer_ids.append(did)

    # Compute home position (offset east from base)
    METERS_PER_DEG_LAT = 111320.0
    cos_lat = math.cos(math.radians(base_lat))
    home_lon_offset = home_spacing_m / (METERS_PER_DEG_LAT * cos_lat)
    home_lat = base_lat
    home_lon = base_lon + home_lon_offset

    return HardwareConfig(
        drone_id=drone_id,
        role=role,
        leader_id=leader_id,
        follower_ids=follower_ids,
        peer_ids=peer_ids,
        offset_n=offset[0] if len(offset) > 0 else 0.0,
        offset_e=offset[1] if len(offset) > 1 else 0.0,
        offset_d=offset[2] if len(offset) > 2 else 0.0,
        home_lat=home_lat,
        home_lon=home_lon,
        base_home_lat=base_lat,
        base_home_lon=base_lon,
        gcs_host=gcs_host,
        gcs_telem_port=gcs_telem_port(drone_id),
        gcs_cmd_port=gcs_cmd_port(drone_id),
        takeoff_alt_m=takeoff_alt_m,
        control_hz=control_hz,
        peer_stale_timeout=peer_stale_timeout,
    )
