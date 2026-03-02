"""Swarm topology and per-drone configuration library.

Core API:
    SwarmTopology.from_yaml()  — load hierarchy definition
    load_drone_config()        — per-drone runtime config from hierarchy + env
"""

from .hierarchy import SwarmTopology, DroneNode, DroneRole
from .swarm_config import DroneConfig, load_drone_config
