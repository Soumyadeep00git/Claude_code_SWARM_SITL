"""Drone 1 — per-drone configuration derived from shared config."""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from math import cos, radians
from config import (
    SITL_BASE_PORT, SITL_PORT_STEP,
    AGENT_BASE_PORT, AGENT_PORT_STEP,
    HOME_LAT, HOME_LON, HOME_ALT, HOME_HEADING,
)

DRONE_ID = 1

SITL_PORT = SITL_BASE_PORT + DRONE_ID * SITL_PORT_STEP
AGENT_PORT = AGENT_BASE_PORT + DRONE_ID * AGENT_PORT_STEP

_offset_east = (DRONE_ID - 1) * 10.0  # 10m spacing between home positions
DRONE_HOME_LAT = HOME_LAT
DRONE_HOME_LON = HOME_LON + (_offset_east / (111320.0 * cos(radians(HOME_LAT))))
DRONE_HOME_ALT = HOME_ALT
DRONE_HOME_HEADING = HOME_HEADING
DRONE_HOME = f"{DRONE_HOME_LAT},{DRONE_HOME_LON},{DRONE_HOME_ALT},{DRONE_HOME_HEADING}"
