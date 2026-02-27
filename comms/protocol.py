"""
Message protocol for GCS <-> Drone Agent communication.
All messages are JSON over UDP. MAVLink is never used on this layer.

Message format:
{
    "type": "<MSG_TYPE>",
    "src":  <int>,          # 0 = GCS, 1-N = drone ID
    "ts":   <float>,        # Unix timestamp
    "data": { ... }         # Payload (varies by type)
}
"""

import json
import time
import logging

log = logging.getLogger(__name__)

# ── Valid message types ─────────────────────────────────────
# GCS -> Drone
TAKEOFF_CMD = "TAKEOFF_CMD"
LAND_CMD = "LAND_CMD"
WAYPOINT_CMD = "WAYPOINT_CMD"
VELOCITY_CMD = "VELOCITY_CMD"
FORMATION_CMD = "FORMATION_CMD"
SWARM_WAYPOINT_CMD = "SWARM_WAYPOINT_CMD"

# Drone -> GCS
STATE_REPORT = "STATE_REPORT"
ALERT = "ALERT"

# Drone -> Drone (P2P mesh)
PEER_HEARTBEAT = "PEER_HEARTBEAT"

# Mesh routing
NEIGHBOR_AD = "NEIGHBOR_AD"
MESH_FORWARD = "MESH_FORWARD"
MESH_CONFIG_CMD = "MESH_CONFIG_CMD"

# Safety / collision avoidance
SAFETY_CONFIG_CMD = "SAFETY_CONFIG_CMD"    # GCS -> Drone: update isolation radius
PROXIMITY_ALERT = "PROXIMITY_ALERT"        # Drone -> Drones: cooperative collision alarm

# RL controller
RL_MODE_CMD = "RL_MODE_CMD"                # GCS -> Drone: toggle RL controller

# Slot negotiation (Drone <-> Drone, P2P + GCS relay)
SLOT_BID = "SLOT_BID"                      # "I want slot X" (distance + nonce)
SLOT_TIEBREAK = "SLOT_TIEBREAK"            # "My nonce is N for contested slot"
SLOT_CONFIRM = "SLOT_CONFIRM"              # "I own slot X (final)"

ALL_TYPES = {
    TAKEOFF_CMD, LAND_CMD, WAYPOINT_CMD, VELOCITY_CMD,
    FORMATION_CMD, SWARM_WAYPOINT_CMD, STATE_REPORT, ALERT,
    PEER_HEARTBEAT, NEIGHBOR_AD, MESH_FORWARD, MESH_CONFIG_CMD,
    SAFETY_CONFIG_CMD, PROXIMITY_ALERT, RL_MODE_CMD,
    SLOT_BID, SLOT_TIEBREAK, SLOT_CONFIRM,
}


def make_msg(msg_type: str, src: int, data: dict) -> bytes:
    """Build a message and serialize to UTF-8 JSON bytes."""
    msg = {
        "type": msg_type,
        "src": src,
        "ts": time.time(),
        "data": data,
    }
    return json.dumps(msg).encode("utf-8")


def encode_msg(msg: dict) -> bytes:
    """Serialize an existing message dict to UTF-8 JSON bytes."""
    return json.dumps(msg).encode("utf-8")


def parse_msg(raw: bytes) -> dict | None:
    """Deserialize bytes to a message dict. Returns None on bad data."""
    try:
        msg = json.loads(raw.decode("utf-8"))
        if {"type", "src", "data"}.issubset(msg):
            return msg
        log.warning("Message missing required fields: %s", msg)
        return None
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        log.warning("Failed to parse message: %s", e)
        return None
