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

# Drone -> GCS
STATE_REPORT = "STATE_REPORT"
ALERT = "ALERT"

ALL_TYPES = {
    TAKEOFF_CMD, LAND_CMD, WAYPOINT_CMD, VELOCITY_CMD,
    FORMATION_CMD, STATE_REPORT, ALERT,
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
