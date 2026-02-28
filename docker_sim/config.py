"""Environment-driven configuration for dockerized per-drone execution."""

import math
import os

# ── Drone identity (from env) ────────────────────────────────
DRONE_ID = int(os.environ.get("DRONE_ID", "1"))
DRONE_ROLE = os.environ.get("DRONE_ROLE", "leader")  # "leader" or "follower"

# ── Peer networking ──────────────────────────────────────────
PEER_HOST = os.environ.get("PEER_HOST", "localhost")
BROADCAST_PORT = int(os.environ.get("BROADCAST_PORT", "14560"))
LISTEN_PORT = int(os.environ.get("LISTEN_PORT", "14561"))

# ── GCS networking ──────────────────────────────────────────
GCS_HOST = os.environ.get("GCS_HOST", "gcs")
GCS_TELEM_PORT = int(os.environ.get("GCS_TELEM_PORT", "0"))  # 0 = disabled
GCS_CMD_PORT = int(os.environ.get("GCS_CMD_PORT", "0"))       # 0 = disabled

# ── ArduPilot SITL ──────────────────────────────────────────
ARDUCOPTER_BIN = os.environ.get(
    "ARDUCOPTER_BIN",
    "/home/ardupilot/ardupilot/build/sitl/bin/arducopter")
COPTER_DEFAULTS = os.environ.get(
    "COPTER_DEFAULTS",
    "/home/ardupilot/ardupilot/Tools/autotest/default_params/copter.parm")

SITL_BASE_PORT = 5760
SITL_PORT_STEP = 10

# ── Home position (Canberra default) ────────────────────────
HOME_LAT = float(os.environ.get("HOME_LAT", "-35.3632620"))
HOME_LON = float(os.environ.get("HOME_LON", "149.1652370"))
HOME_ALT = int(os.environ.get("HOME_ALT", "584"))
HOME_HEADING = 270
HOME_SPACING_M = float(os.environ.get("HOME_SPACING_M", "1.0"))

# ── Flight parameters ───────────────────────────────────────
TAKEOFF_ALT_M = 10.0
TAKEOFF_COMPLETE_FRAC = 0.80
CONTROL_HZ = 10

# ── Follower offset (NED meters from leader) ────────────────
FOLLOW_OFFSET_N = -5.0
FOLLOW_OFFSET_E = 3.0
FOLLOW_OFFSET_D = 0.0

# ── Peer networking (staleness) ────────────────────────────
PEER_STALE_TIMEOUT = float(os.environ.get("PEER_STALE_TIMEOUT", "5.0"))

# ── Feedforward ─────────────────────────────────────────────
FEEDFORWARD_GAIN = 0.8

# ── Leader mission ──────────────────────────────────────────
HOVER_TIME_S = 20.0
LEADER_SPEED_MS = 1.5
MISSION_NORTH_M = 20.0
MISSION_EAST_M = 20.0
MOVE_ARRIVAL_M = 2.0
MOVE_TIMEOUT_S = 30.0

# ── Erratic leader behavior ────────────────────────────────
ERRATIC_SPRINT_SPEED = 4.0
ERRATIC_ZIGZAG_PERIOD = 2.0
ERRATIC_ZIGZAG_SPEED = 2.5
ERRATIC_CHARGE_SPEED = 2.0
ERRATIC_PHASE_TIME = 10.0

# ── Derived ─────────────────────────────────────────────────
METERS_PER_DEG_LAT = 111320.0
SYSID = DRONE_ID + 1
SITL_PORT = SITL_BASE_PORT + DRONE_ID * SITL_PORT_STEP

# Compute per-drone home position (follower offset east)
_cos_lat = math.cos(math.radians(HOME_LAT))
HOME_LON_OFFSET = (DRONE_ID - 1) * HOME_SPACING_M / (METERS_PER_DEG_LAT * _cos_lat)
MY_HOME_LAT = HOME_LAT
MY_HOME_LON = HOME_LON + HOME_LON_OFFSET
HOME_STR = f"{MY_HOME_LAT},{MY_HOME_LON},{HOME_ALT},{HOME_HEADING}"

# ── Directories ─────────────────────────────────────────────
WORK_DIR = os.environ.get("WORK_DIR", f"/tmp/sitl_instance_{DRONE_ID}")
OUTPUT_DIR = os.environ.get("OUTPUT_DIR", "/app/output")
