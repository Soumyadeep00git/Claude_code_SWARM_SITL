"""All constants for the SITL simulation test."""

import os

# ── ArduPilot paths ───────────────────────────────────────────
ARDUPILOT_DIR = os.environ.get(
    "ARDUPILOT_DIR", os.path.expanduser("~/ardupilot"))
ARDUCOPTER_BIN = os.path.join(
    ARDUPILOT_DIR, "build", "sitl", "bin", "arducopter")
COPTER_DEFAULTS = os.path.join(
    ARDUPILOT_DIR, "Tools", "autotest", "default_params", "copter.parm")

# ── Home position (Canberra — ArduPilot default) ─────────────
HOME_LAT = -35.3632620
HOME_LON = 149.1652370
HOME_ALT = 584
HOME_HEADING = 270

# ── SITL instance config ─────────────────────────────────────
# Port formula: 5760 + drone_id * 10
SITL_BASE_PORT = 5760
SITL_PORT_STEP = 10
LEADER_ID = 1   # → port 5770, sysid 2
FOLLOWER_ID = 2  # → port 5780, sysid 3

# Spacing between home positions (meters east)
HOME_SPACING_M = 1.0

# ── Flight parameters ────────────────────────────────────────
TAKEOFF_ALT_M = 10.0
TAKEOFF_COMPLETE_FRAC = 0.80

# ── Follower offset (NED meters from leader) ─────────────────
FOLLOW_OFFSET_N = -5.0   # 5m behind (south)
FOLLOW_OFFSET_E = 3.0    # 3m east
FOLLOW_OFFSET_D = 0.0    # Same altitude

# ── Feedforward ──────────────────────────────────────────────
FEEDFORWARD_GAIN = 0.8

# ── Control rate ─────────────────────────────────────────────
CONTROL_HZ = 10

# ── Leader mission ───────────────────────────────────────────
HOVER_TIME_S = 20.0
LEADER_SPEED_MS = 1.5
MISSION_NORTH_M = 20.0
MISSION_EAST_M = 20.0
MOVE_ARRIVAL_M = 2.0     # Close enough to waypoint
MOVE_TIMEOUT_S = 30.0

# ── Erratic leader behavior ─────────────────────────────────
ERRATIC_SPRINT_SPEED = 4.0   # m/s — exceeds follower max_speed (3.0)
ERRATIC_ZIGZAG_PERIOD = 2.0  # seconds per zig or zag
ERRATIC_ZIGZAG_SPEED = 2.5   # m/s lateral
ERRATIC_CHARGE_SPEED = 2.0   # m/s toward follower
ERRATIC_PHASE_TIME = 10.0    # seconds per erratic phase

# ── Directories ──────────────────────────────────────────────
SIM_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SIM_DIR)
LOG_DIR = os.path.join(SIM_DIR, "logs")
OUTPUT_DIR = os.path.join(SIM_DIR, "output")

# Flat-earth constant (shared with geo_utils.py)
METERS_PER_DEG_LAT = 111320.0
