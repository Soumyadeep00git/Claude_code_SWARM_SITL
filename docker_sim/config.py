"""Environment-driven configuration — backed by swarm hierarchy.

All existing import names still work (DRONE_ID, SITL_PORT, etc.).
Internally delegates to swarm_lib.swarm_config.load_drone_config().
"""

import os

from swarm_lib.swarm_config import load_drone_config, DroneConfig

_cfg: DroneConfig = load_drone_config()

# ── Drone identity ───────────────────────────────────────────────
DRONE_ID = _cfg.drone_id
DRONE_ROLE = _cfg.role.value                        # "leader" or "follower"

# ── Peer networking (multi-peer from hierarchy) ──────────────────
PEER_TARGETS = _cfg.peer_targets                    # {peer_id: (host, port)}
PEER_IDS = _cfg.peer_ids
PEER_LISTEN_PORT = _cfg.peer_listen_port
PEER_STALE_TIMEOUT = _cfg.peer_stale_timeout

# ── GCS networking ───────────────────────────────────────────────
GCS_HOST = _cfg.gcs_host
GCS_TELEM_PORT = _cfg.gcs_telem_port
GCS_CMD_PORT = _cfg.gcs_cmd_port

# ── ArduPilot SITL ───────────────────────────────────────────────
ARDUCOPTER_BIN = os.environ.get(
    "ARDUCOPTER_BIN",
    "/home/ardupilot/ardupilot/build/sitl/bin/arducopter")
COPTER_DEFAULTS = os.environ.get(
    "COPTER_DEFAULTS",
    "/home/ardupilot/ardupilot/Tools/autotest/default_params/copter.parm")
SITL_BASE_PORT = 5760
SITL_PORT_STEP = 10
SITL_PORT = _cfg.sitl_port
MP_PORT = _cfg.mp_port
SYSID = _cfg.sysid

# ── Home position ────────────────────────────────────────────────
HOME_LAT = _cfg.base_home_lat
HOME_LON = _cfg.base_home_lon
HOME_ALT = _cfg.home_alt
HOME_HEADING = 270
MY_HOME_LAT = _cfg.home_lat
MY_HOME_LON = _cfg.home_lon
HOME_STR = f"{MY_HOME_LAT},{MY_HOME_LON},{HOME_ALT},{HOME_HEADING}"

# ── Flight parameters ────────────────────────────────────────────
TAKEOFF_ALT_M = _cfg.takeoff_alt_m
TAKEOFF_COMPLETE_FRAC = 0.90
CONTROL_HZ = _cfg.control_hz

# ── Follower offset (NED meters from leader, from hierarchy) ────
FOLLOW_OFFSET_N = _cfg.offset_n
FOLLOW_OFFSET_E = _cfg.offset_e
FOLLOW_OFFSET_D = _cfg.offset_d

# ── Leader identity (for followers) ─────────────────────────────
LEADER_ID = _cfg.leader_id

# ── Leader mission (unchanged) ──────────────────────────────────
HOVER_TIME_S = 20.0
LEADER_SPEED_MS = 1.5
MISSION_NORTH_M = 20.0
MISSION_EAST_M = 20.0
MOVE_ARRIVAL_M = 2.0
MOVE_TIMEOUT_S = 30.0

# ── Erratic leader behavior (unchanged) ─────────────────────────
ERRATIC_SPRINT_SPEED = 4.0
ERRATIC_ZIGZAG_PERIOD = 2.0
ERRATIC_ZIGZAG_SPEED = 2.5
ERRATIC_CHARGE_SPEED = 2.0
ERRATIC_PHASE_TIME = 10.0

# ── Feedforward ──────────────────────────────────────────────────
FEEDFORWARD_GAIN = 0.8

# ── Derived ──────────────────────────────────────────────────────
METERS_PER_DEG_LAT = 111320.0

# ── Directories ──────────────────────────────────────────────────
WORK_DIR = os.environ.get("WORK_DIR", f"/tmp/sitl_instance_{DRONE_ID}")
OUTPUT_DIR = os.environ.get("OUTPUT_DIR", "/app/output")

# ── Full config for swarm-aware code ─────────────────────────────
SWARM_CONFIG = _cfg

# ── Backward compat: HOME_SPACING_M ─────────────────────────────
HOME_SPACING_M = float(os.environ.get("HOME_SPACING_M", "1.0"))
