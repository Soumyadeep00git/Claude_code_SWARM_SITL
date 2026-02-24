"""
Swarm SITL Stack - Global Configuration
All constants in one place. No logic.
"""

import os

# ── Swarm size ──────────────────────────────────────────────
NUM_DRONES = 5

# ── Home position (ArduPilot default: Canberra, Australia) ──
HOME_LAT = -35.3632620
HOME_LON = 149.1652370
HOME_ALT = 584
HOME_HEADING = 270

# ── SITL ports ──────────────────────────────────────────────
# Drone i connects via TCP to 127.0.0.1:(SITL_BASE_PORT + i * SITL_PORT_STEP)
# Drone IDs are 1-based: drone 1 on 5770, drone 2 on 5780, etc.
SITL_BASE_PORT = 5760
SITL_PORT_STEP = 10

# ── Swarm UDP ports ────────────────────────────────────────
# GCS listens on GCS_PORT
# Drone i listens on AGENT_BASE_PORT + i * AGENT_PORT_STEP
GCS_HOST = "127.0.0.1"
GCS_PORT = 15000
AGENT_BASE_PORT = 15100
AGENT_PORT_STEP = 10
ORCHESTRATOR_PORT = 14999  # Mission orchestrator (isolated demo mode)
WEB_GCS_PORT = 5000       # Flask-SocketIO web GCS

# ── Loop rates (Hz) ────────────────────────────────────────
AGENT_LOOP_HZ = 10
STATE_REPORT_HZ = 4
GCS_LOOP_HZ = 10

# ── Failsafe thresholds ───────────────────────────────────
SAFE_DISTANCE_M = 3.0         # Min distance between any two drones
SAFE_DISTANCE_CLEAR_M = 4.5   # Hysteresis: clear when > this
COMMS_TIMEOUT_S = 5.0         # Seconds before declaring comms lost
LOW_BATTERY_PCT = 20
PEER_STALE_TIMEOUT_S = 5.0           # Seconds before declaring a peer dead
COMMS_RECOVERY_TIMEOUT_S = 30.0      # Max time to rejoin after comms restored

# ── Formation defaults ─────────────────────────────────────
DEFAULT_SPACING_M = 5.0
DEFAULT_FORMATION = "LINE"

# ── ArduPilot paths ────────────────────────────────────────
ARDUPILOT_DIR = os.path.expanduser("~/ardupilot")

# ── Project paths ──────────────────────────────────────────
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(PROJECT_DIR, "logs")

# ── Process monitor ──────────────────────────────────
MONITOR_SAMPLE_HZ = 2  # Samples per second for CPU/memory monitoring
