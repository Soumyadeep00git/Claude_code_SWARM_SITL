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
SAFE_DISTANCE_M = 2.5         # Legacy hard failsafe (used as floor)
SAFE_DISTANCE_CLEAR_M = 4.5   # Legacy hysteresis clear
ISOLATION_RADIUS_M = 5.0      # Hard no-cross boundary (configurable from UI)
ISOLATION_SOFT_ZONE_M = 8.0   # Soft repulsion starts here (> isolation)
COMMS_TIMEOUT_S = 5.0         # Seconds before declaring comms lost
LOW_BATTERY_PCT = 0               # 0 = disabled (SITL battery is simulated)
PEER_STALE_TIMEOUT_S = 5.0           # Seconds before declaring a peer dead
COMMS_RECOVERY_TIMEOUT_S = 30.0      # Max time to rejoin after comms restored
GHOST_PRUNE_TIMEOUT_S = 15.0         # Remove drones silent > 3× comms timeout

# ── MPPI Controller ──────────────────────────────────────
MPPI_CONFIG = {
    "K_SAMPLES": 256,                   # Rollout samples (power of 2)
    "HORIZON_STEPS": 15,                # Prediction steps (15 × 0.1s = 1.5s lookahead)
    "DT_S": 0.1,                        # Time step (matches AGENT_LOOP_HZ)
    "LAMBDA_TEMP": 5.0,                 # Temperature (lower = more exploitation)
    "SIGMA_NOISE": [2.0, 2.0, 0.3],    # Noise std [N, E, D] m/s²
    "MAX_VELOCITY_MS": 3.0,             # Max horizontal speed
    "MAX_ACCEL_MS2": 2.5,               # Max acceleration
    "W_FORMATION": 20.0,                # Formation slot tracking — dominant weight
    "W_COLLISION": 80.0,                # Collision avoidance — must be < formation at spacing distance
    "W_EFFORT": 0.05,                   # Control effort weight
    "W_CONNECTIVITY": 3.0,              # Network connectivity weight
    "W_SMOOTHNESS": 0.8,                # Jerk penalty weight
    "W_TIME_PRESSURE": 2.0,             # Time urgency weight
    "W_SPEED_INCENTIVE": 1.5,           # Speed incentive
    "SAFE_RADIUS_M": 3.0,               # Soft avoidance — MUST be < spacing (5m)
    "COLLISION_RADIUS_M": 2.0,          # Hard penalty — below failsafe threshold (2.5m)
    "COMM_RANGE_M": 50.0,               # Connectivity penalty threshold
}

# ── Hybrid A* Path Planner ───────────────────────────────
ASTAR_CONFIG = {
    "GRID_CELL_M": 2.0,                # Grid resolution (matches GPS accuracy)
    "OBSTACLE_RADIUS_M": 3.0,          # Clearance around peers — must be < spacing (5m)
    "GRID_EXTENT_M": 80.0,             # Half-width of planning grid
    "NUM_HEADINGS": 8,                  # Heading discretization
    "STEP_LENGTH_M": 3.0,              # Distance per A* expansion step
    "MAX_NODES": 2000,                  # Node budget (caps compute time)
}
ASTAR_REPLAN_INTERVAL_S = 1.0          # Replan global path every 1 second

# ── P2P Mesh ──────────────────────────────────────────────
P2P_ENABLED = True                    # Drones send state directly to peers

# ── Mesh Network Simulation ──────────────────────────────
MESH_SIM_ENABLED = True               # Use SimulatedUDPNode with network effects
MESH_SIM_RANGE_M = 100.0             # Simulated radio range (meters)
MESH_SIM_BANDWIDTH_BPS = 250_000     # 250 kbps simulated radio

# ── RL Controller ────────────────────────────────────────────
RL_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "models", "rl_policy.onnx")
RL_MAX_PEERS = 4            # Max peer slots in observation vector
RL_OBS_DIM = 3 + 3 + 3 + RL_MAX_PEERS * 3  # own_pos + own_vel + goal + peers = 21
RL_MAX_SPEED = 3.0          # Clamp RL output velocity (m/s)

# ── Slot Negotiation ────────────────────────────────────
SLOT_NEG_CORNER_PHASE_MS = 100       # Phase 0: corner bid duration
SLOT_NEG_MIDDLE_PHASE_MS = 150       # Phase 1: middle bid duration
SLOT_NEG_TIEBREAK_PHASE_MS = 100     # Phase 2: tiebreak duration
SLOT_NEG_CONFIRM_PHASE_MS = 150      # Phase 3: confirm duration
SLOT_NEG_EQUIDISTANT_M = 0.5        # Distance difference below = equidistant
SLOT_NEG_REDUNDANT_SENDS = 3        # Broadcasts per phase for loss tolerance

# ── Formation defaults ─────────────────────────────────────
DEFAULT_SPACING_M = 5.0
DEFAULT_FORMATION = "LINE"

# ── ArduPilot paths ────────────────────────────────────────
ARDUPILOT_DIR = os.environ.get("ARDUPILOT_DIR", os.path.expanduser("~/ardupilot"))

# ── Project paths ──────────────────────────────────────────
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(PROJECT_DIR, "logs")

# ── Process monitor ──────────────────────────────────
MONITOR_SAMPLE_HZ = 2  # Samples per second for CPU/memory monitoring

# ── Docker mode ──────────────────────────────────────
# Auto-detect: if running inside a container, drones are managed by
# docker-compose (separate containers), not by DroneManager subprocesses.
DOCKER_MODE = os.path.exists("/.dockerenv")
