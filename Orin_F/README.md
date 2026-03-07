# Orin Swarm — Leader-Follower Drone System

Hardware deployment package for a 2-drone leader-follower swarm running on
**NVIDIA Jetson Orin AGX + CubeOrange+ flight controllers**.

Self-contained: all ROS2 packages, Python libraries, message definitions,
launch files, and configs are in this folder. No external dependencies beyond
ROS2 Humble and MAVROS2.

---

## Hardware Setup

### Per drone

| Component | Connection | Linux device |
|---|---|---|
| CubeOrange+ FC | TELEM2 → USB-TTL adapter → Orin USB | `/dev/ttyUSB0` @ 921600 baud |
| RFD900x radio | USB → Orin USB | `/dev/ttyUSB1` @ 57600 baud |
| WiFi | Orin built-in or USB adapter | `wlan0` → GCS laptop |
| RC receiver | Directly to CubeOrange+ RC input | (no Jetson involvement) |

### Roles

- **Leader drone**: RC pilot flies directly via transmitter. Jetson is passive —
  reads FC position via MAVROS2, relays to follower over RFD900x, relays to GCS
  over WiFi. Only handles emergency commands (KILL/RTL/LAND).

- **Follower drone**: Safety pilot has RC override. When FC is in **GUIDED mode**,
  Jetson runs guidance and sends velocity commands. When safety pilot switches to
  STABILIZE/LOITER/any other mode, Jetson immediately stops sending commands.

### Wiring diagram

```
Leader Drone                              Follower Drone
┌─────────────────────┐                   ┌─────────────────────┐
│  RC Transmitter     │                   │  RC Transmitter     │
│        │            │                   │        │            │
│        ▼            │                   │        ▼            │
│  CubeOrange+ FC     │                   │  CubeOrange+ FC     │
│   │ TELEM2          │                   │   │ TELEM2   ▲      │
│   │                 │                   │   │          │      │
│   ▼ USB-TTL         │                   │   ▼ USB-TTL  │vel   │
│  Jetson Orin AGX    │   RFD900x radio   │  Jetson Orin AGX    │
│   │ /dev/ttyUSB0    │◄────────────────►│   │ /dev/ttyUSB0    │
│   │ (MAVROS2)       │   /dev/ttyUSB1    │   │ (MAVROS2)       │
│   │                 │                   │   │                 │
│   └─── WiFi ─────── │ ──────────────── │ ──└─── WiFi ────────│
└─────────────────────┘                   └─────────────────────┘
          │                                         │
          └───────── UDP to GCS laptop ─────────────┘
               port 14571 (leader telem)
               port 14572 (follower telem)
               port 14581 (leader cmds)
               port 14582 (follower cmds)
```

---

## Quick Start

### 1. Install (one-time, on each Jetson)

```bash
# Copy the entire Orin/ folder to the Jetson
scp -r Orin/ orin@<jetson-ip>:~/Orin/

# SSH into the Jetson
ssh orin@<jetson-ip>

# Run setup
cd ~/Orin
chmod +x install.sh
./install.sh leader    # on Leader Jetson
./install.sh follower  # on Follower Jetson
```

The install script:
- Verifies ROS2 Humble is installed
- Installs MAVROS2, pymavlink, pyyaml
- Downloads GeographicLib datasets
- Creates `~/swarm_ws/` workspace with symlinks
- Copies `hierarchy.yaml` to `~/hierarchy.yaml`
- Adds PYTHONPATH and source lines to `~/.bashrc`
- Builds all packages with `colcon build`

### 2. Configure

```bash
# Edit topology — set your GPS home position
nano ~/hierarchy.yaml

# Edit params — set GCS laptop IP, verify serial devices
nano ~/swarm_ws/src/Leader/config/params.yaml    # Leader
nano ~/swarm_ws/src/Follower/config/params.yaml  # Follower

# Verify serial ports
ls /dev/ttyUSB*
# Expect: /dev/ttyUSB0 (CubeOrange+), /dev/ttyUSB1 (RFD900x)
```

**hierarchy.yaml** (edit home lat/lon for your field):
```yaml
home:
  lat: -35.3632620     # <-- your field GPS latitude
  lon: 149.1652370     # <-- your field GPS longitude

drones:
  1:
    role: leader
    home_spacing_m: 0
  2:
    role: follower
    leader_id: 1
    offset: [-5.0, 3.0, 0.0]   # 5m behind, 3m right (NED)
    home_spacing_m: 20
```

**params.yaml** key fields:
```yaml
swarm_bridge_node:
  ros__parameters:
    drone_id: 1               # 1=Leader, 2=Follower
    gcs_host: "192.168.1.100" # GCS laptop WiFi IP
    radio_device: "/dev/ttyUSB1"
    radio_baud: 57600
```

### 3. Launch

```bash
# Terminal on Leader Jetson:
ros2 launch leader_pkg leader_bringup.launch.py

# Terminal on Follower Jetson:
ros2 launch follower_pkg follower_bringup.launch.py
```

Override serial device if needed:
```bash
ros2 launch leader_pkg leader_bringup.launch.py fcu_url:=/dev/ttyTHS1:921600
```

### 4. Verify

```bash
# Check FC connection
ros2 topic echo /mavros/state --once

# Check battery
ros2 topic echo /mavros/battery --once

# Check peer data + radio RSSI
ros2 topic echo /swarm/peer_states --once

# Check GCS commands arriving
ros2 topic echo /swarm/gcs_command
```

---

## Directory Structure

```
Orin/
├── README.md                  # This file
├── install.sh                 # One-shot setup script
├── hierarchy.yaml             # Swarm topology config (copy to ~/hierarchy.yaml)
│
├── guidance_lib/              # Pure Python guidance library (bundled)
│   ├── __init__.py
│   ├── guidance.py            # 3-mode guidance: TRACKING/CATCHUP/EVASION
│   ├── collision_avoidance.py # 3D collision avoidance between drones
│   ├── command_smoother.py    # EMA output filter with hover deadband
│   ├── geo_utils.py           # GPS <-> NED conversions
│   ├── mode_selector.py       # Priority mode switching logic
│   ├── output_safety.py       # Speed cap, rate limiter, NaN guard
│   ├── target.py              # Leader + offset + feedforward target
│   ├── velocity.py            # Per-mode velocity computation
│   └── config.yaml            # Guidance tuning parameters
│
├── failsafe_lib/              # Pure Python failsafe library (bundled)
│   ├── __init__.py
│   ├── failsafe.py            # 6-check safety monitor (geofence, GPS, stale, etc.)
│   └── config.yaml            # Failsafe thresholds
│
├── swarm_msgs/                # Custom ROS2 message definitions
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/
│       ├── PeerState.msg      # Single peer: pos, vel, heading, FC state, battery
│       ├── PeerStates.msg     # Array + own battery + radio quality
│       └── SwarmCommand.msg   # GCS commands (RTL, LAND, KILL, WASD, WP, etc.)
│
├── Leader/                    # Leader drone ROS2 package
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── config/params.yaml     # ROS2 parameters (drone_id, GCS IP, serial ports)
│   ├── launch/
│   │   └── leader_bringup.launch.py   # Launches MAVROS2 + bridge + control
│   ├── leader_pkg/
│   │   ├── __init__.py
│   │   ├── config_loader.py           # Loads hierarchy.yaml → HardwareConfig
│   │   ├── leader_control_node.py     # Emergency only (KILL/RTL/LAND)
│   │   └── swarm_bridge_node.py       # RFD900x + WiFi GCS bridge (4 threads)
│   └── resource/leader_pkg
│
└── Follower/                  # Follower drone ROS2 package
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── config/params.yaml
    ├── launch/
    │   └── follower_bringup.launch.py
    ├── follower_pkg/
    │   ├── __init__.py
    │   ├── config_loader.py
    │   ├── follower_control_node.py   # Guidance + GUIDED gate (718 lines)
    │   └── swarm_bridge_node.py       # Identical to Leader's bridge
    └── resource/follower_pkg
```

---

## Architecture — Nodes Per Drone

Each Jetson runs 3 ROS2 nodes launched by a single launch file:

### Node 1: mavros_node (system)

Translates MAVLink serial <-> ROS2 topics. Connects to CubeOrange+ FC via
`/dev/ttyUSB0:921600` (TELEM2 port). Publishes:

| ROS2 Topic | Message Type | Source MAVLink | Rate |
|---|---|---|---|
| `/mavros/state` | `State` | HEARTBEAT (ID 0) | 1 Hz |
| `/mavros/battery` | `BatteryState` | SYS_STATUS (ID 1) | 1 Hz |
| `/mavros/global_position/global` | `NavSatFix` | GLOBAL_POSITION_INT (ID 33) | 10 Hz |
| `/mavros/global_position/rel_alt` | `Float64` | GLOBAL_POSITION_INT (ID 33) | 10 Hz |
| `/mavros/local_position/velocity_local` | `TwistStamped` | GLOBAL_POSITION_INT (ID 33) | 10 Hz |
| `/mavros/global_position/compass_hdg` | `Float64` | GLOBAL_POSITION_INT (ID 33) | 10 Hz |

### Node 2: swarm_bridge_node (identical on both drones)

Communication hub — 4 background threads:

| Thread | Direction | Link | Rate | MAVLink Messages |
|---|---|---|---|---|
| Radio TX | Jetson → peer | RFD900x serial | 10 Hz pos, 1 Hz health | GLOBAL_POSITION_INT, NAMED_VALUE_INT/gmode, HEARTBEAT, NAMED_VALUE_FLOAT/battery |
| Radio RX | Peer → Jetson | RFD900x serial | continuous | Same as TX + RADIO_STATUS (ID 109) from modem |
| GCS TX | Jetson → GCS | WiFi UDP | 10 Hz | Custom `!i21d` packet (172 bytes) |
| GCS CMD RX | GCS → Jetson | WiFi UDP | on-demand | Variable-length command packets |

ROS2 subscriptions: `/mavros/state`, `/mavros/battery`, `/mavros/global_position/*`,
`/mavros/local_position/velocity_local`, `/mavros/global_position/compass_hdg`,
`/swarm/guidance_mode`

ROS2 publishers: `/swarm/peer_states` (PeerStates), `/swarm/gcs_command` (SwarmCommand)

### Node 3: Control node (role-specific)

**Leader** (`leader_control_node.py` — 148 lines):
- Purely event-driven, no control loop
- Subscribes: `/mavros/state`, `/swarm/gcs_command`
- Handles only: KILL (force disarm), RTL (set mode), LAND (set mode)
- Ignores: TAKEOFF, WASD, HOVER, FOLLOW (pilot has RC)

**Follower** (`follower_control_node.py` — 718 lines):
- 10 Hz control loop with GUIDED mode gate
- Mission state machine: IDLE → TAKEOFF → HOVER → FOLLOW → GEOFENCE_RETURN → RTL/LAND/KILL
- Uses `guidance_lib` for 3-mode tracking (TRACKING/CATCHUP/EVASION)
- Uses `failsafe_lib` for safety (geofence, GPS loss, leader stale, altitude ceiling)
- Publishes velocity commands to `/mavros/setpoint_raw/local` (MAVLink ID 84)
- **Only when FC is in GUIDED mode** — safety pilot override via RC at any time

---

## MAVLink Message Trace

### HEARTBEAT (ID 0) — "Is everything alive?"

| Source | Destination | How | Where in code |
|---|---|---|---|
| CubeOrange+ FC | MAVROS2 | MAVLink serial (auto) | MAVROS2 internal |
| MAVROS2 | swarm_bridge_node | ROS2 `/mavros/state` | `swarm_bridge_node.py:302` `_on_fc_state()` |
| Jetson → peer Jetson | RFD900x serial | MAVLink HEARTBEAT 1Hz | `swarm_bridge_node.py:466` `_radio_send_heartbeat()` |
| Peer Jetson → Jetson | RFD900x serial | Decoded in radio RX | `swarm_bridge_node.py:629` `_handle_peer_heartbeat()` |

**FC state encoded in HEARTBEAT**: `base_mode` bit 7 = armed, `custom_mode` = ArduPilot mode number,
`system_status` = MAV_STATE enum.

**GCS sees 4 heartbeats**:
- Leader Jetson alive = UDP packets arriving on port 14571
- Leader FC alive = `fc_connected=1.0` in leader's telemetry packet
- Follower Jetson alive = UDP packets arriving on port 14572
- Follower FC alive = `fc_connected=1.0` in follower's telemetry packet

### SYS_STATUS (ID 1) — Battery

| Source | Destination | How | Where in code |
|---|---|---|---|
| CubeOrange+ FC | MAVROS2 | MAVLink serial (auto) | MAVROS2 internal |
| MAVROS2 | swarm_bridge_node | ROS2 `/mavros/battery` | `swarm_bridge_node.py:316` `_on_battery()` |
| Jetson → peer Jetson | RFD900x 1Hz | NAMED_VALUE_FLOAT x3 | `swarm_bridge_node.py:503` `_radio_send_battery()` |
| Jetson → GCS | WiFi UDP 10Hz | Fields 13-15 in !i21d | `swarm_bridge_node.py:699` |

Fields: `batt_voltage` (Volts), `batt_current` (Amps), `batt_remaining` (0.0-1.0).

### GLOBAL_POSITION_INT (ID 33) — Position/Velocity

| Source | Destination | How | Where in code |
|---|---|---|---|
| CubeOrange+ FC | MAVROS2 | MAVLink serial 10Hz | MAVROS2 internal |
| MAVROS2 | swarm_bridge_node | ROS2 topics (4 topics) | `swarm_bridge_node.py:282-300` |
| MAVROS2 | follower_control_node | Same ROS2 topics | `follower_control_node.py:165-178` |
| Jetson → peer Jetson | RFD900x 10Hz | Re-packed MAVLink | `swarm_bridge_node.py:437-459` |
| Peer Jetson → Jetson | RFD900x | Decoded | `swarm_bridge_node.py:599` `_handle_peer_position()` |
| Jetson → GCS | WiFi UDP 10Hz | Fields 0-6 in !i21d | `swarm_bridge_node.py:693-696` |

Encoding: lat/lon as degE7 (int32), alt as mm (int32), velocity as cm/s (int16), heading as cdeg (uint16).

### SET_POSITION_TARGET_LOCAL_NED (ID 84) — Velocity Commands

| Source | Destination | How | Where in code |
|---|---|---|---|
| follower_control_node | MAVROS2 | ROS2 `/mavros/setpoint_raw/local` | `follower_control_node.py:647` `_send_velocity_ned()` |
| MAVROS2 | CubeOrange+ FC | MAVLink serial | MAVROS2 internal |

**Note**: We use ID 84 (LOCAL_NED), not ID 86 (GLOBAL_INT). Velocity commands
in local NED frame are correct for guidance. Only sent when FC is in GUIDED mode.

### RADIO_STATUS (ID 109) — RFD900x Link Quality

| Source | Destination | How | Where in code |
|---|---|---|---|
| RFD900x modem (SiK firmware) | Jetson | Injected into local serial ~1Hz | `swarm_bridge_node.py:573` `_handle_radio_status()` |
| Jetson → GCS | WiFi UDP 10Hz | Fields 16-20 in !i21d | `swarm_bridge_node.py:700-701` |

Fields: `rssi` (0-255, higher=stronger), `remrssi` (remote), `txbuf` (0-100% free),
`noise` (0-255, lower=better), `remnoise` (remote noise), `rxerrors` (cumulative).

### Flight Mode (cube.flight_mode)

| Source | Destination | How | Where in code |
|---|---|---|---|
| CubeOrange+ FC | MAVROS2 | HEARTBEAT `custom_mode` field | MAVROS2 internal |
| MAVROS2 | Nodes | `/mavros/state` → `msg.mode` string | `swarm_bridge_node.py:308`, `follower_control_node.py:211` |
| Jetson → peer | RFD900x HEARTBEAT | `custom_mode` = mode number | `swarm_bridge_node.py:481` |
| Jetson → GCS | WiFi UDP | Field 11 = mode number as float | `swarm_bridge_node.py:697` |

Mode numbers: STABILIZE=0, ALT_HOLD=2, AUTO=3, **GUIDED=4**, LOITER=5, RTL=6, LAND=9.

---

## GCS Telemetry Packet Format (`!i21d` — 172 bytes)

```
Index | Field            | Type    | Description
──────┼──────────────────┼─────────┼──────────────────────────────
  i   | drone_id         | int32   | 1=Leader, 2=Follower
  0   | lat              | float64 | GPS latitude (degrees)
  1   | lon              | float64 | GPS longitude (degrees)
  2   | alt              | float64 | Relative altitude (meters)
  3   | vn               | float64 | North velocity (m/s)
  4   | ve               | float64 | East velocity (m/s)
  5   | vd               | float64 | Down velocity (m/s)
  6   | heading          | float64 | Compass heading (degrees)
  7   | timestamp        | float64 | Unix timestamp (seconds)
  8   | gmode            | float64 | Guidance mode (0=NONE,1=TRACK,2=CATCH,3=EVADE,4=FAILSAFE)
  9   | fc_connected     | float64 | 1.0 = FC alive, 0.0 = dead
 10   | fc_armed         | float64 | 1.0 = motors armed
 11   | fc_mode_code     | float64 | ArduPilot mode (4.0=GUIDED, 5.0=LOITER, etc.)
 12   | fc_system_status | float64 | MAV_STATE (0=UNINIT,3=STANDBY,4=ACTIVE,5=CRITICAL,6=EMERGENCY)
 13   | batt_voltage     | float64 | Battery voltage (Volts)
 14   | batt_current     | float64 | Battery current (Amps)
 15   | batt_remaining   | float64 | Battery fraction (0.0-1.0, -1=unknown)
 16   | radio_rssi       | float64 | Local RSSI (0-255)
 17   | radio_remrssi    | float64 | Remote RSSI (0-255)
 18   | radio_txbuf      | float64 | TX buffer free (0-100%)
 19   | radio_noise      | float64 | Local noise (0-255)
 20   | radio_remnoise   | float64 | Remote noise (0-255)
```

GCS receives two UDP streams: port **14571** (Leader) and **14572** (Follower).

Python to unpack on GCS side:
```python
import struct
fmt = "!i21d"
drone_id, lat, lon, alt, vn, ve, vd, heading, ts, gmode, \
    fc_conn, fc_armed, fc_mode, fc_sys, \
    batt_v, batt_a, batt_r, \
    rssi, remrssi, txbuf, noise, remnoise = struct.unpack(fmt, data)
```

---

## GUIDED Mode Gate (Follower Safety)

The follower's control loop (`follower_control_node.py:261-309`) has a strict gate:

```
Every 100ms:
  ├─ FC not connected? → skip (do nothing)
  │
  ├─ FC mode != GUIDED?
  │   ├─ Was GUIDED before? → "suspending guidance"
  │   │   Reset: smoother, guidance_state, failsafe_state
  │   ├─ Drain all pending GCS commands (discard)
  │   └─ return → ZERO velocity commands sent
  │
  ├─ FC mode == GUIDED (just entered)?
  │   ├─ Airborne (armed + alt > 2m)? → set HOVER
  │   └─ On ground? → set IDLE
  │
  └─ FC mode == GUIDED (steady state):
      └─ Run mission state machine normally
         └─ Velocity commands flow to /mavros/setpoint_raw/local
```

**Key safety property**: The safety pilot can switch the FC to STABILIZE or LOITER
at ANY time to instantly stop all Jetson velocity commands. The guidance state is
fully reset so re-entering GUIDED is always a clean start.

---

## Port Scheme

| Port | Protocol | Direction | Purpose |
|---|---|---|---|
| 14571 | UDP | Jetson → GCS | Leader telemetry (172 bytes @ 10 Hz) |
| 14572 | UDP | Jetson → GCS | Follower telemetry (172 bytes @ 10 Hz) |
| 14581 | UDP | GCS → Jetson | Leader commands |
| 14582 | UDP | GCS → Jetson | Follower commands |

Formula: telem = `14570 + drone_id`, cmd = `14580 + drone_id`.

---

## Failsafe Checks (Priority Order)

Run every tick (10 Hz) in the follower before guidance:

| # | Check | Condition | Action |
|---|---|---|---|
| 1 | Own GPS loss | No GPS fix | HOVER → LAND after 20 ticks |
| 2 | Leader data stale | No peer update for 50 ticks | HOVER |
| 3 | Leader outside geofence | Leader > 200m from home | HOVER |
| 4 | Follower outside geofence | Follower > 200m from home | GEOFENCE_RETURN |
| 5 | Altitude ceiling | Follower alt > 100m | HOVER |
| 6 | Catchup timeout | Stuck in CATCHUP > threshold | RTL |

Thresholds are configurable in `failsafe_lib/config.yaml`.

---

## Troubleshooting

### Serial port issues
```bash
# Check what's connected
ls -la /dev/ttyUSB*
dmesg | grep ttyUSB

# If permission denied
sudo usermod -a -G dialout $USER
# Then logout and login again

# If ports swap after reboot, create udev rules:
# /etc/udev/rules.d/99-swarm.rules
# SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="ttyFC"
# SUBSYSTEM=="tty", ATTRS{idVendor}=="yyyy", ATTRS{idProduct}=="yyyy", SYMLINK+="ttyRADIO"
```

### MAVROS2 won't connect to FC
```bash
# Test direct MAVLink connection
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=921600)
msg = m.recv_match(blocking=True, timeout=5)
print('Got:', msg)
"

# Check CubeOrange+ TELEM2 settings in Mission Planner:
# SERIAL2_PROTOCOL = 2 (MAVLink2)
# SERIAL2_BAUD = 921600
```

### No peer data (RFD900x)
```bash
# Test RFD900x link
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600, source_system=1)
while True:
    msg = m.recv_match(blocking=True, timeout=3)
    if msg: print(msg.get_type(), msg.get_srcSystem())
    else: print('timeout')
"

# Check RFD900x configuration (use AT commands or SiK Mission Planner):
# Both radios must have matching: NetID, Air Speed, Baud Rate
```

### Rebuild after code changes
```bash
cd ~/swarm_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Dependencies

### System packages (installed by install.sh)
- ROS2 Humble (`ros-humble-desktop` or `ros-humble-base`)
- `ros-humble-mavros` + `ros-humble-mavros-extras`
- GeographicLib datasets

### Python packages (installed by install.sh)
- `pymavlink` — MAVLink protocol for RFD900x serial
- `pyyaml` — hierarchy.yaml parsing

### Bundled Python libraries (no install needed)
- `guidance_lib/` — 3-mode follower guidance (TRACKING/CATCHUP/EVASION + collision avoidance)
- `failsafe_lib/` — 6-check safety monitor (geofence, GPS, stale leader, altitude)

Both libraries are pure Python with zero external dependencies (only `math`, `dataclasses`).
