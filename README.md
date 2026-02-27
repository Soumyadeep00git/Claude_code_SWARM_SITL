# Leader-Follower Drone System

2-drone leader-follower formation system using ROS2, CubeOrange+, Jetson Orin, and RFD9000 radios.

## Architecture

```
Leader Orin (SYSID=1)        Follower Orin (SYSID=2)       GCS Laptop
┌──────────────────┐         ┌──────────────────┐          ┌──────────────┐
│ CO+ ↔ USB ↔ Orin │         │ CO+ ↔ USB ↔ Orin │          │ Mission      │
│ RFD9000 ↔ UART   │◄─air──►│ RFD9000 ↔ UART   │◄──air──►│ Planner      │
│ RC Receiver       │         │                  │          │ RFD9000 USB  │
│                   │         │                  │          │              │
│ Nodes:            │         │ Nodes:           │          │ Optional:    │
│  mavlink-router   │         │  mavlink-router  │          │  telemetry   │
│  MAVROS2          │         │  MAVROS2         │          │  bridge      │
│  peer_monitor     │         │  peer_monitor    │          │  command     │
│  leader_node      │         │  follow_controller          │  relay       │
│  rc_relay_node    │         │  failsafe_node   │          │              │
│  failsafe_node    │         │                  │          │              │
└──────────────────┘         └──────────────────┘          └──────────────┘
```

## How It Works

- **mavlink-router** on each Orin connects CubeOrange+ (USB) ↔ RFD9000 (UART) ↔ MAVROS2 (UDP) ↔ custom nodes (UDP)
- **RFD9000 multipoint mesh** carries MAVLink from both drones — Mission Planner sees SYSID 1 and 2 natively
- **Follower** receives leader's GLOBAL_POSITION_INT through the radio mesh, maintains fixed NED offset
- **Adaptive potential field** provides collision avoidance with two-zone repulsion + closing-speed adaptation
- **Single RC transmitter** controls leader by default; CH7 switch relays RC to follower via MAVLink

## Setup

### Prerequisites (each Orin)

```bash
# ROS2 Humble
sudo apt install ros-humble-ros-base ros-humble-mavros
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# mavlink-router
sudo apt install mavlink-router
# OR build from source: https://github.com/mavlink-router/mavlink-router

# pymavlink
pip3 install pymavlink
```

### Build (each Orin)

```bash
# Leader Orin
mkdir -p ~/leader_ws/src
cp -r leader_drone ~/leader_ws/src/
cd ~/leader_ws
source /opt/ros/humble/setup.bash
colcon build
echo "source ~/leader_ws/install/setup.bash" >> ~/.bashrc

# Follower Orin
mkdir -p ~/follower_ws/src
cp -r follower_drone ~/follower_ws/src/
cd ~/follower_ws
source /opt/ros/humble/setup.bash
colcon build
echo "source ~/follower_ws/install/setup.bash" >> ~/.bashrc
```

### ArduPilot Parameters

**Leader CO+ (SYSID=1)**:
```
SYSID_THISMAV = 1
SERIAL1_PROTOCOL = 2      (MAVLink2)
SERIAL1_BAUD = 921
SR1_POSITION = 10          (10Hz position stream)
SR1_EXTRA1 = 10            (10Hz attitude)
SR1_RC_CHAN = 10            (10Hz RC channels)
```

**Follower CO+ (SYSID=2)**:
```
SYSID_THISMAV = 2
SERIAL1_PROTOCOL = 2
SERIAL1_BAUD = 921
SR1_POSITION = 10
SR1_EXTRA1 = 10
SYSID_MYGCS = 254          (accept commands from companion)
RC_OPTIONS = 1              (enable RC override from companion)
```

### Serial Port Configuration

Edit `config/mavlink_router.conf` on each Orin to match your hardware:
- CubeOrange+ USB: usually `/dev/ttyACM0`
- RFD9000 UART: usually `/dev/ttyTHS1` (Orin 40-pin header) or `/dev/ttyUSB0` (USB adapter)

### Run

```bash
# Leader Orin
ros2 launch leader_drone leader.launch.py

# Follower Orin
ros2 launch follower_drone follower.launch.py

# GCS Laptop (optional — Mission Planner is primary)
ros2 launch gcs gcs.launch.py
```

## Configuration

All parameters are in YAML config files:

| File | Key Parameters |
|------|---------------|
| `leader_drone/config/leader_params.yaml` | RC switch channel, peer timeout, collision radii |
| `follower_drone/config/follower_params.yaml` | **Follow offset** (north/east/down), PD gains, feedforward, collision avoidance |
| `*/config/mavlink_router.conf` | Serial port paths, baud rates |

### Changing Follow Offset at Runtime

```bash
ros2 param set /follower/follow_controller offset_north_m -8.0
ros2 param set /follower/follow_controller offset_east_m 0.0
```

## Failsafe Behavior

### Leader
| State | Trigger | Action |
|-------|---------|--------|
| NOMINAL | All healthy | Mission Planner controls flight |
| PROX_AVOID | Follower < 5m | Active repulsion velocity |
| PEER_LOST | No follower for 5s | Warning, continue mission |
| LOW_BATTERY | Battery < 20% | RTL |

### Follower
| State | Trigger | Action |
|-------|---------|--------|
| FOLLOWING | Leader known | Track leader with offset |
| PROX_AVOID | Leader < 5m | Active repulsion, stop following |
| LEADER_LOST 0-10s | No leader for 5s | LOITER in place |
| LEADER_LOST 30s+ | No leader for 30s | RTL |
| RC_OVERRIDE | CH7 switch active | Yield to RC pilot |
| LOW_BATTERY | Battery < 20% | RTL |

## RC Override

- RC transmitter bound to **leader's CO+ receiver**
- **CH7 < 1500**: RC controls leader (default)
- **CH7 > 1500**: Leader Orin relays RC sticks to follower via RFD9000
- Follower's companion computer yields control when override is active
- Override auto-releases 1.5s after RC switch returns to leader mode

## File Structure

```
Leader_Follower_ORIN_CO+/
├── leader_drone/           → Flash to Leader Orin
│   ├── config/
│   │   ├── leader_params.yaml
│   │   └── mavlink_router.conf
│   ├── launch/leader.launch.py
│   └── leader_drone/
│       ├── leader_node.py
│       ├── rc_relay_node.py
│       ├── peer_monitor.py
│       ├── failsafe_node.py
│       ├── potential_field.py
│       └── geo_utils.py
│
├── follower_drone/         → Flash to Follower Orin
│   ├── config/
│   │   ├── follower_params.yaml
│   │   └── mavlink_router.conf
│   ├── launch/follower.launch.py
│   └── follower_drone/
│       ├── follow_controller.py
│       ├── peer_monitor.py
│       ├── failsafe_node.py
│       ├── potential_field.py
│       └── geo_utils.py
│
├── gcs/                    → Runs on GCS laptop
│   ├── config/gcs_params.yaml
│   ├── launch/gcs.launch.py
│   └── gcs/
│       ├── telemetry_bridge.py
│       └── command_relay.py
│
└── README.md
```
