================================================================================
                         SWARM SITL STACK
              5-Drone ArduCopter Formation Flight Simulator
================================================================================

WHAT THIS IS
------------
A swarm of 5 simulated drones that take off, fly formations, avoid collisions,
and land -- all controlled from a ground station. Each drone is a separate
process, just like 5 separate physical machines. A 6th process (the GCS) sends
commands, relays state between drones, visualizes the swarm, and logs everything.

Nothing is faked. The drones run the real ArduCopter flight controller firmware
compiled for your Linux machine (SITL -- Software In The Loop). The Python code
is a companion computer application that talks to ArduCopter over MAVLink, the
same binary protocol used on real Pixhawk hardware. The only difference from a
physical swarm is that the sensor data comes from a physics simulator instead of
real IMUs and GPS receivers.


================================================================================
PREREQUISITES
================================================================================

1. ArduPilot SITL
   Clone and build ArduPilot:
     git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
     cd ~/ardupilot
     git submodule update --init --recursive
     Tools/environment_install/install-prereqs-ubuntu.sh -y
     ./waf configure --board sitl
     ./waf copter

   This produces the binary: ~/ardupilot/build/sitl/bin/arducopter
   And the launch script:   ~/ardupilot/Tools/autotest/sim_vehicle.py

2. Python 3.12+ with pymavlink:
     pip install pymavlink matplotlib Pillow

3. Environment (add to your shell profile):
     export PYTHONPATH="$HOME/.local/lib/python3.12/site-packages:$PYTHONPATH"
     export PATH="$HOME/ardupilot/Tools/autotest:$PATH"


================================================================================
HOW TO RUN
================================================================================

OPTION A: Automated demo (one command, produces GIF + logs)
  python3 -m scripts.run_demo --num-drones 5

  This launches all 5 drones and GCS inline, runs a scripted mission
  (takeoff, V formation, LINE formation, DIAMOND formation, land), saves
  outputs to output/, and shuts everything down. Takes about 3 minutes.

OPTION B: Interactive swarm (GCS CLI for manual control)
  python3 -m scripts.launch_swarm --num-drones 5

  Same launch sequence, but the GCS runs with an interactive CLI. You type
  commands to control the swarm in real time.

OPTION C: Separate terminals (simulates 6 independent machines)
  Terminal 1:  python3 drones/drone_1/run.py
  Terminal 2:  python3 drones/drone_2/run.py
  Terminal 3:  python3 drones/drone_3/run.py
  Terminal 4:  python3 drones/drone_4/run.py
  Terminal 5:  python3 drones/drone_5/run.py
  Terminal 6:  python3 gcs/run_gcs.py

  Each drone process starts its own SITL instance and agent. The GCS
  discovers them as they come online. You control the swarm from the GCS
  terminal.

TO STOP EVERYTHING:
  python3 -m scripts.stop_swarm
  (Kills all ArduCopter, sim_vehicle.py, agent, and GCS processes.)


================================================================================
GCS COMMANDS (interactive mode)
================================================================================

  takeoff <alt>                    All drones take off to <alt> meters
  takeoff <id> <alt>              Drone <id> takes off to <alt> meters
  land                             All drones land
  land <id>                        Drone <id> lands
  formation <shape> <hdg> <spc>   Set formation
      shape:   LINE, V, COLUMN, DIAMOND
      hdg:     heading in degrees (0=north, 90=east)
      spc:     spacing between drones in meters
  waypoint <id> <lat> <lon> <alt> Send drone <id> to GPS coordinates
  velocity <id> <vn> <ve> <vd>    Set drone <id> velocity (north/east/down m/s)
  status                           Print all drone states
  quit                             Shut down GCS

Example session:
  gcs> takeoff 10
  gcs> formation V 0 8
  gcs> formation LINE 90 6
  gcs> formation DIAMOND 45 7
  gcs> land
  gcs> quit


================================================================================
OUTPUTS
================================================================================

After a demo run, the output/ directory contains:
  swarm_demo.gif         Animated 2D visualization of the entire mission
  swarm_final.png        Last frame of the visualization
  flight_log.csv         All drones interleaved: timestamp, drone_id, position,
                         velocity, heading, battery, mode, armed, formation,
                         failsafe
  drone_1_log.csv        Per-drone telemetry (same columns, one drone only)
  drone_2_log.csv        ...
  drone_3_log.csv        ...
  drone_4_log.csv        ...
  drone_5_log.csv        ...
  session_metadata.json  Session summary, alert log, command log

CSV columns:
  timestamp, elapsed_s, drone_id, lat, lon, alt, vx, vy, vz, heading,
  battery_pct, mode, armed, formation_slot, failsafe_active


================================================================================
PROJECT STRUCTURE
================================================================================

Swarm_SITL_stack/
|
|-- config.py                      Global constants (ports, thresholds, paths)
|
|-- comms/
|   |-- protocol.py                JSON message format (make_msg, parse_msg)
|   |-- udp_node.py                Non-blocking UDP socket (send, recv_all)
|
|-- mavlink_layer/
|   |-- drone_connection.py        pymavlink wrapper (connect, arm, takeoff,
|                                   goto, velocity, land, read telemetry)
|
|-- sitl/
|   |-- find_sim_vehicle.py        Locate sim_vehicle.py on the system
|   |-- launcher.py                Launch/kill ArduCopter SITL processes
|   |-- start_sitl.sh              Shell script to launch one SITL instance
|
|-- drone_agent/
|   |-- agent.py                   Main agent loop (10 Hz)
|   |-- state.py                   DroneState dataclass + PeerTable
|   |-- local_planner.py           Takeoff/waypoint/velocity state machine
|   |-- global_planner.py          Formation slot offset calculator
|   |-- failsafe.py                Proximity, comms-lost, battery monitors
|
|-- gcs/
|   |-- gcs.py                     GCS main class (receive, relay, CLI, viz)
|   |-- run_gcs.py                 Standalone GCS entry point
|   |-- state_collector.py         Aggregate drone states
|   |-- command_dispatcher.py      Build and send commands to agents
|   |-- visualizer.py              2D matplotlib plot + GIF capture
|   |-- flight_logger.py           CSV + JSON logging
|
|-- drones/
|   |-- drone_1/
|   |   |-- config_drone.py        DRONE_ID=1, computed ports and home position
|   |   |-- run.py                 Self-contained launcher (starts SITL + agent)
|   |-- drone_2/                   Same pattern, DRONE_ID=2
|   |-- drone_3/                   Same pattern, DRONE_ID=3
|   |-- drone_4/                   Same pattern, DRONE_ID=4
|   |-- drone_5/                   Same pattern, DRONE_ID=5
|
|-- scripts/
|   |-- run_demo.py                Automated demo (launch, fly, capture, stop)
|   |-- launch_swarm.py            Interactive launcher (swarm + GCS CLI)
|   |-- stop_swarm.py              Kill all swarm processes
|
|-- logs/                          Runtime logs (SITL, agent, per-instance dirs)
|-- output/                        Demo outputs (GIF, PNG, CSV, JSON)


================================================================================
HOW IT WORKS — THE FULL CHAIN
================================================================================

There are two completely separate communication layers:

  LAYER 1: MAVLink over TCP (agent <-> ArduCopter SITL)
    This is the real flight controller protocol. Each agent has a dedicated
    TCP connection to its own ArduCopter instance. The agent sends commands
    (arm, takeoff, go-to-waypoint, set-velocity, land) and receives telemetry
    (GPS position, velocity, heading, battery, flight mode, armed state).

  LAYER 2: JSON over UDP (agent <-> GCS)
    This is a custom application protocol. The GCS sends high-level commands
    (takeoff, formation, land) and receives state reports. It also relays
    each drone's state to all other drones for peer awareness.

The GCS never touches MAVLink. It sends "form a V shape" over UDP. Each agent
independently calculates its own slot position and sends the corresponding
MAVLink waypoint to its own ArduCopter. ArduCopter handles all the low-level
flight control (PID loops, motor mixing, EKF state estimation).

--- Data flow for one drone: ---

  +------------------+                     +------------------+
  | ArduCopter SITL  |  MAVLink (TCP)      |   Drone Agent    |
  | (C++ firmware)   |<------------------->| (Python process) |
  | Port 5770        |  Position, HB,      | Port 15110 (UDP) |
  |                  |  Battery, Mode       |                  |
  | Runs:            |  Arm, Takeoff,       | Runs:            |
  |  - Physics sim   |  Goto, Velocity,     |  - Read MAVLink  |
  |  - EKF           |  Set Mode            |  - Read UDP cmds |
  |  - PID control   |                      |  - Failsafe      |
  |  - Motor mixing  |                      |  - Plan movement |
  +------------------+                     |  - Report state  |
                                            +--------+---------+
                                                     |
                                              UDP (JSON)
                                                     |
                                            +--------v---------+
                                            |       GCS        |
                                            | (Python process) |
                                            | Port 15000 (UDP) |
                                            |                  |
                                            | Runs:            |
                                            |  - Collect state |
                                            |  - Relay peers   |
                                            |  - Dispatch cmds |
                                            |  - Visualize     |
                                            |  - Log flights   |
                                            |  - CLI interface |
                                            +------------------+


================================================================================
ARDUCOPTER SITL — WHAT IT IS AND HOW IT IS LAUNCHED
================================================================================

ArduCopter is an open-source autopilot written in C++. When compiled for SITL,
it runs as a native Linux process instead of on a Pixhawk microcontroller. It
includes a physics simulator that models:
  - Multirotor aerodynamics and motor response
  - Simulated GPS, IMU, barometer, compass sensors
  - Extended Kalman Filter (EKF) for state estimation
  - PID controllers for attitude and position hold
  - All flight modes (STABILIZE, GUIDED, LAND, RTL, etc.)
  - Arming checks (GPS lock, EKF convergence, etc.)

The compiled binary lives at:
  ~/ardupilot/build/sitl/bin/arducopter

ArduPilot provides a launch script called sim_vehicle.py that handles
compilation, parameter setup, and process management. This project calls it
with these arguments:

  python3 sim_vehicle.py \
    -v ArduCopter           # Use copter firmware (not plane, rover, etc.)
    -I <drone_id>           # Instance ID (1-5). Sets TCP port to 5760+id*10
    --no-mavproxy           # No MAVProxy intermediary. SITL listens directly.
    --auto-sysid            # Set MAVLink system ID from instance ID
    -l "lat,lon,alt,hdg"    # Home GPS position
    --speedup 1             # Real-time physics (no fast-forward)

The launch happens at sitl/launcher.py, line 68:
  proc = subprocess.Popen(cmd, ...)

sim_vehicle.py then internally runs:
  ~/ardupilot/build/sitl/bin/arducopter  (the actual C++ firmware)

Each SITL instance:
  - Gets its own working directory (logs/sitl_instance_N/) to avoid EEPROM
    conflicts between instances
  - Runs in its own process group (os.setsid) for clean shutdown
  - Listens for MAVLink on TCP port 5760 + drone_id * 10
  - Logs to logs/sitl_N.log

The Python code in this project never simulates flight physics, PID control,
EKF, or any ArduCopter behavior. It only sends MAVLink commands and reads
MAVLink telemetry, exactly as a companion computer would on a real drone.


================================================================================
MAVLINK CONNECTION — drone_connection.py
================================================================================

This file is the only place in the project that talks to ArduCopter. It uses
pymavlink (ArduPilot's official Python MAVLink library). Every method is a
thin wrapper around a pymavlink call.

CONNECTION SETUP:
  conn = mavutil.mavlink_connection("tcp:127.0.0.1:5770", source_system=255)
  conn.wait_heartbeat(timeout=60)

  - source_system=255 identifies the agent as a GCS to ArduCopter
  - wait_heartbeat blocks until ArduCopter sends its first HEARTBEAT message
  - The system ID is read from the heartbeat (with fallback to drone_id+1
    if ArduCopter reports sysid=0, which is a known SITL timing bug)
  - After connecting, requests all data streams at 10 Hz:
    conn.mav.request_data_stream_send(sysid, compid, MAV_DATA_STREAM_ALL, 10, 1)

TELEMETRY RECEIVED (non-blocking reads, called every loop at 10 Hz):

  GLOBAL_POSITION_INT:
    lat (degrees, raw / 1e7)
    lon (degrees, raw / 1e7)
    alt (meters above home, raw / 1000)
    vx, vy, vz (m/s, raw / 100)
    heading (degrees, raw / 100)

  HEARTBEAT:
    mode (string, decoded via mavutil.mode_string_v10)
    armed (boolean, from base_mode bitmask & MAV_MODE_FLAG_SAFETY_ARMED)

  SYS_STATUS:
    battery_remaining (0-100%, or -1 if unknown)

  LOCAL_POSITION_NED:
    x, y, z (meters, NED frame relative to EKF origin)
    vx, vy, vz (m/s)

COMMANDS SENT:

  Set mode (e.g., GUIDED, LAND, RTL):
    command_long_send(MAV_CMD_DO_SET_MODE, mode_id)
    Mode IDs come from conn.mode_mapping() — ArduCopter-specific values

  Arm motors:
    command_long_send(MAV_CMD_COMPONENT_ARM_DISARM, param1=1)

  Disarm motors:
    command_long_send(MAV_CMD_COMPONENT_ARM_DISARM, param1=0)

  Takeoff to altitude:
    command_long_send(MAV_CMD_NAV_TAKEOFF, param7=altitude_meters)
    Must be in GUIDED mode and armed first.

  Go to GPS waypoint:
    set_position_target_global_int_send(
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask=0b0000_1111_1111_1000,   (position only)
        lat_int=lat*1e7, lon_int=lon*1e7, alt=meters
    )

  Set velocity (NED frame):
    set_position_target_local_ned_send(
        frame=MAV_FRAME_LOCAL_NED,
        type_mask=0b0000_1111_1100_0111,   (velocity only)
        vn, ve, vd in m/s
    )
    IMPORTANT: Must be resent every ~200ms or ArduCopter stops moving.

TYPE MASKS explained:
  These 16-bit masks tell ArduCopter which fields to use and which to ignore.
  Bit=1 means IGNORE that field.
  Position-only mask 0b0000_1111_1111_1000:
    Bits 0-2 = 000: USE position (x/y/z or lat/lon/alt)
    Bits 3-5 = 111: IGNORE velocity
    Bits 6-8 = 111: IGNORE acceleration
    Bits 9-10 = 11: IGNORE yaw
  Velocity-only mask 0b0000_1111_1100_0111:
    Bits 0-2 = 111: IGNORE position
    Bits 3-5 = 000: USE velocity
    Bits 6-8 = 111: IGNORE acceleration
    Bits 9-10 = 11: IGNORE yaw


================================================================================
DRONE AGENT — THE 10 Hz CONTROL LOOP
================================================================================

Each drone runs one instance of drone_agent/agent.py. The main loop runs at
10 Hz (100ms per iteration) and executes these steps in strict order:

  STEP 1: READ MAVLINK
    Drain the entire pymavlink receive buffer. Keep only the latest message
    of each type (GLOBAL_POSITION_INT, SYS_STATUS, HEARTBEAT). This ensures
    fresh data and prevents latency from accumulated old messages.

    Update DroneState fields: lat, lon, alt, vx, vy, vz, heading,
    battery_pct, mode, armed.

  STEP 2: READ UDP
    Drain all pending UDP datagrams from the GCS. For each message:
      - If src=0 (from GCS), update GCS heartbeat timer (for comms failsafe)
      - Dispatch by message type:
        TAKEOFF_CMD   -> local_planner.do_takeoff(alt)
        LAND_CMD      -> local_planner.do_land()
        WAYPOINT_CMD  -> disable formation, set waypoint
        VELOCITY_CMD  -> disable formation, set velocity
        FORMATION_CMD -> global_planner.set_formation(data)
        STATE_REPORT  -> update peer table (for proximity failsafe)

  STEP 3: FAILSAFE CHECK
    Run all three failsafe monitors in priority order:
      1. Proximity: Distance to each known peer < 3.0m? -> HOLD
      2. Comms Lost: No GCS message for > 5.0s? -> RTL
      3. Low Battery: Below 20%? -> LAND
    If ANY failsafe is active, skip step 4 entirely. The failsafe action
    overrides the planner.

    Guards: Failsafes are disabled until the drone has valid GPS (lat/lon
    not zero) AND is armed. This prevents false alarms during SITL startup
    when GPS is initializing (20-40 seconds).

  STEP 4: PLAN AND EXECUTE
    Only runs if no failsafe is active.
    a) If in a formation: global_planner computes this drone's target
       lat/lon/alt from its slot assignment, then sets it as a waypoint.
    b) local_planner.tick() executes the current task:
       - TAKEOFF: Run the non-blocking state machine (wait GPS -> GUIDED
         -> ARM -> TAKEOFF command, resending each step every 2-3s)
       - WAYPOINT: Resend send_goto_global() every tick
       - VELOCITY: Resend send_velocity_ned() every tick
       - HOLD: Send zero velocity
       - LAND/IDLE: Do nothing (ArduCopter handles LAND mode internally)

  STEP 5: REPORT STATE (4 Hz)
    Every 250ms, serialize DroneState to JSON and send to GCS via UDP.
    Fields: drone_id, lat, lon, alt, vx, vy, vz, heading, battery_pct,
    mode, armed, formation_slot, failsafe_active.

  STEP 6: SEND ALERTS
    If any failsafe triggered this tick, send ALERT messages to GCS.

  STEP 7: SLEEP
    Sleep for the remainder of the 100ms period to maintain 10 Hz.


================================================================================
TAKEOFF STATE MACHINE — local_planner.py
================================================================================

Takeoff is a 4-step gated sequence. Each step must complete before the next
begins. Commands are resent periodically because MAVLink is fire-and-forget
(no acknowledgment in this implementation).

  Step 0: WAIT FOR GPS
    Check: has_gps (lat != 0 or lon != 0)
    ArduCopter's EKF needs a GPS fix before it will accept arming. SITL GPS
    takes 20-40 seconds to initialize after startup. Log "Waiting for GPS
    fix..." every 5 seconds.

  Step 1: SET GUIDED MODE
    Check: "GUIDED" in current mode string
    Send: set_mode("GUIDED") every 2 seconds until confirmed
    ArduCopter must be in GUIDED mode to accept position/velocity commands.

  Step 2: ARM MOTORS
    Check: armed == True (from heartbeat)
    Send: arm() every 2 seconds until confirmed
    ArduCopter runs pre-arm checks (GPS lock, EKF health, etc.) and may
    reject the arm command. Resending handles this.

  Step 3: SEND TAKEOFF COMMAND
    Send: takeoff(target_alt) every 3 seconds
    The takeoff command tells ArduCopter to climb to the target altitude.
    Resending ensures the command is received even if a packet is dropped.
    Once airborne, the agent transitions to normal waypoint/velocity control.


================================================================================
FORMATION GEOMETRY — global_planner.py
================================================================================

When a FORMATION_CMD is received, each drone independently computes its target
position based on its slot number (drone_id - 1, zero-indexed).

FORMATION TYPES:

  LINE: Drones spread perpendicular to heading, centered on reference point
    Slot 0:  local_e = -(n-1)/2 * spacing   (leftmost)
    Slot 1:  local_e = (-(n-1)/2 + 1) * spacing
    ...
    Slot n-1: local_e = (n-1)/2 * spacing    (rightmost)
    local_n = 0 for all slots

    Example (5 drones, 6m spacing, heading=90 east):
      Drone 1: 12m left
      Drone 2: 6m left
      Drone 3: center
      Drone 4: 6m right
      Drone 5: 12m right

  V: V-shape with slot 0 at the tip
    Slot 0: (0, 0) — tip of the V
    Odd slots (1,3,...): Right wing, 30-degree angle back
    Even slots (2,4,...): Left wing, 30-degree angle back
    Each rank steps back by: spacing * cos(30 degrees)
    Each rank steps out by:  spacing * sin(30 degrees)

    Example (5 drones, 8m spacing):
      Drone 1: tip
      Drone 2: 6.9m back, 4.0m right
      Drone 3: 6.9m back, 4.0m left
      Drone 4: 13.9m back, 8.0m right
      Drone 5: 13.9m back, 8.0m left

  COLUMN: Single file along heading
    Slot 0: front
    Slot 1: 1 * spacing behind
    Slot 2: 2 * spacing behind
    ...

  DIAMOND: Slot 0 front, 1-2 flanks, 3 rear, extras extend further back
    Slot 0: (spacing, 0)    — front point
    Slot 1: (0, -spacing)   — left flank
    Slot 2: (0, +spacing)   — right flank
    Slot 3: (-spacing, 0)   — rear point
    Slot 4+: extend backward from rear

HEADING ROTATION:
  All offsets are computed in a local frame (north/east), then rotated by the
  formation heading using a standard 2D rotation matrix:
    rotated_n = local_n * cos(heading) - local_e * sin(heading)
    rotated_e = local_n * sin(heading) + local_e * cos(heading)

GPS CONVERSION:
  The rotated NED offsets are converted to lat/lon:
    target_lat = ref_lat + (north_meters / 111320.0)
    target_lon = ref_lon + (east_meters / (111320.0 * cos(ref_lat_radians)))
  Where 111320.0 meters = 1 degree of latitude (constant approximation).
  This is accurate for the small distances involved (tens of meters).


================================================================================
FAILSAFE SYSTEM — failsafe.py
================================================================================

Three independent monitors, checked every loop in priority order.

PROXIMITY (highest priority):
  Action: HOLD (stop all motion — send zero velocity)
  Trigger: Haversine distance to any peer < 3.0 meters
  Clear: Distance to all peers > 4.5 meters (hysteresis prevents oscillation)
  Details:
    - Calculates great-circle distance using haversine formula
      (earth radius = 6,371,000 meters)
    - Skips peers with invalid GPS (lat=0, lon=0)
    - Generates ALERT with code "PROXIMITY"

COMMS LOST (medium priority):
  Action: RTL (Return To Launch — ArduCopter flies home autonomously)
  Trigger: No UDP message from GCS (src=0) for > 5.0 seconds
  Clear: Any message from GCS received
  Guard: Only activates AFTER the first GCS message has ever been received.
    This prevents RTL during startup when GCS hasn't connected yet.

LOW BATTERY (lowest priority):
  Action: LAND (ArduCopter descends and disarms autonomously)
  Trigger: battery_pct > 0 and battery_pct < 20%
  Clear: Never (once triggered, stays triggered)
  Guard: Ignores battery_pct == -1 (ArduCopter reports -1 when battery
    monitoring is not configured, which is the case in default SITL).

MASTER GUARDS (apply to all failsafes):
  - Disabled until valid GPS: lat != 0 and lon != 0
  - Disabled until armed: armed == True
  Both must be true before any failsafe can trigger. This prevents false
  alarms during the 20-40 second SITL initialization period.


================================================================================
GCS — GROUND CONTROL STATION
================================================================================

The GCS runs as a single process with two threads:

  MAIN THREAD (10 Hz loop):
    1. Receive all UDP datagrams from drone agents
    2. For STATE_REPORT messages:
       - Update state_collector with latest state
       - Relay to all OTHER drone agents (rewrite src=0 so agents count it
         as a GCS heartbeat for comms-lost failsafe)
       - Log to flight_logger
    3. For ALERT messages:
       - Log warning with drone ID, code, message, and action taken
    4. Check for stale drones (no report in > 5 seconds)
    5. Update visualizer with all drone positions
    6. Sleep to maintain 10 Hz

  CLI THREAD (daemon, blocking input):
    Prints "gcs>" prompt, reads commands, dispatches to command_dispatcher.
    Runs in a daemon thread so it doesn't prevent shutdown.

STATE RELAY:
  When the GCS receives a STATE_REPORT from drone N, it forwards a copy to
  all OTHER drones (not back to N). The forwarded message has src=0 (marked
  as from GCS). This serves two purposes:
    1. Each drone learns the positions of all other drones (needed for
       proximity failsafe collision avoidance)
    2. Each drone gets regular messages from src=0, which resets the
       comms-lost timer (prevents false RTL)

COMMAND DISPATCHER:
  Translates CLI commands into JSON messages sent over UDP:
    takeoff 10      -> TAKEOFF_CMD {target_id: 0, alt: 10}
    land            -> LAND_CMD {target_id: 0}
    formation V 0 8 -> FORMATION_CMD {formation: "V", heading_deg: 0,
                         spacing_m: 8, leader_id: 1, ref_lat/lon/alt: ...}
  Formation commands automatically fetch the leader's current GPS position
  from the state_collector as the formation reference point.

VISUALIZER:
  2D matplotlib scatter plot showing drone positions in meters relative to
  a reference origin. Each drone is a colored dot with a label showing its
  ID, altitude, and failsafe status.
    - Reference origin: Set once from the first drone with valid GPS.
      Never updated (prevents drones jumping off-screen).
    - Coordinate conversion: lat/lon to meters using 111320 m/degree
    - Axis limits: Fixed at -50m to +50m for consistent GIF frames
    - Headless mode: Renders to in-memory buffer, captures frames for GIF
    - Live mode: Renders to TkAgg window with interactive display

FLIGHT LOGGER:
  Records every state report to CSV (both aggregated and per-drone files).
  Also records alerts and commands to in-memory lists, which are written
  to session_metadata.json at the end.

  CSV columns:
    timestamp (unix, 3 decimals), elapsed_s (from session start),
    drone_id, lat (7 decimals), lon (7 decimals), alt (2 decimals),
    vx/vy/vz (3 decimals, m/s), heading (1 decimal, degrees),
    battery_pct (integer), mode (string), armed (bool),
    formation_slot (integer, -1 if none), failsafe_active (bool)


================================================================================
JSON MESSAGE PROTOCOL — comms/protocol.py
================================================================================

All GCS <-> Agent communication uses JSON over UDP. MAVLink is never used
on this layer. Message format:

  {
    "type": "<MSG_TYPE>",
    "src": <integer>,         0 = GCS, 1-5 = drone ID
    "ts": <float>,            Unix timestamp (set at creation)
    "data": { ... }           Payload (varies by type)
  }

MESSAGE TYPES AND PAYLOADS:

  TAKEOFF_CMD (GCS -> Drone)
    data: { "target_id": 0 or drone_id, "alt": meters }

  LAND_CMD (GCS -> Drone)
    data: { "target_id": 0 or drone_id }

  WAYPOINT_CMD (GCS -> Drone)
    data: { "target_id": drone_id, "lat": float, "lon": float, "alt": float }

  VELOCITY_CMD (GCS -> Drone)
    data: { "target_id": drone_id, "vn": m/s, "ve": m/s, "vd": m/s }

  FORMATION_CMD (GCS -> All Drones)
    data: {
      "formation": "LINE" | "V" | "COLUMN" | "DIAMOND",
      "leader_id": drone_id,
      "ref_lat": float, "ref_lon": float, "ref_alt": float,
      "heading_deg": float,
      "spacing_m": float
    }

  STATE_REPORT (Drone -> GCS, also GCS -> Drones as relay)
    data: {
      "drone_id": int, "lat": float, "lon": float, "alt": float,
      "vx": float, "vy": float, "vz": float, "heading": float,
      "battery_pct": int, "mode": string, "armed": bool,
      "formation_slot": int, "failsafe_active": bool
    }

  ALERT (Drone -> GCS)
    data: {
      "level": "WARN",
      "code": "PROXIMITY" | "COMMS_LOST" | "LOW_BATTERY",
      "message": string,
      "action_taken": "HOLD" | "RTL" | "LAND"
    }


================================================================================
PORT ASSIGNMENTS
================================================================================

  Component          Port    Protocol   Direction
  ---------          ----    --------   ---------
  Drone 1 SITL       5770    TCP        Agent <-> ArduCopter (MAVLink)
  Drone 2 SITL       5780    TCP        Agent <-> ArduCopter (MAVLink)
  Drone 3 SITL       5790    TCP        Agent <-> ArduCopter (MAVLink)
  Drone 4 SITL       5800    TCP        Agent <-> ArduCopter (MAVLink)
  Drone 5 SITL       5810    TCP        Agent <-> ArduCopter (MAVLink)
  Drone 1 Agent      15110   UDP        GCS -> Agent (commands, peer relay)
  Drone 2 Agent      15120   UDP        GCS -> Agent (commands, peer relay)
  Drone 3 Agent      15130   UDP        GCS -> Agent (commands, peer relay)
  Drone 4 Agent      15140   UDP        GCS -> Agent (commands, peer relay)
  Drone 5 Agent      15150   UDP        GCS -> Agent (commands, peer relay)
  GCS                15000   UDP        Agents -> GCS (state reports, alerts)

  Port formulas:
    SITL TCP port  = 5760 + drone_id * 10
    Agent UDP port = 15100 + drone_id * 10
    GCS UDP port   = 15000 (fixed)


================================================================================
CONFIGURATION — config.py
================================================================================

  NUM_DRONES = 5                   Number of drones in the swarm

  HOME_LAT = -35.3632620           Home GPS (Canberra, Australia — ArduPilot
  HOME_LON = 149.1652370           default). Each drone is offset 10m east.
  HOME_ALT = 584                   Altitude in meters above sea level.
  HOME_HEADING = 270               Initial heading in degrees.

  SITL_BASE_PORT = 5760            TCP port base for SITL instances
  SITL_PORT_STEP = 10              Port increment per drone

  GCS_HOST = "127.0.0.1"           GCS listen address
  GCS_PORT = 15000                 GCS listen port

  AGENT_BASE_PORT = 15100          UDP port base for drone agents
  AGENT_PORT_STEP = 10             Port increment per drone

  AGENT_LOOP_HZ = 10               Agent main loop frequency
  STATE_REPORT_HZ = 4              State report sending frequency
  GCS_LOOP_HZ = 10                 GCS main loop frequency

  SAFE_DISTANCE_M = 3.0            Proximity alert trigger (meters)
  SAFE_DISTANCE_CLEAR_M = 4.5      Proximity alert clear (hysteresis)
  COMMS_TIMEOUT_S = 5.0            Seconds before comms-lost failsafe
  LOW_BATTERY_PCT = 20             Battery failsafe threshold (%)

  DEFAULT_SPACING_M = 5.0          Default formation spacing
  DEFAULT_FORMATION = "LINE"       Default formation shape

  ARDUPILOT_DIR = ~/ardupilot      Path to ArduPilot source tree
  PROJECT_DIR = <auto-detected>    Path to this project
  LOG_DIR = PROJECT_DIR/logs       Runtime log directory


================================================================================
PER-DRONE CONFIGURATION — drones/drone_N/config_drone.py
================================================================================

Each drone directory has its own config file that derives values from the
global config:

  DRONE_ID = N                     (1, 2, 3, 4, or 5)
  SITL_PORT = 5760 + N * 10       (5770, 5780, 5790, 5800, 5810)
  AGENT_PORT = 15100 + N * 10     (15110, 15120, 15130, 15140, 15150)
  DRONE_HOME_LAT = HOME_LAT       (same for all drones)
  DRONE_HOME_LON = HOME_LON + east_offset / (111320 * cos(lat))
  DRONE_HOME_ALT = 584
  DRONE_HOME_HEADING = 270
  DRONE_HOME = "lat,lon,alt,heading"   (string for sim_vehicle.py -l flag)

The east offset is (DRONE_ID - 1) * 10.0 meters, so:
  Drone 1: home at base position
  Drone 2: 10m east
  Drone 3: 20m east
  Drone 4: 30m east
  Drone 5: 40m east


================================================================================
PER-DRONE RUN SCRIPT — drones/drone_N/run.py
================================================================================

All 5 run.py files are byte-for-byte identical. The drone ID is auto-detected
from the directory name using regex: drone_(\d+).

Execution sequence:
  1. Detect drone ID from directory name
  2. Launch own SITL instance (subprocess via sim_vehicle.py)
  3. Wait up to 90 seconds for SITL TCP port to accept connections
  4. Create DroneAgent(drone_id) — connects to SITL via MAVLink
  5. Run agent main loop (10 Hz, blocking)
  6. On SIGTERM/SIGINT/KeyboardInterrupt:
     a. Stop agent (close UDP, stop loop)
     b. Kill SITL process (SIGTERM entire process group, SIGKILL if needed)


================================================================================
KNOWN ISSUES AND WORKAROUNDS
================================================================================

1. SITL SYSID=0 BUG
   Some ArduCopter SITL instances report system ID 0 in their heartbeat
   due to initialization timing. The workaround in drone_connection.py
   overrides sysid to drone_id + 1 when 0 is detected.

2. EKF INITIALIZATION TIME
   ArduCopter's EKF needs 20-40 seconds after startup to converge with
   simulated GPS data. During this time, lat/lon are reported as 0/0.
   All code guards against this: the takeoff state machine waits for GPS,
   failsafes are disabled, the visualizer skips zero-position drones.

3. EEPROM CONFLICTS
   If multiple SITL instances share a working directory, they corrupt each
   other's EEPROM (parameter storage). Each instance runs in its own
   directory: logs/sitl_instance_N/.

4. PYTHONPATH FOR SUBPROCESSES
   sim_vehicle.py runs as a subprocess and needs access to pymavlink.
   The PYTHONPATH environment variable is explicitly set to include
   ~/.local/lib/python3.12/site-packages before launching.

5. VELOCITY SETPOINT TIMEOUT
   ArduCopter's GUIDED mode velocity setpoints expire after ~200ms.
   The local planner resends them every tick (100ms at 10 Hz) to
   maintain continuous motion.

6. PROXIMITY FAILSAFE OSCILLATION
   Without hysteresis, drones near the safety distance would repeatedly
   trigger and clear the proximity alert. The 1.5m gap between trigger
   (3.0m) and clear (4.5m) thresholds prevents this.

7. VISUALIZER ORIGIN DRIFT
   If the reference origin for the 2D plot updates while drones are
   flying, they appear to jump. The origin is set once from the first
   valid GPS reading and never changed.

8. COMMS-LOST FALSE POSITIVES
   The comms-lost failsafe could trigger during startup before the GCS
   connects. The guard (_gcs_ever_contacted flag) prevents this — RTL
   only triggers if the GCS was previously reachable and then went silent.


================================================================================
COORDINATE SYSTEMS
================================================================================

This project uses three coordinate systems:

1. GPS (WGS84): lat/lon in degrees, alt in meters above home
   Used in: STATE_REPORT, WAYPOINT_CMD, FORMATION_CMD, send_goto_global()

2. NED (North-East-Down): meters relative to EKF origin
   North = +x, East = +y, Down = +z (positive downward)
   Used in: send_velocity_ned(), send_position_ned(), formation offsets

3. Screen (meters): East = +x, North = +y (for visualization)
   Used in: visualizer.py scatter plot

Conversions:
  1 degree latitude  = 111,320 meters (constant)
  1 degree longitude = 111,320 * cos(latitude) meters (varies with lat)
  NED velocity vd > 0 means descending


================================================================================
DEMO MISSION TIMELINE
================================================================================

When you run: python3 -m scripts.run_demo --num-drones 5

  T+0s     Launch drone 1 subprocess (starts SITL + agent)
  T+5s     Launch drone 2
  T+10s    Launch drone 3
  T+15s    Launch drone 4
  T+20s    Launch drone 5
  T+25s    Verify all 5 SITL ports reachable
  T+25s    Start GCS (headless, GIF capture)
  T+25s    Wait for all 5 drones to send at least one STATE_REPORT
  ~T+50s   All drones reporting (EKF initialized, GPS acquired)
  ~T+50s   TAKEOFF command: all drones to 10m altitude
  ~T+80s   All drones at altitude (within 2m tolerance)
  ~T+85s   V formation command (8m spacing, heading 0)
  ~T+105s  LINE formation command (6m spacing, heading 90)
  ~T+120s  DIAMOND formation command (7m spacing, heading 45)
  ~T+135s  LAND command: all drones land
  ~T+165s  Save GIF, PNG, CSV, JSON to output/
  ~T+165s  Terminate all processes

Total runtime: approximately 3 minutes.


================================================================================
END
================================================================================
