================================================================================
  FOLLOWER GUIDANCE LIBRARY
================================================================================

  3-mode hard-switched guidance controller for leader-follower drone formation.
  Pure Python. Zero external dependencies. Works with any autopilot.

================================================================================
  FILES
================================================================================

  guidance.py        - Core algorithm. Mode selection + velocity computation.
  geo_utils.py       - GPS <-> NED flat-earth coordinate conversions.
  command_smoother.py - First-order EMA filter with hover deadband.
  config.yaml        - All tunable parameters in one file. Edit this.
  __init__.py        - Package exports (optional, for import convenience).

================================================================================
  REQUIREMENTS
================================================================================

  Python 3.10+
  No pip install needed. All imports are Python standard library (math, time,
  dataclasses).

================================================================================
  WHAT IT DOES
================================================================================

  Given the follower's state and the leader's state, it returns a velocity
  command (vn, ve, vd) in meters/second that drives the follower to maintain
  a fixed offset behind the leader.

  Three modes (priority order, only one active at a time):

    1. EVASION   - Radial repulsion when peer distance < safety_dist (1.0m).
                   Pushes follower straight away from leader.

    2. CATCHUP   - Max-speed sprint when position error > catchup_dist (8.0m).
                   Follower races toward goal with leader feedforward.

    3. TRACKING  - PD controller + leader velocity feedforward.
                   Normal following mode. Maintains offset precisely.

  Safety features applied to all modes:
    - Altitude floor (soft ramp + hard cutoff)
    - Horizontal speed cap
    - Rate limiter (max acceleration per tick)
    - NaN/Inf guard on all inputs and outputs

================================================================================
  INPUT
================================================================================

  compute_guidance() takes these arguments:

    FOLLOWER STATE (from your autopilot):
      my_lat        float   Follower latitude (degrees)
      my_lon        float   Follower longitude (degrees)
      my_alt        float   Follower altitude (meters, above sea level)
      my_vn         float   Follower velocity north (m/s)
      my_ve         float   Follower velocity east (m/s)
      my_vd         float   Follower velocity down (m/s)

    LEADER STATE (from radio link / UDP / ROS topic):
      peer_lat      float   Leader latitude (degrees)
      peer_lon      float   Leader longitude (degrees)
      peer_alt      float   Leader altitude (meters)
      peer_vn       float   Leader velocity north (m/s)
      peer_ve       float   Leader velocity east (m/s)

    GOAL POSITION (leader + your desired offset, see USAGE below):
      goal_lat      float   Target latitude (degrees)
      goal_lon      float   Target longitude (degrees)
      goal_alt      float   Target altitude (meters)

    OPTIONAL:
      cfg           GuidanceConfig   Tunable parameters (defaults are sane)
      state         GuidanceState    Persistent state (rate limiter memory)

================================================================================
  OUTPUT
================================================================================

  Returns a dict:

    vn          float   Velocity north command (m/s)   --+
    ve          float   Velocity east command (m/s)      |-- SEND THIS TO AUTOPILOT
    vd          float   Velocity down command (m/s)    --+
    speed       float   Horizontal speed magnitude (m/s)
    peer_dist   float   Distance to leader (meters)
    mode        str     'TRACKING', 'CATCHUP', 'EVASION', or 'FAILSAFE'
    emergency   bool    True if critical proximity, altitude, or failsafe
    flags       dict    Warning flags (see FAILSAFES section below)
    w_evasion   float   1.0 if evasion active, else 0.0
    w_tracking  float   1.0 if tracking active, else 0.0
    w_catchup   float   1.0 if catchup active, else 0.0
    tick        int     Tick counter

================================================================================
  USAGE
================================================================================

  from guidance_lib import (
      GuidanceConfig, GuidanceState, compute_guidance,
      ned_to_gps, CommandSmoother,
  )

  # ---- Setup (once at startup) ----

  # Option A: Load from config.yaml (recommended)
  #   pip install pyyaml
  import yaml
  with open('config.yaml') as f:
      params = yaml.safe_load(f)
  cfg = GuidanceConfig(**params['guidance'])
  OFFSET_N = params['offset']['north_m']
  OFFSET_E = params['offset']['east_m']
  smoother = CommandSmoother(
      alpha=params['smoother']['alpha'],
      deadband=params['smoother']['deadband'],
  )

  # Option B: Inline (no yaml dependency)
  # cfg = GuidanceConfig(kp=0.9, safety_dist_m=2.0, home_lat=..., home_lon=...)

  state = GuidanceState()           # persists across ticks (rate limiter memory)
  smoother = CommandSmoother()      # EMA output filter

  # Desired formation offset: 5m behind leader, 3m to the right
  OFFSET_N = -5.0   # meters north (negative = behind)
  OFFSET_E =  3.0   # meters east  (positive = right)


  # ---- Control loop (call at ~10Hz) ----

  while running:
      # Get latest data from your system (MAVLink, ROS, DJI SDK, etc.)
      my_lat, my_lon, my_alt = get_follower_gps()
      my_vn, my_ve, my_vd   = get_follower_velocity()
      leader_lat, leader_lon, leader_alt = get_leader_gps()
      leader_vn, leader_ve   = get_leader_velocity()

      # Step 1: Compute goal = leader position + offset
      goal_lat, goal_lon = ned_to_gps(OFFSET_N, OFFSET_E,
                                       leader_lat, leader_lon)
      goal_alt = leader_alt

      # Step 2: Run guidance
      result = compute_guidance(
          my_lat, my_lon, my_alt,
          my_vn, my_ve, my_vd,
          leader_lat, leader_lon, leader_alt,
          leader_vn, leader_ve,
          goal_lat, goal_lon, goal_alt,
          cfg, state,
      )

      # Step 3: Smooth output
      vn, ve, vd = smoother.filter(result['vn'], result['ve'], result['vd'])

      # Step 4: Send to your autopilot
      send_velocity_command(vn, ve, vd)

      # Step 5: Handle failsafes
      if result['mode'] == 'FAILSAFE':
          # Guidance returned zero velocity. You must decide what to do:
          #   LEADER_TOO_FAR   -> leader is > 50m away, consider RTL
          #   GEOFENCE         -> follower left the safe area, RTL immediately
          #   CATCHUP_TIMEOUT  -> stuck chasing for > 30s, hover or RTL
          print(f"FAILSAFE: {result['flags']}")
          trigger_rtl()  # your platform's return-to-launch
          break

      # Step 6: Monitor
      print(f"Mode: {result['mode']}  Dist: {result['peer_dist']:.1f}m")
      if result['emergency']:
          print(f"WARNING: {result['flags']}")

      sleep(0.1)  # 10Hz

================================================================================
  TUNABLE PARAMETERS (GuidanceConfig)
================================================================================

  Parameter              Default   Description
  ---------------------  -------   -------------------------------------------
  kp                     0.7       Position gain (m/s per m error)
  kd                     0.4       Velocity damping coefficient
  ff_gain                0.8       Leader velocity feedforward (0=off, 1=full)
  catchup_dist_m         8.0       Error threshold to trigger CATCHUP mode
  max_catchup_speed      4.0       Max speed in CATCHUP mode (m/s)
  safety_dist_m          1.0       Peer distance threshold for EVASION mode
  escape_speed           3.0       Evasion repulsion speed (m/s)
  max_speed              3.0       Horizontal speed cap in TRACKING mode (m/s)
  max_vertical_speed     1.5       Vertical speed cap (m/s)
  max_accel              4.0       Rate limiter, max delta per tick (m/s)
  min_altitude_m         3.0       Soft altitude floor (ramp-up push)
  critical_altitude_m    1.5       Hard altitude floor (full push-up)
  deadzone_m             0.5       No tracking output below this error
  max_peer_dist_m        50.0      Hover if leader is farther than this (m)
  geofence_radius_m      200.0     Hover if follower is this far from home (m)
  home_lat               0.0       Home latitude for geofence (degrees)
  home_lon               0.0       Home longitude for geofence (degrees)
  catchup_timeout_ticks  300       Hover after this many CATCHUP ticks (300 = 30s at 10Hz)

================================================================================
  FAILSAFES
================================================================================

  Three failsafes that return mode='FAILSAFE', emergency=True, and zero
  velocity. When triggered, the caller must decide what to do (RTL, land, etc).

  1. LEADER_TOO_FAR (flag)
     Triggers when: peer_dist > max_peer_dist_m (default 50m)
     Meaning:       Leader is too far away. Follower can't keep up or leader
                    data is wrong. Hovering prevents chasing a phantom.
     Action:        RTL or hover and wait for leader to return.

  2. GEOFENCE (flag)
     Triggers when: follower is > geofence_radius_m (default 200m) from home.
     Meaning:       Follower has drifted outside the safe operating area.
     Action:        RTL immediately.
     NOTE:          Requires home_lat and home_lon in GuidanceConfig.
                    If home is (0,0), geofence is disabled.

  3. CATCHUP_TIMEOUT (flag)
     Triggers when: follower has been in CATCHUP mode for >
                    catchup_timeout_ticks consecutive ticks (default 300 = 30s).
     Meaning:       Follower has been sprinting for 30 seconds and still can't
                    close the gap. Leader is faster or data is stale.
     Action:        RTL or hover.
     NOTE:          Counter resets when follower enters TRACKING or EVASION.
                    Set catchup_timeout_ticks=0 to disable.

  Checking for failsafes in your code:

      result = compute_guidance(...)
      if result['mode'] == 'FAILSAFE':
          if 'LEADER_TOO_FAR' in result['flags']:
              # ...
          if 'GEOFENCE' in result['flags']:
              # ...
          if 'CATCHUP_TIMEOUT' in result['flags']:
              # ...

================================================================================
  EXTRA: compute_escape()
================================================================================

  A simpler function for emergency-only use. Pure radial repulsion away from
  the peer. No tracking, no catchup. Use this if you only need proximity
  avoidance (e.g. on the leader side, or as a failsafe fallback).

  from guidance_lib import compute_escape

  result = compute_escape(
      my_lat, my_lon, my_alt,
      peer_lat, peer_lon, peer_alt,
  )
  # result['vn'], result['ve'], result['vd'] = escape velocity

================================================================================
  NOTES
================================================================================

  - Call compute_guidance() at a consistent rate (~10Hz recommended).
  - The GuidanceState object MUST persist across ticks. Do not recreate it
    each call — the rate limiter needs the previous velocity.
  - The CommandSmoother is optional but recommended. It prevents jitter
    when the drone is near the goal (hover deadband = 0.2 m/s).
  - goal_lat/lon is NOT the leader's position. It is leader + your offset.
    You compute it with ned_to_gps() before calling compute_guidance().
  - The function handles bad data gracefully: NaN, Inf, zero GPS, stale
    data all produce safe zero-velocity output with appropriate flags.
