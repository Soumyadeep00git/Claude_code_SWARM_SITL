================================================================================
  FOLLOWER GUIDANCE LIBRARY
================================================================================

  3-mode hard-switched guidance controller for leader-follower drone formation.
  Pure Python. Zero external dependencies. Works with any autopilot.

  Modular architecture: each concern (mode selection, velocity, output safety)
  is a separate class that can be tested independently.

  Failsafe monitoring is in the separate failsafe_lib package.

================================================================================
  FILES
================================================================================

  guidance.py        - Orchestrator. Delegates to modular classes below.
  target.py          - TargetComputer: leader + offset + feedforward -> goal.
  mode_selector.py   - ModeSelector: EVASION > CATCHUP > TRACKING switch.
  velocity.py        - VelocityComputer: per-mode velocity computation.
  output_safety.py   - OutputSafety: speed cap, rate limiter, altitude floor/ceiling.
  command_smoother.py - CommandSmoother: EMA filter with hover deadband.
  geo_utils.py       - GPS <-> NED flat-earth coordinate conversions.
  config.yaml        - Guidance tunable parameters. Edit this.
  __init__.py        - Package exports.

================================================================================
  REQUIREMENTS
================================================================================

  Python 3.10+
  No pip install needed. All imports are Python standard library (math, time,
  dataclasses, enum).

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
    - Altitude ceiling (push down if too high)
    - Horizontal speed cap
    - Rate limiter (max acceleration per tick)
    - NaN/Inf guard on all inputs and outputs

================================================================================
  ARCHITECTURE
================================================================================

  Main loop (docker_sim/follower_main.py):
    1. READ own state from autopilot
    2. READ leader state from bridge (None if stale)
    3. FAILSAFE check (failsafe_lib) — runs FIRST, before guidance
       - Own GPS valid?
       - Leader data fresh?
       - Leader inside geofence?
       - Follower inside geofence?
       - Altitude ceiling?
       - Catchup timeout?
    4. COMPUTE target (TargetComputer) — leader + offset + feedforward
    5. GUIDE (compute_guidance) — mode select + velocity
    6. SMOOTH (CommandSmoother) — EMA filter
    7. SEND velocity to autopilot

  Key design decisions:
    - Leader state passed DIRECTLY each tick (None = stale)
    - No cached stale data in controller
    - Smoother resets on failsafe (no residual velocity)
    - One-shot RTL (not re-sent every tick)
    - Command queue (not single overwrite)
    - All failsafe checks are in failsafe_lib (see failsafe_lib/readme.txt)

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
    mode        str     'TRACKING', 'CATCHUP', or 'EVASION'
    emergency   bool    True if critical proximity or altitude issue
    flags       dict    Warning flags (NO_OWN_GPS, PROX_CRITICAL, etc.)
    w_evasion   float   1.0 if evasion active, else 0.0
    w_tracking  float   1.0 if tracking active, else 0.0
    w_catchup   float   1.0 if catchup active, else 0.0
    tick        int     Tick counter

================================================================================
  USAGE
================================================================================

  from guidance_lib import (
      GuidanceConfig, GuidanceState, compute_guidance,
      ned_to_gps, CommandSmoother, TargetComputer,
  )
  from failsafe_lib import FailsafeState, compute_failsafe, load_config

  # ---- Setup (once at startup) ----

  fs_cfg = load_config(home_lat=-35.363, home_lon=149.165)
  cfg = GuidanceConfig(max_altitude_m=fs_cfg.max_altitude_m)
  state = GuidanceState()
  smoother = CommandSmoother()
  target_computer = TargetComputer()
  fs_state = FailsafeState()

  OFFSET_N = -5.0   # meters north (negative = behind)
  OFFSET_E =  3.0   # meters east  (positive = right)

  # ---- Control loop (call at ~10Hz) ----

  while running:
      my_lat, my_lon, my_alt = get_follower_gps()
      my_vn, my_ve, my_vd   = get_follower_velocity()
      leader = get_leader_state()  # None if stale

      # Step 1: Failsafe check (runs FIRST)
      fs = compute_failsafe(
          own_lat=my_lat, own_lon=my_lon, own_alt=my_alt,
          own_gps_valid=True,
          peer_lat=leader['lat'] if leader else 0.0,
          peer_lon=leader['lon'] if leader else 0.0,
          peer_gps_valid=bool(leader),
          leader_fresh=leader is not None,
          cfg=fs_cfg, state=fs_state,
      )
      if not fs['safe']:
          send_velocity_command(0, 0, 0)
          if fs['action'] == 'RTL':
              trigger_rtl()
              break
          continue  # HOVER

      # Step 2: Compute target = leader + offset + feedforward
      goal_lat, goal_lon, goal_alt = target_computer.compute(
          leader['lat'], leader['lon'], leader['alt'],
          leader['vn'], leader['ve'],
          OFFSET_N, OFFSET_E, 0.0,
          ff_gain=cfg.ff_gain, dt=0.1,
      )

      # Step 3: Run guidance
      result = compute_guidance(
          my_lat, my_lon, my_alt,
          my_vn, my_ve, my_vd,
          leader['lat'], leader['lon'], leader['alt'],
          leader['vn'], leader['ve'],
          goal_lat, goal_lon, goal_alt,
          cfg, state,
      )

      # Step 4: Smooth & send
      vn, ve, vd = smoother.filter(result['vn'], result['ve'], result['vd'])
      send_velocity_command(vn, ve, vd)

      sleep(0.1)

================================================================================
  FAILSAFES
================================================================================

  All failsafe checks are handled by failsafe_lib. See failsafe_lib/readme.txt
  for the full list of checks, priority order, and configuration.

  The guidance module no longer contains embedded failsafe logic. When own GPS
  is invalid, compute_guidance() returns zero velocity with a NO_OWN_GPS flag,
  but does not escalate to DEADMAN or RTL — that is failsafe_lib's job.

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
  max_altitude_m         100.0     Altitude ceiling (sourced from failsafe_lib/config.yaml)
  deadzone_m             0.5       No tracking output below this error

================================================================================
  MODULAR CLASSES
================================================================================

  Each concern is a separate class for independent testing:

  TargetComputer      — Leader + offset + feedforward -> target (target.py)
  ModeSelector        — EVASION > CATCHUP > TRACKING switch (mode_selector.py)
  VelocityComputer    — Per-mode velocity computation (velocity.py)
  OutputSafety        — Speed cap, rate limiter, altitude clamps (output_safety.py)
  CommandSmoother     — EMA filter with hover deadband (command_smoother.py)

  compute_guidance() uses these internally and is backward compatible.
  You can also use the classes directly for custom architectures.

================================================================================
  EXTRA: compute_escape()
================================================================================

  A simpler function for emergency-only use. Pure radial repulsion away from
  the peer. No tracking, no catchup. Use this if you only need proximity
  avoidance (e.g. on the leader side, or as a failsafe fallback).

  from guidance_lib import compute_escape, GuidanceState

  escape_state = GuidanceState()  # REQUIRED — no global fallback

  result = compute_escape(
      my_lat, my_lon, my_alt,
      peer_lat, peer_lon, peer_alt,
      state=escape_state,
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
    Use TargetComputer.compute() or ned_to_gps() before calling compute_guidance().
  - The function handles bad data gracefully: NaN, Inf, zero GPS, stale
    data all produce safe zero-velocity output with appropriate flags.
  - compute_escape() no longer uses a global state variable. Pass your own
    GuidanceState instance for rate limiting to work across ticks.
