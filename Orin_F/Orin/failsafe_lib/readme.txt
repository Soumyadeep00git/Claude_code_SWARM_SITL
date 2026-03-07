================================================================================
  FAILSAFE LIBRARY
================================================================================

  Standalone safety monitor for leader-follower drone formation.
  Pure Python. Zero external dependencies. No coupling to guidance_lib.

  Single source of truth for all failsafe parameters (config.yaml).

================================================================================
  FILES
================================================================================

  failsafe.py    - All safety checks: geofence, deadman, stale data, altitude.
  config.yaml    - Single source of truth for all failsafe tunables.
  __init__.py    - Package exports + load_config().

================================================================================
  API
================================================================================

  load_config(path=None, home_lat=0.0, home_lon=0.0) -> FailsafeConfig

  Load FailsafeConfig from config.yaml. home_lat/lon are deployment params
  (from env), injected at runtime — they are NOT in the YAML file.

  compute_failsafe(
      own_lat, own_lon, own_alt, own_gps_valid,
      peer_lat, peer_lon, peer_gps_valid,
      leader_fresh, in_catchup,
      cfg=FailsafeConfig(), state=FailsafeState(),
  ) -> dict

  Call BEFORE guidance each tick. Returns dict with:
    safe        bool    True if all checks pass
    action      str     'CONTINUE', 'HOVER', or 'RTL'
    flags       dict    Warning flags (NO_OWN_GPS, DEADMAN, LEADER_STALE, etc.)
    emergency   bool    True if critical safety issue
    tick        int     Tick counter
    no_gps_counter   int   Consecutive ticks without own GPS
    stale_counter    int   Consecutive ticks with stale leader data
    catchup_ticks    int   Consecutive ticks in CATCHUP mode

  leader_in_oblivion(peer_lat, peer_lon, home_lat, home_lon, radius_m) -> bool

  Standalone utility. Returns True if leader is outside geofence radius.

================================================================================
  CHECKS (priority order)
================================================================================

  1. Own GPS validity       -> HOVER / RTL (DEADMAN after threshold)
  2. Leader data stale      -> HOVER (after threshold ticks)
  3. Leader in oblivion     -> HOVER (leader outside geofence)
  4. Follower geofence      -> RTL   (follower outside geofence)
  5. Altitude ceiling       -> HOVER (follower too high)
  6. Catchup timeout        -> RTL   (stuck in CATCHUP too long)

================================================================================
  CONFIG.YAML
================================================================================

  All failsafe tunables are defined in config.yaml (single source of truth):

    geofence_radius_m       200.0    HOVER/RTL if beyond this from home (m)
    deadman_ticks           20       Ticks without GPS before DEADMAN (RTL)
    leader_stale_ticks      50       Ticks before LEADER_STALE escalates
    catchup_timeout_ticks   300      RTL after this many CATCHUP ticks (0=off)
    max_altitude_m          100.0    HOVER if altitude exceeds this

  home_lat/lon are NOT in config.yaml — they are deployment params passed
  at runtime via load_config(home_lat=..., home_lon=...).

  max_altitude_m is also used by guidance_lib (OutputSafety) for gradual
  push-down. The caller reads it from fs_cfg.max_altitude_m and passes it
  to GuidanceConfig at construction time.

================================================================================
  USAGE
================================================================================

  from failsafe_lib import FailsafeState, compute_failsafe, load_config

  fs_cfg = load_config(home_lat=-35.363, home_lon=149.165)
  fs_state = FailsafeState()

  # Each tick:
  fs = compute_failsafe(
      own_lat=my_lat, own_lon=my_lon, own_alt=my_alt,
      own_gps_valid=True,
      peer_lat=leader_lat, peer_lon=leader_lon,
      peer_gps_valid=True,
      leader_fresh=True,
      in_catchup=False,
      cfg=fs_cfg, state=fs_state,
  )
  if not fs['safe']:
      if fs['action'] == 'RTL':
          trigger_rtl()
      else:
          hover()

================================================================================
  REQUIREMENTS
================================================================================

  Python 3.10+
  PyYAML (for load_config only; compute_failsafe has zero dependencies).
