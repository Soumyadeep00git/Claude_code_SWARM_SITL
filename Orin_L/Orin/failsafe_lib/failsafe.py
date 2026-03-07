"""Failsafe Monitor — single source of truth for all safety checks.

Mirrors the guidance.py pattern:
  - FailsafeConfig   — all tunables (geofence, stale thresholds, etc.)
  - FailsafeState    — persistent counters across ticks
  - compute_failsafe()  — top-level function, call BEFORE guidance each tick

Checks (priority order):
  1. Own GPS validity       → HOVER / LAND (DEADMAN after threshold)
  2. Leader data stale      → HOVER (after threshold ticks)
  3. Leader in oblivion     → HOVER (leader outside geofence)
  4. Follower geofence      → GEOFENCE_RETURN (radial push toward home)
  5. Altitude ceiling       → HOVER (follower too high)
  6. Catchup timeout        → RTL   (stuck in CATCHUP too long)

Returns dict with: safe, action, flags, emergency, counters.
"""

import math
from dataclasses import dataclass, field

_METERS_PER_DEG_LAT = 111_320.0
_EPS = 1e-6


# ═══════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════

@dataclass
class FailsafeConfig:
    """All failsafe tunables."""

    # Home position
    home_lat: float = 0.0
    home_lon: float = 0.0

    # Geofence
    geofence_radius_m: float = 200.0

    # Own GPS
    deadman_ticks: int = 20          # ticks with no GPS before DEADMAN (RTL)

    # Leader staleness
    leader_stale_ticks: int = 50     # ticks before LEADER_STALE escalates

    # Catchup timeout
    catchup_timeout_ticks: int = 300  # 0 = disabled

    # Altitude
    max_altitude_m: float = 100.0    # ceiling — HOVER if exceeded


# ═══════════════════════════════════════════════════════════════
# Persistent State
# ═══════════════════════════════════════════════════════════════

@dataclass
class FailsafeState:
    """Preserved across ticks. Mirrors GuidanceState pattern."""
    tick_count: int = 0
    no_gps_counter: int = 0        # consecutive ticks without own GPS
    stale_counter: int = 0         # consecutive ticks with stale leader data
    catchup_ticks: int = 0         # consecutive ticks in CATCHUP mode


# ═══════════════════════════════════════════════════════════════
# Internal helpers
# ═══════════════════════════════════════════════════════════════

def _gps_valid(lat: float, lon: float) -> bool:
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    if lat == 0.0 and lon == 0.0:
        return False
    return abs(lat) <= 90.0 and abs(lon) <= 180.0


def _dist_from_home(lat: float, lon: float,
                    home_lat: float, home_lon: float) -> float:
    """Flat-earth distance in meters from home."""
    dn = (lat - home_lat) * _METERS_PER_DEG_LAT
    de = ((lon - home_lon) * _METERS_PER_DEG_LAT
          * math.cos(math.radians(home_lat)))
    return math.sqrt(dn * dn + de * de + _EPS * _EPS)


def _result(safe, action, flags, emergency, state):
    return {
        'safe': safe,
        'action': action,          # 'CONTINUE', 'HOVER', 'GEOFENCE_RETURN', 'LAND', 'RTL'
        'flags': flags,
        'emergency': emergency,
        'tick': state.tick_count,
        'no_gps_counter': state.no_gps_counter,
        'stale_counter': state.stale_counter,
        'catchup_ticks': state.catchup_ticks,
    }


# ═══════════════════════════════════════════════════════════════
# MAIN: compute_failsafe
# ═══════════════════════════════════════════════════════════════

def compute_failsafe(
    own_lat: float,
    own_lon: float,
    own_alt: float,
    own_gps_valid: bool,
    peer_lat: float,
    peer_lon: float,
    peer_gps_valid: bool,
    leader_fresh: bool,
    in_catchup: bool = False,
    cfg: FailsafeConfig = None,
    state: FailsafeState = None,
) -> dict:
    """Run all failsafe checks. Call BEFORE guidance each tick.

    Args:
        own_lat, own_lon, own_alt: Follower GPS position.
        own_gps_valid: True if follower has a GPS fix.
        peer_lat, peer_lon: Leader GPS position (last known).
        peer_gps_valid: True if leader GPS is non-zero.
        leader_fresh: True if bridge returned data this tick (not stale).
        in_catchup: True if guidance was in CATCHUP mode last tick.
        cfg: Failsafe configuration (defaults if None).
        state: Persistent state across ticks (fresh if None).

    Returns:
        dict with keys: safe, action, flags, emergency, tick,
                        no_gps_counter, stale_counter, catchup_ticks
    """
    if cfg is None:
        cfg = FailsafeConfig()
    if state is None:
        state = FailsafeState()

    state.tick_count += 1
    flags = {}

    # ── 1. Own GPS validity ───────────────────────────────────
    # No GPS → HOVER briefly, then LAND (not RTL — can't navigate without GPS)
    if not own_gps_valid:
        state.no_gps_counter += 1
        flags['NO_OWN_GPS'] = True
        if state.no_gps_counter >= cfg.deadman_ticks:
            flags['DEADMAN'] = True
            return _result(False, 'LAND', flags, True, state)
        return _result(False, 'HOVER', flags, False, state)
    state.no_gps_counter = 0

    # ── 2. Leader data stale ──────────────────────────────────
    if not leader_fresh:
        state.stale_counter += 1
        flags['LEADER_STALE'] = True
        if state.stale_counter > cfg.leader_stale_ticks:
            flags['LEADER_STALE_CRITICAL'] = True
            return _result(False, 'HOVER', flags, True, state)
    else:
        state.stale_counter = 0

    # ── 3. Leader outside geofence ────────────────────────────
    home_ok = _gps_valid(cfg.home_lat, cfg.home_lon)
    if peer_gps_valid and home_ok:
        leader_dist = _dist_from_home(
            peer_lat, peer_lon, cfg.home_lat, cfg.home_lon)
        if leader_dist > cfg.geofence_radius_m:
            flags['LEADER_IN_OBLIVION'] = True
            flags['leader_home_dist'] = round(leader_dist, 1)
            return _result(False, 'HOVER', flags, True, state)

    # ── 4. Follower outside geofence → radial return ──────────
    if own_gps_valid and home_ok:
        follower_dist = _dist_from_home(
            own_lat, own_lon, cfg.home_lat, cfg.home_lon)
        if follower_dist > cfg.geofence_radius_m:
            flags['GEOFENCE'] = True
            flags['follower_home_dist'] = round(follower_dist, 1)
            return _result(False, 'GEOFENCE_RETURN', flags, False, state)

    # ── 5. Altitude ceiling ───────────────────────────────────
    if cfg.max_altitude_m > 0 and own_alt > cfg.max_altitude_m:
        flags['ALTITUDE_CEILING'] = True
        flags['own_alt'] = round(own_alt, 1)
        return _result(False, 'HOVER', flags, False, state)

    # ── 6. Catchup timeout ────────────────────────────────────
    if in_catchup:
        state.catchup_ticks += 1
    else:
        state.catchup_ticks = 0

    if (cfg.catchup_timeout_ticks > 0
            and state.catchup_ticks > cfg.catchup_timeout_ticks):
        flags['CATCHUP_TIMEOUT'] = True
        flags['catchup_ticks'] = state.catchup_ticks
        return _result(False, 'RTL', flags, True, state)

    # ── All clear ─────────────────────────────────────────────
    return _result(True, 'CONTINUE', flags, False, state)


# ═══════════════════════════════════════════════════════════════
# Convenience: leader_in_oblivion (standalone check)
# ═══════════════════════════════════════════════════════════════

def leader_in_oblivion(
    peer_lat: float, peer_lon: float,
    home_lat: float, home_lon: float,
    radius_m: float,
) -> bool:
    """Return True if the leader's position is outside the geofence."""
    if not (_gps_valid(peer_lat, peer_lon) and _gps_valid(home_lat, home_lon)):
        return False
    return _dist_from_home(peer_lat, peer_lon, home_lat, home_lon) > radius_m
