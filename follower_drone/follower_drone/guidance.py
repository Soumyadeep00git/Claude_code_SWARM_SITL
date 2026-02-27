"""3-Mode Hard-Switched Guidance Controller.

Modes (priority order):
  1. EVASION   — radial repulsion when peer is too close
  2. CATCHUP   — max speed toward target when error is large
  3. TRACKING  — PD + feedforward to maintain offset from leader

Hard if/else switches — only one mode active at a time.
Safety clamps (altitude floor, speed cap, rate limiter) applied on output.
"""

import math
import time
from dataclasses import dataclass, field

METERS_PER_DEG_LAT = 111320.0
_EPS = 1e-6


# ═══════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════

@dataclass
class GuidanceConfig:
    """All guidance tunables — 14 fields (was APFConfig's 28)."""

    # Tracking (PD + feedforward)
    kp: float = 0.7               # Position gain (m/s per m error)
    kd: float = 0.4               # Velocity damping coefficient
    ff_gain: float = 0.8          # Leader velocity feedforward (0=off, 1=full)

    # Catch-up
    catchup_dist_m: float = 8.0   # Error beyond this ramps catch-up weight
    max_catchup_speed: float = 4.0

    # Evasion
    safety_dist_m: float = 1.0    # Below this peer distance, evasion dominates
    escape_speed: float = 3.0     # Evasion output speed

    # Output limits
    max_speed: float = 3.0        # Horizontal speed cap (m/s)
    max_vertical_speed: float = 1.5
    max_accel: float = 4.0        # Rate limiter — max change per tick (m/s)

    # Safety clamps
    min_altitude_m: float = 3.0   # Soft altitude floor
    critical_altitude_m: float = 1.5  # Hard altitude floor
    deadzone_m: float = 0.5       # No tracking command below this error

    # Failsafes
    max_peer_dist_m: float = 50.0       # Hover if leader farther than this
    geofence_radius_m: float = 200.0    # Hover if this far from home
    home_lat: float = 0.0               # Home position for geofence
    home_lon: float = 0.0               # Home position for geofence
    catchup_timeout_ticks: int = 300    # Hover after this many consecutive CATCHUP ticks (300 = 30s at 10Hz)


# ═══════════════════════════════════════════════════════════════
# Persistent state
# ═══════════════════════════════════════════════════════════════

@dataclass
class GuidanceState:
    """Preserved across ticks for rate limiting."""
    prev_vn: float = 0.0
    prev_ve: float = 0.0
    prev_vd: float = 0.0
    tick_count: int = 0
    no_data_counter: int = 0
    last_peer_dist: float = float('inf')
    catchup_ticks: int = 0


# ═══════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════

def _is_valid(v: float) -> bool:
    return math.isfinite(v) and abs(v) < 1e8


def _sanitize(v: float, fallback: float = 0.0) -> float:
    return v if _is_valid(v) else fallback


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _gps_valid(lat: float, lon: float) -> bool:
    if not (_is_valid(lat) and _is_valid(lon)):
        return False
    if lat == 0.0 and lon == 0.0:
        return False
    return abs(lat) <= 90.0 and abs(lon) <= 180.0


def _gps_to_ned(lat: float, lon: float, ref_lat: float, ref_lon: float):
    dn = (lat - ref_lat) * METERS_PER_DEG_LAT
    de = (lon - ref_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    return dn, de


def _mag(n: float, e: float) -> float:
    return math.sqrt(n * n + e * e + _EPS * _EPS)


# ═══════════════════════════════════════════════════════════════
# MAIN: compute_guidance
# ═══════════════════════════════════════════════════════════════

def compute_guidance(
    my_lat: float, my_lon: float, my_alt: float,
    my_vn: float, my_ve: float, my_vd: float,
    peer_lat: float, peer_lon: float, peer_alt: float,
    peer_vn: float, peer_ve: float,
    goal_lat: float, goal_lon: float, goal_alt: float,
    cfg: GuidanceConfig = None,
    state: GuidanceState = None,
) -> dict:
    """Compute hard-switched velocity command.

    Returns dict with:
        vn, ve, vd, speed, peer_dist, emergency, flags, tick,
        w_evasion, w_tracking, w_catchup, mode
    """
    if cfg is None:
        cfg = GuidanceConfig()
    if state is None:
        state = GuidanceState()

    state.tick_count += 1
    flags = {}

    # ── Sanitize inputs ──────────────────────────────────────
    my_lat = _sanitize(my_lat)
    my_lon = _sanitize(my_lon)
    my_alt = _sanitize(my_alt)
    my_vn = _sanitize(my_vn)
    my_ve = _sanitize(my_ve)
    my_vd = _sanitize(my_vd)
    peer_lat = _sanitize(peer_lat)
    peer_lon = _sanitize(peer_lon)
    peer_vn = _sanitize(peer_vn)
    peer_ve = _sanitize(peer_ve)
    goal_lat = _sanitize(goal_lat)
    goal_lon = _sanitize(goal_lon)
    goal_alt = _sanitize(goal_alt, my_alt)

    my_ok = _gps_valid(my_lat, my_lon)
    peer_ok = _gps_valid(peer_lat, peer_lon)
    goal_ok = _gps_valid(goal_lat, goal_lon)

    if not my_ok:
        flags['NO_OWN_GPS'] = True
        state.no_data_counter += 1
        if state.no_data_counter >= 20:
            flags['DEADMAN'] = True
            return _result(0, 0, 0, float('inf'), True, flags, state,
                           0, 1, 0, 'TRACKING')
        return _result(0, 0, 0, float('inf'), False, flags, state,
                       0, 1, 0, 'TRACKING')
    state.no_data_counter = 0

    # ── Compute vectors ──────────────────────────────────────
    peer_dist = float('inf')
    dn_peer, de_peer = 0.0, 0.0
    if peer_ok:
        dn_peer, de_peer = _gps_to_ned(my_lat, my_lon, peer_lat, peer_lon)
        peer_dist = _mag(dn_peer, de_peer)

    err_n, err_e, err_mag = 0.0, 0.0, 0.0
    if goal_ok:
        err_n, err_e = _gps_to_ned(goal_lat, goal_lon, my_lat, my_lon)
        err_mag = _mag(err_n, err_e)

    state.last_peer_dist = peer_dist
    emergency = False

    # ── Failsafe: leader too far ──────────────────────────────
    if peer_ok and peer_dist > cfg.max_peer_dist_m:
        flags['LEADER_TOO_FAR'] = True
        state.prev_vn, state.prev_ve, state.prev_vd = 0.0, 0.0, 0.0
        return _result(0, 0, 0, peer_dist, True, flags, state,
                       0, 0, 0, 'FAILSAFE')

    # ── Failsafe: geofence ────────────────────────────────────
    if _gps_valid(cfg.home_lat, cfg.home_lon) and my_ok:
        home_dist_n, home_dist_e = _gps_to_ned(my_lat, my_lon,
                                                cfg.home_lat, cfg.home_lon)
        home_dist = _mag(home_dist_n, home_dist_e)
        if home_dist > cfg.geofence_radius_m:
            flags['GEOFENCE'] = True
            state.prev_vn, state.prev_ve, state.prev_vd = 0.0, 0.0, 0.0
            return _result(0, 0, 0, peer_dist, True, flags, state,
                           0, 0, 0, 'FAILSAFE')

    # ── Hard mode selection (priority: EVASION > CATCHUP > TRACKING) ─
    vn, ve = 0.0, 0.0
    w_evasion, w_tracking, w_catchup = 0.0, 0.0, 0.0

    if peer_ok and peer_dist < cfg.safety_dist_m:
        # ── EVASION: radial repulsion (push straight away from peer) ─
        mode = 'EVASION'
        w_evasion = 1.0
        state.catchup_ticks = 0

        rad_n = dn_peer / peer_dist
        rad_e = de_peer / peer_dist

        # Stronger push when closer: full speed at 0m, linear fade to 0 at safety_dist
        strength = cfg.escape_speed * _clamp(1.0 - peer_dist / cfg.safety_dist_m, 0.0, 1.0)
        vn = strength * rad_n
        ve = strength * rad_e

        if peer_dist < cfg.safety_dist_m * 0.5:
            emergency = True
            flags['EMERGENCY_PROXIMITY'] = True

    elif goal_ok and err_mag > cfg.catchup_dist_m:
        # ── CATCHUP: sprint toward target + leader feedforward ───
        state.catchup_ticks += 1

        if cfg.catchup_timeout_ticks > 0 and state.catchup_ticks > cfg.catchup_timeout_ticks:
            flags['CATCHUP_TIMEOUT'] = True
            state.prev_vn, state.prev_ve, state.prev_vd = 0.0, 0.0, 0.0
            return _result(0, 0, 0, peer_dist, True, flags, state,
                           0, 0, 1, 'FAILSAFE')

        mode = 'CATCHUP'
        w_catchup = 1.0

        u_n = err_n / err_mag
        u_e = err_e / err_mag
        vn = cfg.max_catchup_speed * u_n + cfg.ff_gain * peer_vn
        ve = cfg.max_catchup_speed * u_e + cfg.ff_gain * peer_ve
        c_speed = _mag(vn, ve)
        if c_speed > cfg.max_catchup_speed:
            vn *= cfg.max_catchup_speed / c_speed
            ve *= cfg.max_catchup_speed / c_speed

    else:
        # ── TRACKING: PD + feedforward ───────────────────────────
        mode = 'TRACKING'
        w_tracking = 1.0
        state.catchup_ticks = 0

        if goal_ok and err_mag > cfg.deadzone_m:
            vn = cfg.kp * err_n - cfg.kd * (my_vn - peer_vn) + cfg.ff_gain * peer_vn
            ve = cfg.kp * err_e - cfg.kd * (my_ve - peer_ve) + cfg.ff_gain * peer_ve
            t_speed = _mag(vn, ve)
            if t_speed > cfg.max_speed:
                vn *= cfg.max_speed / t_speed
                ve *= cfg.max_speed / t_speed

    # ── Vertical: altitude error + ground floor ──────────────
    vd = 0.0
    if goal_ok:
        alt_err = goal_alt - my_alt
        vd = _clamp(-alt_err * 0.5, -cfg.max_vertical_speed, cfg.max_vertical_speed)

    # Soft altitude floor
    if _is_valid(my_alt) and my_alt < cfg.min_altitude_m:
        range_w = cfg.min_altitude_m - cfg.critical_altitude_m
        if range_w > _EPS:
            t = _clamp((cfg.min_altitude_m - my_alt) / range_w, 0.0, 1.0)
            vd = min(vd, -3.0 * t * t)  # Push up

    # Hard altitude floor
    if _is_valid(my_alt) and my_alt < cfg.critical_altitude_m:
        vd = -cfg.max_vertical_speed
        flags['HARD_ALTITUDE_FLOOR'] = True
        emergency = True

    # ── Output clamps ────────────────────────────────────────
    # Speed cap
    h_speed = _mag(vn, ve)
    speed_limit = cfg.max_catchup_speed if mode == 'CATCHUP' else cfg.max_speed
    if h_speed > speed_limit:
        vn *= speed_limit / h_speed
        ve *= speed_limit / h_speed

    vd = _clamp(vd, -cfg.max_vertical_speed, cfg.max_vertical_speed)

    # Rate limiter
    dvn = _clamp(vn - state.prev_vn, -cfg.max_accel, cfg.max_accel)
    dve = _clamp(ve - state.prev_ve, -cfg.max_accel, cfg.max_accel)
    dvd = _clamp(vd - state.prev_vd, -cfg.max_accel, cfg.max_accel)
    vn = state.prev_vn + dvn
    ve = state.prev_ve + dve
    vd = state.prev_vd + dvd

    # NaN guard
    if not (_is_valid(vn) and _is_valid(ve) and _is_valid(vd)):
        flags['OUTPUT_NAN'] = True
        vn, ve, vd = 0.0, 0.0, 0.0
        emergency = True

    # Update state
    state.prev_vn = vn
    state.prev_ve = ve
    state.prev_vd = vd

    return _result(vn, ve, vd, peer_dist, emergency, flags, state,
                   w_evasion, w_tracking, w_catchup, mode)


def _result(vn, ve, vd, peer_dist, emergency, flags, state,
            w_evasion, w_tracking, w_catchup, mode):
    return {
        'vn': vn, 've': ve, 'vd': vd,
        'speed': _mag(vn, ve),
        'peer_dist': peer_dist,
        'emergency': emergency,
        'flags': flags,
        'tick': state.tick_count,
        'w_evasion': w_evasion,
        'w_tracking': w_tracking,
        'w_catchup': w_catchup,
        'mode': mode,
    }


# ═══════════════════════════════════════════════════════════════
# ESCAPE: Simple radial push for PROX_AVOID (failsafe/leader)
# ═══════════════════════════════════════════════════════════════

_escape_state = GuidanceState()


def compute_escape(
    my_lat: float, my_lon: float, my_alt: float,
    peer_lat: float, peer_lon: float, peer_alt: float,
    my_vn: float = 0.0, my_ve: float = 0.0,
    peer_vn: float = 0.0, peer_ve: float = 0.0,
    safety_dist: float = 1.0,
    escape_speed: float = 3.0,
    max_accel: float = 4.0,
    state: GuidanceState = None,
) -> dict:
    """Simple escape for PROX_AVOID — pure radial repulsion away from peer."""
    if state is None:
        global _escape_state
        state = _escape_state

    state.tick_count += 1

    if not (_gps_valid(my_lat, my_lon) and _gps_valid(peer_lat, peer_lon)):
        return {'vn': 0, 've': 0, 'vd': 0, 'peer_dist': float('inf'),
                'emergency': False, 'flags': {'NO_GPS': True}}

    dn, de = _gps_to_ned(my_lat, my_lon, peer_lat, peer_lon)
    dist = _mag(dn, de)

    # Pure radial repulsion: push straight away from peer
    rad_n = dn / dist
    rad_e = de / dist

    # Linear ramp: full speed at 0m, fading to 0 at safety_dist
    strength = escape_speed * _clamp(1.0 - dist / safety_dist, 0.0, 1.0)

    vn = strength * rad_n
    ve = strength * rad_e
    vd = 0.0

    # Rate limit
    dvn = _clamp(vn - state.prev_vn, -max_accel, max_accel)
    dve = _clamp(ve - state.prev_ve, -max_accel, max_accel)
    vn = state.prev_vn + dvn
    ve = state.prev_ve + dve
    state.prev_vn = vn
    state.prev_ve = ve

    emergency = dist < safety_dist * 0.5
    flags = {}
    if emergency:
        flags['EMERGENCY_PROXIMITY'] = True

    return {'vn': vn, 've': ve, 'vd': vd, 'peer_dist': dist,
            'emergency': emergency, 'flags': flags}
