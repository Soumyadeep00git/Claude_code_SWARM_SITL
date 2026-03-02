"""3-Mode Hard-Switched Guidance Controller.

Orchestrator that delegates to modular sub-systems:
  - ModeSelector:      EVASION > CATCHUP > TRACKING priority switch
  - VelocityComputer:  per-mode velocity computation
  - OutputSafety:      speed cap, rate limiter, altitude floor/ceiling, NaN guard

All failsafe checks (geofence, deadman, catchup timeout, stale leader) are
handled by failsafe_lib. This module is pure guidance — no safety escalation.
"""

import math
from dataclasses import dataclass

from .mode_selector import ModeSelector, GuidanceMode
from .velocity import VelocityComputer
from .output_safety import OutputSafety
from .collision_avoidance import collision_filter

METERS_PER_DEG_LAT = 111_320.0
_EPS = 1e-6


# ═══════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════

@dataclass
class GuidanceConfig:
    """All guidance tunables."""

    # Tracking (PD + feedforward)
    kp: float = 0.7
    kd: float = 0.4
    ff_gain: float = 0.8

    # Catch-up (kinematic speed limiting)
    catchup_dist_m: float = 8.0
    max_catchup_speed: float = 17.0
    catchup_decel: float = 2.5

    # Evasion
    safety_dist_m: float = 1.0
    escape_speed: float = 3.0

    # Output limits
    max_speed: float = 17.0
    max_vertical_speed: float = 1.5
    max_accel: float = 0.25

    # Safety clamps
    min_altitude_m: float = 3.0
    critical_altitude_m: float = 1.5
    max_altitude_m: float = 100.0       # Altitude ceiling — sourced from failsafe_lib at runtime
    deadzone_m: float = 0.5


# ═══════════════════════════════════════════════════════════════
# Persistent state
# ═══════════════════════════════════════════════════════════════

@dataclass
class GuidanceState:
    """Preserved across ticks for rate limiting and counters."""
    prev_vn: float = 0.0
    prev_ve: float = 0.0
    prev_vd: float = 0.0
    tick_count: int = 0
    last_peer_dist: float = float('inf')
    catchup_ticks: int = 0


# ═══════════════════════════════════════════════════════════════
# Internal helpers
# ═══════════════════════════════════════════════════════════════

def _is_valid(v: float) -> bool:
    return math.isfinite(v) and abs(v) < 1e8


def _sanitize(v: float, fallback: float = 0.0) -> float:
    return v if _is_valid(v) else fallback


def _gps_valid(lat: float, lon: float) -> bool:
    if not (_is_valid(lat) and _is_valid(lon)):
        return False
    if lat == 0.0 and lon == 0.0:
        return False
    return abs(lat) <= 90.0 and abs(lon) <= 180.0


def _gps_to_ned(lat, lon, ref_lat, ref_lon):
    dn = (lat - ref_lat) * METERS_PER_DEG_LAT
    de = (lon - ref_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    return dn, de


def _mag(n, e):
    return math.sqrt(n * n + e * e + _EPS * _EPS)


# Shared module instances (stateless — safe to share)
_mode_selector = ModeSelector()
_velocity = VelocityComputer()


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
    neighbors: list = None,
    my_id: int = 0,
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

    # ── No own GPS: zero velocity, flag it (failsafe_lib handles escalation)
    if not my_ok:
        flags['NO_OWN_GPS'] = True
        return _result(0, 0, 0, float('inf'), False, flags, state,
                       0, 1, 0, 'TRACKING')

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

    # ── Mode selection (delegated to ModeSelector) ───────────
    mode = _mode_selector.select(
        peer_dist, err_mag, peer_ok, goal_ok,
        cfg.safety_dist_m, cfg.catchup_dist_m)

    vn, ve = 0.0, 0.0
    w_evasion, w_tracking, w_catchup = 0.0, 0.0, 0.0

    if mode == GuidanceMode.EVASION:
        # ── EVASION (delegated to VelocityComputer) ──────────
        w_evasion = 1.0
        state.catchup_ticks = 0
        vn, ve, ev_emg, ev_flags = _velocity.evasion(
            dn_peer, de_peer, peer_dist,
            cfg.escape_speed, cfg.safety_dist_m)
        emergency |= ev_emg
        flags.update(ev_flags)

    elif mode == GuidanceMode.CATCHUP:
        # ── CATCHUP (delegated to VelocityComputer) ──────────
        state.catchup_ticks += 1
        w_catchup = 1.0
        vn, ve = _velocity.catchup(
            err_n, err_e, err_mag,
            peer_vn, peer_ve,
            cfg.max_catchup_speed, cfg.ff_gain,
            cfg.catchup_dist_m, cfg.catchup_decel)

    else:
        # ── TRACKING (delegated to VelocityComputer) ─────────
        w_tracking = 1.0
        state.catchup_ticks = 0
        if goal_ok and err_mag > cfg.deadzone_m:
            vn, ve = _velocity.tracking(
                err_n, err_e, err_mag,
                my_vn, my_ve, peer_vn, peer_ve,
                cfg.kp, cfg.kd, cfg.ff_gain,
                cfg.max_speed, cfg.deadzone_m)

    # ── Vertical velocity (delegated to VelocityComputer) ────
    vd = 0.0
    if goal_ok:
        vd = _velocity.vertical(goal_alt, my_alt, cfg.max_vertical_speed)

    # ── Collision avoidance filter (3D repulsive blend) ─────
    if neighbors:
        vn, ve, vd, ca_flags = collision_filter(
            vn, ve, vd, my_lat, my_lon, my_alt, my_id, neighbors)
        flags.update(ca_flags)

    # ── Output safety (delegated to OutputSafety) ────────────
    # Use a temporary OutputSafety with state's prev_vel for backward compat
    _out = OutputSafety()
    _out.prev_vn = state.prev_vn
    _out.prev_ve = state.prev_ve
    _out.prev_vd = state.prev_vd

    mode_str = mode.value if isinstance(mode, GuidanceMode) else mode
    vn, ve, vd, out_emg, out_flags = _out.apply(
        vn, ve, vd, mode_str, my_alt,
        cfg.max_speed, cfg.max_catchup_speed,
        cfg.max_vertical_speed, cfg.max_accel,
        cfg.min_altitude_m, cfg.critical_altitude_m,
        cfg.max_altitude_m)

    emergency |= out_emg
    flags.update(out_flags)

    # Write back to state
    state.prev_vn = _out.prev_vn
    state.prev_ve = _out.prev_ve
    state.prev_vd = _out.prev_vd

    return _result(vn, ve, vd, peer_dist, emergency, flags, state,
                   w_evasion, w_tracking, w_catchup, mode_str)


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

def compute_escape(
    my_lat: float, my_lon: float, my_alt: float,
    peer_lat: float, peer_lon: float, peer_alt: float,
    my_vn: float = 0.0, my_ve: float = 0.0,
    peer_vn: float = 0.0, peer_ve: float = 0.0,
    safety_dist: float = 1.0,
    escape_speed: float = 3.0,
    max_accel: float = 0.25,
    state: GuidanceState = None,
) -> dict:
    """Simple escape — pure radial repulsion away from peer.

    NOTE: state parameter is REQUIRED (no global fallback).
    If not provided, a fresh state is created each call (no rate limiting).
    """
    if state is None:
        state = GuidanceState()

    state.tick_count += 1

    if not (_gps_valid(my_lat, my_lon) and _gps_valid(peer_lat, peer_lon)):
        return {'vn': 0, 've': 0, 'vd': 0, 'peer_dist': float('inf'),
                'emergency': False, 'flags': {'NO_GPS': True}}

    dn, de = _gps_to_ned(my_lat, my_lon, peer_lat, peer_lon)
    dist = _mag(dn, de)

    vn, ve, emergency, flags = _velocity.evasion(
        dn, de, dist, escape_speed, safety_dist)

    # Rate limit
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    dvn = _clamp(vn - state.prev_vn, -max_accel, max_accel)
    dve = _clamp(ve - state.prev_ve, -max_accel, max_accel)
    vn = state.prev_vn + dvn
    ve = state.prev_ve + dve
    state.prev_vn = vn
    state.prev_ve = ve

    return {'vn': vn, 've': ve, 'vd': 0.0, 'peer_dist': dist,
            'emergency': emergency, 'flags': flags}
