"""
5-Layer Adaptive Potential Field with Comprehensive Failsafes.

Layers:
  1. ATTRACTIVE   — clamped parabolic pull toward goal offset
  2. REPULSIVE    — velocity-predictive two-zone push from peer
  3. VORTEX       — tangential circulation to escape local minima
  4. GROUND PLANE — altitude floor repulsion
  5. GEOFENCE     — virtual boundary walls

Failsafes:
  - NaN/Inf guard on all inputs and outputs
  - GPS validity check (zero-position rejection)
  - Stale data detection (timestamps)
  - Force saturation (never exceed max acceleration)
  - Velocity hard limiter (absolute speed cap)
  - Altitude hard floor (unconditional vertical override)
  - Divergence detector (position error growing = something wrong)
  - Predictive collision (lookahead T seconds, emergency stop)
  - Output smoothing (rate limiter prevents actuator saturation)
  - Deadman switch (no valid data for N cycles = hold/RTL)
  - Numerical stability (epsilon guards on all divisions)
"""

import math
import time
from dataclasses import dataclass, field

METERS_PER_DEG_LAT = 111320.0
_EPS = 1e-6  # Numerical stability guard


# ═══════════════════════════════════════════════════════════════
# Configuration dataclass — all tunables in one place
# ═══════════════════════════════════════════════════════════════

@dataclass
class APFConfig:
    """All potential field parameters. Set from ROS2 YAML at startup."""

    # ── Layer 1: Attractive ──────────────────────────────────
    kp_attract: float = 0.7          # Position gain (m/s per m error)
    attract_max_force: float = 3.0   # Cap on attractive velocity (m/s)
    attract_deadzone_m: float = 1.0  # No force below this error (hover instead of jitter)
    approach_brake_ms2: float = 1.5  # Assumed deceleration for braking curve (m/s²)

    # ── Layer 2: Repulsive ───────────────────────────────────
    hard_radius_m: float = 1.0       # Exponential barrier zone
    soft_radius_m: float = 2.0      # Gentle repulsion starts
    repel_gain: float = 2.5          # Base repulsion strength (m/s)
    closing_speed_scale: float = 0.5 # Extra repulsion per m/s closing
    soft_zone_expansion: float = 1.5 # Soft zone grows by this * closing_speed
    near_target_rep_atten_m: float = 3.0  # Attenuate repulsive when goal error < this
    lookahead_s: float = 1.5         # Predict peer position T seconds ahead
    emergency_radius_m: float = 2.0  # Absolute emergency — full stop + max repel

    # ── Layer 3: Vortex ──────────────────────────────────────
    vortex_gain: float = 1.5         # Tangential force strength
    vortex_activation: float = 0.5   # Alignment threshold (0-1, higher = more selective)
    vortex_direction: float = 1.0    # +1 = clockwise, -1 = counter-clockwise

    # ── Layer 4: Ground plane ────────────────────────────────
    min_altitude_m: float = 3.0      # Soft floor (repulsion starts)
    critical_altitude_m: float = 1.5 # Hard floor (override all commands)
    ground_repel_gain: float = 3.0   # Upward push strength

    # ── Layer 5: Geofence ────────────────────────────────────
    geofence_enabled: bool = False
    geofence_radius_m: float = 100.0 # Cylindrical geofence radius from home
    geofence_margin_m: float = 20.0  # Repulsion starts this far from boundary
    geofence_gain: float = 2.0       # Inward push strength
    geofence_center_lat: float = 0.0 # Home latitude
    geofence_center_lon: float = 0.0 # Home longitude

    # ── Global limits ────────────────────────────────────────
    max_speed_ms: float = 3.0        # Absolute horizontal speed cap
    max_vertical_speed_ms: float = 1.5  # Absolute vertical speed cap
    max_accel_ms2: float = 4.0       # Rate limiter (m/s per tick)
    total_force_cap: float = 5.0     # Cap on combined force magnitude

    # ── Failsafe thresholds ──────────────────────────────────
    stale_threshold_s: float = 2.0   # Data older than this = stale
    deadman_cycles: int = 20         # No valid data for N ticks = emergency
    divergence_threshold_m: float = 50.0  # Error growing beyond this = abort
    max_position_jump_m: float = 20.0     # GPS jump > this in 1 tick = reject


# ═══════════════════════════════════════════════════════════════
# APF State — persistent across ticks
# ═══════════════════════════════════════════════════════════════

@dataclass
class APFState:
    """Internal state preserved between calls for rate limiting and detection."""
    prev_vn: float = 0.0
    prev_ve: float = 0.0
    prev_vd: float = 0.0
    prev_my_lat: float = 0.0
    prev_my_lon: float = 0.0
    prev_peer_lat: float = 0.0
    prev_peer_lon: float = 0.0
    prev_dist: float = float('inf')
    no_data_counter: int = 0
    tick_count: int = 0
    last_valid_tick: int = 0
    emergency_active: bool = False
    divergence_error: float = 0.0
    flags: dict = field(default_factory=dict)


# ═══════════════════════════════════════════════════════════════
# Helper functions
# ═══════════════════════════════════════════════════════════════

def _is_valid(v: float) -> bool:
    """Check float is not NaN, Inf, or unreasonably large."""
    return math.isfinite(v) and abs(v) < 1e8


def _sanitize(v: float, fallback: float = 0.0) -> float:
    """Replace bad floats with fallback."""
    return v if _is_valid(v) else fallback


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _vec_len(n: float, e: float) -> float:
    return math.sqrt(n * n + e * e + _EPS * _EPS)


def _gps_valid(lat: float, lon: float) -> bool:
    """Reject zero, NaN, and out-of-range GPS."""
    if not (_is_valid(lat) and _is_valid(lon)):
        return False
    if lat == 0.0 and lon == 0.0:
        return False
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return False
    return True


def _gps_to_ned_2d(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """GPS to NED meters (2D)."""
    dn = (lat - ref_lat) * METERS_PER_DEG_LAT
    de = (lon - ref_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    return dn, de


def _gps_dist_2d(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dn, de = _gps_to_ned_2d(lat2, lon2, lat1, lon1)
    return _vec_len(dn, de)


# ═══════════════════════════════════════════════════════════════
# LAYER 1: Attractive field — clamped parabolic toward goal
# ═══════════════════════════════════════════════════════════════

def _attractive_force(
    err_n: float, err_e: float, dist: float, cfg: APFConfig,
    my_vn: float = 0.0, my_ve: float = 0.0, kd: float = 0.0,
) -> tuple[float, float]:
    """Kinematic-aware PD control toward goal.

    Braking curve caps commanded speed to what the drone can decelerate
    to zero within the remaining distance — prevents overshoot.
    When kd > 0, a damping term opposes excess closing velocity.
    """
    if dist < cfg.attract_deadzone_m:
        return 0.0, 0.0

    fn = cfg.kp_attract * err_n
    fe = cfg.kp_attract * err_e

    # Velocity damping (D-term)
    if kd > 0.0:
        u_n = err_n / max(dist, _EPS)
        u_e = err_e / max(dist, _EPS)
        v_closing = my_vn * u_n + my_ve * u_e
        fn -= kd * v_closing * u_n
        fe -= kd * v_closing * u_e

    # Kinematic braking curve: v_safe = sqrt(2 * a_brake * d)
    if cfg.approach_brake_ms2 > 0.0:
        v_safe = math.sqrt(2.0 * cfg.approach_brake_ms2 * dist)
        mag = _vec_len(fn, fe)
        if mag > v_safe:
            fn *= v_safe / mag
            fe *= v_safe / mag

    # Cap magnitude
    mag = _vec_len(fn, fe)
    if mag > cfg.attract_max_force:
        fn *= cfg.attract_max_force / mag
        fe *= cfg.attract_max_force / mag

    return fn, fe


# ═══════════════════════════════════════════════════════════════
# LAYER 2: Repulsive field — velocity-predictive two-zone
# ═══════════════════════════════════════════════════════════════

def _orthogonal_escape(
    un: float, ue: float,
    peer_vn: float, peer_ve: float,
) -> tuple[float, float]:
    """Pick the orthogonal direction that dodges AWAY from the peer's velocity.

    Two candidates: rotate escape vector 90 CW or 90 CCW.
    Pick the one with negative dot product against peer velocity
    (i.e. moves away from where the peer is heading).
    """
    # Candidate A: 90 CW  (-ue, un)
    # Candidate B: 90 CCW (ue, -un)
    peer_spd = math.sqrt(peer_vn * peer_vn + peer_ve * peer_ve + _EPS * _EPS)
    if peer_spd < 0.3:
        # Peer barely moving — default to CW
        return -ue, un

    # Peer velocity unit vector
    pvn = peer_vn / peer_spd
    pve = peer_ve / peer_spd

    # Dot each candidate with peer velocity — pick the more negative one
    dot_a = (-ue) * pvn + un * pve
    dot_b = ue * pvn + (-un) * pve

    if dot_a <= dot_b:
        return -ue, un   # CW
    else:
        return ue, -un    # CCW


def _blend_escape(
    un: float, ue: float,
    on: float, oe: float,
    closing_speed: float,
    strength: float,
) -> tuple[float, float]:
    """Blend direct escape with orthogonal dodge based on closing speed.

    At 0 m/s closing → 100% direct (peer hovering, just push away)
    At 3+ m/s closing → 70% orthogonal (peer charging, dodge sideways)
    """
    # Blend factor: 0.0 = all direct, 1.0 = all orthogonal
    # Ramp from 0 at 0 m/s to 0.7 at 3 m/s closing
    ortho_blend = _clamp(closing_speed / 3.0 * 0.7, 0.0, 0.7)

    bn = (1.0 - ortho_blend) * un + ortho_blend * on
    be = (1.0 - ortho_blend) * ue + ortho_blend * oe

    # Re-normalize and apply strength
    mag = math.sqrt(bn * bn + be * be + _EPS * _EPS)
    return strength * bn / mag, strength * be / mag


def _repulsive_force(
    dn: float, de: float, dist: float,
    my_vn: float, my_ve: float,
    peer_vn: float, peer_ve: float,
    cfg: APFConfig,
) -> tuple[float, float, float, bool]:
    """
    Two-zone repulsion with velocity prediction and orthogonal escape.

    When the peer is closing fast, the escape vector blends toward
    perpendicular to the peer's velocity — dodging off the collision
    line rather than retreating along it.

    Returns:
        (fn, fe, closing_speed, is_emergency)
    """
    if dist < _EPS:
        # Co-located — push north hard
        return cfg.total_force_cap, 0.0, 0.0, True

    # Unit escape vector (FROM peer TO self)
    un = dn / dist
    ue = de / dist

    # Orthogonal escape direction (away from peer's velocity)
    on, oe = _orthogonal_escape(un, ue, peer_vn, peer_ve)

    # Closing speed
    rel_vn = peer_vn - my_vn
    rel_ve = peer_ve - my_ve
    closing_speed = max(-(rel_vn * un + rel_ve * ue), 0.0)

    # Predictive distance: where will peer be in T seconds?
    pred_dist = dist - closing_speed * cfg.lookahead_s
    pred_dist = max(pred_dist, 0.1)

    # EMERGENCY: absolute minimum distance — orthogonal dodge
    if dist < cfg.emergency_radius_m:
        strength = cfg.total_force_cap
        fn, fe = _blend_escape(un, ue, on, oe, closing_speed, strength)
        return fn, fe, closing_speed, True

    # Adaptive soft zone
    adaptive_soft = cfg.soft_radius_m + cfg.soft_zone_expansion * closing_speed

    if dist >= adaptive_soft and pred_dist >= cfg.hard_radius_m:
        return 0.0, 0.0, closing_speed, False

    # Use the more threatening of actual vs predicted distance
    effective_dist = min(dist, pred_dist)

    # Adaptive gain
    gain = cfg.repel_gain + cfg.closing_speed_scale * closing_speed

    if effective_dist < cfg.hard_radius_m:
        # HARD zone: exponential barrier
        strength = gain * math.exp(cfg.hard_radius_m / max(effective_dist, 0.3) - 1.0)
        # Cap to prevent absurd values
        strength = min(strength, cfg.total_force_cap)
        fn, fe = _blend_escape(un, ue, on, oe, closing_speed, strength)
    else:
        # SOFT zone: quadratic ramp (keep direct — gentle nudge, no dodge needed)
        range_width = adaptive_soft - cfg.hard_radius_m
        if range_width < _EPS:
            t = 1.0
        else:
            t = (adaptive_soft - effective_dist) / range_width
        t = _clamp(t, 0.0, 1.0)
        strength = gain * t * t
        fn = strength * un
        fe = strength * ue

    return fn, fe, closing_speed, False


# ═══════════════════════════════════════════════════════════════
# LAYER 3: Vortex field — tangential circulation around obstacle
# ═══════════════════════════════════════════════════════════════

def _vortex_force(
    dn_peer: float, de_peer: float, dist_peer: float,
    err_n: float, err_e: float, dist_goal: float,
    cfg: APFConfig,
) -> tuple[float, float]:
    """
    Tangential force that rotates the drone around the peer
    when the peer is between the drone and its goal.

    Only activates when the goal is "behind" the peer (alignment > threshold).
    """
    if dist_peer < _EPS or dist_goal < cfg.attract_deadzone_m:
        return 0.0, 0.0

    # Unit vector: self → peer (toward obstacle)
    to_peer_n = -dn_peer / dist_peer
    to_peer_e = -de_peer / dist_peer

    # Unit vector: self → goal
    to_goal_n = err_n / max(dist_goal, _EPS)
    to_goal_e = err_e / max(dist_goal, _EPS)

    # Alignment: how much the peer is "in the way" of the goal
    # dot(to_peer, to_goal) → 1.0 = peer directly between us and goal
    alignment = to_peer_n * to_goal_n + to_peer_e * to_goal_e

    if alignment < cfg.vortex_activation:
        return 0.0, 0.0

    # Tangential direction: rotate escape vector 90 degrees
    # direction = +1 for clockwise, -1 for counter-clockwise
    tan_n = -de_peer / dist_peer * cfg.vortex_direction
    tan_e = dn_peer / dist_peer * cfg.vortex_direction

    # Strength: proportional to alignment and inversely to distance
    # Fades as we get farther from the obstacle
    dist_factor = max(0.0, 1.0 - dist_peer / cfg.soft_radius_m)
    strength = cfg.vortex_gain * alignment * dist_factor

    return strength * tan_n, strength * tan_e


# ═══════════════════════════════════════════════════════════════
# LAYER 4: Ground plane — altitude floor repulsion
# ═══════════════════════════════════════════════════════════════

def _ground_force(alt: float, cfg: APFConfig) -> float:
    """
    Upward force when altitude drops below minimum.
    Returns vertical velocity adjustment (negative = up in NED).
    """
    if not _is_valid(alt):
        # Unknown altitude — push up conservatively
        return -cfg.ground_repel_gain

    if alt <= cfg.critical_altitude_m:
        # HARD FLOOR — unconditional maximum upward push
        return -cfg.total_force_cap

    if alt < cfg.min_altitude_m:
        # Soft zone — quadratic ramp
        range_width = cfg.min_altitude_m - cfg.critical_altitude_m
        if range_width < _EPS:
            t = 1.0
        else:
            t = (cfg.min_altitude_m - alt) / range_width
        t = _clamp(t, 0.0, 1.0)
        return -cfg.ground_repel_gain * t * t

    return 0.0


# ═══════════════════════════════════════════════════════════════
# LAYER 5: Geofence — virtual boundary walls
# ═══════════════════════════════════════════════════════════════

def _geofence_force(
    my_lat: float, my_lon: float, cfg: APFConfig
) -> tuple[float, float]:
    """
    Inward repulsion when approaching cylindrical geofence boundary.
    """
    if not cfg.geofence_enabled:
        return 0.0, 0.0
    if not _gps_valid(cfg.geofence_center_lat, cfg.geofence_center_lon):
        return 0.0, 0.0

    dn, de = _gps_to_ned_2d(my_lat, my_lon, cfg.geofence_center_lat, cfg.geofence_center_lon)
    dist_from_center = _vec_len(dn, de)

    # Distance to fence boundary
    dist_to_fence = cfg.geofence_radius_m - dist_from_center

    if dist_to_fence >= cfg.geofence_margin_m:
        return 0.0, 0.0

    if dist_from_center < _EPS:
        return 0.0, 0.0

    # Unit inward vector (toward center)
    in_n = -dn / dist_from_center
    in_e = -de / dist_from_center

    if dist_to_fence <= 0.0:
        # OUTSIDE fence — maximum inward force
        return cfg.total_force_cap * in_n, cfg.total_force_cap * in_e

    # Quadratic ramp inside margin
    t = 1.0 - dist_to_fence / cfg.geofence_margin_m
    t = _clamp(t, 0.0, 1.0)
    strength = cfg.geofence_gain * t * t

    return strength * in_n, strength * in_e


# ═══════════════════════════════════════════════════════════════
# FAILSAFE: Velocity cancellation — remove component toward peer
# ═══════════════════════════════════════════════════════════════

def _cancel_closing_velocity(
    cmd_vn: float, cmd_ve: float,
    un: float, ue: float,
) -> tuple[float, float]:
    """Remove the component of commanded velocity moving toward the peer."""
    closing = -(cmd_vn * un + cmd_ve * ue)
    if closing > 0:
        cmd_vn += closing * un
        cmd_ve += closing * ue
    return cmd_vn, cmd_ve


# ═══════════════════════════════════════════════════════════════
# FAILSAFE: Rate limiter — prevent actuator saturation
# ═══════════════════════════════════════════════════════════════

def _rate_limit(
    vn: float, ve: float, vd: float,
    prev_vn: float, prev_ve: float, prev_vd: float,
    max_accel: float,
) -> tuple[float, float, float]:
    """Limit velocity change per tick to prevent actuator saturation."""
    dvn = _clamp(vn - prev_vn, -max_accel, max_accel)
    dve = _clamp(ve - prev_ve, -max_accel, max_accel)
    dvd = _clamp(vd - prev_vd, -max_accel, max_accel)
    return prev_vn + dvn, prev_ve + dve, prev_vd + dvd


# ═══════════════════════════════════════════════════════════════
# FAILSAFE: GPS jump rejection
# ═══════════════════════════════════════════════════════════════

def _check_gps_jump(
    lat: float, lon: float,
    prev_lat: float, prev_lon: float,
    threshold_m: float,
) -> bool:
    """Return True if GPS jumped unreasonably far in one tick."""
    if prev_lat == 0.0 and prev_lon == 0.0:
        return False  # First reading
    if not (_gps_valid(lat, lon) and _gps_valid(prev_lat, prev_lon)):
        return False
    dist = _gps_dist_2d(prev_lat, prev_lon, lat, lon)
    return dist > threshold_m


# ═══════════════════════════════════════════════════════════════
# MAIN ENTRY POINT
# ═══════════════════════════════════════════════════════════════

def compute_guidance(
    # Own state
    my_lat: float, my_lon: float, my_alt: float,
    my_vn: float, my_ve: float, my_vd: float,
    # Peer state
    peer_lat: float, peer_lon: float, peer_alt: float,
    peer_vn: float, peer_ve: float,
    # Goal (desired position for the follower, or no-op for leader)
    goal_lat: float, goal_lon: float, goal_alt: float,
    # Timestamps for staleness detection
    my_data_time: float = 0.0,
    peer_data_time: float = 0.0,
    # Adaptive damping
    kd_attract: float = 0.0,
    # Config and state
    cfg: APFConfig = None,
    state: APFState = None,
) -> dict:
    """
    Compute the full 5-layer APF guidance output.

    Returns dict with:
        vn, ve, vd      — commanded velocity (NED, m/s)
        speed            — horizontal speed magnitude
        peer_dist        — distance to peer (m)
        emergency        — True if emergency failsafe active
        flags            — dict of active failsafe flags
    """
    if cfg is None:
        cfg = APFConfig()
    if state is None:
        state = APFState()

    now = time.time()
    state.tick_count += 1
    flags = {}

    # ── FAILSAFE: Sanitize all inputs ────────────────────────
    my_lat = _sanitize(my_lat)
    my_lon = _sanitize(my_lon)
    my_alt = _sanitize(my_alt, 0.0)
    my_vn = _sanitize(my_vn)
    my_ve = _sanitize(my_ve)
    my_vd = _sanitize(my_vd)
    peer_lat = _sanitize(peer_lat)
    peer_lon = _sanitize(peer_lon)
    peer_alt = _sanitize(peer_alt, 0.0)
    peer_vn = _sanitize(peer_vn)
    peer_ve = _sanitize(peer_ve)
    goal_lat = _sanitize(goal_lat)
    goal_lon = _sanitize(goal_lon)
    goal_alt = _sanitize(goal_alt, my_alt)

    emergency = False
    vn, ve, vd = 0.0, 0.0, 0.0

    # ── FAILSAFE: GPS validity ───────────────────────────────
    my_gps_ok = _gps_valid(my_lat, my_lon)
    peer_gps_ok = _gps_valid(peer_lat, peer_lon)
    goal_gps_ok = _gps_valid(goal_lat, goal_lon)

    if not my_gps_ok:
        flags['NO_OWN_GPS'] = True
        state.no_data_counter += 1
    else:
        state.last_valid_tick = state.tick_count

    # ── FAILSAFE: GPS jump rejection ─────────────────────────
    if my_gps_ok and _check_gps_jump(
            my_lat, my_lon, state.prev_my_lat, state.prev_my_lon,
            cfg.max_position_jump_m):
        flags['GPS_JUMP_SELF'] = True
        my_gps_ok = False  # Reject this reading

    if peer_gps_ok and _check_gps_jump(
            peer_lat, peer_lon, state.prev_peer_lat, state.prev_peer_lon,
            cfg.max_position_jump_m):
        flags['GPS_JUMP_PEER'] = True
        peer_gps_ok = False

    # ── FAILSAFE: Stale data detection ───────────────────────
    if my_data_time > 0 and (now - my_data_time) > cfg.stale_threshold_s:
        flags['STALE_OWN_DATA'] = True
    if peer_data_time > 0 and (now - peer_data_time) > cfg.stale_threshold_s:
        flags['STALE_PEER_DATA'] = True
        peer_gps_ok = False  # Don't trust stale peer position

    # ── FAILSAFE: Deadman switch ─────────────────────────────
    if not my_gps_ok:
        state.no_data_counter += 1
    else:
        state.no_data_counter = 0

    if state.no_data_counter >= cfg.deadman_cycles:
        flags['DEADMAN_TRIGGERED'] = True
        emergency = True
        # Output zero velocity — let ArduPilot failsafe handle it
        return _build_result(0.0, 0.0, 0.0, float('inf'), True, flags, state)

    # ── Compute distances and vectors ────────────────────────
    peer_dist = float('inf')
    dn_peer, de_peer = 0.0, 0.0
    err_n, err_e, dist_goal = 0.0, 0.0, 0.0

    if my_gps_ok and peer_gps_ok:
        dn_peer, de_peer = _gps_to_ned_2d(my_lat, my_lon, peer_lat, peer_lon)
        peer_dist = _vec_len(dn_peer, de_peer)

    if my_gps_ok and goal_gps_ok:
        err_n, err_e = _gps_to_ned_2d(goal_lat, goal_lon, my_lat, my_lon)
        dist_goal = _vec_len(err_n, err_e)

    # ── FAILSAFE: Divergence detection ───────────────────────
    if goal_gps_ok and my_gps_ok:
        state.divergence_error = dist_goal
        if dist_goal > cfg.divergence_threshold_m:
            flags['DIVERGENCE'] = True
            emergency = True

    # ── LAYER 1: Attractive force ────────────────────────────
    f_att_n, f_att_e = 0.0, 0.0
    if my_gps_ok and goal_gps_ok:
        f_att_n, f_att_e = _attractive_force(
            err_n, err_e, dist_goal, cfg,
            my_vn=my_vn, my_ve=my_ve, kd=kd_attract)

    # ── LAYER 2: Repulsive force ─────────────────────────────
    f_rep_n, f_rep_e = 0.0, 0.0
    rep_emergency = False
    closing_speed = 0.0
    if my_gps_ok and peer_gps_ok:
        f_rep_n, f_rep_e, closing_speed, rep_emergency = _repulsive_force(
            dn_peer, de_peer, peer_dist,
            my_vn, my_ve, peer_vn, peer_ve, cfg)
        if rep_emergency:
            flags['EMERGENCY_PROXIMITY'] = True
            emergency = True

    # ── LAYER 3: Vortex force ────────────────────────────────
    f_vort_n, f_vort_e = 0.0, 0.0
    if my_gps_ok and peer_gps_ok and goal_gps_ok:
        f_vort_n, f_vort_e = _vortex_force(
            dn_peer, de_peer, peer_dist,
            err_n, err_e, dist_goal, cfg)

    # ── LAYER 5: Geofence force ──────────────────────────────
    f_fence_n, f_fence_e = 0.0, 0.0
    if my_gps_ok:
        f_fence_n, f_fence_e = _geofence_force(my_lat, my_lon, cfg)
        if f_fence_n != 0.0 or f_fence_e != 0.0:
            flags['GEOFENCE_ACTIVE'] = True

    # ── Near-target repulsive & vortex attenuation ──────────
    # When the follower is close to its target, the leader is at expected
    # proximity (~5.83m offset).  Soft-zone repulsion at that range creates
    # a force fight with attraction → persistent zigzag.  Attenuate both
    # repulsive and vortex forces when near the goal, but preserve full
    # repulsion when the leader is closing fast (safety).
    if (dist_goal < cfg.near_target_rep_atten_m
            and not rep_emergency
            and peer_dist > cfg.hard_radius_m):
        t = dist_goal / cfg.near_target_rep_atten_m      # 0 at goal, 1 at threshold
        atten = t * t                                     # quadratic ramp
        # Preserve repulsive when leader is closing fast (safety)
        if closing_speed > 0.0:
            preserve = min(closing_speed / 2.0, 1.0)      # full repulsive at ≥2 m/s closing
            atten = atten + (1.0 - atten) * preserve
        f_rep_n *= atten
        f_rep_e *= atten
        f_vort_n *= atten
        f_vort_e *= atten
        if atten < 0.5:
            flags['REP_ATTENUATED'] = True

    # ── Combine horizontal forces ────────────────────────────
    if emergency and rep_emergency:
        # EMERGENCY: only repulsion, override everything else
        vn = f_rep_n
        ve = f_rep_e
    else:
        vn = f_att_n + f_rep_n + f_vort_n + f_fence_n
        ve = f_att_e + f_rep_e + f_vort_e + f_fence_e

    # ── Kinematic proximity brake toward leader ──────────────
    # If the total command closes on the leader faster than what can
    # stop before the hard radius, scale down the closing component.
    if (my_gps_ok and peer_gps_ok and peer_dist > cfg.hard_radius_m
            and cfg.approach_brake_ms2 > 0.0 and not (emergency and rep_emergency)):
        un = dn_peer / peer_dist
        ue = de_peer / peer_dist
        # Closing speed of COMMAND toward leader (positive = closing)
        v_close_cmd = vn * un + ve * ue
        if v_close_cmd > 0.0:
            d_avail = peer_dist - cfg.hard_radius_m
            v_safe_peer = math.sqrt(2.0 * cfg.approach_brake_ms2 * d_avail)
            if v_close_cmd > v_safe_peer:
                excess = v_close_cmd - v_safe_peer
                vn -= excess * un
                ve -= excess * ue
                flags['PROXIMITY_BRAKE'] = True

    # ── FAILSAFE: Velocity cancellation toward peer ──────────
    if my_gps_ok and peer_gps_ok and peer_dist > _EPS:
        un = dn_peer / peer_dist
        ue = de_peer / peer_dist
        # Only cancel if we're in or near the repulsive zone
        if peer_dist < cfg.soft_radius_m:
            vn, ve = _cancel_closing_velocity(vn, ve, un, ue)
            flags['VEL_CANCEL_ACTIVE'] = True

    # ── LAYER 4: Ground force (vertical) ─────────────────────
    f_ground = _ground_force(my_alt, cfg)
    if f_ground != 0.0:
        flags['GROUND_AVOIDANCE'] = True

    # Vertical: altitude error toward goal + ground avoidance
    if my_gps_ok and goal_gps_ok:
        alt_err = goal_alt - my_alt
        vd = _clamp(-alt_err * 0.5, -cfg.max_vertical_speed_ms, cfg.max_vertical_speed_ms)
    vd += f_ground

    # ── FAILSAFE: Hard altitude floor ────────────────────────
    if _is_valid(my_alt) and my_alt < cfg.critical_altitude_m:
        # Unconditional: override vertical to go UP
        vd = -cfg.total_force_cap
        flags['HARD_ALTITUDE_FLOOR'] = True
        emergency = True

    # ── FAILSAFE: Cap total horizontal force ─────────────────
    h_force = _vec_len(vn, ve)
    if h_force > cfg.total_force_cap:
        vn *= cfg.total_force_cap / h_force
        ve *= cfg.total_force_cap / h_force

    # ── FAILSAFE: Hard speed limiter ─────────────────────────
    h_speed = _vec_len(vn, ve)
    if h_speed > cfg.max_speed_ms:
        vn *= cfg.max_speed_ms / h_speed
        ve *= cfg.max_speed_ms / h_speed
    vd = _clamp(vd, -cfg.max_vertical_speed_ms, cfg.max_vertical_speed_ms)

    # ── FAILSAFE: Rate limiter ───────────────────────────────
    vn, ve, vd = _rate_limit(
        vn, ve, vd,
        state.prev_vn, state.prev_ve, state.prev_vd,
        cfg.max_accel_ms2)

    # ── FAILSAFE: Final NaN/Inf check on output ─────────────
    if not (_is_valid(vn) and _is_valid(ve) and _is_valid(vd)):
        flags['OUTPUT_NAN'] = True
        vn, ve, vd = 0.0, 0.0, 0.0
        emergency = True

    # ── Update state ─────────────────────────────────────────
    state.prev_vn = vn
    state.prev_ve = ve
    state.prev_vd = vd
    if my_gps_ok:
        state.prev_my_lat = my_lat
        state.prev_my_lon = my_lon
    if peer_gps_ok:
        state.prev_peer_lat = peer_lat
        state.prev_peer_lon = peer_lon
    state.prev_dist = peer_dist
    state.emergency_active = emergency
    state.flags = flags

    return _build_result(vn, ve, vd, peer_dist, emergency, flags, state)


def _build_result(
    vn: float, ve: float, vd: float,
    peer_dist: float, emergency: bool,
    flags: dict, state: APFState,
) -> dict:
    return {
        'vn': vn,
        've': ve,
        'vd': vd,
        'speed': _vec_len(vn, ve),
        'peer_dist': peer_dist,
        'emergency': emergency,
        'flags': flags,
        'tick': state.tick_count,
    }


# ═══════════════════════════════════════════════════════════════
# BACKWARD COMPATIBLE WRAPPER
# ═══════════════════════════════════════════════════════════════

# Persistent state for the simple API
_default_state = APFState()
_default_config = APFConfig()


def compute_avoidance(
    my_lat: float, my_lon: float, my_alt: float,
    my_vn: float, my_ve: float,
    peer_lat: float, peer_lon: float, peer_alt: float,
    peer_vn: float, peer_ve: float,
    cmd_vn: float, cmd_ve: float, cmd_vd: float,  # noqa: ARG001 — cmd_vd unused (vertical computed by APF)
    hard_radius: float = 1.0,
    soft_radius: float = 2.0,
    repel_gain: float = 2.5,
    max_speed: float = 3.0,
    closing_speed_scale: float = 0.5,
    soft_zone_expansion: float = 1.5,
) -> tuple[float, float, float, float]:
    """
    Backward-compatible wrapper for the full APF.

    Computes goal as current position + commanded velocity direction,
    then runs the full 5-layer APF.

    Returns: (vn, ve, vd, peer_distance)
    """
    global _default_config, _default_state

    _default_config.hard_radius_m = hard_radius
    _default_config.soft_radius_m = soft_radius
    _default_config.repel_gain = repel_gain
    _default_config.max_speed_ms = max_speed
    _default_config.closing_speed_scale = closing_speed_scale
    _default_config.soft_zone_expansion = soft_zone_expansion

    # Synthesize a goal from commanded velocity (1 second ahead)
    if _gps_valid(my_lat, my_lon):
        cos_lat = math.cos(math.radians(my_lat))
        goal_lat = my_lat + cmd_vn / METERS_PER_DEG_LAT
        goal_lon = my_lon + cmd_ve / (METERS_PER_DEG_LAT * max(cos_lat, _EPS))
    else:
        goal_lat = my_lat
        goal_lon = my_lon

    result = compute_guidance(
        my_lat, my_lon, my_alt,
        my_vn, my_ve, 0.0,
        peer_lat, peer_lon, peer_alt,
        peer_vn, peer_ve,
        goal_lat, goal_lon, my_alt,
        cfg=_default_config,
        state=_default_state,
    )

    return result['vn'], result['ve'], result['vd'], result['peer_dist']
