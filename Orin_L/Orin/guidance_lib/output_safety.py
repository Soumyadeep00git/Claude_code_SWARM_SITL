"""Output safety — speed cap, altitude floor/ceiling, rate limiter, NaN guard.

Applied to ALL velocity output regardless of guidance mode.
Must persist across ticks for rate limiter state.
"""

import math

_EPS = 1e-6


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _mag(n: float, e: float) -> float:
    return math.sqrt(n * n + e * e + _EPS * _EPS)


def _is_valid(v: float) -> bool:
    return math.isfinite(v) and abs(v) < 1e8


class OutputSafety:
    """Clamps, rate-limits, and validates velocity output.

    Stores previous velocity for rate limiting. Must persist across ticks.
    """

    def __init__(self):
        self.prev_vn: float = 0.0
        self.prev_ve: float = 0.0
        self.prev_vd: float = 0.0

    def apply(self,
              vn: float, ve: float, vd: float,
              mode: str, my_alt: float,
              max_speed: float, max_catchup_speed: float,
              max_vertical_speed: float, max_accel: float,
              min_altitude_m: float, critical_altitude_m: float,
              max_altitude_m: float = 100.0,
              ) -> tuple[float, float, float, bool, dict]:
        """Apply all output safety clamps.

        Args:
            vn, ve, vd: Raw velocity from guidance.
            mode: Current guidance mode string.
            my_alt: Current altitude (meters).
            max_speed: Horizontal speed cap for TRACKING.
            max_catchup_speed: Horizontal speed cap for CATCHUP.
            max_vertical_speed: Vertical speed cap.
            max_accel: Max velocity change per tick (rate limiter).
            min_altitude_m: Soft altitude floor (gradual push-up).
            critical_altitude_m: Hard altitude floor (full push-up).
            max_altitude_m: Altitude ceiling (push down).

        Returns:
            (vn, ve, vd, emergency, flags)
        """
        emergency = False
        flags = {}

        # ── Altitude floor (soft ramp) ──
        if _is_valid(my_alt) and my_alt < min_altitude_m:
            range_w = min_altitude_m - critical_altitude_m
            if range_w > _EPS:
                t = _clamp((min_altitude_m - my_alt) / range_w, 0.0, 1.0)
                vd = min(vd, -3.0 * t * t)

        # ── Altitude floor (hard cutoff) ──
        if _is_valid(my_alt) and my_alt < critical_altitude_m:
            vd = -max_vertical_speed
            flags['HARD_ALTITUDE_FLOOR'] = True
            emergency = True

        # ── Altitude ceiling ──
        if _is_valid(my_alt) and my_alt > max_altitude_m:
            vd = max_vertical_speed * 0.5
            flags['ALTITUDE_CEILING'] = True

        # ── Horizontal speed cap ──
        # CATCHUP uses kinematic limit (already applied by velocity computer),
        # but we still enforce the hard ceiling here as a safety net.
        h_speed = _mag(vn, ve)
        speed_limit = max_catchup_speed if mode == 'CATCHUP' else max_speed
        if h_speed > speed_limit:
            vn *= speed_limit / h_speed
            ve *= speed_limit / h_speed

        # ── Vertical speed cap ──
        vd = _clamp(vd, -max_vertical_speed, max_vertical_speed)

        # ── Rate limiter ──
        dvn = _clamp(vn - self.prev_vn, -max_accel, max_accel)
        dve = _clamp(ve - self.prev_ve, -max_accel, max_accel)
        dvd = _clamp(vd - self.prev_vd, -max_accel, max_accel)
        vn = self.prev_vn + dvn
        ve = self.prev_ve + dve
        vd = self.prev_vd + dvd

        # ── NaN guard ──
        if not (_is_valid(vn) and _is_valid(ve) and _is_valid(vd)):
            flags['OUTPUT_NAN'] = True
            vn, ve, vd = 0.0, 0.0, 0.0
            emergency = True

        # Update state
        self.prev_vn = vn
        self.prev_ve = ve
        self.prev_vd = vd

        return vn, ve, vd, emergency, flags

    def reset(self):
        """Reset rate limiter state. Call on failsafe transitions."""
        self.prev_vn = 0.0
        self.prev_ve = 0.0
        self.prev_vd = 0.0
