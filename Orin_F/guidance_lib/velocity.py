"""Velocity computation per guidance mode.

Each method is a pure function — no internal state.
"""

import math

_EPS = 1e-6


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _mag(n: float, e: float) -> float:
    return math.sqrt(n * n + e * e + _EPS * _EPS)


class VelocityComputer:
    """Computes raw velocity commands for each guidance mode.

    All methods are stateless — they take inputs and return outputs
    without storing anything.
    """

    def evasion(self,
                dn_peer: float, de_peer: float, peer_dist: float,
                escape_speed: float, safety_dist_m: float,
                peer_vn: float = 0.0, peer_ve: float = 0.0,
                ) -> tuple[float, float, bool, dict]:
        """Opposite-to-sideways evasion with full-strength ramp-up.

        Blend factor m = peer_dist / safety_dist (1 at boundary, 0 at contact):
          - m ~ 1 (far):  mostly run OPPOSITE (away from leader)
          - m ~ 0 (close): mostly dodge SIDEWAYS (matador step)
        Sideways direction chosen to maximize separation from leader's
        velocity vector (dodge to the side the leader ISN'T heading).

        Strength is full escape_speed from the moment evasion triggers —
        no ramp-from-zero.

        Returns (vn, ve, emergency, flags).
        """
        # Unit vector AWAY from leader (from peer toward self)
        opp_n = dn_peer / peer_dist
        opp_e = de_peer / peer_dist

        # Two perpendicular candidates: rotate opposite ±90°
        side_a_n, side_a_e = -opp_e, opp_n    # +90° (right)
        side_b_n, side_b_e = opp_e, -opp_n    # -90° (left)

        # Pick the sideways direction that moves AWAY from leader's velocity.
        # Dot product of sideways with leader_vel: pick the more negative one
        # (the side the leader is NOT heading toward).
        dot_a = side_a_n * peer_vn + side_a_e * peer_ve
        dot_b = side_b_n * peer_vn + side_b_e * peer_ve
        if dot_a <= dot_b:
            side_n, side_e = side_a_n, side_a_e
        else:
            side_n, side_e = side_b_n, side_b_e

        # Blend: m=1 at boundary (pure opposite), m=0 at contact (pure sideways)
        m = _clamp(peer_dist / safety_dist_m, 0.0, 1.0)
        dir_n = m * opp_n + (1.0 - m) * side_n
        dir_e = m * opp_e + (1.0 - m) * side_e

        # Normalize
        dir_mag = _mag(dir_n, dir_e)
        dir_n /= dir_mag
        dir_e /= dir_mag

        # Full escape speed from the start — no ramp-from-zero
        vn = escape_speed * dir_n
        ve = escape_speed * dir_e

        emergency = peer_dist < safety_dist_m * 0.5
        flags = {}
        if emergency:
            flags['EMERGENCY_PROXIMITY'] = True

        return vn, ve, emergency, flags

    def catchup(self,
                err_n: float, err_e: float, err_mag: float,
                peer_vn: float, peer_ve: float,
                max_catchup_speed: float, ff_gain: float,
                catchup_dist_m: float = 8.0,
                catchup_decel: float = 2.5,
                ) -> tuple[float, float]:
        """Kinematic catchup — speed limited by braking distance.

        v_max = min(sqrt(2 * catchup_decel * distance), max_catchup_speed)

        The follower accelerates freely up to whatever speed lets it
        brake to zero within the remaining distance. The rate limiter
        in output_safety handles actual acceleration smoothing.

        Returns (vn, ve) clamped to kinematic speed limit.
        """
        # Kinematic speed limit: can we stop in the remaining distance?
        v_kinematic = math.sqrt(2.0 * catchup_decel * err_mag)
        speed_limit = min(v_kinematic, max_catchup_speed)

        u_n = err_n / err_mag
        u_e = err_e / err_mag
        vn = speed_limit * u_n + ff_gain * peer_vn
        ve = speed_limit * u_e + ff_gain * peer_ve

        speed = _mag(vn, ve)
        if speed > speed_limit:
            vn *= speed_limit / speed
            ve *= speed_limit / speed

        return vn, ve

    def tracking(self,
                 err_n: float, err_e: float, err_mag: float,
                 my_vn: float, my_ve: float,
                 peer_vn: float, peer_ve: float,
                 kp: float, kd: float, ff_gain: float,
                 max_speed: float, deadzone_m: float,
                 ) -> tuple[float, float]:
        """PD controller + leader velocity feedforward.

        Returns (vn, ve) clamped to max_speed. Returns (0, 0) in deadzone.
        """
        if err_mag <= deadzone_m:
            return 0.0, 0.0

        vn = kp * err_n - kd * (my_vn - peer_vn) + ff_gain * peer_vn
        ve = kp * err_e - kd * (my_ve - peer_ve) + ff_gain * peer_ve

        speed = _mag(vn, ve)
        if speed > max_speed:
            vn *= max_speed / speed
            ve *= max_speed / speed

        return vn, ve

    def vertical(self,
                 goal_alt: float, my_alt: float,
                 max_vertical_speed: float,
                 ) -> float:
        """Altitude error -> vertical velocity command.

        Returns vd (positive = down per NED convention).
        """
        alt_err = goal_alt - my_alt
        return _clamp(-alt_err * 0.5, -max_vertical_speed, max_vertical_speed)
