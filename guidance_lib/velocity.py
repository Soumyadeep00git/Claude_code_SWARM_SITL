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
                ) -> tuple[float, float, bool, dict]:
        """Radial repulsion away from peer.

        Returns (vn, ve, emergency, flags).
        """
        rad_n = dn_peer / peer_dist
        rad_e = de_peer / peer_dist

        # Stronger push when closer: full speed at 0m, linear fade at safety_dist
        strength = escape_speed * _clamp(
            1.0 - peer_dist / safety_dist_m, 0.0, 1.0)
        vn = strength * rad_n
        ve = strength * rad_e

        emergency = peer_dist < safety_dist_m * 0.5
        flags = {}
        if emergency:
            flags['EMERGENCY_PROXIMITY'] = True

        return vn, ve, emergency, flags

    def catchup(self,
                err_n: float, err_e: float, err_mag: float,
                peer_vn: float, peer_ve: float,
                max_catchup_speed: float, ff_gain: float,
                ) -> tuple[float, float]:
        """Sprint toward target with leader feedforward.

        Returns (vn, ve) clamped to max_catchup_speed.
        """
        u_n = err_n / err_mag
        u_e = err_e / err_mag
        vn = max_catchup_speed * u_n + ff_gain * peer_vn
        ve = max_catchup_speed * u_e + ff_gain * peer_ve

        speed = _mag(vn, ve)
        if speed > max_catchup_speed:
            vn *= max_catchup_speed / speed
            ve *= max_catchup_speed / speed

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
