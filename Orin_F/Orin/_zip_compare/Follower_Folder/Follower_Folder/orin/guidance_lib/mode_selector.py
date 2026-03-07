"""Hard-switch mode selector: EVASION > CATCHUP > TRACKING.

Only one mode active at a time. Priority order is absolute —
no blending, no hysteresis. Simple and predictable.
"""

from enum import Enum


class GuidanceMode(Enum):
    EVASION = 'EVASION'
    CATCHUP = 'CATCHUP'
    TRACKING = 'TRACKING'


class ModeSelector:
    """Selects guidance mode based on peer distance and goal error.

    Priority: EVASION > CATCHUP > TRACKING (hard if/else).
    """

    def select(self,
               peer_dist: float, goal_error: float,
               peer_ok: bool, goal_ok: bool,
               safety_dist_m: float, catchup_dist_m: float,
               ) -> GuidanceMode:
        """Return the active guidance mode.

        Args:
            peer_dist: Distance to leader (meters). inf if unknown.
            goal_error: Distance to target position (meters).
            peer_ok: True if leader GPS is valid.
            goal_ok: True if goal GPS is valid.
            safety_dist_m: Evasion trigger distance.
            catchup_dist_m: Catchup trigger distance.
        """
        if peer_ok and peer_dist < safety_dist_m:
            return GuidanceMode.EVASION
        if goal_ok and goal_error > catchup_dist_m:
            return GuidanceMode.CATCHUP
        return GuidanceMode.TRACKING
