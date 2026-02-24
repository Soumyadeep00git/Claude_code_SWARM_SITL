"""
Deterministic PD + Potential Field formation controller.

Replaces MPPI for formation station-keeping. Simple, stable, microsecond-fast.

Attraction: PD controller pulls drone toward its formation slot.
Repulsion:  Two-zone potential field pushes away from peers.
  - Soft zone (3-5m): quadratic ramp — gentle correction during transitions
  - Hard zone (<3m): exponential barrier — prevents actual collision
Output:     Velocity command [vn, ve, vd] in NED m/s.

NOTE: The formation controller uses FIXED internal safety zones, NOT the
configurable isolation radius. Formations intentionally place drones close
together — the isolation radius is for non-formation free-flight safety.
"""

import math
import numpy as np

# Tuning constants
KP = 0.7            # Proportional gain (position error → velocity)
KD = 0.4            # Derivative gain (velocity damping)
MAX_SPEED = 3.0     # Max horizontal speed m/s
MAX_VERT = 1.0      # Max vertical speed m/s
ARRIVE_RADIUS = 0.3 # Dead zone — stop correcting below this (meters)

# Internal repulsion zones (fixed, not affected by isolation radius slider)
REPEL_HARD = 3.0    # Exponential barrier below this distance
REPEL_SOFT = 5.0    # Gentle quadratic ramp starts here
REPEL_GAIN = 2.0    # Repulsion strength multiplier


class FormationController:
    """Deterministic PD + two-zone potential field controller for formation tracking."""

    def compute(self, my_pos_ned, my_vel_ned, goal_ned, peer_positions_ned):
        """
        Compute velocity command to reach formation slot while avoiding peers.

        Args:
            my_pos_ned:         (3,) own position [n, e, d] meters
            my_vel_ned:         (3,) own velocity [vn, ve, vd] m/s
            goal_ned:           (3,) target slot position [n, e, d]
            peer_positions_ned: (P, 3) peer positions or empty array

        Returns:
            (3,) velocity command [vn, ve, vd] m/s
        """
        # ── Attraction: PD control toward goal ──
        error = goal_ned - my_pos_ned
        dist = np.linalg.norm(error[:2])

        if dist < ARRIVE_RADIUS:
            vel_attract = -KD * my_vel_ned
        else:
            vel_attract = KP * error - KD * my_vel_ned

        # ── Repulsion: two-zone potential field from peers ──
        vel_repel = np.zeros(3, dtype=np.float64)

        if peer_positions_ned is not None and len(peer_positions_ned) > 0:
            diffs = my_pos_ned[:2] - peer_positions_ned[:, :2]  # (P, 2) FROM peers
            dists = np.linalg.norm(diffs, axis=1)               # (P,)

            for i in range(len(dists)):
                d = float(dists[i])
                if d >= REPEL_SOFT or d < 0.01:
                    continue

                direction = diffs[i] / d  # Unit vector away from peer

                if d < REPEL_HARD:
                    # Hard zone: exponential barrier
                    strength = REPEL_GAIN * math.exp(REPEL_HARD / max(d, 0.3) - 1.0)
                else:
                    # Soft zone: quadratic ramp (0 at REPEL_SOFT, REPEL_GAIN at REPEL_HARD)
                    t = (REPEL_SOFT - d) / (REPEL_SOFT - REPEL_HARD)
                    strength = REPEL_GAIN * t * t

                vel_repel[:2] += strength * direction

        # ── Combine ──
        vel_cmd = vel_attract + vel_repel

        # ── Clamp speeds ──
        h_speed = np.linalg.norm(vel_cmd[:2])
        if h_speed > MAX_SPEED:
            vel_cmd[:2] *= MAX_SPEED / h_speed
        vel_cmd[2] = np.clip(vel_cmd[2], -MAX_VERT, MAX_VERT)

        return vel_cmd
