"""
Model Predictive Path Integral (MPPI) controller for drone formation tracking.

Uses a double-integrator kinematic model and fully vectorized NumPy rollouts.
All K trajectory samples are evaluated in parallel — no Python loops in the
hot path (cost computation).

Key design: PREDICTIVE peer positions. Forward-propagates peer positions using
their current velocities so collision cost accounts for peer motion.

State:   [x, y, z, vx, vy, vz]  in NED meters / m/s
Control: [ax, ay, az]            acceleration commands in m/s²
Output:  [vn, ve, vd]            velocity command sent to MAVLink
"""

import numpy as np
import logging

log = logging.getLogger(__name__)


class MPPIController:
    """Sampling-based MPC for smooth formation tracking with collision avoidance."""

    def __init__(self, config: dict):
        # Hyperparameters
        self.K = config.get("K_SAMPLES", 256)
        self.T = config.get("HORIZON_STEPS", 15)
        self.dt = config.get("DT_S", 0.1)
        self.lam = config.get("LAMBDA_TEMP", 5.0)
        self.sigma = np.array(config.get("SIGMA_NOISE", [2.0, 2.0, 0.3]),
                              dtype=np.float64)

        # Physical limits
        self.max_vel = config.get("MAX_VELOCITY_MS", 3.0)
        self.max_accel = config.get("MAX_ACCEL_MS2", 2.5)

        # Cost weights
        self.w_formation = config.get("W_FORMATION", 20.0)
        self.w_collision = config.get("W_COLLISION", 80.0)
        self.w_effort = config.get("W_EFFORT", 0.05)
        self.w_connectivity = config.get("W_CONNECTIVITY", 3.0)
        self.w_smoothness = config.get("W_SMOOTHNESS", 0.8)
        self.w_time_pressure = config.get("W_TIME_PRESSURE", 2.0)
        self.w_speed_incentive = config.get("W_SPEED_INCENTIVE", 1.5)

        # Safety radii
        self.safe_radius = config.get("SAFE_RADIUS_M", 3.0)
        self.collision_radius = config.get("COLLISION_RADIUS_M", 2.0)
        self.comm_range = config.get("COMM_RANGE_M", 50.0)

        # Warm start: previous optimal control sequence (T, 3)
        self._prev_controls = np.zeros((self.T, 3), dtype=np.float64)

        # Pre-compute time weights: (T+1,) — later timesteps weighted more
        self._time_weights = np.linspace(0.3, 1.0, self.T + 1)

    def optimize(self, state_6d, goal_ned, peer_positions_ned,
                 path_waypoints_ned=None, peer_velocities_ned=None):
        """
        Run one MPPI optimization step.

        Returns:
            (3,) velocity command [vn, ve, vd] clipped to max_vel
        """
        pos = state_6d[:3].copy()
        vel = state_6d[3:6].copy()

        goal_dist = np.linalg.norm((goal_ned - pos)[:2])

        # Pre-compute predicted peer positions: (T+1, P, 3)
        has_peers = peer_positions_ned is not None and len(peer_positions_ned) > 0
        pred_peers = None
        if has_peers:
            P = peer_positions_ned.shape[0]
            # Vectorized time steps: (T+1, 1, 1)
            t_steps = (np.arange(self.T + 1) * self.dt)[:, np.newaxis, np.newaxis]
            if peer_velocities_ned is not None and len(peer_velocities_ned) == P:
                # (T+1, P, 3) = (1, P, 3) + (T+1, 1, 1) * (1, P, 3)
                pred_peers = peer_positions_ned[np.newaxis, :, :] + \
                    t_steps * peer_velocities_ned[np.newaxis, :, :]
            else:
                pred_peers = np.broadcast_to(
                    peer_positions_ned[np.newaxis, :, :],
                    (self.T + 1, P, 3)
                ).copy()

        # Warm start: shift previous controls forward
        shifted = np.roll(self._prev_controls, -1, axis=0)
        shifted[-1] = 0.0
        mean_controls = shifted

        # Sample: (K, T, 3)
        noise = np.random.randn(self.K, self.T, 3) * self.sigma
        controls = mean_controls[np.newaxis, :, :] + noise
        np.clip(controls, -self.max_accel, self.max_accel, out=controls)

        # Rollout: (K, T+1, 3)
        traj_pos, traj_vel = self._rollout(pos, vel, controls)

        # Cost: (K,)
        costs = self._compute_costs(
            traj_pos, traj_vel, controls, goal_ned,
            path_waypoints_ned, pred_peers, goal_dist
        )

        # MPPI weighting
        costs_shifted = costs - np.min(costs)
        weights = np.exp(-costs_shifted / self.lam)
        w_sum = np.sum(weights)
        if w_sum < 1e-10:
            weights = np.ones(self.K) / self.K
        else:
            weights /= w_sum

        # Weighted average: (T, 3)
        optimal = np.einsum('k,ktc->tc', weights, controls)
        self._prev_controls = optimal.copy()

        # First-step velocity command
        cmd_vel = vel + optimal[0] * self.dt
        h_speed = np.linalg.norm(cmd_vel[:2])
        if h_speed > self.max_vel:
            cmd_vel[:2] *= self.max_vel / h_speed
        cmd_vel[2] = np.clip(cmd_vel[2], -1.0, 1.0)

        return cmd_vel

    def _rollout(self, pos0, vel0, controls):
        """Forward simulate K trajectories. Loop over T is unavoidable (sequential)."""
        K, T, _ = controls.shape
        positions = np.empty((K, T + 1, 3), dtype=np.float64)
        velocities = np.empty((K, T + 1, 3), dtype=np.float64)

        positions[:, 0, :] = pos0
        velocities[:, 0, :] = vel0

        for t in range(T):
            v_new = velocities[:, t, :] + controls[:, t, :] * self.dt
            h_speed = np.linalg.norm(v_new[:, :2], axis=1, keepdims=True)
            scale = np.where(h_speed > self.max_vel,
                             self.max_vel / np.maximum(h_speed, 1e-6), 1.0)
            v_new[:, :2] *= scale
            v_new[:, 2] = np.clip(v_new[:, 2], -1.0, 1.0)
            velocities[:, t + 1, :] = v_new
            positions[:, t + 1, :] = positions[:, t, :] + v_new * self.dt

        return positions, velocities

    def _compute_costs(self, traj_pos, traj_vel, controls,
                       goal_ned, path_wps, pred_peers, goal_dist):
        """
        Fully vectorized cost computation — no Python loops over timesteps.

        Returns: (K,) cost array
        """
        K = traj_pos.shape[0]
        costs = np.zeros(K, dtype=np.float64)

        # Time weights broadcast shape: (1, T+1)
        tw = self._time_weights[np.newaxis, :]

        # ── 1. Formation tracking (vectorized over all timesteps) ──────
        # (K, T+1, 3) - (1, 1, 3) → (K, T+1)
        goal_diffs = traj_pos - goal_ned[np.newaxis, np.newaxis, :]
        goal_dists_sq = np.sum(goal_diffs ** 2, axis=2)  # (K, T+1)
        # Time-weighted running cost: sum over timesteps
        costs += self.w_formation * np.sum(tw * goal_dists_sq, axis=1)
        # Terminal bonus
        costs += self.w_formation * 2.0 * goal_dists_sq[:, -1]

        # Path waypoint tracking
        if path_wps is not None and len(path_wps) > 1:
            # (K, T+1, 1, 3) - (1, 1, W, 3) → (K, T+1, W, 3)
            wp_diffs = traj_pos[:, 1:, np.newaxis, :] - path_wps[np.newaxis, np.newaxis, :, :]
            wp_dists = np.linalg.norm(wp_diffs, axis=3)  # (K, T, W)
            nearest = np.min(wp_dists, axis=2)  # (K, T)
            costs += self.w_formation * 0.02 * np.sum(nearest ** 2, axis=1)

        # ── 2. Collision avoidance (fully vectorized) ──────────────────
        if pred_peers is not None:
            P = pred_peers.shape[1]
            alpha = 4.0

            # traj_pos horizontal: (K, T+1, 2)
            traj_2d = traj_pos[:, :, :2]
            # pred_peers horizontal: (T+1, P, 2)
            peers_2d = pred_peers[:, :, :2]

            # (K, T+1, 1, 2) - (1, T+1, P, 2) → (K, T+1, P, 2)
            diffs = traj_2d[:, :, np.newaxis, :] - peers_2d[np.newaxis, :, :, :]
            dists = np.linalg.norm(diffs, axis=3)  # (K, T+1, P)

            # Exponential barrier
            barrier = np.exp(-alpha * (dists - self.safe_radius))
            # Hard penalty inside collision radius
            barrier = np.where(dists < self.collision_radius, 1000.0, barrier)
            costs += self.w_collision * np.sum(barrier, axis=(1, 2))

            # Closing rate penalty (t=1..T compared to t=0..T-1)
            if dists.shape[1] > 1:
                closing_rate = dists[:, :-1, :] - dists[:, 1:, :]  # positive = closing
                close_mask = dists[:, 1:, :] < self.safe_radius * 1.5
                closing_cost = np.where(
                    close_mask & (closing_rate > 0),
                    closing_rate * 20.0,
                    0.0
                )
                costs += self.w_collision * 0.2 * np.sum(closing_cost, axis=(1, 2))

            # ── 5. Connectivity (use final timestep peer distances) ────
            final_dists = dists[:, -1, :]  # (K, P)
            max_peer_dist = np.max(final_dists, axis=1)
            over_range = np.maximum(max_peer_dist - self.comm_range * 0.7, 0.0)
            costs += self.w_connectivity * over_range ** 2

        # ── 3. Control effort ──────────────────────────────────────────
        costs += self.w_effort * np.sum(controls ** 2, axis=(1, 2))

        # ── 4. Smoothness (jerk) ───────────────────────────────────────
        if controls.shape[1] > 1:
            jerk = np.diff(controls, axis=1)
            costs += self.w_smoothness * np.sum(jerk ** 2, axis=(1, 2))

        # ── 6. Time pressure (vectorized) ──────────────────────────────
        if goal_dist > 2.0:
            goal_dists_2d = np.linalg.norm(
                traj_pos[:, 1:, :2] - goal_ned[np.newaxis, np.newaxis, :2], axis=2
            )  # (K, T)
            costs += self.w_time_pressure * np.sum(
                self._time_weights[np.newaxis, 1:] * goal_dists_2d, axis=1
            )

        # ── 7. Speed incentive (vectorized) ────────────────────────────
        if goal_dist > 3.0:
            h_speeds = np.linalg.norm(traj_vel[:, 1:, :2], axis=2)  # (K, T)
            speed_deficit = np.maximum(self.max_vel * 0.5 - h_speeds, 0.0)
            costs += self.w_speed_incentive * np.sum(speed_deficit ** 2, axis=1)

        return costs

    def reset(self):
        """Clear warm start. Call on mode transitions or after proximity clear."""
        self._prev_controls[:] = 0.0
