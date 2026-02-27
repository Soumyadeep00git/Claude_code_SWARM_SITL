"""Tests for drone_agent/formation_controller.py — PD + potential field."""

import numpy as np
import pytest
from drone_agent.formation_controller import (
    FormationController, KP, KD, MAX_SPEED, MAX_VERT,
    REPEL_HARD, REPEL_SOFT, ARRIVE_RADIUS,
)


@pytest.fixture
def fc():
    return FormationController()


class TestAttraction:
    def test_at_goal_near_zero_velocity(self, fc):
        pos = np.array([10.0, 5.0, -10.0])
        vel = np.zeros(3)
        goal = np.array([10.0, 5.0, -10.0])
        cmd = fc.compute(pos, vel, goal, np.empty((0, 3)))
        assert np.linalg.norm(cmd) < 0.1

    def test_moves_toward_goal(self, fc):
        pos = np.array([0.0, 0.0, -10.0])
        vel = np.zeros(3)
        goal = np.array([10.0, 0.0, -10.0])
        cmd = fc.compute(pos, vel, goal, np.empty((0, 3)))
        assert cmd[0] > 0  # northward

    def test_damping_reduces_overshoot(self, fc):
        pos = np.array([0.0, 0.0, -10.0])
        vel = np.array([3.0, 0.0, 0.0])  # already moving fast
        goal = np.array([2.0, 0.0, -10.0])
        cmd = fc.compute(pos, vel, goal, np.empty((0, 3)))
        # Damping should reduce forward velocity
        assert cmd[0] < vel[0]

    def test_arrive_radius_deadzone(self, fc):
        pos = np.array([10.0, 5.0, -10.0])
        vel = np.array([0.1, 0.0, 0.0])
        goal = np.array([10.0 + ARRIVE_RADIUS * 0.5, 5.0, -10.0])
        cmd = fc.compute(pos, vel, goal, np.empty((0, 3)))
        # Inside arrive radius — should just damp velocity
        assert abs(cmd[0]) < 0.5


class TestRepulsion:
    def test_no_repulsion_beyond_soft_zone(self, fc):
        pos = np.array([0.0, 0.0, -10.0])
        vel = np.zeros(3)
        goal = np.array([0.0, 0.0, -10.0])
        peers = np.array([[REPEL_SOFT + 1.0, 0.0, -10.0]])
        cmd = fc.compute(pos, vel, goal, peers)
        # No repulsion, just damping near goal
        assert np.linalg.norm(cmd[:2]) < 0.5

    def test_repulsion_in_soft_zone(self, fc):
        pos = np.array([0.0, 0.0, -10.0])
        vel = np.zeros(3)
        goal = np.array([0.0, 0.0, -10.0])  # at goal
        peers = np.array([[4.0, 0.0, -10.0]])  # between hard and soft
        cmd = fc.compute(pos, vel, goal, peers)
        assert cmd[0] < 0  # pushed south (away from peer at north)

    def test_repulsion_in_hard_zone(self, fc):
        pos = np.array([0.0, 0.0, -10.0])
        vel = np.zeros(3)
        goal = np.array([0.0, 0.0, -10.0])
        peers = np.array([[2.0, 0.0, -10.0]])  # inside hard zone
        cmd = fc.compute(pos, vel, goal, peers)
        # Strong repulsion southward
        assert cmd[0] < -0.5

    def test_hard_zone_stronger_than_soft(self, fc):
        pos = np.zeros(3)
        vel = np.zeros(3)
        goal = np.zeros(3)

        peers_soft = np.array([[4.0, 0.0, 0.0]])
        cmd_soft = fc.compute(pos, vel, goal, peers_soft)

        peers_hard = np.array([[2.0, 0.0, 0.0]])
        cmd_hard = fc.compute(pos, vel, goal, peers_hard)

        assert abs(cmd_hard[0]) > abs(cmd_soft[0])

    def test_multiple_peers_combine(self, fc):
        pos = np.array([0.0, 0.0, 0.0])
        vel = np.zeros(3)
        goal = np.array([0.0, 0.0, 0.0])
        # Peers on both sides — forces should partially cancel
        peers = np.array([[3.0, 0.0, 0.0], [-3.0, 0.0, 0.0]])
        cmd = fc.compute(pos, vel, goal, peers)
        assert abs(cmd[0]) < 0.5  # roughly symmetric → cancel


class TestSpeedLimits:
    def test_horizontal_clamp(self, fc):
        pos = np.array([0.0, 0.0, 0.0])
        vel = np.zeros(3)
        goal = np.array([100.0, 100.0, 0.0])  # very far
        cmd = fc.compute(pos, vel, goal, np.empty((0, 3)))
        h_speed = np.linalg.norm(cmd[:2])
        assert h_speed <= MAX_SPEED + 0.01

    def test_vertical_clamp(self, fc):
        pos = np.array([0.0, 0.0, 0.0])
        vel = np.zeros(3)
        goal = np.array([0.0, 0.0, -100.0])  # far down
        cmd = fc.compute(pos, vel, goal, np.empty((0, 3)))
        assert abs(cmd[2]) <= MAX_VERT + 0.01


class TestEdgeCases:
    def test_empty_peers(self, fc):
        pos = np.array([0.0, 0.0, 0.0])
        vel = np.zeros(3)
        goal = np.array([5.0, 0.0, 0.0])
        cmd = fc.compute(pos, vel, goal, np.empty((0, 3)))
        assert cmd.shape == (3,)

    def test_none_peers(self, fc):
        pos = np.array([0.0, 0.0, 0.0])
        vel = np.zeros(3)
        goal = np.array([5.0, 0.0, 0.0])
        cmd = fc.compute(pos, vel, goal, None)
        assert cmd.shape == (3,)
