"""Tests for drone_agent/rl_controller.py — RL inference wrapper."""

import numpy as np
import pytest
from drone_agent.rl_controller import RLController, MockRLModel
from config import RL_OBS_DIM, RL_MAX_PEERS, RL_MAX_SPEED


class TestMockRLModel:
    def test_moves_toward_goal(self):
        model = MockRLModel()
        obs = np.zeros(RL_OBS_DIM)
        obs[0:3] = [0, 0, 0]    # own pos
        obs[6:9] = [10, 0, 0]   # goal north
        vel = model.infer(obs)
        assert vel[0] > 0  # moving north

    def test_zero_at_goal(self):
        model = MockRLModel()
        obs = np.zeros(RL_OBS_DIM)
        obs[0:3] = [10, 5, -10]
        obs[6:9] = [10, 5, -10]
        vel = model.infer(obs)
        assert np.linalg.norm(vel) < 0.1

    def test_speed_clamped(self):
        model = MockRLModel()
        obs = np.zeros(RL_OBS_DIM)
        obs[0:3] = [0, 0, 0]
        obs[6:9] = [100, 100, 0]  # very far
        vel = model.infer(obs)
        assert np.linalg.norm(vel[:2]) <= RL_MAX_SPEED + 0.01

    def test_vertical_clamped(self):
        model = MockRLModel()
        obs = np.zeros(RL_OBS_DIM)
        obs[0:3] = [0, 0, 0]
        obs[6:9] = [0, 0, -50]
        vel = model.infer(obs)
        assert abs(vel[2]) <= 1.0 + 0.01


class TestRLController:
    @pytest.fixture
    def rl(self):
        return RLController()

    def test_build_observation_shape(self, rl):
        obs = rl.build_observation(
            np.array([1, 2, 3]),
            np.array([0.1, 0.2, 0.0]),
            np.array([10, 0, -10]),
            np.empty((0, 3)),
        )
        assert obs.shape == (RL_OBS_DIM,)

    def test_build_observation_own_state(self, rl):
        obs = rl.build_observation(
            np.array([1, 2, 3]),
            np.array([0.1, 0.2, 0.0]),
            np.array([10, 0, -10]),
            np.empty((0, 3)),
        )
        np.testing.assert_array_equal(obs[0:3], [1, 2, 3])
        np.testing.assert_array_equal(obs[3:6], [0.1, 0.2, 0.0])
        np.testing.assert_array_equal(obs[6:9], [10, 0, -10])

    def test_build_observation_peers(self, rl):
        own_pos = np.array([0, 0, 0])
        peers = np.array([[5, 0, 0], [0, 10, 0]])
        obs = rl.build_observation(
            own_pos, np.zeros(3), np.zeros(3), peers
        )
        # Peers are relative and sorted by distance
        # Peer at [5,0,0] is closer → slot 0 (index 9:12)
        np.testing.assert_array_equal(obs[9:12], [5, 0, 0])
        np.testing.assert_array_equal(obs[12:15], [0, 10, 0])

    def test_build_observation_max_peers(self, rl):
        own_pos = np.array([0, 0, 0])
        # More peers than RL_MAX_PEERS
        peers = np.array([[i, 0, 0] for i in range(1, RL_MAX_PEERS + 5)])
        obs = rl.build_observation(own_pos, np.zeros(3), np.zeros(3), peers)
        # Only closest RL_MAX_PEERS should be filled
        assert obs.shape == (RL_OBS_DIM,)
        # Slot for peer beyond max should be zero
        beyond_offset = 9 + RL_MAX_PEERS * 3
        assert np.all(obs[beyond_offset:] == 0)

    def test_build_observation_none_peers(self, rl):
        obs = rl.build_observation(
            np.zeros(3), np.zeros(3), np.array([10, 0, 0]), None
        )
        # Peer slots should be zero
        assert np.all(obs[9:] == 0)

    def test_infer_returns_3d(self, rl):
        obs = np.zeros(RL_OBS_DIM)
        obs[6:9] = [10, 0, 0]
        vel = rl.infer(obs)
        assert vel.shape == (3,)

    def test_infer_type(self, rl):
        obs = np.zeros(RL_OBS_DIM)
        vel = rl.infer(obs)
        assert vel.dtype == np.float64
