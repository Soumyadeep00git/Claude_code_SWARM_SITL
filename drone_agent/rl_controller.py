"""
RL controller — ONNX inference wrapper with mock fallback.

Provides velocity commands [vn, ve, vd] from an observation vector.
If onnxruntime is available and a model file exists, uses ONNX inference.
Otherwise falls back to a simple proportional goal-seeking mock.

Observation vector layout (RL_OBS_DIM = 21):
  [0:3]   own_pos_ned   (north, east, down) meters
  [3:6]   own_vel_ned   (vn, ve, vd) m/s
  [6:9]   goal_ned      (north, east, down) meters
  [9:21]  peer_relative_positions (max 4 peers x 3) zero-padded
"""

import os
import logging
import numpy as np

from config import RL_MODEL_PATH, RL_OBS_DIM, RL_MAX_PEERS, RL_MAX_SPEED

log = logging.getLogger(__name__)

# Try importing onnxruntime; gracefully degrade if unavailable
try:
    import onnxruntime as ort
    _ORT_AVAILABLE = True
except ImportError:
    _ORT_AVAILABLE = False
    log.info("onnxruntime not installed — RL controller will use mock model")


class MockRLModel:
    """Simple proportional goal-seeking controller as RL fallback.
    Same interface as OnnxRLModel: infer(obs) -> velocity."""

    def infer(self, obs: np.ndarray) -> np.ndarray:
        own_pos = obs[0:3]
        goal = obs[6:9]
        goal_vec = goal - own_pos
        dist = np.linalg.norm(goal_vec[:2])

        if dist < 0.5:
            return np.zeros(3, dtype=np.float64)

        speed = min(0.5 * dist, RL_MAX_SPEED)
        direction = goal_vec / max(dist, 0.01)
        vel = direction * speed
        vel[2] = np.clip(vel[2], -1.0, 1.0)
        return vel


class OnnxRLModel:
    """ONNX Runtime inference session wrapper."""

    def __init__(self, model_path: str):
        self._session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"],
        )
        self._input_name = self._session.get_inputs()[0].name
        log.info("ONNX RL model loaded from %s", model_path)

    def infer(self, obs: np.ndarray) -> np.ndarray:
        obs_f32 = obs.astype(np.float32).reshape(1, -1)
        outputs = self._session.run(None, {self._input_name: obs_f32})
        vel = np.array(outputs[0].flatten()[:3], dtype=np.float64)
        h_speed = np.linalg.norm(vel[:2])
        if h_speed > RL_MAX_SPEED:
            vel[:2] *= RL_MAX_SPEED / h_speed
        vel[2] = np.clip(vel[2], -1.0, 1.0)
        return vel


class RLController:
    """Unified RL controller: loads ONNX model if available, else uses mock."""

    def __init__(self):
        self._model = None
        self._using_onnx = False
        self._load_model()

    def _load_model(self):
        if _ORT_AVAILABLE and os.path.isfile(RL_MODEL_PATH):
            try:
                self._model = OnnxRLModel(RL_MODEL_PATH)
                self._using_onnx = True
                return
            except Exception as e:
                log.warning("Failed to load ONNX model: %s — using mock", e)
        self._model = MockRLModel()
        self._using_onnx = False
        log.info("RL controller using mock model (goal-seeking PD)")

    @property
    def is_onnx(self) -> bool:
        return self._using_onnx

    def build_observation(self, own_pos_ned: np.ndarray, own_vel_ned: np.ndarray,
                          goal_ned: np.ndarray,
                          peer_positions_ned: np.ndarray) -> np.ndarray:
        """Build fixed-size observation vector from agent state.

        Args:
            own_pos_ned:        (3,) own position NED
            own_vel_ned:        (3,) own velocity NED
            goal_ned:           (3,) goal position NED
            peer_positions_ned: (P, 3) peer positions in NED (absolute)

        Returns:
            (RL_OBS_DIM,) observation vector, zero-padded for missing peers.
        """
        obs = np.zeros(RL_OBS_DIM, dtype=np.float64)
        obs[0:3] = own_pos_ned
        obs[3:6] = own_vel_ned
        obs[6:9] = goal_ned

        if peer_positions_ned is not None and len(peer_positions_ned) > 0:
            n_peers = min(len(peer_positions_ned), RL_MAX_PEERS)
            dists = np.linalg.norm(
                peer_positions_ned[:, :2] - own_pos_ned[:2], axis=1)
            sorted_idx = np.argsort(dists)[:n_peers]
            for i, idx in enumerate(sorted_idx):
                offset = 9 + i * 3
                obs[offset:offset + 3] = peer_positions_ned[idx] - own_pos_ned

        return obs

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """Run inference. Returns (3,) velocity [vn, ve, vd] in m/s."""
        return self._model.infer(obs)
