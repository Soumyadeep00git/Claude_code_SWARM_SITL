"""
Generate a mock ONNX model for RL controller testing.

Creates a simple linear model: vel = W @ obs + b
where W is hand-tuned to do goal-seeking (extract goal - pos from obs).

Usage:
    python -m drone_agent.rl_mock_model

Requires: pip install onnx  (only for model generation, not for inference)
"""

import os
import sys
import logging
import numpy as np

from config import RL_OBS_DIM, RL_MODEL_PATH

log = logging.getLogger(__name__)


def generate_mock_model(output_path: str = RL_MODEL_PATH):
    """Create a minimal ONNX model that does goal-seeking from observation vector.

    The model computes: output = input @ W_T + b
    where W extracts (goal - own_pos) and applies velocity damping.
    """
    try:
        import onnx
        from onnx import helper, TensorProto, numpy_helper
    except ImportError:
        print("ERROR: 'onnx' package required for model generation.")
        print("  pip install onnx")
        sys.exit(1)

    # Weight matrix: 3 x RL_OBS_DIM
    # output[i] = scale * (goal[i] - pos[i]) - damp * vel[i]
    # obs layout: [pos(3), vel(3), goal(3), peers...]
    scale = 0.5
    damp = 0.1
    W = np.zeros((3, RL_OBS_DIM), dtype=np.float32)
    for i in range(3):
        W[i, i] = -scale         # -scale * own_pos[i]
        W[i, 6 + i] = scale      # +scale * goal[i]
        W[i, 3 + i] = -damp      # -damp * own_vel[i]

    b = np.zeros(3, dtype=np.float32)

    # Transpose W for MatMul: obs (1, D) @ W_T (D, 3) -> (1, 3)
    W_T = W.T.copy()
    W_T_init = numpy_helper.from_array(W_T, name="W_T")
    b_init = numpy_helper.from_array(b, name="b")

    X = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, RL_OBS_DIM])
    Y = helper.make_tensor_value_info("vel", TensorProto.FLOAT, [1, 3])

    matmul_node = helper.make_node("MatMul", ["obs", "W_T"], ["matmul_out"])
    add_node = helper.make_node("Add", ["matmul_out", "b"], ["vel"])

    graph = helper.make_graph(
        [matmul_node, add_node],
        "rl_mock_policy",
        [X], [Y],
        initializer=[W_T_init, b_init],
    )

    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 7

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    onnx.save(model, output_path)
    print(f"Mock ONNX model saved to {output_path}")
    print(f"  Input:  obs ({RL_OBS_DIM},) float32")
    print(f"  Output: vel (3,) float32")
    print(f"  Logic:  vel = {scale} * (goal - pos) - {damp} * vel")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    generate_mock_model()
