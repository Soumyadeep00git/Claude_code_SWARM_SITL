"""Tests for gcs/visualizer.py — crash guards when disabled."""

from gcs.visualizer import SwarmVisualizer


class TestVisualizerDisabled:
    def test_save_final_frame_disabled(self):
        """save_final_frame with enabled=False should return None, not crash."""
        viz = SwarmVisualizer(num_drones=3, enabled=False)
        assert viz.save_final_frame() is None

    def test_save_gif_disabled(self):
        """save_gif with enabled=False should return None, not crash."""
        viz = SwarmVisualizer(num_drones=3, enabled=False)
        assert viz.save_gif() is None
