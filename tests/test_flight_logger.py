"""Tests for gcs/flight_logger.py — lazy CSV creation."""

import os
import pytest
from gcs.flight_logger import FlightLogger


class TestFlightLogger:
    def test_lazy_csv_creation(self, tmp_path):
        """No per-drone CSVs should exist until first log_state."""
        logger = FlightLogger(num_drones=3, output_dir=str(tmp_path))
        # Only aggregated CSV should exist
        assert os.path.exists(tmp_path / "flight_log.csv")
        assert not os.path.exists(tmp_path / "drone_1_log.csv")
        assert not os.path.exists(tmp_path / "drone_2_log.csv")
        assert not os.path.exists(tmp_path / "drone_3_log.csv")
        logger.close()

    def test_unknown_drone_gets_csv(self, tmp_path):
        """Drone ID outside 1..N should still get a CSV created."""
        logger = FlightLogger(num_drones=2, output_dir=str(tmp_path))
        state = {"lat": -35.363, "lon": 149.165, "alt": 10.0}
        logger.log_state(99, state)
        logger._flush_buffer()
        assert os.path.exists(tmp_path / "drone_99_log.csv")
        logger.close()
