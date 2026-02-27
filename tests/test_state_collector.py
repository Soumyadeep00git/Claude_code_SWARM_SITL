"""Tests for gcs/state_collector.py — state aggregation."""

import time
import pytest
from gcs.state_collector import StateCollector


class TestStateCollector:
    def test_empty(self):
        sc = StateCollector(num_drones=5)
        assert sc.get_all_states() == {}
        assert sc.get_state(1) is None

    def test_update_and_get(self):
        sc = StateCollector(num_drones=5)
        data = {"lat": -35.363, "lon": 149.165, "alt": 10.0, "armed": True}
        sc.update(1, data, time.time())
        assert sc.get_state(1) == data

    def test_get_all_states(self):
        sc = StateCollector(num_drones=5)
        sc.update(1, {"lat": -35.363}, time.time())
        sc.update(2, {"lat": -35.364}, time.time())
        all_states = sc.get_all_states()
        assert len(all_states) == 2
        assert 1 in all_states
        assert 2 in all_states

    def test_stale_drones(self):
        sc = StateCollector(num_drones=5)
        now = time.time()
        sc.update(1, {"lat": -35.363}, now)
        sc.update(2, {"lat": -35.364}, now - 10)
        sc.update(3, {"lat": -35.365}, now - 20)
        stale = sc.get_stale_drones(5.0)
        assert 1 not in stale
        assert 2 in stale
        assert 3 in stale

    def test_centroid_valid(self):
        sc = StateCollector(num_drones=3)
        sc.update(1, {"lat": -35.360, "lon": 149.160, "alt": 10.0}, time.time())
        sc.update(2, {"lat": -35.370, "lon": 149.170, "alt": 20.0}, time.time())
        centroid = sc.get_centroid()
        assert centroid is not None
        assert centroid[0] == pytest.approx(-35.365, abs=0.001)
        assert centroid[1] == pytest.approx(149.165, abs=0.001)
        assert centroid[2] == pytest.approx(15.0, abs=0.01)

    def test_centroid_skips_zero_gps(self):
        sc = StateCollector(num_drones=3)
        sc.update(1, {"lat": -35.360, "lon": 149.160, "alt": 10.0}, time.time())
        sc.update(2, {"lat": 0.0, "lon": 0.0, "alt": 0.0}, time.time())
        centroid = sc.get_centroid()
        assert centroid is not None
        assert centroid[0] == pytest.approx(-35.360, abs=0.001)

    def test_centroid_all_zero_returns_none(self):
        sc = StateCollector(num_drones=2)
        sc.update(1, {"lat": 0.0, "lon": 0.0}, time.time())
        assert sc.get_centroid() is None

    def test_valid_drone_ids(self):
        sc = StateCollector(num_drones=3)
        sc.update(1, {"lat": -35.360, "lon": 149.160}, time.time())
        sc.update(2, {"lat": 0.0, "lon": 0.0}, time.time())
        sc.update(3, {"lat": -35.365, "lon": 149.165}, time.time())
        valid = sc.get_valid_drone_ids()
        assert 1 in valid
        assert 3 in valid
        assert 2 not in valid

    def test_remove_drone(self):
        sc = StateCollector(num_drones=3)
        sc.update(1, {"lat": -35.360}, time.time())
        sc.update(2, {"lat": -35.370}, time.time())
        sc.remove_drone(1)
        assert sc.get_state(1) is None
        assert sc.get_state(2) is not None
        # Removing non-existent drone is a no-op
        sc.remove_drone(99)

    def test_prune_stale(self):
        sc = StateCollector(num_drones=3)
        now = time.time()
        sc.update(1, {"lat": -35.360}, now)        # fresh
        sc.update(2, {"lat": -35.370}, now - 20)    # stale
        sc.update(3, {"lat": -35.380}, now - 20)    # stale
        pruned = sc.prune_stale(15.0)
        assert sorted(pruned) == [2, 3]
        assert sc.get_state(1) is not None
        assert sc.get_state(2) is None
        assert sc.get_state(3) is None

    def test_prune_returns_pruned_ids(self):
        sc = StateCollector(num_drones=2)
        now = time.time()
        sc.update(1, {"lat": -35.360}, now - 30)
        sc.update(2, {"lat": -35.370}, now)
        result = sc.prune_stale(15.0)
        assert result == [1]
        # Second prune returns empty (already removed)
        assert sc.prune_stale(15.0) == []
