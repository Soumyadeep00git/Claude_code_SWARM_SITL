"""Tests for drone_agent/state.py — DroneState and PeerTable."""

import time
import pytest
from drone_agent.state import DroneState, PeerTable


class TestDroneState:
    def test_defaults(self):
        s = DroneState(drone_id=1)
        assert s.lat == 0.0
        assert s.lon == 0.0
        assert s.alt == 0.0
        assert s.armed is False
        assert s.mode == "STABILIZE"
        assert s.battery_pct == 100
        assert s.swarm_state == "NOMINAL"

    def test_to_dict_keys(self):
        s = DroneState(drone_id=1)
        d = s.to_dict()
        expected_keys = {
            "drone_id", "lat", "lon", "alt", "vx", "vy", "vz",
            "heading", "battery_pct", "mode", "armed", "formation_slot",
            "failsafe_active", "swarm_state", "leader_id", "alive_count",
            "rl_mode",
        }
        assert set(d.keys()) == expected_keys

    def test_to_dict_no_mesh_stats_by_default(self):
        d = DroneState(drone_id=1).to_dict()
        assert "mesh_stats" not in d

    def test_to_dict_includes_mesh_stats(self):
        s = DroneState(drone_id=1)
        s.mesh_stats = {"links": 3}
        d = s.to_dict()
        assert "mesh_stats" in d
        assert d["mesh_stats"]["links"] == 3

    def test_update_from_position(self):
        s = DroneState(drone_id=1)
        pos = {"lat": -35.363, "lon": 149.165, "alt": 10.5,
               "vx": 1.0, "vy": -0.5, "vz": 0.1, "heading": 90.0}
        s.update_from_position(pos)
        assert s.lat == -35.363
        assert s.lon == 149.165
        assert s.alt == 10.5
        assert s.vx == 1.0
        assert s.heading == 90.0

    def test_update_from_heartbeat(self):
        s = DroneState(drone_id=1)
        s.update_from_heartbeat({"mode": "GUIDED", "armed": True})
        assert s.mode == "GUIDED"
        assert s.armed is True


class TestPeerTable:
    def test_empty_table(self):
        pt = PeerTable()
        assert pt.get_peer(1) is None
        assert pt.get_all_positions() == []
        assert pt.get_alive_ids(5.0) == set()

    def test_update_and_get(self):
        pt = PeerTable()
        pt.update_peer(2, {"lat": -35.364, "lon": 149.166, "alt": 10.0}, time.time())
        peer = pt.get_peer(2)
        assert peer is not None
        assert peer.lat == -35.364

    def test_update_preserves_existing_fields(self):
        pt = PeerTable()
        pt.update_peer(2, {"lat": -35.364, "lon": 149.166}, time.time())
        pt.update_peer(2, {"alt": 15.0}, time.time())
        peer = pt.get_peer(2)
        assert peer.lat == -35.364
        assert peer.alt == 15.0

    def test_get_all_positions(self):
        pt = PeerTable()
        pt.update_peer(1, {"lat": -35.363, "lon": 149.165, "alt": 10.0}, time.time())
        pt.update_peer(2, {"lat": -35.364, "lon": 149.166, "alt": 12.0}, time.time())
        positions = pt.get_all_positions()
        assert len(positions) == 2
        ids = {p[0] for p in positions}
        assert ids == {1, 2}

    def test_get_all_states(self):
        pt = PeerTable()
        pt.update_peer(1, {"lat": -35.363, "lon": 149.165, "alt": 10.0,
                           "vx": 1.0, "vy": 0.5, "vz": 0.0}, time.time())
        states = pt.get_all_states()
        assert len(states) == 1
        assert states[0][4] == 1.0  # vx

    def test_is_stale_true(self):
        pt = PeerTable()
        pt.update_peer(1, {"lat": -35.363}, time.time() - 10)
        assert pt.is_stale(1, 5.0) is True

    def test_is_stale_false(self):
        pt = PeerTable()
        pt.update_peer(1, {"lat": -35.363}, time.time())
        assert pt.is_stale(1, 5.0) is False

    def test_is_stale_unknown_peer(self):
        pt = PeerTable()
        assert pt.is_stale(99, 5.0) is True

    def test_alive_and_stale_ids(self):
        pt = PeerTable()
        now = time.time()
        pt.update_peer(1, {"lat": -35.363}, now)
        pt.update_peer(2, {"lat": -35.364}, now - 10)
        pt.update_peer(3, {"lat": -35.365}, now)

        alive = pt.get_alive_ids(5.0)
        stale = pt.get_stale_ids(5.0)
        assert alive == {1, 3}
        assert stale == {2}
