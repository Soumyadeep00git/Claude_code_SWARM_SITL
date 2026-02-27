"""Tests for drone_agent/mesh_router.py — OLSR-lite routing."""

import time
import pytest
from drone_agent.mesh_router import MeshRouter, AD_INTERVAL_S, MAX_TTL


class TestNeighborManagement:
    def test_empty_neighbors(self):
        mr = MeshRouter(drone_id=1, num_drones=5)
        assert mr.neighbors == {}

    def test_update_neighbors(self):
        mr = MeshRouter(drone_id=1, num_drones=5)
        mr.update_neighbors({
            2: {"quality": 0.9, "distance_m": 10},
            3: {"quality": 0.5, "distance_m": 50},
        })
        assert mr.neighbors == {2: 0.9, 3: 0.5}

    def test_zero_quality_excluded(self):
        mr = MeshRouter(drone_id=1, num_drones=5)
        mr.update_neighbors({
            2: {"quality": 0.9, "distance_m": 10},
            3: {"quality": 0.0, "distance_m": 200},
        })
        assert 3 not in mr.neighbors


class TestAdvertisement:
    def test_should_advertise_initially(self):
        mr = MeshRouter(drone_id=1, num_drones=5)
        assert mr.should_advertise() is True

    def test_should_not_advertise_immediately_after(self):
        mr = MeshRouter(drone_id=1, num_drones=5)
        mr.should_advertise()  # resets timer
        assert mr.should_advertise() is False

    def test_advertisement_data_format(self):
        mr = MeshRouter(drone_id=1, num_drones=5)
        mr.update_neighbors({2: {"quality": 0.8, "distance_m": 20}})
        data = mr.get_advertisement_data()
        assert "neighbors" in data
        assert "2" in data["neighbors"]
        assert data["neighbors"]["2"] == 0.8


class TestRouting:
    def _setup_linear_chain(self):
        """Create a 1→2→3 chain topology."""
        # Drone 1: direct neighbor of 2
        mr1 = MeshRouter(drone_id=1, num_drones=3)
        mr1.update_neighbors({2: {"quality": 0.9, "distance_m": 10}})

        # Drone 2: neighbor of 1 and 3
        mr2 = MeshRouter(drone_id=2, num_drones=3)
        mr2.update_neighbors({
            1: {"quality": 0.9, "distance_m": 10},
            3: {"quality": 0.8, "distance_m": 20},
        })

        # Drone 3: neighbor of 2 only
        mr3 = MeshRouter(drone_id=3, num_drones=3)
        mr3.update_neighbors({2: {"quality": 0.8, "distance_m": 20}})

        # Exchange advertisements
        ad2 = mr2.get_advertisement_data()
        mr1.handle_neighbor_ad(2, ad2)
        mr3.handle_neighbor_ad(2, ad2)

        ad1 = mr1.get_advertisement_data()
        mr2.handle_neighbor_ad(1, ad1)

        ad3 = mr3.get_advertisement_data()
        mr2.handle_neighbor_ad(3, ad3)

        return mr1, mr2, mr3

    def test_direct_neighbor_route(self):
        mr1, mr2, mr3 = self._setup_linear_chain()
        assert mr1.get_next_hop(2) == 2

    def test_multi_hop_route(self):
        mr1, mr2, mr3 = self._setup_linear_chain()
        assert mr1.get_next_hop(3) == 2  # 1→2→3

    def test_unreachable_returns_none(self):
        mr = MeshRouter(drone_id=1, num_drones=5)
        assert mr.get_next_hop(99) is None

    def test_reachable_peers(self):
        mr1, mr2, mr3 = self._setup_linear_chain()
        reachable = mr1.get_reachable_peers()
        assert 2 in reachable
        assert 3 in reachable

    def test_routing_table_structure(self):
        mr1, mr2, mr3 = self._setup_linear_chain()
        rt = mr1.get_routing_table()
        assert 2 in rt
        assert "next_hop" in rt[2]
        assert "hops" in rt[2]
        assert "metric" in rt[2]


class TestMeshForward:
    def test_wrap_forward(self):
        mr1 = MeshRouter(drone_id=1, num_drones=3)
        mr1.update_neighbors({2: {"quality": 0.9, "distance_m": 10}})
        mr1.handle_neighbor_ad(2, {"neighbors": {"3": 0.8}})

        fwd = mr1.wrap_forward(3, {"cmd": "test"})
        assert fwd is not None
        assert fwd["origin"] == 1
        assert fwd["dest"] == 3
        assert fwd["ttl"] == MAX_TTL
        assert fwd["payload"]["cmd"] == "test"

    def test_wrap_forward_no_route(self):
        mr = MeshRouter(drone_id=1, num_drones=3)
        assert mr.wrap_forward(99, {"cmd": "test"}) is None

    def test_dedup_prevents_reprocess(self):
        mr = MeshRouter(drone_id=2, num_drones=3)
        mr.update_neighbors({
            1: {"quality": 0.9, "distance_m": 10},
            3: {"quality": 0.8, "distance_m": 20},
        })
        mr.handle_neighbor_ad(1, {"neighbors": {}})
        mr.handle_neighbor_ad(3, {"neighbors": {}})

        data = {
            "origin": 1, "dest": 3, "next_hop": 2,
            "ttl": 3, "msg_id": "test-msg-1", "payload": {},
        }
        result1 = mr.handle_mesh_forward(data)
        result2 = mr.handle_mesh_forward(data)
        # First should forward, second should be deduped
        assert result2 is None

    def test_ttl_expiry(self):
        mr = MeshRouter(drone_id=2, num_drones=3)
        mr.update_neighbors({3: {"quality": 0.8, "distance_m": 20}})
        mr.handle_neighbor_ad(3, {"neighbors": {}})

        data = {
            "origin": 1, "dest": 3, "next_hop": 2,
            "ttl": 1, "msg_id": "ttl-test", "payload": {},
        }
        result = mr.handle_mesh_forward(data)
        assert result is None  # TTL=1 decrements to 0 → expired

    def test_destination_returns_none(self):
        mr = MeshRouter(drone_id=3, num_drones=3)
        data = {
            "origin": 1, "dest": 3, "next_hop": 3,
            "ttl": 3, "msg_id": "dest-test", "payload": {"cmd": "hello"},
        }
        result = mr.handle_mesh_forward(data)
        assert result is None  # We're the destination
