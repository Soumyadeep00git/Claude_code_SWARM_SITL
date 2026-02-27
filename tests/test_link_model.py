"""Tests for comms/link_model.py — radio link quality math."""

import math
import pytest
from comms.link_model import (
    haversine_m, link_quality, packet_loss_probability,
    should_drop, compute_latency_ms, bandwidth_available_bps,
    DEFAULT_MAX_RANGE_M, DEFAULT_BANDWIDTH_BPS,
)


class TestHaversine:
    def test_same_point_is_zero(self):
        assert haversine_m(-35.363, 149.165, -35.363, 149.165) == pytest.approx(0.0, abs=0.01)

    def test_known_distance(self):
        # ~111km per degree of latitude
        d = haversine_m(0.0, 0.0, 1.0, 0.0)
        assert d == pytest.approx(111194.9, rel=0.01)

    def test_symmetric(self):
        d1 = haversine_m(-35.363, 149.165, -35.364, 149.166)
        d2 = haversine_m(-35.364, 149.166, -35.363, 149.165)
        assert d1 == pytest.approx(d2, abs=0.001)

    def test_small_distance(self):
        # 10m north at reference lat
        lat2 = -35.363 + 10 / 111320.0
        d = haversine_m(-35.363, 149.165, lat2, 149.165)
        assert d == pytest.approx(10.0, rel=0.01)


class TestLinkQuality:
    def test_zero_distance(self):
        assert link_quality(0.0) == 1.0

    def test_negative_distance(self):
        assert link_quality(-5.0) == 1.0

    def test_at_max_range(self):
        assert link_quality(DEFAULT_MAX_RANGE_M) == 0.0

    def test_beyond_max_range(self):
        assert link_quality(DEFAULT_MAX_RANGE_M + 50) == 0.0

    def test_monotonically_decreasing(self):
        prev = 1.0
        for d in range(1, int(DEFAULT_MAX_RANGE_M) + 1):
            q = link_quality(float(d))
            assert q <= prev
            prev = q

    def test_mid_range(self):
        q = link_quality(50.0, max_range_m=100.0)
        assert 0.0 < q < 1.0

    def test_custom_range(self):
        assert link_quality(0.0, max_range_m=200.0) == 1.0
        assert link_quality(200.0, max_range_m=200.0) == 0.0


class TestPacketLoss:
    def test_perfect_quality(self):
        assert packet_loss_probability(1.0) == 0.0

    def test_zero_quality(self):
        assert packet_loss_probability(0.0) == 1.0

    def test_mid_quality(self):
        loss = packet_loss_probability(0.5)
        assert 0.0 < loss < 1.0
        assert loss == pytest.approx(0.25)  # (1-0.5)^2


class TestShouldDrop:
    def test_perfect_never_drops(self):
        for _ in range(100):
            assert should_drop(1.0) is False

    def test_zero_always_drops(self):
        for _ in range(100):
            assert should_drop(0.0) is True

    def test_mid_quality_probabilistic(self):
        drops = sum(should_drop(0.5) for _ in range(1000))
        # Expected ~25% drop rate (0.25 * 1000 = 250), allow wide margin
        assert 100 < drops < 500


class TestLatency:
    def test_perfect_quality_base_latency(self):
        lat = compute_latency_ms(0.0, 1.0)
        assert lat == pytest.approx(2.0, abs=0.01)

    def test_degraded_quality_adds_jitter(self):
        latencies = [compute_latency_ms(50.0, 0.5) for _ in range(100)]
        assert all(lat >= 2.0 for lat in latencies)
        assert max(latencies) > 2.0  # jitter should make some > base


class TestBandwidth:
    def test_perfect_quality(self):
        assert bandwidth_available_bps(1.0) == DEFAULT_BANDWIDTH_BPS

    def test_zero_quality(self):
        assert bandwidth_available_bps(0.0) == 0.0

    def test_half_quality(self):
        assert bandwidth_available_bps(0.5) == pytest.approx(DEFAULT_BANDWIDTH_BPS * 0.5)
