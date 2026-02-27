"""Tests for drone_agent/geo.py — GPS/NED coordinate conversions."""

import math
import numpy as np
import pytest
from drone_agent.geo import gps_to_ned, ned_to_gps, gps_to_ned_2d, METERS_PER_DEG_LAT

REF_LAT = -35.3632620
REF_LON = 149.1652370
REF_ALT = 584.0


class TestGpsToNed:
    def test_origin_is_zero(self):
        ned = gps_to_ned(REF_LAT, REF_LON, REF_ALT, REF_LAT, REF_LON, REF_ALT)
        np.testing.assert_allclose(ned, [0, 0, 0], atol=1e-6)

    def test_north_positive(self):
        lat = REF_LAT + 100 / METERS_PER_DEG_LAT
        ned = gps_to_ned(lat, REF_LON, REF_ALT, REF_LAT, REF_LON, REF_ALT)
        assert ned[0] == pytest.approx(100.0, abs=0.01)
        assert ned[1] == pytest.approx(0.0, abs=0.01)

    def test_east_positive(self):
        m_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(REF_LAT))
        lon = REF_LON + 50 / m_per_deg_lon
        ned = gps_to_ned(REF_LAT, lon, REF_ALT, REF_LAT, REF_LON, REF_ALT)
        assert ned[0] == pytest.approx(0.0, abs=0.01)
        assert ned[1] == pytest.approx(50.0, abs=0.01)

    def test_down_is_negative_alt_increase(self):
        ned = gps_to_ned(REF_LAT, REF_LON, REF_ALT + 20, REF_LAT, REF_LON, REF_ALT)
        assert ned[2] == pytest.approx(-20.0, abs=0.01)

    def test_returns_float64_array(self):
        ned = gps_to_ned(REF_LAT, REF_LON, REF_ALT, REF_LAT, REF_LON, REF_ALT)
        assert ned.dtype == np.float64
        assert ned.shape == (3,)


class TestNedToGps:
    def test_origin_roundtrip(self):
        lat, lon, alt = ned_to_gps(0, 0, 0, REF_LAT, REF_LON, REF_ALT)
        assert lat == pytest.approx(REF_LAT, abs=1e-8)
        assert lon == pytest.approx(REF_LON, abs=1e-8)
        assert alt == pytest.approx(REF_ALT, abs=1e-4)

    def test_round_trip_accuracy(self):
        offsets = [(100, 0, 0), (0, 100, 0), (100, 50, -10),
                   (-50, -75, 5), (200, 200, -30)]
        for n, e, d in offsets:
            lat, lon, alt = ned_to_gps(n, e, d, REF_LAT, REF_LON, REF_ALT)
            ned_back = gps_to_ned(lat, lon, alt, REF_LAT, REF_LON, REF_ALT)
            np.testing.assert_allclose(ned_back, [n, e, d], atol=0.01,
                                       err_msg=f"Failed roundtrip for NED=({n},{e},{d})")


class TestGpsToNed2d:
    def test_origin_zero(self):
        n, e = gps_to_ned_2d(REF_LAT, REF_LON, REF_LAT, REF_LON)
        assert n == pytest.approx(0.0, abs=1e-6)
        assert e == pytest.approx(0.0, abs=1e-6)

    def test_north_offset(self):
        lat = REF_LAT + 50 / METERS_PER_DEG_LAT
        n, e = gps_to_ned_2d(lat, REF_LON, REF_LAT, REF_LON)
        assert n == pytest.approx(50.0, abs=0.01)
        assert e == pytest.approx(0.0, abs=0.01)
