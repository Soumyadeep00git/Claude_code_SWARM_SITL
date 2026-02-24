"""
GPS ↔ NED coordinate conversions for drone_agent package.
Uses flat-earth approximation (valid within ~10 km of reference point).
"""

import numpy as np
from math import cos, radians

METERS_PER_DEG_LAT = 111320.0


def gps_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """Convert GPS to NED numpy array [north, east, down] relative to ref."""
    n = (lat - ref_lat) * METERS_PER_DEG_LAT
    e = (lon - ref_lon) * (METERS_PER_DEG_LAT * cos(radians(ref_lat)))
    d = -(alt - ref_alt)
    return np.array([n, e, d], dtype=np.float64)


def ned_to_gps(n, e, d, ref_lat, ref_lon, ref_alt):
    """Convert NED meters back to GPS. Returns (lat, lon, alt)."""
    lat = ref_lat + n / METERS_PER_DEG_LAT
    lon = ref_lon + e / (METERS_PER_DEG_LAT * cos(radians(ref_lat)))
    alt = ref_alt - d
    return (lat, lon, alt)


def gps_to_ned_2d(lat, lon, ref_lat, ref_lon):
    """2D GPS → (north, east) in meters. Ignores altitude."""
    n = (lat - ref_lat) * METERS_PER_DEG_LAT
    e = (lon - ref_lon) * (METERS_PER_DEG_LAT * cos(radians(ref_lat)))
    return (n, e)
