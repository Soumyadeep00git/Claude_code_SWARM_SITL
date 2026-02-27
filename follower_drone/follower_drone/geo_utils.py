"""GPS <-> NED flat-earth coordinate conversions."""

import math

METERS_PER_DEG_LAT = 111320.0


def gps_to_ned(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Convert GPS to NED meters relative to reference point (2D, ignores alt)."""
    north = (lat - ref_lat) * METERS_PER_DEG_LAT
    east = (lon - ref_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    return north, east


def ned_to_gps(north: float, east: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Convert NED meters back to GPS coordinates."""
    lat = ref_lat + north / METERS_PER_DEG_LAT
    cos_lat = math.cos(math.radians(ref_lat))
    lon = ref_lon + east / (METERS_PER_DEG_LAT * cos_lat) if cos_lat > 1e-10 else ref_lon
    return lat, lon


def gps_distance_2d(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Flat-earth 2D distance between two GPS points in meters."""
    dn = (lat2 - lat1) * METERS_PER_DEG_LAT
    de = (lon2 - lon1) * METERS_PER_DEG_LAT * math.cos(math.radians(lat1))
    return math.sqrt(dn * dn + de * de)
