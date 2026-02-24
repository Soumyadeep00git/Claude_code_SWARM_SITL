"""
Geographic coordinate utilities for the GCS.
NED offset to/from GPS conversion.
"""

from math import cos, radians

METERS_PER_DEG_LAT = 111320.0


def ned_to_gps(ref_lat: float, ref_lon: float,
               north_m: float, east_m: float) -> tuple[float, float]:
    """Convert NED offset from reference point to GPS lat/lon."""
    lat = ref_lat + north_m / METERS_PER_DEG_LAT
    lon = ref_lon + east_m / (METERS_PER_DEG_LAT * cos(radians(ref_lat)))
    return lat, lon


def centroid(positions: list[tuple[float, float, float]]
             ) -> tuple[float, float, float]:
    """Compute centroid of GPS positions. Skips (0, 0) entries."""
    valid = [(lat, lon, alt) for lat, lon, alt in positions
             if lat != 0.0 or lon != 0.0]
    if not valid:
        return (0.0, 0.0, 0.0)
    n = len(valid)
    return (
        sum(p[0] for p in valid) / n,
        sum(p[1] for p in valid) / n,
        sum(p[2] for p in valid) / n,
    )
