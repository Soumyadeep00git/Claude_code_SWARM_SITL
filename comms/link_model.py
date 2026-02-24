"""
Radio link quality model for mesh network simulation.
Distance-based packet loss, latency, and bandwidth estimation.
Pure math — no I/O, no state.
"""

import random
from math import radians, cos, sin, sqrt, atan2

# Defaults
DEFAULT_MAX_RANGE_M = 100.0
DEFAULT_MIN_LATENCY_MS = 2.0
DEFAULT_BANDWIDTH_BPS = 250_000  # 250 kbps simulated radio


def haversine_m(lat1, lon1, lat2, lon2):
    """Distance in meters between two GPS coordinates."""
    R = 6371000.0
    rlat1, rlat2 = radians(lat1), radians(lat2)
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(rlat1) * cos(rlat2) * sin(dlon / 2) ** 2
    return R * 2 * atan2(sqrt(a), sqrt(1 - a))


def link_quality(distance_m, max_range_m=DEFAULT_MAX_RANGE_M):
    """
    Link quality [0.0, 1.0] based on distance.
    Smooth falloff: 1.0 at 0m, ~0.5 at 60% range, 0.0 at max range.
    """
    if distance_m <= 0:
        return 1.0
    if distance_m >= max_range_m:
        return 0.0
    ratio = distance_m / max_range_m
    # Quadratic falloff
    return max(0.0, 1.0 - ratio * ratio)


def packet_loss_probability(quality):
    """Packet loss probability from quality. loss = (1 - quality)^2."""
    return (1.0 - quality) ** 2


def should_drop(quality):
    """Roll dice: True if packet should be dropped."""
    if quality <= 0.0:
        return True
    if quality >= 1.0:
        return False
    return random.random() < packet_loss_probability(quality)


def compute_latency_ms(distance_m, quality):
    """
    One-way latency in ms.
    Base 2ms + quality-dependent jitter (0-5ms as quality drops).
    """
    base = DEFAULT_MIN_LATENCY_MS
    jitter = random.uniform(0, 5.0 * (1.0 - quality)) if quality < 1.0 else 0.0
    return base + jitter


def bandwidth_available_bps(quality, base_bps=DEFAULT_BANDWIDTH_BPS):
    """Effective bandwidth = base * quality."""
    return base_bps * max(quality, 0.0)
