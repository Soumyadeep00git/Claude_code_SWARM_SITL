"""Failsafe library — standalone safety monitor for leader-follower drones.

Core API:
    compute_failsafe()    — run all safety checks (call BEFORE guidance each tick)
    leader_in_oblivion()  — standalone geofence check for leader position
    load_config()         — load FailsafeConfig from config.yaml

Config/State:
    FailsafeConfig  — all tunables (geofence, stale thresholds, etc.)
    FailsafeState   — persistent counters across ticks
"""

import os
import yaml

from .failsafe import (
    FailsafeConfig,
    FailsafeState,
    compute_failsafe,
    leader_in_oblivion,
)

_DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config.yaml')


def load_config(path=None, home_lat=0.0, home_lon=0.0):
    """Load FailsafeConfig from YAML, injecting runtime home position.

    Args:
        path: Path to config.yaml. Defaults to failsafe_lib/config.yaml.
        home_lat: Home latitude (deployment param, from env).
        home_lon: Home longitude (deployment param, from env).

    Returns:
        FailsafeConfig with values from YAML + caller-supplied home_lat/lon.
    """
    if path is None:
        path = _DEFAULT_CONFIG_PATH

    with open(path) as f:
        raw = yaml.safe_load(f)

    params = raw.get('failsafe', {})
    return FailsafeConfig(
        home_lat=home_lat,
        home_lon=home_lon,
        geofence_radius_m=params.get('geofence_radius_m', 200.0),
        deadman_ticks=params.get('deadman_ticks', 20),
        leader_stale_ticks=params.get('leader_stale_ticks', 50),
        catchup_timeout_ticks=params.get('catchup_timeout_ticks', 300),
        max_altitude_m=params.get('max_altitude_m', 100.0),
    )
