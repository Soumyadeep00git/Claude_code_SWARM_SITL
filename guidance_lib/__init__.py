"""Follower guidance library — pure Python, zero external dependencies.

Core API:
    compute_guidance()    — 3-mode hard-switched follower guidance
    compute_escape()      — simple radial repulsion for proximity avoidance

Config/State:
    GuidanceConfig / GuidanceState  — guidance tunables and tick state

Modular classes (for advanced usage / testing):
    TargetComputer      — leader + offset + feedforward -> target position
    ModeSelector        — EVASION > CATCHUP > TRACKING priority switch
    VelocityComputer    — per-mode velocity computation
    OutputSafety        — speed cap, rate limiter, altitude floor, NaN guard
    CommandSmoother     — EMA output filter with hover deadband

Failsafe monitoring is in the separate failsafe_lib package.
"""

from .guidance import (
    GuidanceConfig,
    GuidanceState,
    compute_guidance,
    compute_escape,
)
from .geo_utils import gps_to_ned, ned_to_gps, gps_distance_2d
from .command_smoother import CommandSmoother
from .target import TargetComputer
from .mode_selector import ModeSelector, GuidanceMode
from .velocity import VelocityComputer
from .output_safety import OutputSafety
