"""Leader guidance — re-exports from follower_drone (single source of truth)."""
from follower_drone.guidance import (   # noqa: F401
    GuidanceConfig,
    GuidanceState,
    compute_escape,
)

__all__ = ["GuidanceConfig", "GuidanceState", "compute_escape"]
