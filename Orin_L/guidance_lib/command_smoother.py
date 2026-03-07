"""First-order EMA command smoother with velocity deadband.

Smooths APF output and snaps horizontal to zero when below deadband
so the drone hovers cleanly instead of jittering.

With alpha=0.3 at 10Hz: tau~0.23s, 95% response in ~0.7s.
"""

import math


class CommandSmoother:
    """First-order EMA with velocity deadband for clean hover."""

    def __init__(self, alpha: float = 0.3, deadband: float = 0.2,
                 dt: float = 0.1):
        self.alpha = alpha
        self.deadband = deadband
        self.dt = dt
        self._v = [0.0, 0.0, 0.0]

    def filter(self, vn: float, ve: float, vd: float
               ) -> tuple[float, float, float]:
        raw = [vn, ve, vd]
        a = self.alpha
        for i in range(3):
            self._v[i] += a * (raw[i] - self._v[i])
        # Horizontal deadband: hover instead of jitter
        h = math.sqrt(self._v[0] * self._v[0] + self._v[1] * self._v[1])
        if h < self.deadband:
            return 0.0, 0.0, self._v[2]
        return self._v[0], self._v[1], self._v[2]

    def reset(self):
        """Reset filter state (e.g. after tracking lost / mode switch)."""
        self._v = [0.0, 0.0, 0.0]
