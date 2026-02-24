"""
Swarm visualizer — renders drone positions to frames.
Supports both live display (TkAgg) and headless GIF capture (Agg).
"""

import os
import logging
from io import BytesIO

log = logging.getLogger(__name__)

try:
    import matplotlib
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    log.warning("matplotlib not available — visualization disabled")

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


# Drone colors for up to 10 drones
COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00",
           "#a65628", "#f781bf", "#999999", "#66c2a5", "#fc8d62"]


class SwarmVisualizer:
    """2D plot of drone positions. Can save frames for GIF output."""

    def __init__(self, num_drones: int, enabled: bool = True,
                 headless: bool = False, output_dir: str = "output"):
        self.enabled = enabled and HAS_MPL
        self.num_drones = num_drones
        self.headless = headless
        self.output_dir = output_dir
        self._frames: list = []  # PIL Image frames for GIF
        self._ref_lat = None
        self._ref_lon = None
        self._frame_count = 0

        if not self.enabled:
            return

        # Use non-interactive backend for headless mode
        if headless:
            matplotlib.use("Agg")
        else:
            try:
                matplotlib.use("TkAgg")
            except Exception:
                matplotlib.use("Agg")
                self.headless = True

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        if not headless:
            try:
                self.fig.canvas.manager.set_window_title("Swarm SITL - GCS View")
                plt.ion()
            except Exception:
                self.headless = True

        log.info("Visualizer initialized (headless=%s)", self.headless)

    def update(self, states: dict[int, dict]):
        """Redraw the plot with latest drone positions."""
        if not self.enabled or not states:
            return

        self.ax.clear()

        # Use first drone with valid GPS as reference origin
        if self._ref_lat is None or (self._ref_lat == 0 and self._ref_lon == 0):
            for s in states.values():
                lat, lon = s.get("lat", 0), s.get("lon", 0)
                if lat != 0 or lon != 0:
                    self._ref_lat = lat
                    self._ref_lon = lon
                    break
            else:
                # No valid GPS yet — skip this frame
                self._frame_count += 1
                return

        for did, s in sorted(states.items()):
            lat, lon = s.get("lat", 0), s.get("lon", 0)
            # Skip drones without valid GPS
            if lat == 0 and lon == 0:
                continue
            dn = (lat - self._ref_lat) * 111320.0
            de = (lon - self._ref_lon) * 111320.0 * 0.8

            color = COLORS[(did - 1) % len(COLORS)]
            self.ax.plot(de, dn, "o", color=color, markersize=12)
            label = f"D{did}\n{s.get('alt', 0):.1f}m"
            if s.get("failsafe_active"):
                label += "\n[FS!]"
            self.ax.annotate(label, (de, dn), textcoords="offset points",
                             xytext=(8, 8), fontsize=8, color=color)

        # Build title from current drone modes
        modes = set(s.get("mode", "?") for s in states.values())
        alts = [s.get("alt", 0) for s in states.values()
                if s.get("lat", 0) != 0 or s.get("lon", 0) != 0]
        avg_alt = sum(alts) / len(alts) if alts else 0
        title = f"Swarm SITL — {len(states)} drones — avg alt {avg_alt:.1f}m"

        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")
        self.ax.set_title(title)
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)

        # Set consistent axis limits so GIF doesn't jump around
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(-50, 50)

        if self.headless:
            self._capture_frame()
        else:
            try:
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
            except Exception:
                pass

        self._frame_count += 1

    def _capture_frame(self):
        """Capture current figure as a PIL Image for GIF assembly."""
        if not HAS_PIL:
            return
        buf = BytesIO()
        self.fig.savefig(buf, format="png", dpi=80, bbox_inches="tight")
        buf.seek(0)
        img = Image.open(buf).copy()
        buf.close()
        # Capture every 5th frame to keep GIF size reasonable
        if self._frame_count % 5 == 0:
            self._frames.append(img)

    def save_gif(self, filename: str = "swarm_output.gif", fps: float = 5.0):
        """Save captured frames as an animated GIF."""
        if not self._frames:
            log.warning("No frames captured — cannot save GIF")
            return None

        os.makedirs(self.output_dir, exist_ok=True)
        filepath = os.path.join(self.output_dir, filename)

        duration_ms = int(1000.0 / fps)
        self._frames[0].save(
            filepath,
            save_all=True,
            append_images=self._frames[1:],
            duration=duration_ms,
            loop=0,
        )
        log.info("GIF saved: %s (%d frames)", filepath, len(self._frames))
        return filepath

    def save_final_frame(self, filename: str = "swarm_final.png"):
        """Save the last frame as a static PNG."""
        os.makedirs(self.output_dir, exist_ok=True)
        filepath = os.path.join(self.output_dir, filename)
        self.fig.savefig(filepath, dpi=120, bbox_inches="tight")
        log.info("Final frame saved: %s", filepath)
        return filepath
