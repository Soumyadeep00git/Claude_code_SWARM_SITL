"""Real-time 6-panel matplotlib visualization for SITL test.

Supports both live display (TkAgg) and headless GIF/PNG capture (Agg).

Layout (3x2):
  Top-left:     Map view (NED) with leader/follower/target trails
  Top-right:    Offset error (N/E/total) vs time
  Mid-left:     Speed profiles (leader/follower/limit)
  Mid-right:    Peer distance + guidance flags
  Bottom-left:  Guidance mode weights (evasion/tracking/catchup)
  Bottom-right: Feedforward gain + peer distance correlation

Usage:
  LiveVisualizer(enabled=True)                  — auto-detect display
  LiveVisualizer(enabled=True, headless=True)   — force headless + GIF
"""

import logging
import math
import os
import time
from collections import deque
from io import BytesIO

from sim.config import METERS_PER_DEG_LAT, OUTPUT_DIR

log = logging.getLogger(__name__)

# Try to import matplotlib with appropriate backend
_VIZ_AVAILABLE = False
try:
    import matplotlib
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    log.warning("matplotlib not available, visualization disabled")

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# Guidance mode colors
_MODE_COLORS = {
    "TRACKING": "#e8f5e9",   # Light green
    "CATCHUP": "#fff3e0",    # Light orange
    "EVASION": "#ffebee",    # Light red
}


def _setup_backend(headless: bool) -> bool:
    """Configure matplotlib backend. Returns True if headless."""
    if not HAS_MPL:
        return True

    if headless:
        matplotlib.use("Agg")
        return True

    # Auto-detect: try TkAgg, fall back to Agg
    try:
        matplotlib.use("TkAgg")
        return False
    except Exception:
        try:
            matplotlib.use("Agg")
            log.info("No display detected, using headless mode (Agg)")
            return True
        except Exception:
            return True


class LiveVisualizer:
    """6-panel real-time flight visualization with behavior monitoring.

    Supports headless GIF capture for WSL2 / CI environments.
    """

    def __init__(self, enabled: bool = True, headless: bool = False,
                 save_gif: bool = False):
        self.enabled = enabled and HAS_MPL
        self.save_gif_flag = save_gif
        self._frames: list = []  # PIL Image frames for GIF
        self._frame_count = 0

        if not self.enabled:
            self.headless = True
            log.info("Visualization disabled")
            return

        self.headless = _setup_backend(headless)
        if self.headless:
            self.save_gif_flag = True  # Always save GIF in headless mode

        import matplotlib.pyplot as plt
        self._plt = plt

        plt.ion()
        self.fig, self.axes = plt.subplots(3, 2, figsize=(14, 14))
        self.fig.suptitle(
            "Leader-Follower SITL Test — 3-Mode Sigmoid Guidance",
            fontsize=14)

        self.ax_map = self.axes[0, 0]
        self.ax_err = self.axes[0, 1]
        self.ax_spd = self.axes[1, 0]
        self.ax_dist = self.axes[1, 1]
        self.ax_behav = self.axes[2, 0]
        self.ax_gains = self.axes[2, 1]

        # Trail buffers (60s at 5Hz = 300 points)
        self._maxlen = 300
        self.leader_trail_n = deque(maxlen=self._maxlen)
        self.leader_trail_e = deque(maxlen=self._maxlen)
        self.follower_trail_n = deque(maxlen=self._maxlen)
        self.follower_trail_e = deque(maxlen=self._maxlen)
        self.target_trail_n = deque(maxlen=self._maxlen)
        self.target_trail_e = deque(maxlen=self._maxlen)

        # Time series
        self.time_buf = deque(maxlen=600)
        self.err_n_buf = deque(maxlen=600)
        self.err_e_buf = deque(maxlen=600)
        self.err_total_buf = deque(maxlen=600)
        self.leader_spd_buf = deque(maxlen=600)
        self.follower_spd_buf = deque(maxlen=600)
        self.peer_dist_buf = deque(maxlen=600)

        # Guidance mode time series
        self.w_evasion_buf = deque(maxlen=600)
        self.w_tracking_buf = deque(maxlen=600)
        self.w_catchup_buf = deque(maxlen=600)
        self.guidance_mode_buf = deque(maxlen=600)
        self.ff_gain_buf = deque(maxlen=600)

        self._start_time = time.time()
        self._ref_lat = None
        self._ref_lon = None
        self._update_count = 0

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        mode_str = "headless+GIF" if self.headless else "live display"
        log.info("Visualization initialized (6-panel, %s)", mode_str)

    def _to_ned(self, lat, lon):
        """Convert GPS to NED relative to first valid leader position."""
        if self._ref_lat is None:
            return 0.0, 0.0
        n = (lat - self._ref_lat) * METERS_PER_DEG_LAT
        e = (lon - self._ref_lon) * METERS_PER_DEG_LAT * math.cos(
            math.radians(self._ref_lat))
        return n, e

    def update(self, leader_pos: dict | None, follower_pos: dict | None,
               apf_result: dict | None, target_lat: float, target_lon: float,
               phase: str):
        """Update all 6 panels. Call at ~5Hz."""
        if not self.enabled:
            return

        self._update_count += 1
        t = time.time() - self._start_time

        # Set reference on first valid leader GPS
        if self._ref_lat is None and leader_pos and leader_pos['lat'] != 0:
            self._ref_lat = leader_pos['lat']
            self._ref_lon = leader_pos['lon']

        # Accumulate data
        if leader_pos and self._ref_lat:
            ln, le = self._to_ned(leader_pos['lat'], leader_pos['lon'])
            self.leader_trail_n.append(ln)
            self.leader_trail_e.append(le)
            l_spd = math.sqrt(leader_pos['vx']**2 + leader_pos['vy']**2)
        else:
            l_spd = 0.0

        if follower_pos and self._ref_lat:
            fn, fe = self._to_ned(follower_pos['lat'], follower_pos['lon'])
            self.follower_trail_n.append(fn)
            self.follower_trail_e.append(fe)
            f_spd = math.sqrt(follower_pos['vx']**2 + follower_pos['vy']**2)
        else:
            f_spd = 0.0

        if target_lat != 0 and self._ref_lat:
            tn, te = self._to_ned(target_lat, target_lon)
            self.target_trail_n.append(tn)
            self.target_trail_e.append(te)

        self.time_buf.append(t)
        self.leader_spd_buf.append(l_spd)
        self.follower_spd_buf.append(f_spd)

        # Offset error + behavior data
        if apf_result:
            self.peer_dist_buf.append(apf_result.get('peer_dist', 0))
            # Compute offset error from follower to target
            if follower_pos and target_lat != 0 and self._ref_lat:
                fn, fe = self._to_ned(follower_pos['lat'], follower_pos['lon'])
                tn, te = self._to_ned(target_lat, target_lon)
                en = tn - fn
                ee = te - fe
                self.err_n_buf.append(en)
                self.err_e_buf.append(ee)
                self.err_total_buf.append(math.sqrt(en * en + ee * ee))
            else:
                self.err_n_buf.append(0)
                self.err_e_buf.append(0)
                self.err_total_buf.append(0)

            # Guidance mode data
            self.w_evasion_buf.append(apf_result.get('w_evasion', 0.0))
            self.w_tracking_buf.append(apf_result.get('w_tracking', 1.0))
            self.w_catchup_buf.append(apf_result.get('w_catchup', 0.0))
            self.guidance_mode_buf.append(apf_result.get('mode', 'TRACKING'))
            self.ff_gain_buf.append(apf_result.get('ff_gain', 0.8))
        else:
            self.peer_dist_buf.append(0)
            self.err_n_buf.append(0)
            self.err_e_buf.append(0)
            self.err_total_buf.append(0)
            self.w_evasion_buf.append(0.0)
            self.w_tracking_buf.append(1.0)
            self.w_catchup_buf.append(0.0)
            self.guidance_mode_buf.append('TRACKING')
            self.ff_gain_buf.append(0.8)

        # Only redraw every 2nd call (~2.5Hz actual redraw)
        if self._update_count % 2 != 0:
            return

        try:
            self._draw(phase, apf_result)

            if self.headless:
                self._capture_frame()
            else:
                self.fig.canvas.flush_events()
                self._plt.pause(0.001)
        except Exception:
            pass  # Don't let viz crash the control loop

    def _draw(self, phase: str, apf_result: dict | None):
        # Current guidance info for panel titles
        g_mode = apf_result.get('mode', 'TRACKING') if apf_result else 'TRACKING'

        # ── Panel 1: Map view ─────────────────────────────
        ax = self.ax_map
        ax.clear()
        mode_tag = f" | {g_mode}" if g_mode != "TRACKING" else ""
        ax.set_title(f"Map View (NED)  [{phase}]{mode_tag}")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # Tint map background based on guidance mode
        bg_color = _MODE_COLORS.get(g_mode, '#ffffff')
        ax.set_facecolor(bg_color)

        if self.leader_trail_n:
            ax.plot(list(self.leader_trail_e), list(self.leader_trail_n),
                    'r-', alpha=0.4, linewidth=1)
            ax.plot(self.leader_trail_e[-1], self.leader_trail_n[-1],
                    'ro', markersize=8, label='Leader')

        if self.follower_trail_n:
            ax.plot(list(self.follower_trail_e), list(self.follower_trail_n),
                    'b-', alpha=0.4, linewidth=1)
            ax.plot(self.follower_trail_e[-1], self.follower_trail_n[-1],
                    'bs', markersize=8, label='Follower')

        if self.target_trail_n:
            ax.plot(self.target_trail_e[-1], self.target_trail_n[-1],
                    'g+', markersize=12, markeredgewidth=2, label='Target')

        ax.legend(loc='upper left', fontsize=8)

        # ── Panel 2: Offset error ─────────────────────────
        ax = self.ax_err
        ax.clear()
        ax.set_title("Offset Error")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.grid(True, alpha=0.3)

        t = list(self.time_buf)
        if t:
            ax.plot(t, list(self.err_n_buf), 'r-', label='North err', alpha=0.7)
            ax.plot(t, list(self.err_e_buf), 'b-', label='East err', alpha=0.7)
            ax.plot(t, list(self.err_total_buf), 'k-', label='Total', linewidth=2)
            ax.axhline(y=0.3, color='gray', linestyle='--', alpha=0.5, label='Deadzone')
        ax.legend(loc='upper right', fontsize=8)

        # ── Panel 3: Speed profiles ───────────────────────
        ax = self.ax_spd
        ax.clear()
        ax.set_title("Speed Profiles")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (m/s)")
        ax.grid(True, alpha=0.3)

        if t:
            ax.plot(t, list(self.leader_spd_buf), 'r-', label='Leader')
            ax.plot(t, list(self.follower_spd_buf), 'b-', label='Follower')
            ax.axhline(y=3.0, color='gray', linestyle='--', alpha=0.5, label='Speed limit')
        ax.legend(loc='upper right', fontsize=8)

        # ── Panel 4: Peer distance + flags ────────────────
        ax = self.ax_dist
        ax.clear()
        ax.set_title("Peer Distance & Guidance Status")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance (m)")
        ax.grid(True, alpha=0.3)

        if t:
            ax.plot(t, list(self.peer_dist_buf), 'g-', label='Peer dist', linewidth=2)
            ax.axhline(y=1.0, color='red', linestyle='--', alpha=0.5, label='Hard radius')
            ax.axhline(y=2.0, color='orange', linestyle='--', alpha=0.3, label='Soft radius')

        if apf_result and apf_result.get('emergency'):
            ax.set_facecolor('#ffeeee')
        if apf_result and apf_result.get('flags'):
            flags_str = ', '.join(apf_result['flags'].keys())
            ax.text(0.02, 0.98, f"Flags: {flags_str}",
                    transform=ax.transAxes, fontsize=7,
                    verticalalignment='top', color='red')

        ax.legend(loc='upper right', fontsize=8)

        # ── Panel 5: Guidance mode weights ──────────────────
        ax = self.ax_behav
        ax.clear()
        ax.set_title("Guidance Mode Weights")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Weight (0-1)")
        ax.set_ylim(-0.05, 1.1)
        ax.grid(True, alpha=0.3)

        if t:
            modes = list(self.guidance_mode_buf)
            self._draw_mode_bands(ax, t, modes)

            ax.plot(t, list(self.w_tracking_buf), 'g-', linewidth=2, label='Tracking')
            ax.plot(t, list(self.w_catchup_buf), 'orange', linewidth=2, label='Catch-up')
            ax.plot(t, list(self.w_evasion_buf), 'r-', linewidth=2, label='Evasion')

        ax.legend(loc='upper right', fontsize=7)

        # ── Panel 6: Feedforward gain ────────────────────
        ax = self.ax_gains
        ax.clear()
        ax.set_title("Feedforward & Distance")
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)

        if t:
            ff = list(self.ff_gain_buf)
            ff_norm = [v / 0.8 for v in ff]
            ax.plot(t, ff_norm, 'm-', linewidth=1.5, label='FF gain (norm)')

            # Normalize peer_dist to 0-1 range for overlay
            dists = list(self.peer_dist_buf)
            d_norm = [min(d / 20.0, 1.5) for d in dists]
            ax.plot(t, d_norm, 'g--', alpha=0.5, label='Peer dist (norm)')

            ax.set_ylim(-0.05, 1.5)

        ax.legend(loc='upper right', fontsize=7)

    def _draw_mode_bands(self, ax, t: list, modes: list):
        """Draw colored background bands for behavior mode regions."""
        if len(t) < 2:
            return

        # Find contiguous mode regions
        i = 0
        while i < len(modes):
            mode = modes[i]
            j = i + 1
            while j < len(modes) and modes[j] == mode:
                j += 1
            # Draw band from t[i] to t[j-1]
            color = _MODE_COLORS.get(mode, '#ffffff')
            if mode != "TRACKING":  # Only color non-tracking regions
                ax.axvspan(t[i], t[min(j - 1, len(t) - 1)],
                           color=color, alpha=0.4)
            i = j

    # ── Frame capture & output ──────────────────────────────

    def _capture_frame(self):
        """Capture current figure as a PIL Image for GIF assembly."""
        if not HAS_PIL:
            return
        self._frame_count += 1
        # Capture every 5th drawn frame to keep GIF size reasonable
        if self._frame_count % 5 != 0:
            return
        buf = BytesIO()
        self.fig.savefig(buf, format="png", dpi=80, bbox_inches="tight")
        buf.seek(0)
        img = Image.open(buf).copy()
        buf.close()
        self._frames.append(img)

    def save_gif(self, filename: str = None, fps: float = 4.0) -> str | None:
        """Save captured frames as an animated GIF. Returns filepath or None."""
        if not self._frames:
            log.warning("No frames captured — cannot save GIF")
            return None

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        if filename is None:
            ts = time.strftime('%Y%m%d_%H%M%S')
            filename = f"leader_follower_{ts}.gif"
        filepath = os.path.join(OUTPUT_DIR, filename)

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

    def save_final_frame(self, filename: str = "leader_follower_final.png") -> str | None:
        """Save the last frame as a high-res static PNG."""
        if not self.enabled:
            return None
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        filepath = os.path.join(OUTPUT_DIR, filename)
        try:
            self.fig.savefig(filepath, dpi=150, bbox_inches="tight")
            log.info("Final frame saved: %s", filepath)
            return filepath
        except Exception as e:
            log.warning("Could not save final frame: %s", e)
            return None

    def close(self):
        if self.enabled:
            try:
                self._plt.ioff()
                self._plt.close(self.fig)
            except Exception:
                pass
