"""
Post-run visualization of process monitor data.
Reads CSV files produced by process_monitor.py and generates a multi-panel
matplotlib figure: Gantt chart, CPU% timeline, and per-core heatmap.

Usage:
    python -m scripts.monitor_visualizer --input-dir output/
"""

import os
import csv
import warnings
import argparse
import logging
from collections import defaultdict

log = logging.getLogger(__name__)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.colors import Normalize
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# Colors for roles — agents get warm colors, SITL children get cool ones
ROLE_COLORS = {
    "orchestrator": "#2c3e50",
    "gcs": "#8e44ad",
}
DRONE_COLORS = ["#e74c3c", "#e67e22", "#f1c40f", "#2ecc71", "#1abc9c",
                "#3498db", "#9b59b6", "#e84393", "#00cec9", "#fd79a8"]
SITL_COLORS = ["#c0392b", "#d35400", "#f39c12", "#27ae60", "#16a085",
               "#2980b9", "#8e44ad", "#d63031", "#00b894", "#e17055"]


def _get_color(role: str) -> str:
    """Assign a consistent color to a role name."""
    if role in ROLE_COLORS:
        return ROLE_COLORS[role]
    # drone_N -> index N-1
    for prefix, palette in [("drone_", DRONE_COLORS), ("sitl_", SITL_COLORS)]:
        if role.startswith(prefix):
            rest = role[len(prefix):]
            # Handle "drone_1", "drone_1_sitl", etc.
            num_str = rest.split("_")[0]
            try:
                idx = int(num_str) - 1
                return palette[idx % len(palette)]
            except ValueError:
                pass
    # Fallback
    return "#7f8c8d"


def load_process_csv(path: str) -> list[dict]:
    """Load process_monitor.csv into list of dicts."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            row["timestamp"] = float(row["timestamp"])
            row["elapsed_s"] = float(row["elapsed_s"])
            row["pid"] = int(row["pid"])
            row["cpu_pct"] = float(row["cpu_pct"])
            row["mem_rss_kb"] = int(row["mem_rss_kb"])
            row["mem_peak_kb"] = int(row["mem_peak_kb"])
            row["threads"] = int(row["threads"])
            row["core"] = int(row["core"])
            row["children"] = int(row["children"])
            rows.append(row)
    return rows


def load_core_csv(path: str) -> list[dict]:
    """Load core_utilization.csv into list of dicts."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            row["timestamp"] = float(row["timestamp"])
            row["elapsed_s"] = float(row["elapsed_s"])
            row["core_id"] = int(row["core_id"])
            row["cpu_pct"] = float(row["cpu_pct"])
            rows.append(row)
    return rows


def generate_charts(input_dir: str, output_name: str = "process_timeline.png"):
    """Generate the multi-panel process timeline chart."""
    if not HAS_MPL:
        log.error("matplotlib not available — cannot generate charts")
        return

    proc_path = os.path.join(input_dir, "process_monitor.csv")
    core_path = os.path.join(input_dir, "core_utilization.csv")

    if not os.path.exists(proc_path):
        log.error("Process CSV not found: %s", proc_path)
        return

    proc_data = load_process_csv(proc_path)
    if not proc_data:
        log.error("No process data to plot")
        return

    has_core_data = os.path.exists(core_path)
    core_data = load_core_csv(core_path) if has_core_data else []

    n_panels = 3 if core_data else 2
    fig, axes = plt.subplots(n_panels, 1, figsize=(14, 4 * n_panels),
                             gridspec_kw={"hspace": 0.35})
    if n_panels == 2:
        axes = [axes[0], axes[1]]

    # ── Panel 1: Process Lifetime Gantt Chart ──────────

    ax1 = axes[0]
    # Group by role: find first and last elapsed_s for each
    role_times: dict[str, tuple[float, float]] = {}
    for r in proc_data:
        role = r["role"]
        t = r["elapsed_s"]
        if role not in role_times:
            role_times[role] = (t, t)
        else:
            role_times[role] = (min(role_times[role][0], t), max(role_times[role][1], t))

    # Sort: orchestrator, gcs, then drone_1, drone_1_sitl, drone_2, ...
    def sort_key(role):
        if role == "orchestrator":
            return (0, 0, "")
        if role == "gcs":
            return (1, 0, "")
        parts = role.replace("drone_", "").replace("_sitl", ".1").replace("_child", ".2")
        try:
            num = float(parts.split(".")[0])
            sub = float(parts) if "." in parts else num
            return (2, sub, role)
        except ValueError:
            return (3, 0, role)

    sorted_roles = sorted(role_times.keys(), key=sort_key)
    y_positions = {role: i for i, role in enumerate(sorted_roles)}

    for role in sorted_roles:
        start, end = role_times[role]
        y = y_positions[role]
        width = max(end - start, 0.5)  # minimum bar width for visibility
        color = _get_color(role)
        ax1.barh(y, width, left=start, height=0.6, color=color, alpha=0.85,
                 edgecolor="white", linewidth=0.5)

    ax1.set_yticks(range(len(sorted_roles)))
    ax1.set_yticklabels(sorted_roles, fontsize=8)
    ax1.set_xlabel("Elapsed Time (s)")
    ax1.set_title("Process Lifetimes", fontweight="bold")
    ax1.invert_yaxis()
    ax1.grid(axis="x", alpha=0.3)

    # ── Panel 2: CPU% Over Time ────────────────────────

    ax2 = axes[1]
    # Group by role, collect (elapsed_s, cpu_pct) series
    role_series: dict[str, tuple[list[float], list[float]]] = defaultdict(lambda: ([], []))
    for r in proc_data:
        times, pcts = role_series[r["role"]]
        times.append(r["elapsed_s"])
        pcts.append(r["cpu_pct"])

    for role in sorted_roles:
        if role not in role_series:
            continue
        times, pcts = role_series[role]
        color = _get_color(role)
        linewidth = 1.5 if "_sitl" not in role and "_child" not in role else 1.0
        alpha = 0.9 if "_sitl" not in role and "_child" not in role else 0.6
        ax2.plot(times, pcts, color=color, linewidth=linewidth, alpha=alpha,
                 label=role)

    ax2.set_xlabel("Elapsed Time (s)")
    ax2.set_ylabel("CPU %")
    ax2.set_title("CPU Usage Over Time", fontweight="bold")
    ax2.grid(alpha=0.3)
    # Legend outside plot
    ax2.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7,
               ncol=1, framealpha=0.8)

    # ── Panel 3: Per-Core Utilization Heatmap ──────────

    if core_data and n_panels == 3:
        ax3 = axes[2]

        # Build 2D grid: rows=core_ids, cols=time bins
        core_ids = sorted(set(r["core_id"] for r in core_data))
        times = sorted(set(r["elapsed_s"] for r in core_data))
        time_to_idx = {t: i for i, t in enumerate(times)}
        core_to_idx = {c: i for i, c in enumerate(core_ids)}

        import numpy as np
        grid = np.full((len(core_ids), len(times)), float("nan"))
        for r in core_data:
            ci = core_to_idx[r["core_id"]]
            ti = time_to_idx[r["elapsed_s"]]
            grid[ci, ti] = r["cpu_pct"]

        im = ax3.imshow(grid, aspect="auto", cmap="YlOrRd",
                        vmin=0, vmax=100,
                        extent=[times[0], times[-1], len(core_ids) - 0.5, -0.5],
                        interpolation="nearest")
        ax3.set_yticks(range(len(core_ids)))
        ax3.set_yticklabels([f"Core {c}" for c in core_ids], fontsize=8)
        ax3.set_xlabel("Elapsed Time (s)")
        ax3.set_title("Per-Core CPU Utilization", fontweight="bold")
        fig.colorbar(im, ax=ax3, label="CPU %", shrink=0.8)

    fig.suptitle("Swarm Process Monitor — Timeline", fontsize=14,
                 fontweight="bold", y=0.98)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        fig.tight_layout(rect=[0, 0, 0.92, 0.96])

    output_path = os.path.join(input_dir, output_name)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    log.info("Timeline chart saved: %s", output_path)
    return output_path


# ── CLI ───────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Generate process monitor charts")
    parser.add_argument("--input-dir", default="output",
                        help="Directory containing process_monitor.csv and core_utilization.csv")
    parser.add_argument("--output", default="process_timeline.png",
                        help="Output filename for the chart")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="[VISUALIZER] %(asctime)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    generate_charts(args.input_dir, args.output)


if __name__ == "__main__":
    main()
