#!/usr/bin/env python3
"""Plot raw Jetson CSVs — leader from leader file, follower from follower file.
No merging, no timestamp correction. Just the truth.

Usage:
  python plot_raw.py
  python plot_raw.py --ldr leader_events_20260305_173826.csv --fol follower_events_20260305_173834.csv
"""

import argparse
import glob
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

METERS_PER_DEG = 111_320.0


def latest_csv(prefix):
    """Find the latest CSV matching prefix."""
    files = sorted(glob.glob(f"{prefix}_*.csv"))
    return files[-1] if files else None


def load_gps(path):
    """Load CSV → GPS-event rows only, add t_s column (seconds since start)."""
    df = pd.read_csv(path)
    gps = df[df["event"] == "GPS"].copy()
    gps["t_s"] = gps["gcs_ts"] - gps["gcs_ts"].iloc[0]
    return gps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ldr", default=None, help="Leader CSV path")
    ap.add_argument("--fol", default=None, help="Follower CSV path")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    ldr_path = args.ldr or latest_csv("leader_events")
    fol_path = args.fol or latest_csv("follower_events")
    if not ldr_path or not fol_path:
        sys.exit("Cannot find CSV files. Use --ldr / --fol flags.")

    print(f"Leader : {ldr_path}")
    print(f"Follower: {fol_path}")

    lg = load_gps(ldr_path)
    fg = load_gps(fol_path)

    print(f"Leader GPS rows : {len(lg)}")
    print(f"Follower GPS rows: {len(fg)}")

    # ── Convert lat/lon to meters (NE) relative to leader start ──
    ref_lat = lg["lat"].iloc[0]
    ref_lon = lg["lon"].iloc[0]
    cos_lat = np.cos(np.radians(ref_lat))

    lg["n"] = (lg["lat"] - ref_lat) * METERS_PER_DEG
    lg["e"] = (lg["lon"] - ref_lon) * METERS_PER_DEG * cos_lat
    fg["n"] = (fg["lat"] - ref_lat) * METERS_PER_DEG
    fg["e"] = (fg["lon"] - ref_lon) * METERS_PER_DEG * cos_lat

    # ── FIGURE 1: Trajectory (East vs North) ─────────────────
    fig1, ax1 = plt.subplots(figsize=(10, 8))
    ax1.plot(lg["e"], lg["n"], "r-", lw=1.0, alpha=0.8, label="Leader")
    ax1.plot(fg["e"], fg["n"], "b-", lw=1.0, alpha=0.8, label="Follower")
    ax1.plot(lg["e"].iloc[0], lg["n"].iloc[0], "ro", ms=8, label="Leader start")
    ax1.plot(fg["e"].iloc[0], fg["n"].iloc[0], "bs", ms=8, label="Follower start")
    ax1.plot(lg["e"].iloc[-1], lg["n"].iloc[-1], "rx", ms=10, mew=2, label="Leader end")
    ax1.plot(fg["e"].iloc[-1], fg["n"].iloc[-1], "bx", ms=10, mew=2, label="Follower end")
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.set_title("Raw Trajectory — Leader from Leader CSV, Follower from Follower CSV")
    ax1.legend()
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)
    fig1.tight_layout()
    fig1.savefig("raw_trajectory.png", dpi=150)
    print("Saved raw_trajectory.png")

    # ── FIGURE 2: Velocities + Speed vs time ─────────────────
    fig2, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)

    axes[0].plot(lg["t_s"], lg["vn"], "r-", lw=0.7, alpha=0.8, label="Leader vn")
    axes[0].plot(fg["t_s"], fg["vn"], "b-", lw=0.7, alpha=0.8, label="Follower vn")
    axes[0].set_ylabel("vn (m/s)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(lg["t_s"], lg["ve"], "r-", lw=0.7, alpha=0.8, label="Leader ve")
    axes[1].plot(fg["t_s"], fg["ve"], "b-", lw=0.7, alpha=0.8, label="Follower ve")
    axes[1].set_ylabel("ve (m/s)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    lg_spd = np.sqrt(lg["vn"]**2 + lg["ve"]**2)
    fg_spd = np.sqrt(fg["vn"]**2 + fg["ve"]**2)
    axes[2].plot(lg["t_s"], lg_spd, "r-", lw=0.7, alpha=0.8, label="Leader speed")
    axes[2].plot(fg["t_s"], fg_spd, "b-", lw=0.7, alpha=0.8, label="Follower speed")
    axes[2].set_ylabel("speed (m/s)")
    axes[2].set_xlabel("Time since start (s)")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    fig2.suptitle("Raw Velocities — Each drone from its own CSV", fontsize=13)
    fig2.tight_layout()
    fig2.savefig("raw_velocities.png", dpi=150)
    print("Saved raw_velocities.png")

    # ── Quick stats ──────────────────────────────────────────
    print(f"\n--- Leader ---")
    print(f"  lat: {lg['lat'].min():.8f} → {lg['lat'].max():.8f}")
    print(f"  vn:  {lg['vn'].min():.2f} → {lg['vn'].max():.2f} m/s")
    print(f"  duration: {lg['t_s'].iloc[-1]:.1f}s")

    print(f"--- Follower ---")
    print(f"  lat: {fg['lat'].min():.8f} → {fg['lat'].max():.8f}")
    print(f"  vn:  {fg['vn'].min():.2f} → {fg['vn'].max():.2f} m/s")
    print(f"  duration: {fg['t_s'].iloc[-1]:.1f}s")

    # Check for DDS crosstalk: does follower lat match leader lat?
    fg_far = fg[fg["lat"] > ref_lat + 0.00002]
    print(f"\n⚠ Follower rows with lat > leader_home+2m: {len(fg_far)} / {len(fg)}")
    if len(fg_far) > 10:
        print("  → This means the follower logger is RECEIVING leader's GPS via DDS multicast!")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
