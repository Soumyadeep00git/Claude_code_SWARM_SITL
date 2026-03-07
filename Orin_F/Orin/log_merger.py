#!/usr/bin/env python3
"""Log Merger & Analyzer — merges leader + follower event logs by GCS time.

Reads leader_events_*.csv and follower_events_*.csv (produced by
leader_logger.py and follower_logger.py), aligns rows by GCS-synced
timestamp using nearest-neighbor matching, and produces:

  1. merged_<ts>.csv  — every follower GPS event matched with nearest
                        leader GPS event, TRUE distance computed from
                        each drone's own GPS (not relayed peer data)
  2. Console report   — rate stats, distance stats, sync quality,
                        event breakdowns, data freshness analysis

Architecture:
  ┌─────────────┐                     ┌──────────────┐
  │ Leader Orin  │  leader_events.csv  │ Follower Orin│  follower_events.csv
  │ (logger.py)  │──────────────┐     │  (logger.py) │──────────────┐
  └──────────────┘              │     └──────────────┘              │
                                ▼                                   ▼
                         ┌──────────────────────────────────────────┐
                         │     log_merger.py  (this script)         │
                         │     Runs on LAPTOP after SCP'ing CSVs    │
                         │     Uses pandas for nearest-neighbor merge│
                         └──────────────────────────────────────────┘
                                           │
                                  merged_<ts>.csv   + console report

Usage:
  # Explicit paths:
  python log_merger.py leader_events_20260305_143000.csv follower_events_20260305_143000.csv

  # Auto-discover latest pair:
  python log_merger.py --auto

  # With full timeline (all events, not just GPS):
  python log_merger.py --auto --timeline

Requirements:
  pip install pandas numpy
"""

import argparse
import glob
import math
import os
import sys
from datetime import datetime

try:
    import pandas as pd
    import numpy as np
except ImportError:
    print("ERROR: pandas and numpy required. Install with:")
    print("  pip install pandas numpy")
    sys.exit(1)


METERS_PER_DEG = 111_320.0


# ═══════════════════════════════════════════════════════════════
# DDS Cross-Talk Filter
# ═══════════════════════════════════════════════════════════════
# When both Jetsons share the same WiFi + ROS_DOMAIN_ID (default 0),
# DDS multicast causes the follower's /mavros/global_position/global
# to receive GPS messages from BOTH MAVROS nodes.  Each logger then
# records alternating own-GPS and other-drone-GPS, producing ~12 m
# jumps every other row.
#
# Fix for future flights: set different ROS_DOMAIN_ID per Jetson.
# This filter cleans already-collected logs.
# ═══════════════════════════════════════════════════════════════

def remove_dds_crosstalk(gps_df: pd.DataFrame,
                         max_jump_m: float = 8.0) -> pd.DataFrame:
    """Remove GPS rows caused by DDS cross-talk between two Jetsons.

    When both Jetsons are on the same ROS_DOMAIN_ID + WiFi, each drone's
    logger receives interleaved GPS from BOTH MAVROS nodes.  The result
    is lat/lon that alternates between the drone's real position and
    the OTHER drone's position (~10-100 m per jump).

    Strategy: walk through sorted GPS events and reject any row whose
    lat/lon jumps more than *max_jump_m* from the last *accepted* row.
    The first GPS point seeds the reference.

    Returns a filtered copy (rows are dropped, not modified).
    """
    if gps_df.empty:
        return gps_df

    df = gps_df.sort_values('gcs_ts').reset_index(drop=True)
    lats = df['lat'].values
    lons = df['lon'].values

    keep = np.ones(len(df), dtype=bool)
    ref_lat = lats[0]
    ref_lon = lons[0]

    cos_lat = np.cos(np.radians(ref_lat))

    for i in range(1, len(df)):
        dn = (lats[i] - ref_lat) * METERS_PER_DEG
        de = (lons[i] - ref_lon) * METERS_PER_DEG * cos_lat
        jump = np.sqrt(dn * dn + de * de)

        if jump > max_jump_m:
            # Check if the NEXT accepted row continues from this one
            # (i.e., maybe WE are the stale one and this is the real move)
            # Heuristic: if 3 consecutive rows all agree with the new pos,
            # accept the new pos as the real move.
            ahead_ok = 0
            for j in range(i + 1, min(i + 4, len(df))):
                djn = (lats[j] - lats[i]) * METERS_PER_DEG
                dje = (lons[j] - lons[i]) * METERS_PER_DEG * cos_lat
                if np.sqrt(djn * djn + dje * dje) < max_jump_m:
                    ahead_ok += 1
                else:
                    break
            if ahead_ok >= 2:
                # Genuine position change — accept and update reference
                ref_lat = lats[i]
                ref_lon = lons[i]
                cos_lat = np.cos(np.radians(ref_lat))
            else:
                keep[i] = False
        else:
            ref_lat = lats[i]
            ref_lon = lons[i]

    removed = int(np.sum(~keep))
    if removed > 0:
        print(f"  DDS cross-talk filter: removed {removed}/{len(df)} GPS rows "
              f"(max_jump={max_jump_m}m)")
    return df[keep].reset_index(drop=True)


def dist_3d(lat1, lon1, alt1, lat2, lon2, alt2):
    """Flat-earth 3D distance in meters."""
    if (lat1 == 0 and lon1 == 0) or (lat2 == 0 and lon2 == 0):
        return -1.0
    dn = (lat2 - lat1) * METERS_PER_DEG
    de = (lon2 - lon1) * METERS_PER_DEG * math.cos(math.radians(lat1))
    dd = alt2 - alt1
    return math.sqrt(dn*dn + de*de + dd*dd)


def dist_horiz(lat1, lon1, lat2, lon2):
    """Flat-earth 2D horizontal distance in meters."""
    if (lat1 == 0 and lon1 == 0) or (lat2 == 0 and lon2 == 0):
        return -1.0
    dn = (lat2 - lat1) * METERS_PER_DEG
    de = (lon2 - lon1) * METERS_PER_DEG * math.cos(math.radians(lat1))
    return math.sqrt(dn*dn + de*de)


def load_events(path: str) -> pd.DataFrame:
    """Load event CSV with proper numeric types."""
    df = pd.read_csv(path, low_memory=False)
    df['gcs_ts'] = pd.to_numeric(df['gcs_ts'], errors='coerce')
    df = df.dropna(subset=['gcs_ts'])
    df = df.sort_values('gcs_ts').reset_index(drop=True)
    return df


def merge_gps_events(leader_df: pd.DataFrame, follower_df: pd.DataFrame,
                     max_dt_s: float = 0.5) -> pd.DataFrame:
    """Merge leader+follower GPS events by nearest GCS timestamp.

    For each follower GPS event, find the nearest leader GPS event
    within max_dt_s. Compute TRUE distance from each drone's own GPS.
    """
    ldr_gps = leader_df[leader_df['event'] == 'GPS'].copy()
    fol_gps = follower_df[follower_df['event'] == 'GPS'].copy()

    if ldr_gps.empty or fol_gps.empty:
        print("WARNING: No GPS events in one or both files")
        return pd.DataFrame()

    # Remove DDS cross-talk rows (caused by shared ROS_DOMAIN_ID + WiFi)
    print("Leader GPS filtering:")
    ldr_gps = remove_dds_crosstalk(ldr_gps)
    print("Follower GPS filtering:")
    fol_gps = remove_dds_crosstalk(fol_gps)

    ldr_gps = ldr_gps.sort_values('gcs_ts')
    fol_gps = fol_gps.sort_values('gcs_ts')

    merged = pd.merge_asof(
        fol_gps, ldr_gps,
        on='gcs_ts', direction='nearest', tolerance=max_dt_s,
        suffixes=('_fol', '_ldr')
    )
    merged = merged.dropna(subset=['lat_ldr'])

    # TRUE distances
    merged['true_dist_3d_m'] = merged.apply(
        lambda r: dist_3d(r['lat_ldr'], r['lon_ldr'], r.get('rel_alt_ldr', 0),
                          r['lat_fol'], r['lon_fol'], r.get('rel_alt_fol', 0)),
        axis=1)
    merged['true_dist_horiz_m'] = merged.apply(
        lambda r: dist_horiz(r['lat_ldr'], r['lon_ldr'],
                             r['lat_fol'], r['lon_fol']),
        axis=1)

    return merged


def full_timeline_merge(leader_df: pd.DataFrame, follower_df: pd.DataFrame) -> pd.DataFrame:
    """Full timeline: for each follower event, attach latest known leader state."""
    ldr = leader_df.copy().sort_values('gcs_ts').reset_index(drop=True)
    fol = follower_df.copy().sort_values('gcs_ts').reset_index(drop=True)

    ldr_r = ldr.rename(columns={c: f'ldr_{c}' if c != 'gcs_ts' else c
                                 for c in ldr.columns})
    fol_r = fol.rename(columns={c: f'fol_{c}' if c != 'gcs_ts' else c
                                 for c in fol.columns})

    timeline = pd.merge_asof(
        fol_r, ldr_r, on='gcs_ts', direction='backward'
    )

    if 'ldr_lat' in timeline.columns and 'fol_lat' in timeline.columns:
        timeline['true_dist_m'] = timeline.apply(
            lambda r: dist_3d(
                r.get('ldr_lat', 0), r.get('ldr_lon', 0), r.get('ldr_rel_alt', 0),
                r.get('fol_lat', 0), r.get('fol_lon', 0), r.get('fol_rel_alt', 0)),
            axis=1)

    return timeline


def print_report(leader_df, follower_df, merged_gps):
    """Print comprehensive analysis report."""

    def safe_dur(df):
        d = df['gcs_ts'].max() - df['gcs_ts'].min()
        return max(d, 0.1)

    ldr_dur = safe_dur(leader_df)
    fol_dur = safe_dur(follower_df)

    print("\n" + "=" * 70)
    print("  LOG MERGER ANALYSIS REPORT")
    print("=" * 70)

    print(f"\nLeader:   {len(leader_df):,} events over {ldr_dur:.1f}s "
          f"({len(leader_df)/ldr_dur:.1f} evt/s)")
    print(f"Follower: {len(follower_df):,} events over {fol_dur:.1f}s "
          f"({len(follower_df)/fol_dur:.1f} evt/s)")

    # Event breakdown
    for label, df, dur in [("Leader", leader_df, ldr_dur),
                           ("Follower", follower_df, fol_dur)]:
        counts = df['event'].value_counts()
        print(f"\n{label} event breakdown:")
        for evt, cnt in counts.items():
            print(f"  {evt:12s}: {cnt:6d} ({cnt/dur:6.1f} Hz)")

    # Sync quality
    for label, df in [("Leader", leader_df), ("Follower", follower_df)]:
        off = pd.to_numeric(df['sync_off_ms'], errors='coerce').dropna()
        off = off[off != 0]
        if not off.empty:
            print(f"\n{label} clock sync: mean={off.mean():+.1f}ms "
                  f"std={off.std():.1f}ms range=[{off.min():.1f}, {off.max():.1f}]")
        else:
            print(f"\n{label}: NO TIME SYNC")

    # GPS merge distance results
    if not merged_gps.empty:
        valid = merged_gps[merged_gps['true_dist_3d_m'] >= 0]
        if not valid.empty:
            d = valid['true_dist_3d_m']
            print(f"\n{'─' * 50}")
            print(f"TRUE DISTANCE ({len(valid)} GPS-aligned pairs):")
            print(f"  Min={d.min():.2f}m  Max={d.max():.2f}m  "
                  f"Mean={d.mean():.2f}m  Median={d.median():.2f}m  Std={d.std():.2f}m")

            if len(valid) >= 10:
                n = len(valid)
                q = n // 4
                print(f"  Q1={d.iloc[:q].mean():.2f}m  Q2={d.iloc[q:2*q].mean():.2f}m  "
                      f"Q3={d.iloc[2*q:3*q].mean():.2f}m  Q4={d.iloc[3*q:].mean():.2f}m")

        # Follower-reported vs true distance
        if 'dist_m_fol' in merged_gps.columns:
            rep = pd.to_numeric(merged_gps['dist_m_fol'], errors='coerce')
            rep = rep[rep >= 0]
            if not rep.empty and not valid.empty:
                n = min(len(valid), len(rep))
                errs = np.abs(valid['true_dist_3d_m'].iloc[:n].values - rep.iloc[:n].values)
                print(f"\n  Follower-reported mean: {rep.mean():.2f}m  "
                      f"True mean: {d.mean():.2f}m")
                print(f"  Abs error: mean={errs.mean():.2f}m  max={errs.max():.2f}m")

    # Leader data freshness
    if 'ldr_data_age_s' in follower_df.columns:
        ages = pd.to_numeric(follower_df['ldr_data_age_s'], errors='coerce')
        ages = ages[(ages >= 0) & (ages < 60)]
        if not ages.empty:
            print(f"\nLEADER DATA AGE at follower ({len(ages)} samples):")
            print(f"  Mean={ages.mean()*1000:.1f}ms  Median={ages.median()*1000:.1f}ms  "
                  f"P95={ages.quantile(0.95)*1000:.1f}ms  Max={ages.max()*1000:.1f}ms")
            stale = (ages > 2.0).sum()
            if stale: print(f"  WARNING: {stale} samples >2s stale")

    # RSSI
    for label, df in [("Leader", leader_df), ("Follower", follower_df)]:
        if 'rssi' in df.columns:
            rs = pd.to_numeric(df['rssi'], errors='coerce')
            rs = rs[rs > 0]
            if not rs.empty:
                print(f"\n{label} RSSI: mean={rs.mean():.0f} "
                      f"min={rs.min():.0f} max={rs.max():.0f}")

    # Velocity commands (follower)
    if 'cmd_vn' in follower_df.columns:
        cmd = follower_df[follower_df['event'] == 'CMD']
        if not cmd.empty:
            vn = pd.to_numeric(cmd['cmd_vn'], errors='coerce')
            ve = pd.to_numeric(cmd['cmd_ve'], errors='coerce')
            vd = pd.to_numeric(cmd['cmd_vd'], errors='coerce')
            spd = np.sqrt(vn**2 + ve**2 + vd**2)
            print(f"\nFOLLOWER CMD ({len(cmd)} events): "
                  f"mean_speed={spd.mean():.3f}m/s  max={spd.max():.3f}m/s")

    # Mission/guidance breakdown
    if 'mission' in follower_df.columns:
        ms = follower_df['mission'].value_counts()
        print(f"\nMISSION STATES: {dict(ms)}")

    if 'guid_mode' in follower_df.columns:
        gm = follower_df['guid_mode'].value_counts()
        names = {0: 'NONE', 1: 'TRACKING', 2: 'CATCHUP', 3: 'EVASION'}
        mapped = {names.get(int(k), f'?{k}'): v for k, v in gm.items()}
        print(f"GUIDANCE MODES: {mapped}")

    print(f"\n{'=' * 70}")


def find_latest_pair():
    """Find the most recent leader/follower event CSV pair."""
    leaders = sorted(glob.glob('leader_events_*.csv'))
    followers = sorted(glob.glob('follower_events_*.csv'))
    if not leaders:
        print("No leader_events_*.csv found"); sys.exit(1)
    if not followers:
        print("No follower_events_*.csv found"); sys.exit(1)
    return leaders[-1], followers[-1]


def main():
    parser = argparse.ArgumentParser(
        description='Merge and analyze leader + follower event logs')
    parser.add_argument('leader_csv', nargs='?', help='Leader events CSV')
    parser.add_argument('follower_csv', nargs='?', help='Follower events CSV')
    parser.add_argument('--auto', action='store_true',
                        help='Auto-discover latest log pair in cwd')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='Output merged CSV path')
    parser.add_argument('--timeline', action='store_true',
                        help='Also produce full timeline merge (all events)')
    args = parser.parse_args()

    if args.auto or (not args.leader_csv and not args.follower_csv):
        leader_csv, follower_csv = find_latest_pair()
    else:
        if not args.leader_csv or not args.follower_csv:
            parser.print_help(); sys.exit(1)
        leader_csv, follower_csv = args.leader_csv, args.follower_csv

    print(f"Leader:   {leader_csv}")
    print(f"Follower: {follower_csv}")

    leader_df = load_events(leader_csv)
    follower_df = load_events(follower_csv)
    print(f"Loaded {len(leader_df):,} leader + {len(follower_df):,} follower events")

    merged_gps = merge_gps_events(leader_df, follower_df)

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = args.output or f'merged_{ts}.csv'

    if not merged_gps.empty:
        merged_gps.to_csv(out_path, index=False)
        print(f"GPS merge: {len(merged_gps)} matched pairs → {out_path}")

    if args.timeline:
        tl = full_timeline_merge(leader_df, follower_df)
        tl_path = out_path.replace('.csv', '_timeline.csv')
        tl.to_csv(tl_path, index=False)
        print(f"Timeline:  {len(tl)} rows → {tl_path}")

    print_report(leader_df, follower_df, merged_gps)


if __name__ == '__main__':
    main()
