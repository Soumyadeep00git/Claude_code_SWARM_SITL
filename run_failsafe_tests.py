#!/usr/bin/env python3
"""Comprehensive failsafe + guidance simulation — ALL 6 failure modes.

Simulates the follower control loop (no Docker / SITL required) with
injected failures. Generates CSV telemetry logs and GIF visualizations
for each test scenario.

Scenarios:
  0. NOMINAL    — normal following, no failures
  1. GPS_LOSS   — own GPS goes invalid → HOVER then DEADMAN (RTL)
  2. LEADER_STALE — leader data stops arriving → HOVER then STALE_CRITICAL
  3. LEADER_GEOFENCE — leader flies outside geofence → HOVER
  4. FOLLOWER_GEOFENCE — follower drifts outside geofence → RTL
  5. ALTITUDE_CEILING — follower rises above max altitude → HOVER
  6. CATCHUP_TIMEOUT — follower stuck in CATCHUP too long → RTL

Each scenario runs the FULL pipeline:
  failsafe_lib.compute_failsafe() → guidance_lib.compute_guidance()
and logs every tick to CSV.
"""

import csv
import math
import os
import sys
import time
from dataclasses import dataclass, field

_PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _PROJECT_DIR)

from failsafe_lib import FailsafeState, compute_failsafe, load_config
from guidance_lib import (
    GuidanceConfig, GuidanceState, compute_guidance, CommandSmoother,
)
from guidance_lib.target import TargetComputer
from guidance_lib.geo_utils import ned_to_gps, gps_distance_2d

# ── Constants ────────────────────────────────────────────────────
HOME_LAT = -35.3632620
HOME_LON = 149.1652370
CONTROL_HZ = 10
DT = 1.0 / CONTROL_HZ
OFFSET_N = -5.0
OFFSET_E = 3.0

OUTPUT_DIR = os.path.join(_PROJECT_DIR, "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ═══════════════════════════════════════════════════════════════
# Simulated leader trajectory
# ═══════════════════════════════════════════════════════════════

def _leader_trajectory(tick, scenario):
    """Return (lat, lon, alt, vn, ve) for the leader at this tick."""
    t = tick * DT
    # Default: leader circles slowly
    radius_m = 15.0
    omega = 0.1  # rad/s
    cx_n = radius_m * math.cos(omega * t)
    cx_e = radius_m * math.sin(omega * t)
    vn = -radius_m * omega * math.sin(omega * t)
    ve = radius_m * omega * math.cos(omega * t)
    lat, lon = ned_to_gps(cx_n, cx_e, HOME_LAT, HOME_LON)
    alt = 10.0

    if scenario == "LEADER_GEOFENCE":
        # Leader flies straight north fast — crosses 200m geofence at ~tick 100
        speed = 20.0  # m/s (fast enough to cross geofence before catchup timeout)
        n = speed * t
        lat, lon = ned_to_gps(n, 0.0, HOME_LAT, HOME_LON)
        vn = speed
        ve = 0.0

    return lat, lon, alt, vn, ve


# ═══════════════════════════════════════════════════════════════
# Simulated follower physics (simple integrator)
# ═══════════════════════════════════════════════════════════════

@dataclass
class SimFollower:
    lat: float = HOME_LAT
    lon: float = HOME_LON
    alt: float = 10.0
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0

    def step(self, cmd_vn, cmd_ve, cmd_vd, dt):
        """Integrate velocity command (simple Euler)."""
        # Smooth acceleration toward commanded velocity
        alpha = 0.5
        self.vn += alpha * (cmd_vn - self.vn)
        self.ve += alpha * (cmd_ve - self.ve)
        self.vd += alpha * (cmd_vd - self.vd)

        dn = self.vn * dt
        de = self.ve * dt
        self.lat += dn / 111320.0
        self.lon += de / (111320.0 * math.cos(math.radians(self.lat)))
        self.alt -= self.vd * dt  # NED: positive vd is down


# ═══════════════════════════════════════════════════════════════
# Failure injectors
# ═══════════════════════════════════════════════════════════════

def _inject_failure(tick, scenario, follower, leader_lat, leader_lon,
                    leader_alt, leader_vn, leader_ve):
    """Return modified values based on failure scenario.

    Returns:
        own_gps_valid, leader_fresh, peer_gps_valid,
        leader_lat, leader_lon, leader_alt, leader_vn, leader_ve,
        follower_alt_override
    """
    own_gps_valid = True
    leader_fresh = True
    peer_gps_valid = True
    alt_override = None

    if scenario == "GPS_LOSS":
        # GPS goes bad at tick 50, stays bad
        if tick >= 50:
            own_gps_valid = False

    elif scenario == "LEADER_STALE":
        # Leader data stops arriving at tick 60
        if tick >= 60:
            leader_fresh = False
            peer_gps_valid = False
            leader_lat = 0.0
            leader_lon = 0.0
            leader_vn = 0.0
            leader_ve = 0.0

    elif scenario == "LEADER_GEOFENCE":
        # Leader flies away (handled by trajectory), no injection needed
        pass

    elif scenario == "FOLLOWER_GEOFENCE":
        # At tick 30, push follower 250m east (outside 200m geofence)
        if tick == 30:
            cos_lat = math.cos(math.radians(HOME_LAT))
            follower.lon = HOME_LON + 250.0 / (111320.0 * cos_lat)

    elif scenario == "ALTITUDE_CEILING":
        # At tick 40, push follower to 120m (above 100m ceiling)
        if tick == 40:
            follower.alt = 120.0
            alt_override = 120.0

    elif scenario == "CATCHUP_TIMEOUT":
        # Leader circles at 100m from home (inside 200m geofence) but fast
        # enough that follower can never converge → stays in CATCHUP → timeout.
        t_s = tick * DT
        r = 100.0  # circle radius (well inside 200m geofence)
        omega = 0.3  # fast enough that follower can't converge
        cx_n = r * math.cos(omega * t_s)
        cx_e = r * math.sin(omega * t_s)
        leader_lat, leader_lon = ned_to_gps(cx_n, cx_e, HOME_LAT, HOME_LON)
        leader_vn = -r * omega * math.sin(omega * t_s)
        leader_ve = r * omega * math.cos(omega * t_s)

    return (own_gps_valid, leader_fresh, peer_gps_valid,
            leader_lat, leader_lon, leader_alt, leader_vn, leader_ve,
            alt_override)


# ═══════════════════════════════════════════════════════════════
# Run one scenario
# ═══════════════════════════════════════════════════════════════

def run_scenario(scenario, max_ticks=500):
    """Run a single failure scenario and return tick-by-tick log."""

    # Load failsafe config (single source of truth)
    fs_cfg = load_config(home_lat=HOME_LAT, home_lon=HOME_LON)
    fs_state = FailsafeState()

    # Guidance config — max_altitude_m from failsafe config
    g_cfg = GuidanceConfig(max_altitude_m=fs_cfg.max_altitude_m)
    g_state = GuidanceState()
    smoother = CommandSmoother(dt=DT)
    target = TargetComputer()

    follower = SimFollower()
    log = []
    outcome = "COMPLETED"
    last_guidance_mode = "TRACKING"
    # Track last known leader position for logging (avoid 0,0 blowing up viz)
    last_known_l_lat = HOME_LAT
    last_known_l_lon = HOME_LON

    for tick in range(max_ticks):
        # Leader trajectory
        l_lat, l_lon, l_alt, l_vn, l_ve = _leader_trajectory(tick, scenario)

        # Inject failure
        (own_gps_valid, leader_fresh, peer_gps_valid,
         l_lat, l_lon, l_alt, l_vn, l_ve, alt_ovr) = _inject_failure(
            tick, scenario, follower,
            l_lat, l_lon, l_alt, l_vn, l_ve)

        if alt_ovr is not None:
            follower.alt = alt_ovr

        # Track last known leader position (for viz — don't log 0,0)
        if l_lat != 0.0 or l_lon != 0.0:
            last_known_l_lat = l_lat
            last_known_l_lon = l_lon

        # ── FAILSAFE (runs first) ────────────────────────────
        fs = compute_failsafe(
            own_lat=follower.lat, own_lon=follower.lon, own_alt=follower.alt,
            own_gps_valid=own_gps_valid,
            peer_lat=l_lat, peer_lon=l_lon,
            peer_gps_valid=peer_gps_valid,
            leader_fresh=leader_fresh,
            in_catchup=(last_guidance_mode == 'CATCHUP'),
            cfg=fs_cfg, state=fs_state,
        )

        fs_action = fs['action']
        fs_flags = ','.join(sorted(k for k, v in fs['flags'].items()
                                    if isinstance(v, bool) and v))
        fs_safe = fs['safe']

        cmd_vn, cmd_ve, cmd_vd = 0.0, 0.0, 0.0
        g_mode = "FAILSAFE"
        g_flags_str = ""
        peer_dist = float('inf')

        if fs_safe:
            # ── GUIDANCE ──────────────────────────────────────
            has_leader = (leader_fresh and l_lat != 0.0)
            if has_leader:
                t_lat, t_lon, t_alt = target.compute(
                    l_lat, l_lon, l_alt, l_vn, l_ve,
                    OFFSET_N, OFFSET_E, 0.0,
                    ff_gain=g_cfg.ff_gain, dt=DT)
            else:
                t_lat, t_lon, t_alt = follower.lat, follower.lon, follower.alt

            result = compute_guidance(
                my_lat=follower.lat, my_lon=follower.lon, my_alt=follower.alt,
                my_vn=follower.vn, my_ve=follower.ve, my_vd=follower.vd,
                peer_lat=l_lat, peer_lon=l_lon, peer_alt=l_alt,
                peer_vn=l_vn, peer_ve=l_ve,
                goal_lat=t_lat, goal_lon=t_lon, goal_alt=t_alt,
                cfg=g_cfg, state=g_state,
            )

            g_mode = result['mode']
            last_guidance_mode = g_mode
            g_flags_str = ','.join(sorted(k for k, v in result['flags'].items()
                                          if isinstance(v, bool) and v))
            peer_dist = result['peer_dist']

            sm_vn, sm_ve, sm_vd = smoother.filter(
                result['vn'], result['ve'], result['vd'])
            cmd_vn, cmd_ve, cmd_vd = sm_vn, sm_ve, sm_vd
        else:
            # Failsafe active — zero velocity
            smoother.reset()
            cmd_vn = cmd_ve = cmd_vd = 0.0

        # ── Physics step ──────────────────────────────────────
        follower.step(cmd_vn, cmd_ve, cmd_vd, DT)

        # ── Distance from home ────────────────────────────────
        home_dist = gps_distance_2d(follower.lat, follower.lon, HOME_LAT, HOME_LON)

        # ── Log ───────────────────────────────────────────────
        log.append({
            'tick': tick,
            'time_s': round(tick * DT, 2),
            'f_lat': round(follower.lat, 7),
            'f_lon': round(follower.lon, 7),
            'f_alt': round(follower.alt, 2),
            'f_vn': round(follower.vn, 3),
            'f_ve': round(follower.ve, 3),
            'l_lat': round(last_known_l_lat, 7),
            'l_lon': round(last_known_l_lon, 7),
            'l_alt': round(l_alt, 2),
            'peer_dist': round(peer_dist, 2) if peer_dist < 1e6 else 999.0,
            'home_dist': round(home_dist, 2),
            'fs_safe': int(fs_safe),
            'fs_action': fs_action,
            'fs_flags': fs_flags,
            'g_mode': g_mode,
            'g_flags': g_flags_str,
            'cmd_vn': round(cmd_vn, 3),
            'cmd_ve': round(cmd_ve, 3),
            'no_gps_ctr': fs_state.no_gps_counter,
            'stale_ctr': fs_state.stale_counter,
            'catchup_ticks': fs_state.catchup_ticks,
        })

        # ── Check for terminal failsafe ───────────────────────
        if not fs_safe and fs_action == 'RTL':
            outcome = f"RTL @ tick {tick} ({fs_flags})"
            break
        if not fs_safe:
            # Non-RTL failsafe — continue a few ticks for visualization,
            # then terminate if stuck in failsafe for 30+ consecutive ticks
            unsafe_tail = [r for r in log[-30:] if not r['fs_safe']]
            if len(unsafe_tail) >= 30:
                ftype = "HOVER_EMERGENCY" if fs['emergency'] else "HOVER"
                outcome = f"{ftype} @ tick {tick} ({fs_flags})"
                break

    return log, outcome


# ═══════════════════════════════════════════════════════════════
# CSV writer
# ═══════════════════════════════════════════════════════════════

def write_csv(log, path):
    if not log:
        return
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=log[0].keys())
        w.writeheader()
        w.writerows(log)


# ═══════════════════════════════════════════════════════════════
# Visualization (GIF)
# ═══════════════════════════════════════════════════════════════

def make_gif(log, scenario, outcome, path, fps=10):
    """Generate an animated GIF showing the scenario."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        from PIL import Image
        import io
    except ImportError as e:
        print(f"  [SKIP GIF] missing dependency: {e}")
        return

    frames = []
    total = len(log)
    # Sample every N ticks for manageable GIF size
    step = max(1, total // 100)

    # Extract data
    f_lats = [r['f_lat'] for r in log]
    f_lons = [r['f_lon'] for r in log]
    l_lats = [r['l_lat'] for r in log]
    l_lons = [r['l_lon'] for r in log]

    # Convert to meters from home
    def to_m(lats, lons):
        ns = [(la - HOME_LAT) * 111320.0 for la in lats]
        es = [(lo - HOME_LON) * 111320.0 * math.cos(math.radians(HOME_LAT))
              for lo in lons]
        return ns, es

    f_ns, f_es = to_m(f_lats, f_lons)
    l_ns, l_es = to_m(l_lats, l_lons)

    # Compute plot bounds
    all_n = f_ns + l_ns + [0]
    all_e = f_es + l_es + [0]
    margin = 30
    n_min = min(all_n) - margin
    n_max = max(all_n) + margin
    e_min = min(all_e) - margin
    e_max = max(all_e) + margin

    # Color map for failsafe state
    def _color(row):
        if not row['fs_safe']:
            return 'red'
        if row['g_mode'] == 'CATCHUP':
            return 'orange'
        if row['g_mode'] == 'EVASION':
            return 'yellow'
        return 'limegreen'

    for i in range(0, total, step):
        fig, ax = plt.subplots(1, 1, figsize=(6, 6), dpi=80)

        # Geofence circle
        circle = Circle((0, 0), 200.0, fill=False, edgecolor='red',
                         linestyle='--', linewidth=1.5, alpha=0.5)
        ax.add_patch(circle)

        # Home marker
        ax.plot(0, 0, 'k^', markersize=10, label='HOME')

        # Leader trail + current position
        ax.plot(l_es[:i+1], l_ns[:i+1], 'b-', alpha=0.3, linewidth=1)
        ax.plot(l_es[i], l_ns[i], 'bs', markersize=8, label='Leader')

        # Follower trail (colored by state)
        for j in range(1, min(i+1, len(log))):
            ax.plot([f_es[j-1], f_es[j]], [f_ns[j-1], f_ns[j]],
                    color=_color(log[j]), linewidth=1.5, alpha=0.6)
        c = _color(log[i])
        ax.plot(f_es[i], f_ns[i], 'o', color=c, markersize=8,
                markeredgecolor='black', label='Follower')

        # Info text
        row = log[i]
        info = (f"Scenario: {scenario}\n"
                f"t={row['time_s']:.1f}s  tick={row['tick']}\n"
                f"Mode: {row['g_mode']}  Safe: {'Y' if row['fs_safe'] else 'N'}\n"
                f"Action: {row['fs_action']}")
        if row['fs_flags']:
            info += f"\nFlags: {row['fs_flags']}"
        ax.text(0.02, 0.98, info, transform=ax.transAxes,
                fontsize=8, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

        # Telemetry
        telem = (f"Home: {row['home_dist']:.0f}m  Alt: {row['f_alt']:.1f}m\n"
                 f"Peer: {row['peer_dist']:.0f}m\n"
                 f"GPS_ctr: {row['no_gps_ctr']}  Stale_ctr: {row['stale_ctr']}")
        ax.text(0.98, 0.02, telem, transform=ax.transAxes,
                fontsize=7, verticalalignment='bottom', horizontalalignment='right',
                fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

        ax.set_xlim(e_min, e_max)
        ax.set_ylim(n_min, n_max)
        ax.set_aspect('equal')
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title(f'{scenario} — {outcome}', fontsize=10, fontweight='bold')
        ax.legend(loc='lower left', fontsize=7)
        ax.grid(True, alpha=0.3)

        buf = io.BytesIO()
        fig.savefig(buf, format='png', bbox_inches='tight')
        plt.close(fig)
        buf.seek(0)
        frames.append(Image.open(buf).copy())
        buf.close()

    if frames:
        # Hold last frame longer
        durations = [int(1000 / fps)] * len(frames)
        durations[-1] = 2000  # 2s hold on final frame
        frames[0].save(path, save_all=True, append_images=frames[1:],
                       duration=durations, loop=0)


# ═══════════════════════════════════════════════════════════════
# Make a static summary plot (all scenarios on one figure)
# ═══════════════════════════════════════════════════════════════

def make_summary(all_results, path):
    """Single PNG with all scenarios side by side."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError:
        return

    n_scenarios = len(all_results)
    cols = 3
    rows = math.ceil(n_scenarios / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows), dpi=100)
    if rows == 1:
        axes = [axes]
    axes_flat = [ax for row in axes for ax in (row if hasattr(row, '__iter__') else [row])]

    for idx, (scenario, log, outcome) in enumerate(all_results):
        ax = axes_flat[idx]

        f_ns = [(r['f_lat'] - HOME_LAT) * 111320.0 for r in log]
        f_es = [(r['f_lon'] - HOME_LON) * 111320.0 * math.cos(math.radians(HOME_LAT))
                for r in log]
        l_ns = [(r['l_lat'] - HOME_LAT) * 111320.0 for r in log]
        l_es = [(r['l_lon'] - HOME_LON) * 111320.0 * math.cos(math.radians(HOME_LAT))
                for r in log]

        circle = Circle((0, 0), 200.0, fill=False, edgecolor='red',
                         linestyle='--', linewidth=1, alpha=0.4)
        ax.add_patch(circle)
        ax.plot(0, 0, 'k^', markersize=8)

        ax.plot(l_es, l_ns, 'b-', alpha=0.4, linewidth=1, label='Leader')
        ax.plot(f_es, f_ns, 'g-', alpha=0.6, linewidth=1.5, label='Follower')

        # Mark failure point
        for i, r in enumerate(log):
            if not r['fs_safe']:
                ax.plot(f_es[i], f_ns[i], 'rx', markersize=12, markeredgewidth=2)
                break

        # Mark start/end
        ax.plot(f_es[0], f_ns[0], 'go', markersize=6)
        ax.plot(f_es[-1], f_ns[-1], 'ro', markersize=6)

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'{scenario}\n{outcome}', fontsize=9, fontweight='bold')
        ax.legend(fontsize=7, loc='lower left')
        ax.set_xlabel('East (m)', fontsize=8)
        ax.set_ylabel('North (m)', fontsize=8)

    # Hide unused axes
    for idx in range(len(all_results), len(axes_flat)):
        axes_flat[idx].set_visible(False)

    fig.suptitle('Failsafe Test Suite — All 6 Failure Modes + Nominal',
                 fontsize=14, fontweight='bold')
    fig.tight_layout()
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

SCENARIOS = [
    ("NOMINAL",             500),
    ("GPS_LOSS",            200),
    ("LEADER_STALE",        200),
    ("LEADER_GEOFENCE",     500),
    ("FOLLOWER_GEOFENCE",   100),
    ("ALTITUDE_CEILING",    150),
    ("CATCHUP_TIMEOUT",     500),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    all_results = []

    print("=" * 70)
    print("  FAILSAFE TEST SUITE — ALL FAILURE MODES")
    print(f"  {len(SCENARIOS)} scenarios | Output → {OUTPUT_DIR}")
    print("=" * 70)

    for scenario, max_ticks in SCENARIOS:
        print(f"\n{'─'*60}")
        print(f"  Scenario: {scenario}  (max {max_ticks} ticks)")
        print(f"{'─'*60}")

        t0 = time.time()
        log, outcome = run_scenario(scenario, max_ticks)
        sim_time = time.time() - t0

        print(f"  Result  : {outcome}")
        print(f"  Ticks   : {len(log)}")
        print(f"  Sim time: {sim_time:.2f}s")

        # Summary of flags seen
        all_flags = set()
        for r in log:
            if r['fs_flags']:
                all_flags.update(r['fs_flags'].split(','))
            if r['g_flags']:
                all_flags.update(r['g_flags'].split(','))
        if all_flags:
            print(f"  Flags   : {', '.join(sorted(all_flags))}")

        # Final state
        last = log[-1]
        print(f"  Final   : safe={last['fs_safe']} action={last['fs_action']} "
              f"mode={last['g_mode']} home_dist={last['home_dist']:.0f}m "
              f"alt={last['f_alt']:.1f}m")

        # Write CSV
        csv_name = f"failsafe_test_{scenario.lower()}_{ts}.csv"
        csv_path = os.path.join(OUTPUT_DIR, csv_name)
        write_csv(log, csv_path)
        print(f"  CSV     : {csv_name}")

        # Write GIF
        gif_name = f"failsafe_test_{scenario.lower()}_{ts}.gif"
        gif_path = os.path.join(OUTPUT_DIR, gif_name)
        make_gif(log, scenario, outcome, gif_path)
        print(f"  GIF     : {gif_name}")

        all_results.append((scenario, log, outcome))

    # Summary plot
    summary_path = os.path.join(OUTPUT_DIR, f"failsafe_summary_{ts}.png")
    make_summary(all_results, summary_path)
    print(f"\n{'='*70}")
    print(f"  SUMMARY : {summary_path}")
    print(f"{'='*70}")

    # Final report
    print(f"\n  {'SCENARIO':<25} {'TICKS':<8} {'OUTCOME'}")
    print(f"  {'─'*25} {'─'*8} {'─'*40}")
    for scenario, log, outcome in all_results:
        print(f"  {scenario:<25} {len(log):<8} {outcome}")

    print(f"\n  All outputs in: {OUTPUT_DIR}/")
    print("  Done!")


if __name__ == "__main__":
    main()
