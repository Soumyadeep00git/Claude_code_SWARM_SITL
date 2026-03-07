#!/usr/bin/env python3
"""
Standalone leader-follower analysis GIF.  No guidance_lib imports.

Layout (2 rows x 3 cols):
  Row 1:  Leader+Follower(real) | Leader+Ghost OL | Leader+Ghost CL
          (distance shown live in legend)
  Row 2:  Leader vel & accel    | Follower vel & accel | CL ghost vel & accel

Guidance params (from config.yaml, pasted here for portability):
  kp=0.7  kd=0.4  ff_gain=0.8  max_speed=2.0  deadzone=0.5
  catchup_dist=8.0  max_catchup_speed=2.0  catchup_decel=2.5
  offset: north=-2m  east=0m
"""

import math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# ═══════════════════════════════════════════════════════════════════════════
#  GUIDANCE PARAMS  (copy-pasted from guidance_lib/config.yaml)
# ═══════════════════════════════════════════════════════════════════════════
KP              = 0.7
KD              = 0.4
FF_GAIN         = 0.8
MAX_SPEED       = 2.0
DEADZONE_M      = 0.5
CATCHUP_DIST_M  = 8.0
MAX_CATCHUP_SPD = 2.0
CATCHUP_DECEL   = 2.5
OFFSET_N        = -2.0
OFFSET_E        =  0.0

# ═══════════════════════════════════════════════════════════════════════════
#  GUIDANCE FUNCTIONS  (copy-pasted from guidance_lib/velocity.py)
# ═══════════════════════════════════════════════════════════════════════════
_EPS = 1e-6

def _mag(n, e):
    return math.sqrt(n * n + e * e + _EPS * _EPS)

def guidance_tracking(err_n, err_e, err_mag, my_vn, my_ve, peer_vn, peer_ve):
    if err_mag <= DEADZONE_M:
        return 0.0, 0.0
    vn = KP * err_n - KD * (my_vn - peer_vn) + FF_GAIN * peer_vn
    ve = KP * err_e - KD * (my_ve - peer_ve) + FF_GAIN * peer_ve
    speed = _mag(vn, ve)
    if speed > MAX_SPEED:
        vn *= MAX_SPEED / speed
        ve *= MAX_SPEED / speed
    return vn, ve

def guidance_catchup(err_n, err_e, err_mag, peer_vn, peer_ve):
    v_kin = math.sqrt(2.0 * CATCHUP_DECEL * err_mag)
    speed_limit = min(v_kin, MAX_CATCHUP_SPD)
    u_n = err_n / err_mag
    u_e = err_e / err_mag
    vn = speed_limit * u_n + FF_GAIN * peer_vn
    ve = speed_limit * u_e + FF_GAIN * peer_ve
    speed = _mag(vn, ve)
    if speed > speed_limit:
        vn *= speed_limit / speed
        ve *= speed_limit / speed
    return vn, ve

def guidance_step(err_n, err_e, my_vn, my_ve, peer_vn, peer_ve):
    err_mag = _mag(err_n, err_e)
    if err_mag > CATCHUP_DIST_M:
        return guidance_catchup(err_n, err_e, err_mag, peer_vn, peer_ve)
    return guidance_tracking(err_n, err_e, err_mag, my_vn, my_ve, peer_vn, peer_ve)

# ═══════════════════════════════════════════════════════════════════════════
#  LOAD DATA
# ═══════════════════════════════════════════════════════════════════════════
leader  = pd.read_csv("/mnt/d/LAT_D/Orin_working/leader_events_20260306_205751.csv")
follower = pd.read_csv("/mnt/d/LAT_D/Orin_working/follower_events_20260306_205753.csv")

def ll2m(lat, lon, rlat, rlon):
    n = (lat - rlat) * 111320.0
    e = (lon - rlon) * 111320.0 * np.cos(np.radians(rlat))
    return n, e

lg = leader[leader["event"] == "GPS"].sort_values("local_ts").reset_index(drop=True)
RL, RN = lg["lat"].iloc[0], lg["lon"].iloc[0]

# Leader position & velocity
ln, le = ll2m(lg["lat"].values, lg["lon"].values, RL, RN)
lt = lg["local_ts"].values
lv = leader[leader["event"] == "VEL"][["local_ts", "vn", "ve"]].sort_values("local_ts").reset_index(drop=True)

# Follower real position & velocity
fg = follower[follower["event"] == "GPS"].sort_values("local_ts").reset_index(drop=True)
fn, fe = ll2m(fg["lat"].values, fg["lon"].values, RL, RN)
ft = fg["local_ts"].values
fv = follower[follower["event"] == "VEL"][["local_ts", "vn", "ve"]].sort_values("local_ts").reset_index(drop=True)

# Follower logged CMD
fc = follower[follower["event"] == "CMD"][["local_ts", "cmd_vn", "cmd_ve"]].sort_values("local_ts").reset_index(drop=True)

# ═══════════════════════════════════════════════════════════════════════════
#  GHOST OPEN-LOOP
# ═══════════════════════════════════════════════════════════════════════════
mask = fg["local_ts"] <= fc["local_ts"].iloc[0]
sr = fg[mask].iloc[-1] if mask.any() else fg.iloc[0]
sn, se = ll2m(sr["lat"], sr["lon"], RL, RN)

gn_ol, ge_ol, gt_ol = [sn], [se], [fc["local_ts"].iloc[0]]
for i in range(1, len(fc)):
    dt = fc["local_ts"].iloc[i] - fc["local_ts"].iloc[i - 1]
    if dt <= 0 or dt > 2.0:
        dt = 0.0
    gn_ol.append(gn_ol[-1] + fc["cmd_vn"].iloc[i - 1] * dt)
    ge_ol.append(ge_ol[-1] + fc["cmd_ve"].iloc[i - 1] * dt)
    gt_ol.append(fc["local_ts"].iloc[i])
gn_ol, ge_ol, gt_ol = np.array(gn_ol), np.array(ge_ol), np.array(gt_ol)

# ═══════════════════════════════════════════════════════════════════════════
#  GHOST CLOSED-LOOP
# ═══════════════════════════════════════════════════════════════════════════
ldr_n_at_cmd = np.interp(fc["local_ts"].values, lt, ln)
ldr_e_at_cmd = np.interp(fc["local_ts"].values, lt, le)
ldr_vn_at_cmd = np.interp(fc["local_ts"].values, lv["local_ts"].values, lv["vn"].values)
ldr_ve_at_cmd = np.interp(fc["local_ts"].values, lv["local_ts"].values, lv["ve"].values)

gn_cl, ge_cl, gt_cl = [sn], [se], [fc["local_ts"].iloc[0]]
gcl_vn, gcl_ve = [0.0], [0.0]
gcl_cmd_vn, gcl_cmd_ve = [0.0], [0.0]

for i in range(1, len(fc)):
    dt = fc["local_ts"].iloc[i] - fc["local_ts"].iloc[i - 1]
    if dt <= 0 or dt > 2.0:
        dt = 0.0
    goal_n = ldr_n_at_cmd[i - 1] + OFFSET_N
    goal_e = ldr_e_at_cmd[i - 1] + OFFSET_E
    err_n = goal_n - gn_cl[-1]
    err_e = goal_e - ge_cl[-1]
    cmd_vn, cmd_ve = guidance_step(
        err_n, err_e, gcl_vn[-1], gcl_ve[-1],
        ldr_vn_at_cmd[i - 1], ldr_ve_at_cmd[i - 1])
    gn_cl.append(gn_cl[-1] + cmd_vn * dt)
    ge_cl.append(ge_cl[-1] + cmd_ve * dt)
    gt_cl.append(fc["local_ts"].iloc[i])
    gcl_vn.append(cmd_vn); gcl_ve.append(cmd_ve)
    gcl_cmd_vn.append(cmd_vn); gcl_cmd_ve.append(cmd_ve)

gn_cl = np.array(gn_cl); ge_cl = np.array(ge_cl); gt_cl = np.array(gt_cl)
gcl_cmd_vn = np.array(gcl_cmd_vn); gcl_cmd_ve = np.array(gcl_cmd_ve)

# ═══════════════════════════════════════════════════════════════════════════
#  TIME ALIGNMENT & INTERPOLATION
# ═══════════════════════════════════════════════════════════════════════════
t0 = min(lt[0], ft[0], gt_ol[0], gt_cl[0],
         lv["local_ts"].iloc[0], fv["local_ts"].iloc[0], fc["local_ts"].iloc[0])
lt    = lt - t0;    ft    = ft - t0
gt_ol = gt_ol - t0; gt_cl = gt_cl - t0
lv_t  = lv["local_ts"].values - t0
fv_t  = fv["local_ts"].values - t0
fc_t  = fc["local_ts"].values - t0
t_max = max(lt[-1], ft[-1], gt_ol[-1], gt_cl[-1])

fps = 20
n_frames = int(t_max * fps)
tg = np.linspace(0, t_max, n_frames)
dt_grid = tg[1] - tg[0]

# Positions
iln    = np.interp(tg, lt, ln);       ile    = np.interp(tg, lt, le)
ifn    = np.interp(tg, ft, fn);       ife    = np.interp(tg, ft, fe)
ign_ol = np.interp(tg, gt_ol, gn_ol); ige_ol = np.interp(tg, gt_ol, ge_ol)
ign_cl = np.interp(tg, gt_cl, gn_cl); ige_cl = np.interp(tg, gt_cl, ge_cl)

# Velocities
i_lv_vn = np.interp(tg, lv_t, lv["vn"].values)
i_lv_ve = np.interp(tg, lv_t, lv["ve"].values)
i_fv_vn = np.interp(tg, fv_t, fv["vn"].values)
i_fv_ve = np.interp(tg, fv_t, fv["ve"].values)
i_gcl_vn = np.interp(tg, gt_cl, gcl_cmd_vn)
i_gcl_ve = np.interp(tg, gt_cl, gcl_cmd_ve)

# Speed (magnitude)
i_lv_spd  = np.hypot(i_lv_vn, i_lv_ve)
i_fv_spd  = np.hypot(i_fv_vn, i_fv_ve)
i_gcl_spd = np.hypot(i_gcl_vn, i_gcl_ve)

# Acceleration (d(speed)/dt via finite difference)
i_lv_acc  = np.gradient(i_lv_spd, dt_grid)
i_fv_acc  = np.gradient(i_fv_spd, dt_grid)
i_gcl_acc = np.gradient(i_gcl_spd, dt_grid)

# Distances to leader
dist_fol = np.hypot(ife - ile, ifn - iln)
dist_ol  = np.hypot(ige_ol - ile, ign_ol - iln)
dist_cl  = np.hypot(ige_cl - ile, ign_cl - iln)

# ═══════════════════════════════════════════════════════════════════════════
#  AXIS LIMITS
# ═══════════════════════════════════════════════════════════════════════════
pad = 5

def square_lims(*arrays_e_n):
    ae = np.concatenate(arrays_e_n[0::2])
    an = np.concatenate(arrays_e_n[1::2])
    emin, emax = ae.min() - pad, ae.max() + pad
    nmin, nmax = an.min() - pad, an.max() + pad
    span = max(emax - emin, nmax - nmin)
    ec, nc = (emin + emax) / 2, (nmin + nmax) / 2
    return ec - span/2, ec + span/2, nc - span/2, nc + span/2

e1min, e1max, n1min, n1max = square_lims(ile, iln, ife, ifn)
e2min, e2max, n2min, n2max = square_lims(ile, iln, ige_ol, ign_ol)
e3min, e3max, n3min, n3max = square_lims(ile, iln, ige_cl, ign_cl)

# Vel/accel y-limits (shared across bottom row for comparison)
all_spd = np.concatenate([i_lv_spd, i_fv_spd, i_gcl_spd])
all_acc = np.concatenate([i_lv_acc, i_fv_acc, i_gcl_acc])
spd_max = all_spd.max() + 0.3
acc_lo, acc_hi = all_acc.min() - 0.3, all_acc.max() + 0.3

# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE  (2 rows x 3 cols)
# ═══════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(2, 3, figsize=(20, 12))
(ax1, ax2, ax3), (ax4, ax5, ax6) = axes
fig.suptitle("Leader-Follower Analysis", fontsize=15, fontweight="bold")

# ── Row 1: trajectory plots with live distance in legend ─────────────────

# Top-left: leader + real follower
ax1.set_xlim(e1min, e1max); ax1.set_ylim(n1min, n1max)
ax1.set_aspect("equal"); ax1.set_xlabel("East (m)"); ax1.set_ylabel("North (m)")
ax1.set_title("Leader + Follower (real)"); ax1.grid(True, alpha=0.3)
tr1_l, = ax1.plot([], [], "-",  color="blue",  lw=2, label="Leader")
tr1_f, = ax1.plot([], [], "-",  color="green", lw=2, label="Follower")
mk1_l, = ax1.plot([], [], "o",  color="blue",  ms=10, mec="k", zorder=5)
mk1_f, = ax1.plot([], [], "s",  color="green", ms=10, mec="k", zorder=5)
leg1 = ax1.legend(loc="upper right", fontsize=8)

# Top-mid: leader + ghost OL
ax2.set_xlim(e2min, e2max); ax2.set_ylim(n2min, n2max)
ax2.set_aspect("equal"); ax2.set_xlabel("East (m)"); ax2.set_ylabel("North (m)")
ax2.set_title("Leader + Ghost OPEN-loop"); ax2.grid(True, alpha=0.3)
tr2_l, = ax2.plot([], [], "-",  color="blue", lw=2, label="Leader")
tr2_g, = ax2.plot([], [], "--", color="red",  lw=2, label="Ghost OL")
mk2_l, = ax2.plot([], [], "o",  color="blue", ms=10, mec="k", zorder=5)
mk2_g, = ax2.plot([], [], "D",  color="red",  ms=9,  mec="k", zorder=5)
leg2 = ax2.legend(loc="upper right", fontsize=8)

# Top-right: leader + ghost CL
ax3.set_xlim(e3min, e3max); ax3.set_ylim(n3min, n3max)
ax3.set_aspect("equal"); ax3.set_xlabel("East (m)"); ax3.set_ylabel("North (m)")
ax3.set_title("Leader + Ghost CLOSED-loop"); ax3.grid(True, alpha=0.3)
tr3_l, = ax3.plot([], [], "-",  color="blue",   lw=2, label="Leader")
tr3_g, = ax3.plot([], [], "--", color="orange", lw=2, label="Ghost CL")
mk3_l, = ax3.plot([], [], "o",  color="blue",   ms=10, mec="k", zorder=5)
mk3_g, = ax3.plot([], [], "D",  color="orange", ms=9,  mec="k", zorder=5)
leg3 = ax3.legend(loc="upper right", fontsize=8)

# ── Row 2: vel & accel (twin y-axis for accel) ──────────────────────────

def setup_vel_accel_ax(ax, title, spd_color, acc_color):
    ax.set_xlim(0, t_max); ax.set_ylim(-0.1, spd_max)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed (m/s)", color=spd_color)
    ax.set_title(title); ax.grid(True, alpha=0.3)
    ax.tick_params(axis="y", labelcolor=spd_color)
    ax2 = ax.twinx()
    ax2.set_ylim(acc_lo, acc_hi)
    ax2.set_ylabel("Accel (m/s²)", color=acc_color)
    ax2.tick_params(axis="y", labelcolor=acc_color)
    return ax2

# Leader vel & accel
ax4t = setup_vel_accel_ax(ax4, "Leader vel & accel", "blue", "royalblue")
l_spd, = ax4.plot([], [], "-",  color="blue",      lw=1.5, label="speed")
l_acc, = ax4t.plot([], [], "-", color="royalblue",  lw=1.0, alpha=0.7, label="accel")
ax4.legend([l_spd, l_acc], ["speed", "accel"], loc="upper right", fontsize=8)

# Follower vel & accel
ax5t = setup_vel_accel_ax(ax5, "Follower vel & accel", "green", "darkgreen")
f_spd, = ax5.plot([], [], "-",  color="green",     lw=1.5, label="speed")
f_acc, = ax5t.plot([], [], "-", color="darkgreen",  lw=1.0, alpha=0.7, label="accel")
ax5.legend([f_spd, f_acc], ["speed", "accel"], loc="upper right", fontsize=8)

# Ghost CL vel & accel
ax6t = setup_vel_accel_ax(ax6, "Ghost CL vel & accel", "orange", "sienna")
g_spd, = ax6.plot([], [], "-",  color="orange",    lw=1.5, label="speed")
g_acc, = ax6t.plot([], [], "-", color="sienna",     lw=1.0, alpha=0.7, label="accel")
ax6.legend([g_spd, g_acc], ["speed", "accel"], loc="upper right", fontsize=8)

time_txt = fig.text(0.5, 0.005, "", ha="center", fontsize=12, fontfamily="monospace")
fig.tight_layout(rect=[0, 0.02, 1, 0.96])

# ═══════════════════════════════════════════════════════════════════════════
#  ANIMATION
# ═══════════════════════════════════════════════════════════════════════════
skip = max(1, n_frames // 400)
fidx = list(range(0, n_frames, skip))

# blit=False because we update legend text each frame
def init():
    for a in [tr1_l, tr1_f, mk1_l, mk1_f,
              tr2_l, tr2_g, mk2_l, mk2_g,
              tr3_l, tr3_g, mk3_l, mk3_g,
              l_spd, l_acc, f_spd, f_acc, g_spd, g_acc]:
        a.set_data([], [])
    time_txt.set_text("")

def update(f):
    i = fidx[f]

    # Row 1: trajectories
    tr1_l.set_data(ile[:i+1], iln[:i+1])
    tr1_f.set_data(ife[:i+1], ifn[:i+1])
    mk1_l.set_data([ile[i]], [iln[i]])
    mk1_f.set_data([ife[i]], [ifn[i]])

    tr2_l.set_data(ile[:i+1], iln[:i+1])
    tr2_g.set_data(ige_ol[:i+1], ign_ol[:i+1])
    mk2_l.set_data([ile[i]], [iln[i]])
    mk2_g.set_data([ige_ol[i]], [ign_ol[i]])

    tr3_l.set_data(ile[:i+1], iln[:i+1])
    tr3_g.set_data(ige_cl[:i+1], ign_cl[:i+1])
    mk3_l.set_data([ile[i]], [iln[i]])
    mk3_g.set_data([ige_cl[i]], [ign_cl[i]])

    # Update legend text with live distance
    leg1.get_texts()[0].set_text(f"Leader")
    leg1.get_texts()[1].set_text(f"Follower  d={dist_fol[i]:.1f}m")

    leg2.get_texts()[0].set_text(f"Leader")
    leg2.get_texts()[1].set_text(f"Ghost OL  d={dist_ol[i]:.1f}m")

    leg3.get_texts()[0].set_text(f"Leader")
    leg3.get_texts()[1].set_text(f"Ghost CL  d={dist_cl[i]:.1f}m")

    # Row 2: vel & accel
    l_spd.set_data(tg[:i+1], i_lv_spd[:i+1])
    l_acc.set_data(tg[:i+1], i_lv_acc[:i+1])

    f_spd.set_data(tg[:i+1], i_fv_spd[:i+1])
    f_acc.set_data(tg[:i+1], i_fv_acc[:i+1])

    g_spd.set_data(tg[:i+1], i_gcl_spd[:i+1])
    g_acc.set_data(tg[:i+1], i_gcl_acc[:i+1])

    # Time
    time_txt.set_text(f"t = {tg[i]:.1f} s")

print(f"Rendering {len(fidx)} frames ...")
anim = FuncAnimation(fig, update, init_func=init,
                     frames=len(fidx), interval=50, blit=False)
anim.save("/mnt/d/LAT_D/Orin_working/leader_follower_ghost.gif",
          writer=PillowWriter(fps=20))
plt.close()
print("Done -> leader_follower_ghost.gif")
