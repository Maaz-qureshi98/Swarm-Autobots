"""Figures and LaTeX tables from results/*.json (all numbers in the paper come from here)."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
    "font.size": 7, "axes.labelsize": 7, "axes.titlesize": 7, "legend.fontsize": 6.5,
    "xtick.labelsize": 6.5, "ytick.labelsize": 6.5, "lines.linewidth": 1.1, "axes.linewidth": 0.5,
    "xtick.major.width": 0.5, "ytick.major.width": 0.5, "xtick.major.size": 2.5, "ytick.major.size": 2.5,
    "xtick.direction": "in", "ytick.direction": "in", "axes.spines.top": False, "axes.spines.right": False,
    "axes.titlepad": 3, "axes.labelpad": 2, "legend.handlelength": 1.8, "legend.columnspacing": 1.2,
    "pdf.fonttype": 42, "savefig.bbox": "tight", "savefig.pad_inches": 0.015,
})
COL = 3.45   # IEEE column width [in]; figures are drawn at their printed size
TW = 7.0     # usable text width for figure* [in]
R = "results/"
F = "figs/"
os.makedirs(F, exist_ok=True)
C = {"asa": "#B23A48", "lpsi_zoh": "#E39B2D", "lpsi_pred": "#2F6690", "pred_nocbf": "#7A7A7A", "pred_stop": "#6A994E", "pred_nobyp": "#9B7EBD", "consensus": "#5B8E7D"}
LBL = {"asa": "ASA (original)", "lpsi_zoh": "Point + ZOH", "lpsi_pred": "Proposed",
       "pred_nocbf": "No safety layer", "pred_stop": "20 cm stop rule", "pred_nobyp": "Speed filter only", "consensus": "Consensus"}
ROB = ["#1b1b1b", "#2F6690", "#E39B2D", "#6A994E"]


def load(n):
    return json.load(open(R + n + ".json"))


def arr(x):
    return np.array([np.nan if v is None else v for v in x], float)


def ci95(x):
    x = x[np.isfinite(x)]
    if len(x) < 2:
        return np.nan
    return 1.96 * x.std(ddof=1) / np.sqrt(len(x))


stats = {}

# ------------------------------------------------------------------ E1 loss sweep
E7 = []
for part in ["a", "b"]:
    if os.path.exists(R + f"e7_consensus_{part}.json"):
        E7 += load(f"e7_consensus_{part}")
d = load("e1_loss") + [r for r in E7 if r["exp"] == "e1"]
E1D = d
for r in d:
    stats[f"e1_{r['form']}_{r['p']}_{r['method']}"] = dict(
        est_med=float(np.median(arr(r["rmse_est"]))), est_mean=float(np.nanmean(arr(r["rmse_est"]))),
        gt_med=float(np.median(arr(r["rmse_gt"]))), gt_mean=float(np.nanmean(arr(r["rmse_gt"]))),
        succ=float(np.mean(arr(r["success"]))), coll=float(np.mean(arr(r["collisions"]))),
        reach=float(np.mean(arr(r["reached"]))), false_el=float(np.mean(arr(r["false_elections"]))),
        final_med=float(np.nanmedian(arr(r["final_gt"]))), interv=float(np.mean(arr(r["interv"]))),
        T=float(np.nanmean(arr(r["goal_time"]))))

# ------------------------------------------------------------------ E1b burst
d = load("e1b_burst")
for r in d:
    stats[f"e1b_{r['form']}_{r['burst']}_{r['method']}"] = dict(
        est_med=float(np.median(arr(r["rmse_est"]))), est_p90=float(np.percentile(arr(r["rmse_est"]), 90)),
        succ=float(np.mean(arr(r["success"]))))

# ------------------------------------------------------------------ E2 safety
# collision studies use the 0.142 m circumscribed footprint (E13); the 0.12 m runs are kept as e2r012_*
for r in load("e2_safety") + [r for r in E7 if r["exp"] == "e2"]:
    stats[f"e2r012_{r['form']}_{r['n_unknown']}_{r['method']}"] = dict(
        coll=float(np.mean(arr(r["collisions"]))), free=float(np.mean(arr(r["collisions"]) == 0)),
        succ=float(np.mean(arr(r["success"]))))
d = load("e13_footprint")["e2"]
for r in d:
    stats[f"e2_{r['form']}_{r['n_unknown']}_{r['method']}"] = dict(
        coll=float(np.mean(arr(r["collisions"]))), coll_ci=float(ci95(arr(r["collisions"]))),
        free=float(np.mean(arr(r["collisions"]) == 0)),
        minsep=float(np.nanmedian(arr(r["min_sep"]))), succ=float(np.mean(arr(r["success"]))),
        reach=float(np.mean(arr(r["reached"]))), est_med=float(np.median(arr(r["rmse_est"]))),
        interv=float(np.mean(arr(r["interv"]))))
_rc5 = plt.rcParams.copy()   # larger text for the safety figure
plt.rcParams.update({"font.size": 7.7, "axes.labelsize": 7.7, "legend.fontsize": 7.2,
                     "xtick.labelsize": 7.2, "ytick.labelsize": 7.2})
fig, axs = plt.subplots(2, 1, figsize=(COL, 2.35), sharex=True)
ms = ["asa", "pred_nocbf", "pred_stop", "pred_nobyp", "lpsi_pred"]
w = 0.155
for k, m in enumerate(ms):
    xs = np.arange(4) + (k - 2) * w
    # missions with at least one contact: a stalled robot can log many brief contacts in one mission
    yc = [100 * (1 - np.mean([stats[f"e2_{f}_{nu}_{m}"]["free"] for f in ["triangle", "Y"]])) for nu in range(4)]
    ys = [100 * np.mean([stats[f"e2_{f}_{nu}_{m}"]["succ"] for f in ["triangle", "Y"]]) for nu in range(4)]
    axs[0].bar(xs, yc, w * 0.92, color=C[m], label=LBL[m], lw=0)
    axs[1].bar(xs, ys, w * 0.92, color=C[m], lw=0)
    for x, v in zip(xs, yc):          # label empty or near-empty bars so they are not read as missing data
        if v < 3:
            axs[0].text(x, v + 2, f"{v:.0f}", ha="center", va="bottom", fontsize=6.2, color=C[m])
    for x, v in zip(xs, ys):
        if v < 3:
            axs[1].text(x, v + 2, f"{v:.0f}", ha="center", va="bottom", fontsize=6.2, color=C[m])
for ax in axs:
    ax.set_xticks(range(4)); ax.grid(axis="y", alpha=0.3, lw=0.4); ax.set_axisbelow(True)
    ax.tick_params(axis="x", length=0)
axs[1].set_xlabel("Unmapped obstacles near the path")
axs[0].set_ylabel("Missions with\ncontact [%]"); axs[0].set_ylim(0, 100)
axs[1].set_ylabel("Pattern held [%]"); axs[1].set_ylim(0, 100)
axs[0].text(0.01, 0.97, "(a)", transform=axs[0].transAxes, va="top", fontweight="bold")
axs[1].text(0.01, 0.97, "(b)", transform=axs[1].transAxes, va="top", fontweight="bold")
h, l = axs[0].get_legend_handles_labels()
fig.subplots_adjust(hspace=0.12, top=0.84)
fig.legend(h, l, loc="lower center", ncol=3, bbox_to_anchor=(0.54, 0.845), frameon=False, handlelength=1.0,
           columnspacing=0.9, handletextpad=0.4)
fig.align_ylabels(axs)
fig.savefig(F + "fig_safety.pdf"); plt.close(fig)
plt.rcParams.update(_rc5)

# ------------------------------------------------------------------ E3 election
d = load("e3_election")
for r in d:
    rt = arr(r["recover_t"])
    stats[f"e3_{r['form']}_{r['p']}_{r['election']}"] = dict(
        reach=float(np.mean(arr(r["reached"]))), succ=float(np.mean(arr(r["success"]))),
        rec_med=float(np.nanmedian(rt)) if np.isfinite(rt).any() else None,
        rec_p90=float(np.nanpercentile(rt, 90)) if np.isfinite(rt).any() else None,
        rec_frac=float(np.mean(np.isfinite(rt))), coll=float(np.mean(arr(r["collisions"]))),
        conflicts=float(np.mean(arr(r["false_elections"]))))
ex = load("e3_example")
t = np.array(ex["t"]); eg = np.array(ex["e_gt"]); fol = np.array(ex["fol"]).astype(bool)
isl = np.array(ex["is_leader"]).astype(bool); alive = np.array(ex["alive"]).astype(bool)
NAMES = ["Alpha", "Beta-1", "Beta-2", "Beta-3"]
fig, axs = plt.subplots(2, 1, figsize=(COL, 1.6), sharex=True, gridspec_kw=dict(height_ratios=[2.1, 1]))
LD = np.array(ex["leader"])
agree = np.all(LD[:, 1:] == 1, axis=1)            # all survivors follow Beta-1
t_new = t[(t > 12.0) & agree][0]
gap = (t >= 12.0) & (t < t_new)                    # no agreed leader: slot error undefined
for i in range(1, 4):
    y = np.where(fol[:, i] & ~gap, eg[:, i], np.nan)
    axs[0].plot(t, y, color=ROB[i], lw=0.9, label=NAMES[i])
for ax in axs:
    ax.axvspan(12.0, t_new, color="0.85", lw=0)
    ax.axvline(12.0, color="k", ls=":", lw=0.7)
    ax.grid(alpha=0.3, lw=0.4); ax.set_axisbelow(True)
axs[0].text(t_new + 0.4, 0.55, "Alpha removed; election", fontsize=6, va="top", ha="left")
axs[0].set_ylabel("GT slot error [m]")
axs[0].set_ylim(0, 0.6); axs[0].set_yticks([0, 0.2, 0.4, 0.6])
stats["e3_example_newleader_t"] = float(t_new)
axs[0].legend(frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0), handlelength=1.4)
for i in range(4):
    axs[1].fill_between(t, i - 0.32, i + 0.32, where=isl[:, i] & alive[:, i], color=ROB[i], step="post", lw=0)
axs[1].set_yticks(range(4)); axs[1].set_yticklabels(NAMES, fontsize=6)
axs[1].set_ylim(-0.6, 3.6); axs[1].set_ylabel("Leader")
axs[1].set_xlabel("Time [s]")
axs[0].set_xlim(0, t[-1])
fig.align_ylabels(axs)
fig.subplots_adjust(hspace=0.15)
fig.savefig(F + "fig_election.pdf"); plt.close(fig)
stats["e3_example_recover"] = ex["recover_t"]

# ------------------------------------------------------------------ E4 scale
d = load("e4_scale") + [r for r in E7 if r["exp"] == "e4"]
for form in ["triangle", "Y"]:
    for m in ["asa", "consensus", "lpsi_pred"]:
        for r in [r for r in d if r["form"] == form and r["method"] == m]:
            stats[f"e4_{form}_{r['N']}_{m}"] = dict(est_med=float(np.median(arr(r["rmse_est"]))),
                                                     gt_med=float(np.median(arr(r["rmse_gt"]))),
                                                     succ=float(np.mean(arr(r["success"]))),
                                                     coll=float(np.mean(arr(r["collisions"]))),
                                                     reach=float(np.mean(arr(r["reached"]))))
E4D = d

# ------------------------------------------------------------------ Fig: packet loss + scalability (one row)
from matplotlib.lines import Line2D
_rc = plt.rcParams.copy()   # larger text for this figure* (read at full page width)
plt.rcParams.update({"font.size": 8.8, "axes.labelsize": 8.8, "axes.titlesize": 8.8, "legend.fontsize": 9.4,
                     "xtick.labelsize": 8.8, "ytick.labelsize": 8.8})
fig, axs = plt.subplots(1, 4, figsize=(TW, 1.85))
MS = ["asa", "lpsi_zoh", "consensus", "lpsi_pred"]
for form, ls, mk in [("triangle", "-", "o"), ("Y", "--", "s")]:
    for m in MS:
        rows = [r for r in E1D if r["form"] == form and r["method"] == m]
        ps = 100 * np.array([r["p"] for r in rows])
        axs[0].plot(ps, [np.median(arr(r["rmse_gt"])) for r in rows], ls, color=C[m], marker=mk, ms=2.2, lw=0.9)
        axs[1].plot(ps, [100 * np.mean(arr(r["success"])) for r in rows], ls, color=C[m], marker=mk, ms=2.2, lw=0.9)
        if form == "triangle" and m == "lpsi_pred":
            axs[0].fill_between(ps, [np.percentile(arr(r["rmse_gt"]), 25) for r in rows],
                                [np.percentile(arr(r["rmse_gt"]), 75) for r in rows], color=C[m], alpha=0.13, lw=0)
    for m in ["asa", "consensus", "lpsi_pred"]:
        rows = [r for r in E4D if r["form"] == form and r["method"] == m]
        Ns = [r["N"] for r in rows]
        axs[2].plot(Ns, [np.median(arr(r["rmse_gt"])) for r in rows], ls, color=C[m], marker=mk, ms=2.2, lw=0.9)
        axs[3].plot(Ns, [100 * np.mean(arr(r["reached"])) for r in rows], ls, color=C[m], marker=mk, ms=2.2, lw=0.9)
for ax in axs[:2]:
    ax.set_xlabel("Mean packet loss [%]"); ax.set_xticks([0, 20, 40, 60])
for ax in axs[2:]:
    ax.set_xlabel("Swarm size $N$"); ax.set_xticks([4, 8, 12, 16, 20])
axs[0].set_ylabel("GT slot RMSE [m]"); axs[0].set_ylim(0.1, 0.5)
axs[1].set_ylabel("Pattern held [%]"); axs[1].set_ylim(0, 100)
axs[2].set_ylabel("GT slot RMSE [m]"); axs[2].set_ylim(0.1, 0.5)
axs[3].set_ylabel("Completed [%]"); axs[3].set_ylim(0, 105)
for k, ax in enumerate(axs):
    ax.grid(alpha=0.3, lw=0.4); ax.set_axisbelow(True)
    ax.set_title(["(a) Slot error vs. loss", "(b) Pattern held vs. loss", "(c) Slot error vs. size",
                  "(d) Completion vs. size"][k])
hm = [Line2D([], [], color=C[m], lw=1.4, label=LBL[m]) for m in MS]
hf = [Line2D([], [], color="0.3", ls="-", marker="o", ms=2.2, lw=0.9, label="triangle"),
      Line2D([], [], color="0.3", ls="--", marker="s", ms=2.2, lw=0.9, label="Y")]
fig.legend(handles=hm + hf, loc="lower center", ncol=6, bbox_to_anchor=(0.5, 0.99), frameon=False)
fig.subplots_adjust(wspace=0.42, left=0.06, right=0.995)
fig.savefig(F + "fig_loss.pdf"); plt.close(fig)
plt.rcParams.update(_rc)

# ------------------------------------------------------------------ E5 drift + trajectories
d = load("e5_drift")
fig, ax = plt.subplots(figsize=(COL, 1.5))
for form, ls in [("triangle", "-"), ("Y", "--")]:
    b = np.array(d[form]["bins"]); xc = 0.5 * (b[1:] + b[:-1])
    med = arr(d[form]["med"]); q1 = arr(d[form]["q25"]); q3 = arr(d[form]["q75"])
    keep = xc <= 7.0
    ax.plot(xc[keep], med[keep], ls, color="#2F6690", label=f"{form}")
    ax.fill_between(xc[keep], q1[keep], q3[keep], color="#2F6690", alpha=0.12, lw=0)
    stats[f"e5_{form}_drift_at"] = {f"{x:.2f}": (None if not np.isfinite(m) else float(m)) for x, m in zip(xc, med)}
ax.set_xlabel("Distance travelled [m]"); ax.set_ylabel("Dead-reckoning error [m]")
ax.grid(alpha=0.25, lw=0.4); ax.legend(frameon=False)
fig.savefig(F + "fig_drift.pdf"); plt.close(fig)

from matplotlib.patches import Wedge, FancyArrowPatch, Patch
from matplotlib.lines import Line2D
import core as _core
fig, axs = plt.subplots(1, 3, figsize=(TW, 1.75), gridspec_kw=dict(width_ratios=[0.82, 1.0, 1.0]))
# ---- (a) simulator scene: sensors, radio links, corridor, unmapped obstacle (snapshot of one mission)
ax = axs[0]
sim = _core.SwarmSim(1, 4, form="triangle", p_loss=0.2, burst=4, seed=0, n_unknown=1)
for step in range(377):
    if step % 2 == 0:
        sim.comm_step()
    sim.election_step(); sim.ranges = sim.sense()
    cmd, _ = _core.control_step(sim, "lpsi_pred")
    sim.physics(cmd)
sim.ranges = sim.sense()
P = sim.paths[0]
half_w = np.abs(sim.offsets[0][:, 1]).max()
ax.plot(P[:, 0], P[:, 1], color="0.55", ls=":", lw=0.8)
# corridor the formation-aware planner reserves: points within half-width + R + margin of the path
gx, gy = np.meshgrid(np.linspace(0, sim.W, 400), np.linspace(0, sim.H, 300))
dmin = np.full(gx.shape, np.inf)
for (x0, y0), (x1, y1) in zip(P[:-1:3], P[3::3]):
    vx, vy = x1 - x0, y1 - y0
    t = np.clip(((gx - x0) * vx + (gy - y0) * vy) / max(vx * vx + vy * vy, 1e-9), 0, 1)
    dmin = np.minimum(dmin, np.hypot(gx - x0 - t * vx, gy - y0 - t * vy))
ax.contourf(gx, gy, dmin, levels=[0, half_w + sim.c.R + 0.10], colors=["#2F6690"], alpha=0.10)
unk = set(map(tuple, sim.unknown_list[0]))
for (x, y, r) in sim.obs_list[0]:
    ax.add_patch(Circle((x, y), r, color=("#B23A48" if (x, y, r) in unk else "0.35"), lw=0))
rays = sim.true_rays[0]
c = sim.c
for i in range(4):
    x, y, th = sim.pos[0, i, 0], sim.pos[0, i, 1], sim.th[0, i]
    # ultrasonic: cone of +-12 deg cut at the measured (closest) range
    rus = min(rays[i, :3].min(), c.us_rng[1])
    ax.add_patch(Wedge((x, y), c.mount + rus, np.degrees(th) - 12, np.degrees(th) + 12,
                       facecolor="#E39B2D", alpha=0.35, lw=0, zorder=2))
    # infrared: rays at +-60 deg to the hit point or 0.8 m
    for k in (3, 4):
        a = c.ray_ang[k]
        L = min(rays[i, k], c.ir_rng[1])
        ox, oy = x + c.mount * np.cos(th + a), y + c.mount * np.sin(th + a)
        ax.plot([ox, ox + L * np.cos(th + a)], [oy, oy + L * np.sin(th + a)], color="#6A994E", lw=0.7, zorder=2)
    ax.add_patch(Circle((x, y), c.R, facecolor="white", edgecolor=ROB[i], lw=0.9, zorder=3))
    ax.plot([x, x + 0.16 * np.cos(th)], [y, y + 0.16 * np.sin(th)], color=ROB[i], lw=0.9, zorder=4)
for i in range(1, 4):
    ax.annotate("", xy=sim.pos[0, i], xytext=sim.pos[0, 0],
                arrowprops=dict(arrowstyle="-|>", color="#2F6690", lw=0.5, ls="--", alpha=0.7, mutation_scale=5))
cx, cy = sim.pos[0, 0]
ax.set_xlim(cx - 2.4, cx + 1.4); ax.set_ylim(cy - 1.45, cy + 1.45)
ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
ax.set_title("(a) Simulator scene")
for sp in ax.spines.values():
    sp.set_visible(True); sp.set_linewidth(0.5)
# 0.5 m scale bar
sx0, sy0 = cx - 2.25, cy - 1.35
ax.plot([sx0, sx0 + 0.5], [sy0, sy0], color="k", lw=1.0)
ax.text(sx0 + 0.25, sy0 + 0.06, "0.5 m", ha="center", va="bottom", fontsize=5.5)
for ax, form in zip(axs[1:], ["triangle", "Y"]):
    ex = d[form]["example"]
    pos = np.array(ex["pos"]); est = np.array(ex["est"]); tt = np.array(ex["t"])
    for (x, y, r) in ex["obs"]:
        ax.add_patch(Circle((x, y), r, color="0.45", lw=0))
    P = np.array(ex["path"]); ax.plot(P[:, 0], P[:, 1], color="0.5", ls=":", lw=0.8)
    for i in range(4):
        ax.plot(pos[:, i, 0], pos[:, i, 1], color=ROB[i], lw=0.9)
        ax.plot(est[:, i, 0], est[:, i, 1], color=ROB[i], lw=0.6, ls="--", alpha=0.85)
    for k in np.linspace(0, len(tt) - 1, 6).astype(int):
        ax.plot(pos[k, :, 0], pos[k, :, 1], color="0.65", lw=0.4, zorder=1)
        for i in range(4):
            ax.add_patch(Circle(pos[k, i], 0.12, fill=False, color=ROB[i], lw=0.55))
    ax.set_aspect("equal"); ax.set_xlim(0, 8.0); ax.set_ylim(0.6, 4.4); ax.set_yticks([1, 2, 3, 4])
    nm = "Triangle" if form == "triangle" else "Dynamic Y"
    ax.set_title(f"({'b' if form == 'triangle' else 'c'}) {nm}: RMSE {ex['rmse_gt']:.2f} m (GT), {ex['rmse_est']:.2f} m (est.)")
    ax.set_xlabel("$x$ [m]"); ax.set_ylabel("$y$ [m]")
    ax.grid(alpha=0.25, lw=0.4); ax.set_axisbelow(True)
hl = [Line2D([], [], color="0.5", ls=":", lw=0.9, label="leader A* path")]
hl += [Line2D([], [], color=ROB[i], lw=1.2, label=["Alpha", "Beta-1", "Beta-2", "Beta-3"][i]) for i in range(4)]
hl += [Line2D([], [], color="0.3", lw=0.9, label="true path"), Line2D([], [], color="0.3", lw=0.7, ls="--", label="dead-reckoned")]
hl += [Patch(facecolor="#E39B2D", alpha=0.35, label="ultrasonic cone"),
       Line2D([], [], color="#6A994E", lw=0.8, label="IR ray"),
       Line2D([], [], color="#2F6690", lw=0.6, ls="--", label="ESP-NOW heartbeat"),
       Patch(facecolor="#2F6690", alpha=0.12, label="A* corridor"),
       Line2D([], [], marker="o", color="0.45", lw=0, ms=4, label="mapped obstacle"),
       Line2D([], [], marker="o", color="#B23A48", lw=0, ms=4, label="unmapped obstacle")]
r1, r2 = hl[:7], hl[7:]
hl = [h for pair in zip(r1, r2 + [Line2D([], [], lw=0, label=" ")]) for h in pair]
fig.legend(handles=hl, loc="lower center", ncol=7, bbox_to_anchor=(0.5, 0.94), frameon=False, columnspacing=1.0,
           handlelength=1.6)
fig.subplots_adjust(wspace=0.2, left=0.01, right=0.995)
fig.savefig(F + "fig_traj.pdf"); plt.close(fig)

d = load("e6_switch")
for r in d:
    lat = arr(r["switch_lat"])
    stats[f"e6_{r['src']}_{r['p']}"] = dict(med=float(np.nanmedian(lat)), p90=float(np.nanpercentile(lat, 90)),
                                            done=float(np.mean(np.isfinite(lat))), coll=float(np.mean(arr(r["collisions"]))),
                                            succ=float(np.mean(arr(r["success"]))))
json.dump(stats, open(R + "stats.json", "w"), indent=1)
print("figures and stats written")
