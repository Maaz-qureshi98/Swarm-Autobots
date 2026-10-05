"""Render labelled simulation videos of the swarm layer (top view) for the submission video.
    python3 make_sim_videos.py OUT_DIR"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Rectangle, Circle, Polygon
from runner import run

OUT = sys.argv[1] if len(sys.argv) > 1 else "sim_videos"
os.makedirs(OUT, exist_ok=True)
NAMES = ["Alpha", "Beta-1", "Beta-2", "Beta-3"]
COLS = ["#222222", "#1f5fa8", "#e08a1e", "#3f8f3a"]


def pick(o, need_switch=False, need_fail=False):
    ok = o["success"] > 0
    if need_fail:
        ok &= np.isfinite(o["recover_t"])
    if need_switch:
        ok &= np.isfinite(o["switch_lat"])
    return int(np.argmax(ok)) if ok.any() else 0


def render(fn, title, subtitle, rec, sim, e, events):
    T = len(rec)
    pos = np.array([r["pos"][e] for r in rec]); th = np.array([r["th"][e] for r in rec])
    est = np.array([r["est"][e] for r in rec])
    lead = np.array([r["is_leader"][e] for r in rec]); alive = np.array([r["alive"][e] for r in rec])
    t = np.array([r["t"][e] for r in rec]); done = np.array([r["done"][e] for r in rec])
    last = int(np.argmax(done)) if done.any() else T
    W, H = sim.W, sim.H
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.02, 0.04, 0.70, 0.84])
    side = fig.add_axes([0.74, 0.04, 0.24, 0.84]); side.axis("off")
    fig.text(0.02, 0.95, title, fontsize=18, weight="bold")
    fig.text(0.02, 0.91, subtitle, fontsize=11, color="#444")
    fig.text(0.98, 0.95, "SIMULATION", fontsize=16, weight="bold", color="#c0002a", ha="right")
    path = sim.paths[e]; mapped = [o for o in sim.obs_list[e] if o not in sim.unknown_list[e]]
    writer = FFMpegWriter(fps=20, bitrate=4000)
    with writer.saving(fig, fn, dpi=100):
        for k in range(0, last, 1):
            ax.clear(); ax.set_xlim(0, W); ax.set_ylim(0, H); ax.set_aspect("equal")
            ax.set_xticks([]); ax.set_yticks([]); ax.set_facecolor("#f7f3dc")
            ax.plot(path[:, 0], path[:, 1], ":", color="#999", lw=1)
            for (ox, oy, r) in mapped:
                ax.add_patch(Circle((ox, oy), r, color="#777"))
            for (ox, oy, r) in sim.unknown_list[e]:
                ax.add_patch(Circle((ox, oy), r, color="#b5543a"))
            for i in range(4):
                if not alive[k, i]:
                    continue
                ax.plot(pos[max(0, k - 400):k + 1, i, 0], pos[max(0, k - 400):k + 1, i, 1], "-", color=COLS[i], lw=1, alpha=0.5)
                ax.plot(est[max(0, k - 400):k + 1, i, 0], est[max(0, k - 400):k + 1, i, 1], "--", color=COLS[i], lw=0.8, alpha=0.4)
                c, s = np.cos(th[k, i]), np.sin(th[k, i])
                corners = np.array([[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]])
                P = pos[k, i] + corners @ np.array([[c, s], [-s, c]])
                ax.add_patch(Polygon(P, closed=True, color=COLS[i], alpha=0.95))
                ax.plot([pos[k, i, 0], pos[k, i, 0] + 0.16 * c], [pos[k, i, 1], pos[k, i, 1] + 0.16 * s], color="w", lw=2)
                if lead[k, i]:
                    ph = (t[k] % 0.1) / 0.1
                    ax.add_patch(Circle(pos[k, i], 0.2 + 0.25 * ph, fill=False, ec=COLS[i], lw=1.5, alpha=1 - ph))
                ax.text(pos[k, i, 0], pos[k, i, 1] + 0.2, NAMES[i] + (" (leader)" if lead[k, i] else ""),
                        ha="center", fontsize=8, color=COLS[i], weight="bold")
            side.clear(); side.axis("off")
            side.text(0, 0.97, f"t = {t[k]:5.1f} s", fontsize=14, family="monospace")
            ln = [NAMES[i] for i in range(4) if lead[k, i] and alive[k, i]]
            side.text(0, 0.90, "Leader: " + (", ".join(ln) if ln else "none (election)"), fontsize=11)
            y = 0.80
            for (te, msg) in events:
                if t[k] >= te:
                    side.text(0, y, f"{te:4.1f} s  {msg}", fontsize=10, color="#c0002a"); y -= 0.06
            side.text(0, 0.40, "solid: true path\ndashed: robot's own estimate\ngrey: mapped obstacle\nred: unmapped obstacle\nrings: leader heartbeat (10 Hz)",
                      fontsize=9, color="#444", va="top")
            side.text(0, 0.02, "Bursty packet loss p = 0.2, L = 4\nPython simulator (sim/core.py) used for\nthe paper's simulation results;\nrendered from recorded simulator states", fontsize=9, color="#444")
            writer.grab_frame()
    plt.close(fig)
    print("wrote", fn, last / 20.0, "s")


# 1) dynamic Y with unmapped obstacles
o, sim, rec = run("lpsi_pred", E=8, N=4, form="Y", p_loss=0.2, burst=4.0, seed=31, n_unknown=2, record=True)
e = pick(o)
render(f"{OUT}/sim1_obstacles.mp4", "Swarm layer: Y formation through obstacles",
       "Speed filter and deadlock recovery around unmapped obstacles", rec, sim, e, [])
# 2) leader failure
o, sim, rec = run("lpsi_pred", E=8, N=4, form="Y", p_loss=0.2, burst=4.0, seed=32, n_unknown=0,
                  fail_time=12.0, max_time=90.0, record=True)
e = pick(o, need_fail=True)
render(f"{OUT}/sim2_leader_failure.mp4", "Swarm layer: leader failure and election",
       f"Alpha removed at 12 s; recovered after {o['recover_t'][e]:.2f} s", rec, sim, e,
       [(12.0, "Alpha switched off"), (12.0 + o["recover_t"][e], "all follow Beta-1")])
# 3) pattern switch
o, sim, rec = run("lpsi_pred", E=8, N=4, form="triangle", p_loss=0.2, burst=4.0, seed=33, n_unknown=0,
                  switch_time=12.0, switch_to="Y", record=True)
e = pick(o, need_switch=True)
render(f"{OUT}/sim3_pattern_switch.mp4", "Swarm layer: pattern switch triangle to Y",
       f"Pattern ID carried in the heartbeat; switch completed after {o['switch_lat'][e]:.2f} s", rec, sim, e,
       [(12.0, "operator: switch to Y"), (12.0 + o["switch_lat"][e], "all Betas in new slots")])
