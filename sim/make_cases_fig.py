"""Hardware/simulation/measurement figure for the four cases -> figs/fig_cases.pdf
Row 1: labelled frames of the hardware runs (original ASA video). Row 2: swarm-layer simulation of
the same pattern. Row 3: ground-truth spacing deviations of the swarm-layer runs, measured on the floor grid.
Style follows make_figs.py / make_robot_fig.py (serif, robot colours, plain labels with leader lines)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle
from PIL import Image
import matplotlib.patheffects as pe
from runner import run
from core import formation

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
    "font.size": 8.8, "axes.titlesize": 8.8, "axes.labelsize": 8.8, "xtick.labelsize": 8.2, "ytick.labelsize": 8.8,
    "legend.fontsize": 8.8, "axes.linewidth": 0.5, "xtick.major.width": 0.5, "ytick.major.width": 0.5,
    "xtick.major.size": 2, "ytick.major.size": 2, "xtick.direction": "in", "ytick.direction": "in",
    "axes.titlepad": 3, "pdf.fonttype": 42,
})
ROB = ["#1b1b1b", "#2F6690", "#E39B2D", "#6A994E"]
RED = "#B23A48"; GREY = "#4d4d4d"
D = "figs_hw/"
LBOX = dict(boxstyle="square,pad=0.15", fc="white", ec="none")


def load(f, crop):
    im = Image.open(D + f).convert("RGB"); s = im.size[0] / 1000
    x0, y0, x1, y1 = [int(v * s) for v in crop]; im = im.crop((x0, y0, x1, y1)); im.thumbnail((1400, 1400))
    k = im.size[0] / (x1 - x0) * s
    return im, (lambda x, y: ((x - crop[0]) * k, (y - crop[1]) * k))


def label(ax, P, x, y, txt, dx, dy, col="black"):
    px, py = P(x, y); tx, ty = P(x + dx, y + dy)
    ax.annotate(txt, (px, py), (tx, ty), ha="center", va="center", fontsize=8.8, color="black",
                arrowprops=dict(arrowstyle="-", lw=0.6, color="black", shrinkA=3, shrinkB=0), zorder=5)
    ax.plot(px, py, "o", ms=2.2, color="black", zorder=6)


def dim(ax, P, a, b, txt, o=(0, 0)):
    pa, pb = P(*a), P(*b)
    ax.annotate("", pa, pb, arrowprops=dict(arrowstyle="<|-|>,head_length=0.25,head_width=0.12", lw=0.6,
                                            color="black", shrinkA=0, shrinkB=0), zorder=4)
    m = ((pa[0] + pb[0]) / 2 + o[0], (pa[1] + pb[1]) / 2 + o[1])
    ax.text(*m, txt, fontsize=8.8, color="black", ha="center", va="center", zorder=6)


def frame(ax):
    ax.set_xticks([]); ax.set_yticks([])
    for s_ in ax.spines.values():
        s_.set_visible(True); s_.set_linewidth(0.5); s_.set_color("#333333")


fig = plt.figure(figsize=(7.0, 3.45))
gs = fig.add_gridspec(3, 4, left=0.085, right=0.997, top=0.94, bottom=0.075, hspace=0.14, wspace=0.05,
                      height_ratios=[1.0, 0.95, 0.72])
titles = ["(a) Case 1: teleoperated triangle", "(b) Case 2: static Y", "(c) Case 3: dynamic Y, obstacle",
          "(d) Case 4: dynamic Y, fire"]

# ---------------- row 1: hardware frames
H = []
ax = fig.add_subplot(gs[0, 0]); im, P = load("hw_case1_v2.jpg", (30, 60, 960, 645)); ax.imshow(im)
label(ax, P, 820, 150, "Alpha (on table)", -400, -40, ROB[0])
label(ax, P, 140, 280, "Beta", 0, -80, GREY); label(ax, P, 460, 265, "Beta", 40, -80, GREY)
label(ax, P, 250, 520, "Beta", -135, 35, GREY)
dim(ax, P, (205, 300), (395, 285), "30 cm", (0, -45)); dim(ax, P, (165, 350), (280, 500), "30 cm", (-150, 25))
dim(ax, P, (440, 340), (340, 490), "30 cm", (150, 25))
H.append(ax)
ax = fig.add_subplot(gs[0, 1]); im, P = load("hw_case2_v2.jpg", (20, 40, 960, 645)); ax.imshow(im)
label(ax, P, 390, 145, "Alpha", -120, -40, ROB[0]); label(ax, P, 450, 300, "Beta-1", 200, -20, ROB[1])
label(ax, P, 160, 520, "Beta", 140, 85, GREY); label(ax, P, 790, 500, "Beta", 80, -120, GREY)
dim(ax, P, (395, 190), (420, 240), "30 cm", (-190, 10)); dim(ax, P, (360, 345), (210, 470), "60 cm", (-100, -55))
dim(ax, P, (480, 345), (740, 450), "60 cm", (30, 65))
H.append(ax)
ax = fig.add_subplot(gs[0, 2]); im, P = load("hw_case3_v2.jpg", (20, 40, 960, 645)); ax.imshow(im)
label(ax, P, 320, 220, "Alpha", 60, -140, ROB[0]); label(ax, P, 660, 280, "Beta-1", -130, 165, ROB[1])
label(ax, P, 860, 190, "Beta", 0, -120, GREY); label(ax, P, 790, 500, "Beta", -60, 110, GREY)
label(ax, P, 140, 250, "obstacle", 10, 200, "black")
dim(ax, P, (400, 240), (570, 270), "30 cm", (10, -90))
H.append(ax)
ax = fig.add_subplot(gs[0, 3]); im, P = load("hw_case4_v2.jpg", (110, 10, 1000, 566)); ax.imshow(im)
label(ax, P, 500, 255, "Alpha", -40, -150, ROB[0])
label(ax, P, 565, 135, "Beta", 90, -85, GREY); label(ax, P, 660, 215, "Beta", 20, 110, GREY)
label(ax, P, 865, 215, "Beta", 0, 110, GREY)
label(ax, P, 255, 400, "flame", -60, 100, "black")
dim(ax, P, (440, 300), (300, 395), "0–80 cm", (-155, -80))
H.append(ax)
for a, t in zip(H, titles):
    frame(a); a.set_title(t)


# ---------------- row 2: simulation snapshots of the swarm layer
def snapshot(ax, form, seed, n_unknown, when, label_txt):
    o, sim, rec = run("lpsi_pred", E=6, N=4, form=form, p_loss=0.2, burst=4.0, seed=seed, n_unknown=n_unknown, record=True)
    e = int(np.argmax(o["success"] > 0))
    pos = np.array([r["pos"][e] for r in rec]); th = np.array([r["th"][e] for r in rec]); t = np.array([r["t"][e] for r in rec])
    est = np.array([r["est"][e] for r in rec])
    if when == "obstacle" and sim.unknown_list[e]:
        u = np.array(sim.unknown_list[e][0][:2])
        k = int(np.argmin(np.min(np.hypot(*(pos[:, 1:] - u).transpose(2, 0, 1)), axis=1)))
    elif when == "start":
        k = int(3.0 / sim.c.dt)
    else:
        k = int(when / sim.c.dt)
    off = formation(form, 4)
    c, s = np.cos(th[k, 0]), np.sin(th[k, 0])
    cen = pos[k].mean(0)
    ax.set_xlim(cen[0] - 1.5, cen[0] + 1.5); ax.set_ylim(cen[1] - 0.9, cen[1] + 0.9); ax.set_aspect("equal")
    ax.set_facecolor("#f4f4f4")
    for (ox, oy, r) in sim.obs_list[e]:
        unk = (ox, oy, r) in sim.unknown_list[e]
        ax.add_patch(Circle((ox, oy), r, color=RED if unk else "#9a9a9a", lw=0))
    k0 = max(0, k - 160)
    for i in range(4):
        ax.plot(pos[k0:k + 1, i, 0], pos[k0:k + 1, i, 1], "-", color=ROB[i], lw=0.8)
        ax.plot(est[k0:k + 1, i, 0], est[k0:k + 1, i, 1], "--", color=ROB[i], lw=0.5, alpha=0.7)
        ci, si = np.cos(th[k, i]), np.sin(th[k, i])
        corners = np.array([[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]])
        ax.add_patch(Polygon(pos[k, i] + corners @ np.array([[ci, si], [-si, ci]]), closed=True, color=ROB[i], zorder=4))
        if i > 0:
            sl = pos[k, 0] + np.array([c * off[i, 0] - s * off[i, 1], s * off[i, 0] + c * off[i, 1]])
            ax.add_patch(Rectangle(sl - 0.1, 0.2, 0.2, fill=False, ec=ROB[i], lw=0.5, ls=(0, (1.5, 1.2)), zorder=3))
    ax.plot([cen[0] - 1.4, cen[0] - 0.9], [cen[1] - 0.8] * 2, "k-", lw=0.8)
    ax.text(cen[0] - 1.15, cen[1] - 0.76, "0.5 m", ha="center", va="bottom", fontsize=8.8)
    ax.text(cen[0] + 1.45, cen[1] + 0.84, f"{label_txt}, $t$ = {t[k]:.1f} s", ha="right", va="top", fontsize=8.8, zorder=7,
            bbox=dict(boxstyle="square,pad=0.12", fc="white", ec="none", alpha=0.85))
    frame(ax)


snapshot(fig.add_subplot(gs[1, 0]), "triangle", 41, 0, 10.0, "triangle")
snapshot(fig.add_subplot(gs[1, 1]), "Y", 42, 0, "start", "Y after settling")
snapshot(fig.add_subplot(gs[1, 2]), "Y", 43, 2, "obstacle", "unmapped obstacle")
snapshot(fig.add_subplot(gs[1, 3]), "Y", 44, 0, 20.0, "Y, $p$ = 0.2")

# ---------------- row 3: ground-truth spacing deviation on the floor grid (swarm layer) [cm]
NOM = {"tri": [0.5, np.hypot(.5, .5), np.hypot(.5, .5), 1.0], "Y": [0.5, np.hypot(.8, .5), np.hypot(.8, .5), 1.0]}
LAB = ["A–B1", "B1–B2", "B1–B3", "B2–B3"]
meas = [
    ("tri", {"end": [np.nan, 0.72, 0.70, 0.99]}, "slot dev. 2–4 cm (to Beta-1)"),
    ("Y", {"0 s": [0.50, 0.94, 0.94, 1.00], "30 s": [0.51, 0.95, 0.93, 0.99], "60 s": [0.50, 0.95, 0.94, 1.01]}, "slot dev. 2–4 cm"),
    ("Y", {"end": [0.52, 0.96, 0.92, 0.98]}, "slot dev. 4–7 cm after 2–3 m"),
    ("Y", {"end": [0.52, 0.96, 0.95, 1.02]}, "slot dev. 4–8 cm"),
]
GC = {"end": ROB[1], "0 s": "#a9c1d6", "30 s": "#6b97bd", "60 s": ROB[1]}
for j, (f, groups, note) in enumerate(meas):
    ax = fig.add_subplot(gs[2, j])
    n = len(groups); w = 0.8 / n
    for g, (name, d) in enumerate(groups.items()):
        dev = 100 * (np.array(d) - np.array(NOM[f]))
        x = np.arange(4) + (g - (n - 1) / 2) * w
        ax.bar(x, dev, w * 0.9, color=GC[name], label=name if n > 1 else None, lw=0)
    ax.axhline(0, color="black", lw=0.5)
    ax.set_ylim(-3, 3.2); ax.set_xlim(-0.6, 3.6); ax.set_xticks(range(4)); ax.set_xticklabels(LAB)
    ax.set_yticks([-2, 0, 2]); ax.grid(axis="y", lw=0.3, color="#cccccc")
    ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)
    if j == 0:
        ax.set_ylabel("Spacing dev. [cm]")
        ax.text(0, -0.3, "Alpha on\ntable", ha="center", va="top", fontsize=8.2, color=GREY)
    else:
        ax.set_yticklabels([])
    if n > 1:
        ax.legend(ncol=3, loc="lower left", frameon=False, handlelength=0.9, columnspacing=0.7, borderaxespad=0.1)
    ax.text(0.99, 0.99, note, transform=ax.transAxes, ha="right", va="top", fontsize=8.8)

# ---------------- vertical row labels
for r, lab in enumerate(["Hardware", "Simulation", "Measured"]):
    bb = gs[r, 0].get_position(fig)
    fig.patches.append(Rectangle((0.001, bb.y0), 0.017, bb.height, transform=fig.transFigure, fc="#e6e6e6", ec="none"))
    fig.text(0.0095, bb.y0 + bb.height / 2, lab, rotation=90, ha="center", va="center", fontsize=8.8)

fig.savefig("figs/fig_cases.pdf", dpi=300)
fig.savefig("figs/fig_cases.png", dpi=250)
print("ok")
