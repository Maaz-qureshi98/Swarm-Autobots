"""Fig. 1: (a) the four robots, (b) heartbeat broadcast and self-healing after leader loss (schematic),
(c)-(f) the four hardware cases. Drawn at printed size (IEEE column width)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Arc
from PIL import Image, ImageOps

plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
                     "font.size": 6.5, "pdf.fonttype": 42})
SRC = "figs_hw/"
W, H = 3.45, 2.24
ROB = ["#1b1b1b", "#2F6690", "#E39B2D", "#6A994E"]
BL = "#2F6690"
fig = plt.figure(figsize=(W, H))


def ax_in(x, y, w, h):
    a = fig.add_axes([x / W, y / H, w / W, h / H]); return a


def photo(name):
    im = Image.open(SRC + name).convert("RGB")
    return ImageOps.autocontrast(im, cutoff=0.5)


def crop_to(im, aspect, cx=0.5, cy=0.5):
    w, h = im.size
    if w / h > aspect:
        nw = int(h * aspect); x0 = int(np.clip(cx * w - nw / 2, 0, w - nw)); return im.crop((x0, 0, x0 + nw, h))
    nh = int(w / aspect); y0 = int(np.clip(cy * h - nh / 2, 0, h - nh)); return im.crop((0, y0, w, y0 + nh))


def head(ax, s):
    ax.text(0.0, 1.0, s, transform=ax.transAxes, ha="left", va="bottom", fontsize=6.8)


# ------------------------------------------------------------------ (a) hero photo
top_h = 1.33
wa = 1.80
axa = ax_in(0.0, H - top_h - 0.13, wa, top_h)
axa.imshow(np.asarray(crop_to(photo("hero.jpg"), wa / top_h, cy=0.47)), interpolation="none")
axa.set_xticks([]); axa.set_yticks([])
for sp in axa.spines.values():
    sp.set_linewidth(0.4)
head(axa, "(a) Four identical ESP32 robots")

# ------------------------------------------------------------------ (b) schematic: nominal and after leader loss
xb = wa + 0.07
wb = W - xb
axb = ax_in(xb, H - top_h - 0.13, wb, top_h)
axb.set_xlim(0, 2.0); axb.set_ylim(0, 1.6); axb.set_aspect("equal"); axb.axis("off")
head(axb, "(b) Self-healing after leader loss")


def robot(ax, x, y, c, lab, alpha=1.0, failed=False, side="below"):
    s = 0.11
    ax.add_patch(FancyBboxPatch((x - s / 2, y - s / 2), s, s, boxstyle="round,pad=0,rounding_size=0.025",
                                fc="white" if failed else c, ec=c, lw=0.8, alpha=alpha, zorder=4))
    ax.plot([x, x], [y + s / 2, y + s / 2 + 0.05], color=c, lw=0.8, alpha=alpha, zorder=4)     # heading tick
    if side == "below":
        ax.text(x, y - s / 2 - 0.03, lab, ha="center", va="top", fontsize=5.2, alpha=alpha, zorder=5)
    else:
        ax.text(x + s / 2 + 0.035, y, lab, ha="left", va="center", fontsize=5.2, alpha=alpha, zorder=5)
    if failed:
        ax.plot([x - 0.05, x + 0.05], [y - 0.05, y + 0.05], color="#B23A48", lw=1.0, zorder=6)
        ax.plot([x - 0.05, x + 0.05], [y + 0.05, y - 0.05], color="#B23A48", lw=1.0, zorder=6)


def rings(ax, x, y):
    for r, a in [(0.11, 0.95), (0.165, 0.65), (0.22, 0.35)]:
        ax.add_patch(Arc((x, y), 2 * r, 2 * r, theta1=20, theta2=160, color=BL, lw=0.7, alpha=a, ls=(0, (2.5, 1.5)),
                         zorder=3))


def scene(ox, after):
    # Y pattern, heading up; slot offsets drawn to scale (0.5 m lateral, 0.5/0.8 m longitudinal) * 0.42
    k = 0.70
    P = {0: (0, 0), 1: (0, -0.5), 2: (0.5, -1.3), 3: (-0.5, -1.3)}
    pos = {i: (ox + 0.62 * px, 1.24 + k * py) for i, (px, py) in P.items()}
    lead = 1 if after else 0
    # slot lines from the current leader frame
    for i in (2, 3) + (() if after else (1,)):
        a, b = pos[lead], pos[i]
        axb.plot([a[0], b[0]], [a[1], b[1]], color="0.55", lw=0.5, ls=(0, (1, 1.2)), zorder=2)
    rings(axb, *pos[lead])
    robot(axb, *pos[0], ROB[0], "Alpha", alpha=0.45 if after else 1.0, failed=after, side="right")
    robot(axb, *pos[1], ROB[1], "Beta-1", side="right")
    for i in (2, 3):
        robot(axb, *pos[i], ROB[i], f"Beta-{i}")
    return pos


p0 = scene(0.50, after=False)
p1 = scene(1.50, after=True)
axb.text(0.50, 1.47, "Alpha leads", ha="center", va="bottom", fontsize=5.6, style="italic")
axb.text(1.50, 1.47, "Beta-1 takes over", ha="center", va="bottom", fontsize=5.6, style="italic")
axb.annotate("", xy=(1.09, 0.62), xytext=(0.91, 0.62),
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color="k", mutation_scale=6))
axb.text(1.0, 0.67, "1.65 s", ha="center", va="bottom", fontsize=5.2)
axb.plot([1.0, 1.0], [0.80, 1.50], color="0.85", lw=0.5, zorder=1)
axb.plot([1.0, 1.0], [0.18, 0.55], color="0.85", lw=0.5, zorder=1)
axb.text(1.0, 0.0, "", fontsize=1)
# small key
axb.plot([0.10, 0.24], [0.04, 0.04], color=BL, lw=0.7, ls=(0, (2.5, 1.5)))
axb.text(0.27, 0.04, "heartbeat", va="center", fontsize=5.0)
axb.plot([1.06, 1.20], [0.04, 0.04], color="0.55", lw=0.5, ls=(0, (1, 1.2)))
axb.text(1.23, 0.04, "slot (kept)", va="center", fontsize=5.0)

# The four hardware cases are shown in the separate case figure (make_cases_fig.py).

fig.savefig("figs/fig_teaser.png", dpi=600)
from PIL import ImageChops
full = Image.open("figs/fig_teaser.png").convert("RGB")
full = full.crop(ImageChops.difference(full, Image.new("RGB", full.size, "white")).getbbox())
full.save("figs/fig_teaser.jpg", quality=93, dpi=(600, 600))
fig.savefig("figs/fig_teaser_preview.png", dpi=300)
print("ok")
