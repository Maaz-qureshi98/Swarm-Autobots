"""Fig. 2: (a) labelled top view of one robot, (b) sensor layout to scale, (c) power and signal wiring."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, Rectangle, FancyBboxPatch
from PIL import Image

plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
                     "font.size": 6.5, "pdf.fonttype": 42})
W, H = 3.45, 3.05                          # printed size [in]
INK, GRAY = "#24292E", "#E5E7E9"           # same ink and box fill as the Fig. 3 flowchart
SIG = "#5A6B7B"                            # muted slate for signal paths
US_C, IR_C = "#C9822B", "#5E8C4A"          # ultrasonic / infrared accents
fig = plt.figure(figsize=(W, H))


def ax_in(x, y, w, h):                     # axes placed in inches from the lower-left corner
    return fig.add_axes([x / W, y / H, w / W, h / H])


def title(ax, s):
    ax.text(0.5, 1.0, s, transform=ax.transAxes, ha="center", va="bottom", fontsize=7)


# ---------------------------------------------------------------- (a) labelled photo (top view, front up)
im = np.asarray(Image.open("figs_hw/robot_top.jpg").convert("RGB"))     # 766 x 907 px
ph, pw = im.shape[:2]
img_h = 1.42
img_w = img_h * pw / ph
x0 = (W - img_w) / 2
axp = ax_in(x0, 1.45, img_w, img_h)
axp.imshow(im, interpolation="none"); axp.set_axis_off()
title(axp, "(a) Robot, top view (front up)")
LEFT, RIGHT = -0.06, 1.06                  # label anchors in photo-axes fraction
LAB = [  # text, point on photo (px), side, label height (axes fraction)
    ("HC-SR04\nultrasonic ranger", (372, 70), "L", 0.93),
    ("Tracked chassis\n(T101)", (95, 520), "L", 0.62),
    ("LM2596 buck\nconverter", (172, 700), "L", 0.33),
    ("Hall-encoder\nDC motor", (330, 752), "L", 0.08),
    ("ESP32\ncontroller", (368, 445), "R", 0.86),
    ("Relay\n(pump)", (603, 458), "R", 0.64),
    ("L298N dual\nH-bridge", (572, 705), "R", 0.40),
    ("12 V LiPo (below deck)", (430, 692), "B", -0.05),
]
for txt, (px, py), side, yl in LAB:
    if side == "B":
        axp.annotate(txt, xy=(px / pw, 1 - py / ph), xycoords="axes fraction", xytext=(px / pw, yl),
                     textcoords="axes fraction", ha="center", va="top", fontsize=6,
                     arrowprops=dict(arrowstyle="-", lw=0.5, color="k", shrinkA=1, shrinkB=0), annotation_clip=False)
    else:
        xt = LEFT if side == "L" else RIGHT
        axp.annotate(txt, xy=(px / pw, 1 - py / ph), xycoords="axes fraction", xytext=(xt, yl), textcoords="axes fraction",
                     ha="right" if side == "L" else "left", va="center", fontsize=6, linespacing=1.0,
                     arrowprops=dict(arrowstyle="-", lw=0.5, color="k", shrinkA=1, shrinkB=0), annotation_clip=False)
    axp.plot(px, py, "o", ms=2.4, mfc="white", mec="k", mew=0.6)
axp.set_xlim(0, pw); axp.set_ylim(ph, 0)
# arrows drawn in axes-fraction need data limits matching the image extent
axp.set_xlim(-0.5, pw - 0.5); axp.set_ylim(ph - 0.5, -0.5)

# ---------------------------------------------------------------- (b) sensor layout (to scale, rays shortened)
ax = ax_in(0.02, 0.02, 1.55, 1.12)
ax.set_aspect("equal"); ax.axis("off")
title(ax, "(b) Sensor layout")
R, m = 0.142, 0.10
ax.add_patch(Rectangle((-0.10, -0.10), 0.20, 0.20, fc=GRAY, ec=INK, lw=0.65))
ax.add_patch(Circle((0, 0), R, fill=False, ls="--", lw=0.5, ec="0.45"))
ax.add_patch(Wedge((m, 0), 0.34, -12, 12, fc=US_C, alpha=0.30, lw=0))
ax.plot(m, 0, "o", ms=2.6, color=US_C, zorder=5)
for sg in (1, -1):
    a = np.radians(60 * sg)
    p0 = np.array([m * np.cos(a), m * np.sin(a)]); p1 = p0 + 0.30 * np.array([np.cos(a), np.sin(a)])
    ax.plot(*zip(p0, p1), color=IR_C, lw=1.0)
    ax.plot(*p0, "o", ms=2.6, color=IR_C, zorder=5)
ax.plot(0, 0, "+", ms=4, color="k", mew=0.6)
ax.annotate("", xy=(0.0, 0.215), xytext=(-0.13, 0.215), arrowprops=dict(arrowstyle="-|>", lw=0.6, color="k", mutation_scale=6))
ax.text(-0.135, 0.215, "heading ", ha="right", va="center", fontsize=5.6)
ax.text(0.47, 0.03, "US ±12°\n0.02–2 m", fontsize=5.6, ha="left", va="center", linespacing=1.0)
ax.text(0.27, 0.36, "IR +60°\n0.1–0.8 m", fontsize=5.6, ha="left", va="center", linespacing=1.0)
ax.text(0.27, -0.36, "IR −60°", fontsize=5.6, ha="left", va="center")
ax.annotate("mounts at\n0.10 m", xy=(m * np.cos(np.radians(-60)), m * np.sin(np.radians(-60))), xytext=(-0.05, -0.33),
            fontsize=5.6, ha="right", va="center", linespacing=1.0, arrowprops=dict(arrowstyle="-", lw=0.4))
ax.text(-0.17, 0.0, "$R$ = 0.142 m\n20×20 cm", fontsize=5.6, ha="right", va="center", linespacing=1.0)
ax.set_xlim(-0.47, 0.72); ax.set_ylim(-0.44, 0.44)

# ---------------------------------------------------------------- (c) power and signal wiring
ax = ax_in(1.62, 0.02, 1.80, 1.12)
ax.set_xlim(0, 1.8); ax.set_ylim(0, 1.2); ax.axis("off")
title(ax, "(c) Power and signal wiring")


def box(x, y, w, h, title_s, detail=""):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.0,rounding_size=0.03", fc=GRAY, ec=INK, lw=0.65))
    if detail:
        ax.text(x + w / 2, y + h / 2 + 0.005, title_s, ha="center", va="bottom", fontsize=5.8, fontweight="bold",
                color=INK, linespacing=1.0)
        ax.text(x + w / 2, y + h / 2 - 0.015, detail, ha="center", va="top", fontsize=5.4, color=INK, linespacing=1.0)
    else:
        ax.text(x + w / 2, y + h / 2, title_s, ha="center", va="center", fontsize=5.8, fontweight="bold", color=INK,
                linespacing=1.0)
    return (x, y, w, h)


def route(pts, c=INK, ls="-"):
    """Orthogonal connector: polyline with one solid arrowhead at the last point."""
    pts = np.asarray(pts, float)
    d = pts[-1] - pts[-2]; d /= np.linalg.norm(d)
    line = pts.copy(); line[-1] = pts[-1] - 0.03 * d                # stop the stroke under the head
    ax.plot(line[:, 0], line[:, 1], color=c, lw=0.65, ls=ls, solid_capstyle="butt", zorder=2)
    ax.annotate("", xy=pts[-1], xytext=pts[-1] - 0.001 * d,
                arrowprops=dict(arrowstyle="-|>", lw=0.65, color=c, mutation_scale=5, shrinkA=0, shrinkB=0), zorder=3)


def lab(x, y, s, c="0.25", **kw):
    ax.text(x, y, s, fontsize=5.2, color=c, **kw)


BL = SIG
box(0.02, 0.42, 0.38, 0.36, "Battery", "12 V LiPo\n4.2 Ah")
box(0.46, 0.86, 0.44, 0.26, "2$\\times$LM2596", "12$\\to$5 V")
box(1.08, 0.86, 0.62, 0.26, "ESP32", "sensors + relay")
box(1.08, 0.47, 0.54, 0.26, "L298N", "dual H-bridge")
box(1.08, 0.06, 0.54, 0.26, "Motors", "2 Hall-encoder")
# power (solid)
route([(0.21, 0.78), (0.21, 0.99), (0.46, 0.99)]);  lab(0.335, 1.005, "12 V", ha="center", va="bottom")
route([(0.40, 0.60), (1.08, 0.60)]);                lab(0.74, 0.615, "12 V", ha="center", va="bottom")
route([(0.90, 0.99), (1.08, 0.99)]);                lab(0.99, 1.005, "5 V", ha="center", va="bottom")
route([(1.35, 0.47), (1.35, 0.32)])
# signal (dashed)
route([(1.22, 0.86), (1.22, 0.73)], c=BL, ls="--"); lab(1.20, 0.795, "PWM", c=BL, ha="right", va="center")
route([(1.62, 0.19), (1.77, 0.19), (1.77, 0.99), (1.70, 0.99)], c=BL, ls="--")
lab(1.75, 0.55, "encoders", c=BL, ha="right", va="center", rotation=90)
ax.plot([0.04, 0.13], [0.24, 0.24], color=INK, lw=0.65); ax.text(0.16, 0.24, "power", va="center", fontsize=5.2, color="0.25")
ax.plot([0.04, 0.13], [0.13, 0.13], color=BL, lw=0.6, ls="--"); ax.text(0.16, 0.13, "signal", va="center", fontsize=5.2, color=BL)

fig.savefig("figs/fig_robot.pdf", dpi=600)
fig.savefig("figs/fig_robot_preview.png", dpi=300)
print("ok")
