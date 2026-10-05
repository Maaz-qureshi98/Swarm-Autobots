"""Per-robot compass bias from firmware/compass_cal/compass_cal.ino logs.

    python3 tools/analyze_compass.py compass_0.csv compass_1.csv compass_2.csv compass_3.csv

Input lines: "CAL,rank,true_deg,measured_deg,sample_std_deg,motors" (other lines are ignored).

For each robot and motor condition it reports
  bias      circular mean of (measured - true): the constant heading error of that robot,
  ripple    RMS of the heading-dependent part around that bias (hard/soft-iron distortion),
  noise     mean within-sample std.
Across robots it reports the quantity the simulator uses: the spread (std) of the per-robot
biases after removing the bias common to all robots, which only rotates the whole formation."""
import sys
import numpy as np


def wrap(d):
    return (np.asarray(d) + 180.0) % 360.0 - 180.0


def main(files):
    rows = []
    for fn in files:
        for line in open(fn):
            p = line.strip().split(",")
            if len(p) == 6 and p[0] == "CAL":
                rows.append((int(p[1]), float(p[2]), float(p[3]), float(p[4]), int(p[5])))
    if not rows:
        print("no CAL lines found"); return
    R = np.array(rows)
    out = {}
    for motors in (0, 1):
        res = {}
        for rank in sorted(set(R[:, 0].astype(int))):
            m = (R[:, 0] == rank) & (R[:, 4] == motors)
            if m.sum() < 3:
                continue
            err = wrap(R[m, 2] - R[m, 1])
            bias = np.degrees(np.arctan2(np.sin(np.radians(err)).mean(), np.cos(np.radians(err)).mean()))
            ripple = float(np.sqrt(np.mean(wrap(err - bias) ** 2)))
            res[rank] = dict(bias=float(bias), ripple=ripple, noise=float(R[m, 3].mean()), n=int(m.sum()),
                             worst=float(np.abs(err).max()))
        if not res:
            continue
        b = np.array([v["bias"] for v in res.values()])
        common = np.degrees(np.arctan2(np.sin(np.radians(b)).mean(), np.cos(np.radians(b)).mean()))
        rel = wrap(b - common)
        spread = float(np.std(rel, ddof=1)) if len(rel) > 1 else float("nan")
        print(f"\nmotors {'ON' if motors else 'OFF'}")
        print(" rank   bias[deg]  rel.bias[deg]  ripple RMS[deg]  noise[deg]  worst |err|[deg]  headings")
        for (rank, v), r in zip(res.items(), rel):
            print(f" {rank:4d}  {v['bias']:9.2f}  {r:13.2f}  {v['ripple']:15.2f}  {v['noise']:10.2f}  "
                  f"{v['worst']:16.2f}  {v['n']:8d}")
        print(f" common bias {common:.2f} deg (rotates the whole formation; harmless)")
        print(f" between-robot bias spread (std, n={len(rel)}): {spread:.2f} deg; "
              f"range {rel.min():.2f} to {rel.max():.2f} deg")
        print(f" heading-dependent ripple across robots: {np.mean([v['ripple'] for v in res.values()]):.2f} deg RMS")
        out[motors] = dict(spread=spread, ripple=float(np.mean([v['ripple'] for v in res.values()])),
                           range=(float(rel.min()), float(rel.max())))
    if out:
        k = 1 if 1 in out else 0
        o = out[k]
        print("\nFor main.tex (motors %s):" % ("ON" if k else "OFF"))
        print(f"  \\renewcommand{{\\HWbiasSpread}}{{{o['spread']:.2f}}}")
        print(f"  \\renewcommand{{\\HWbiasRipple}}{{{o['ripple']:.2f}}}")


if __name__ == "__main__":
    main(sys.argv[1:])
