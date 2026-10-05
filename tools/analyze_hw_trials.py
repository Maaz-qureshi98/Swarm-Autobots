"""Analysis of the hardware loss-injection trials (ZOH vs prediction) and of onboard cycle time.

    python3 tools/analyze_hw_trials.py logs/*.csv --grid grid.csv [--settle 3]

Logs: one serial log per robot per run, named <run>_<rank>.csv (e.g. zoh1_1.csv, pred2_3.csv).
Runs zohK and predK form a pair (same injected-loss seed, start cell, and robot order) and are
compared pairwise; run them back to back, alternating which mode goes first.
holding the CSV lines written by hw::logLine (firmware/swarm_agent/HwExperiment.h):
    ms,rank,is_leader,leader,pkt_age_ms,rx,injected_drops,slot_err_mm,v_mm_s,w_mrad_s,cycle_us,predict
Lines starting with '#' and malformed lines are ignored.

Grid file (optional): run,slot_dev_cm,spacing_dev_cm  - the end-of-run floor-grid measurements
(largest Beta slot deviation relative to Alpha, largest centre-spacing deviation), as in Sec. VI.

Reports per mode: injected loss rate, packet-age statistics, controller-level slot RMSE of the
followers (onboard estimate, after the settling time), grid ground truth, and the cycle time of
every robot; then LaTeX macros for main.tex."""
import argparse
import os
import re
from collections import defaultdict
import numpy as np

COLS = ["ms", "rank", "is_leader", "leader", "age", "rx", "drops", "err", "v", "w", "cyc", "predict"]


def load(fn):
    rows = []
    for line in open(fn, errors="ignore"):
        if line.startswith("#"):
            continue
        p = line.strip().split(",")
        if len(p) != len(COLS):
            continue
        try:
            rows.append([float(x) for x in p])
        except ValueError:
            continue
    return np.array(rows) if rows else np.zeros((0, len(COLS)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--grid", default=None)
    ap.add_argument("--settle", type=float, default=3.0, help="seconds excluded at the start of each run")
    a = ap.parse_args()

    runs = defaultdict(dict)                    # run -> rank -> array
    for fn in a.logs:
        m = re.match(r"(.+)_(\d+)\.csv$", os.path.basename(fn))
        if not m:
            print("skipping (name must be <run>_<rank>.csv):", fn); continue
        d = load(fn)
        if len(d):
            runs[m.group(1)][int(m.group(2))] = d
    grid = {}
    if a.grid:
        for line in open(a.grid):
            p = [x.strip() for x in line.split(",")]
            if len(p) >= 3 and p[0] != "run":
                grid[p[0]] = (float(p[1]), float(p[2]))

    per_mode = defaultdict(lambda: defaultdict(list))
    pair_rmse = {}
    cyc_all = []
    print(f"{'run':10s} {'mode':5s} {'inj. loss':>9s} {'age med/p95/max [ms]':>22s} {'ctrl RMSE [cm]':>15s} "
          f"{'grid slot/spacing [cm]':>23s}")
    for run in sorted(runs):
        R = runs[run]
        modes = {int(d[:, 11].max()) for d in R.values()}
        if len(modes) != 1:
            print(f"{run}: robots disagree on the predict flag, skipped"); continue
        mode = "pred" if modes.pop() else "zoh"
        errs, ages, rx, dr = [], [], 0, 0
        for rank, d in R.items():
            cyc_all.append(d[:, 10])
            t = (d[:, 0] - d[0, 0]) / 1000.0
            fol = (d[:, 2] == 0) & (t > a.settle) & (d[:, 4] >= 0)
            if fol.sum() == 0:
                continue
            errs.append(d[fol, 7] / 1000.0)
            ages.append(d[fol, 4])
            rx += d[-1, 5]; dr += d[-1, 6]
        if not errs:
            continue
        e = np.concatenate(errs); g = np.concatenate(ages)
        rmse = float(np.sqrt(np.mean(e ** 2)))
        p_inj = dr / rx if rx else float("nan")
        gs = grid.get(run)
        per_mode[mode]["rmse"].append(rmse); per_mode[mode]["p"].append(p_inj); pair_rmse[run] = rmse
        per_mode[mode]["age_med"].append(float(np.median(g))); per_mode[mode]["age_p95"].append(float(np.percentile(g, 95)))
        per_mode[mode]["age_max"].append(float(g.max()))
        if gs:
            per_mode[mode]["slot"].append(gs[0]); per_mode[mode]["spacing"].append(gs[1])
        print(f"{run:10s} {mode:5s} {p_inj:9.2f} {np.median(g):7.0f}/{np.percentile(g, 95):5.0f}/{g.max():6.0f}"
              f" {100 * rmse:15.1f} {('%.1f / %.1f' % gs) if gs else '-':>23s}")

    print()
    for mode in ("zoh", "pred"):
        M = per_mode.get(mode)
        if not M:
            continue
        s = f"{mode:5s} runs={len(M['rmse'])}  inj. loss {np.mean(M['p']):.2f}  ctrl RMSE {100 * np.mean(M['rmse']):.1f} cm " \
            f"(runs {', '.join('%.1f' % (100 * x) for x in M['rmse'])})"
        if M["slot"]:
            s += f"  grid slot dev {np.mean(M['slot']):.1f} cm (runs {', '.join('%.1f' % x for x in M['slot'])})"
        print(s)
    # paired comparison: zohK is paired with predK (same drop seed, start cell, and robot order)
    pairs = []
    for run in sorted(runs):
        m = re.match(r"zoh(\d+)$", run)
        if m and f"pred{m.group(1)}" in pair_rmse and run in pair_rmse:
            pairs.append((pair_rmse[run], pair_rmse[f"pred{m.group(1)}"], grid.get(run), grid.get(f"pred{m.group(1)}")))
    if pairs:
        dz = np.array([q - z for z, q, _, _ in pairs])
        print(f"\npaired runs: {len(pairs)}; prediction lower in {int((dz < 0).sum())} of {len(pairs)} pairs; "
              f"mean paired reduction {100 * np.mean([1 - q / z for z, q, _, _ in pairs]):.0f}%")
        gp = [(gz[0], gq[0]) for _, _, gz, gq in pairs if gz and gq]
        if gp:
            print(f"grid slot deviation lower with prediction in {sum(q < z for z, q in gp)} of {len(gp)} pairs")
        try:
            from scipy.stats import wilcoxon
            if len(pairs) >= 3:
                print(f"Wilcoxon signed-rank (one-sided) on controller-level RMSE: "
                      f"p = {wilcoxon(dz, alternative='less').pvalue:.3f} (smallest attainable with "
                      f"{len(pairs)} pairs: {0.5 ** len(pairs):.3f})")
        except Exception:
            pass
    if "zoh" in per_mode and "pred" in per_mode:
        z, q = np.array(per_mode["zoh"]["rmse"]), np.array(per_mode["pred"]["rmse"])
        red = 100 * (1 - q.mean() / z.mean())
        sep = q.max() < z.min()
        print(f"\ncontroller-level reduction with prediction: {red:.0f}%  "
              f"({'every prediction run below every ZOH run' if sep else 'runs overlap'})")
        try:
            from scipy.stats import mannwhitneyu
            print(f"Mann-Whitney U (one-sided, unpaired): p = {mannwhitneyu(q, z, alternative='less').pvalue:.3f}")
        except Exception:
            pass
    if cyc_all:
        c = np.concatenate(cyc_all)
        print(f"\ncycle time over {len(c)} cycles: mean {c.mean():.0f} us, p99 {np.percentile(c, 99):.0f} us, "
              f"max {c.max():.0f} us (budget 50000 us at 20 Hz)")

    print("\nFor main.tex:")
    if "zoh" in per_mode and "pred" in per_mode:
        Z, P = per_mode["zoh"], per_mode["pred"]
        print(f"  \\renewcommand{{\\HWlossRuns}}{{{len(pairs) if pairs else min(len(Z['rmse']), len(P['rmse']))}}}")
        if pairs:
            print(f"  \\renewcommand{{\\HWlossWins}}{{{int((dz < 0).sum())}}}")
        print(f"  \\renewcommand{{\\HWlossP}}{{{np.mean(Z['p'] + P['p']):.2f}}}")
        print(f"  \\renewcommand{{\\HWctrlZoh}}{{{100 * np.mean(Z['rmse']):.1f}}}")
        print(f"  \\renewcommand{{\\HWctrlPred}}{{{100 * np.mean(P['rmse']):.1f}}}")
        print(f"  \\renewcommand{{\\HWctrlRed}}{{{100 * (1 - np.mean(P['rmse']) / np.mean(Z['rmse'])):.0f}}}")
        if Z["slot"] and P["slot"]:
            print(f"  \\renewcommand{{\\HWslotZoh}}{{{np.mean(Z['slot']):.1f}}}")
            print(f"  \\renewcommand{{\\HWslotPred}}{{{np.mean(P['slot']):.1f}}}")
        print(f"  \\renewcommand{{\\HWageMax}}{{{max(Z['age_max'] + P['age_max']) / 1000:.1f}}}")
    if cyc_all:
        c = np.concatenate(cyc_all)
        print(f"  \\renewcommand{{\\HWcycMean}}{{{c.mean() / 1000:.2f}}}")
        print(f"  \\renewcommand{{\\HWcycMax}}{{{c.max() / 1000:.2f}}}")


if __name__ == "__main__":
    main()
