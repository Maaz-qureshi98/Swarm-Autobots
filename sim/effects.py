"""Effect sizes for the paper: relative reduction of the median RMSE (prediction vs ZOH) with paired
bootstrap 95% confidence intervals (missions are paired through common random numbers).
Reads results/e14_calib_*.json and results/e15_locsweep*.json -> results/effects.json"""
import json, glob
import numpy as np

rng = np.random.default_rng(0)


def red_ci(z, q, B=4000):
    z, q = np.asarray(z), np.asarray(q)
    n = min(len(z), len(q)); z, q = z[:n], q[:n]
    pt = 1 - np.median(q) / np.median(z)
    idx = rng.integers(n, size=(B, n))
    bs = 1 - np.median(q[idx], 1) / np.median(z[idx], 1)
    return [float(pt), float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))]


OUT = {}
for m in ["nominal", "cal025", "cal05", "ideal"]:
    d = json.load(open(f"results/e14_calib_{m}.json"))
    for f in ["triangle", "Y"]:
        for p in [0.2, 0.4, 0.6]:
            for key in ["gt_all", "est_all"]:
                OUT[f"{m}|{f}|{p}|{key}"] = red_ci(d[f"{m}|{f}|{p}|lpsi_zoh"][key], d[f"{m}|{f}|{p}|lpsi_pred"][key])
loc = {}
for fn in glob.glob("results/e15_locsweep_*.json"):
    loc.update(json.load(open(fn)))
for k in loc:
    lev, f, p, m = k.split("|")
    if m == "lpsi_zoh":
        OUT[f"loc|{lev}|{f}|{p}|gt_all"] = red_ci(loc[k]["gt_all"], loc[f"{lev}|{f}|{p}|lpsi_pred"]["gt_all"])
json.dump(OUT, open("results/effects.json", "w"), indent=0)
for k, v in OUT.items():
    if "|0.6|" in k:
        print(k, " %.0f%% [%.0f, %.0f]" % tuple(100 * np.array(v)))
