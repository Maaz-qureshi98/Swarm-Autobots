"""Second-round revision experiments (E15). Results -> results/e15_<part>.json
  calcurve : loss sweep under the hardware-calibrated model (Fig. 4b)
  burst    : constant-twist vs constant-acceleration predictor at burst lengths 1, 4, 12
  locsweep : prediction gain versus localization quality (compass bias std 0-4 deg, slip <= 2%,
             plus an ideal model), ZOH vs prediction at p = 0 and 0.6 (Fig. 4c, d)
  hwplan   : pre-registration of the hardware loss-injection trial (Case 3 scale: first 2.5 m of
             leader travel, p = 0.6, L = 4, ZOH vs prediction, calibrated model); per-run metrics and
             the chance that 3 runs per mode separate the two
Usage: python3 e15_revision2.py calcurve|burst|locsweep [bias ...]|hwplan"""
import json, sys
import numpy as np
from core import Cfg, DEG
from runner import run

CAL = type("C", (Cfg,), dict(compass_bias_std=0.25 * DEG, slip_v=(0.98, 1.0)))
IDEAL = type("C", (Cfg,), dict(compass_bias_std=0.0, compass_noise_std=0.0, slip_v=(1.0, 1.0),
                               slip_w=(1.0, 1.0), slip_noise=(0.0, 0.0)))
part = sys.argv[1]
OUT = {}


def med(x):
    return [float(np.median(x)), float(np.percentile(x, 25)), float(np.percentile(x, 75))]


if part == "calcurve":
    for form in ["triangle", "Y"]:
        for p in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]:
            for m in ["asa", "lpsi_zoh", "lpsi_pred", "consensus"]:
                o, _, _ = run(m, E=100, N=4, form=form, p_loss=p, burst=4.0, seed=11, n_unknown=0, cfg=CAL)
                OUT[f"{form}|{p}|{m}"] = dict(gt=med(o["rmse_gt"]), est=med(o["rmse_est"]),
                                              succ=float(o["success"].mean()))
                print(form, p, m, OUT[f"{form}|{p}|{m}"], flush=True)
elif part == "burst":
    for mname, C in [("nominal", Cfg), ("cal025", CAL), ("ideal", IDEAL)]:
        for form in ["triangle", "Y"]:
            for L in [1.0, 4.0, 12.0]:
                for m in ["lpsi_zoh", "lpsi_pred", "lpsi_ca"]:
                    o, _, _ = run(m, E=100, N=4, form=form, p_loss=0.6, burst=L, seed=12, n_unknown=0, cfg=C)
                    OUT[f"{mname}|{form}|{L}|{m}"] = dict(gt=float(np.median(o["rmse_gt"])),
                                                          est=float(np.median(o["rmse_est"])))
                    print(mname, form, L, m, OUT[f"{mname}|{form}|{L}|{m}"], flush=True)
elif part == "locsweep":
    levels = sys.argv[2:] or ["ideal", "0.0", "0.25", "0.5", "1.0", "2.0", "4.0"]
    for lev in levels:
        C = IDEAL if lev == "ideal" else type("C", (Cfg,), dict(compass_bias_std=float(lev) * DEG, slip_v=(0.98, 1.0)))
        for form in ["triangle", "Y"]:
            for p in [0.0, 0.6]:
                for m in ["lpsi_zoh", "lpsi_pred"]:
                    o, _, _ = run(m, E=100, N=4, form=form, p_loss=p, burst=4.0, seed=11, n_unknown=0, cfg=C)
                    OUT[f"{lev}|{form}|{p}|{m}"] = dict(gt_all=[float(x) for x in o["rmse_gt"]],
                                                        est_all=[float(x) for x in o["rmse_est"]])
                    print(lev, form, p, m, f"gt {np.median(o['rmse_gt']):.4f}", flush=True)
    tag = "_" + "_".join(sys.argv[2:]) if len(sys.argv) > 2 else ""
    json.dump(OUT, open(f"results/e15_locsweep{tag}.json", "w"))
    sys.exit(0)
elif part == "hwplan":
    rng = np.random.default_rng(0)
    for m in ["lpsi_zoh", "lpsi_pred"]:
        _, sim, rec = run(m, E=200, N=4, form="Y", p_loss=0.6, burst=4.0, seed=31, n_unknown=1,
                          record=True, cfg=CAL)
        pos = np.array([r["pos"] for r in rec]); done = np.array([r["done"] for r in rec])
        ee = np.array([r["e_est"] for r in rec]); eg = np.array([r["e_gt"] for r in rec])
        fol = np.array([r["fol"] for r in rec]); t = np.array([r["t"] for r in rec])
        step = np.hypot(*np.diff(pos[:, :, 0], axis=0).transpose(2, 0, 1))
        dist = np.vstack([np.zeros((1, pos.shape[1])), np.cumsum(step, 0)])
        est_rmse, gt_end = [], []
        for e in range(pos.shape[1]):
            k = int(np.argmax(dist[:, e] >= 2.5))
            if dist[k, e] < 2.5:
                continue
            w = (t[:k, e] > 3.0) & ~done[:k, e]
            msk = fol[:k, e] & w[:, None]
            est_rmse.append(float(np.sqrt((ee[:k, e][msk] ** 2).mean())))
            gt_end.append(float(eg[k, e][fol[k, e]].max()))
        OUT[m] = dict(est_rmse=est_rmse, gt_end=gt_end, est=med(est_rmse), gt=med(gt_end), n=len(est_rmse))
        print(m, "ctrl RMSE first 2.5 m", med(est_rmse), "end slot dev", med(gt_end), len(est_rmse), flush=True)
    for key in ["est_rmse", "gt_end"]:
        z, q = np.array(OUT["lpsi_zoh"][key]), np.array(OUT["lpsi_pred"][key])
        for n in [3, 5]:
            win = np.mean([q[rng.integers(len(q), size=n)].mean() < z[rng.integers(len(z), size=n)].mean()
                           for _ in range(20000)])
            OUT[f"p_sep_{key}_{n}"] = float(win)
            print(f"P(mean of {n} prediction runs < mean of {n} ZOH runs) on {key}: {win:.3f}", flush=True)
json.dump(OUT, open(f"results/e15_{part}.json", "w"))
