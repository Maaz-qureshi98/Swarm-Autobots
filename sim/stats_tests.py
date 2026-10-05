"""Paired significance tests and success-threshold check used in the paper -> results/stats_tests.json.

Wilcoxon signed-rank tests are paired by mission (common random numbers across methods).
"""
import json
import numpy as np
from scipy.stats import wilcoxon

E1 = json.load(open("results/e1_loss.json"))
E7 = json.load(open("results/e7_consensus_a.json"))


def get(src, form, p, method, key):
    for r in src:
        if r["form"] == form and abs(r["p"] - p) < 1e-9 and r["method"] == method and r.get("exp", "e1") == "e1":
            return np.array([np.nan if v is None else v for v in r[key]], float)
    raise KeyError((form, p, method))


def pv(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    return float(wilcoxon(a[m], b[m]).pvalue)


out = {}
for form in ["triangle", "Y"]:
    for p in [0.0, 0.2, 0.6]:
        cp, cz = get(E1, form, p, "lpsi_pred", "rmse_est"), get(E1, form, p, "lpsi_zoh", "rmse_est")
        gp, gz = get(E1, form, p, "lpsi_pred", "rmse_gt"), get(E1, form, p, "lpsi_zoh", "rmse_gt")
        gc = get(E7, form, p, "consensus", "rmse_gt")
        out[f"{form}_{p}"] = dict(ctrl_pred_vs_zoh=pv(cp, cz), gt_pred_vs_zoh=pv(gp, gz),
                                  gt_pred_vs_cons=pv(gp, gc), gt_zoh_vs_cons=pv(gz, gc),
                                  med=[float(np.nanmedian(x)) for x in (gp, gz, gc)])

# success under stricter final-error thresholds at p = 0.2, averaged over both formations
for thr in [0.3, 0.4, 0.5]:
    row = []
    for m in ["asa", "lpsi_zoh", "consensus", "lpsi_pred"]:
        src = E7 if m == "consensus" else E1
        s = []
        for form in ["triangle", "Y"]:
            fin = get(src, form, 0.2, m, "final_gt")
            ok = (fin < thr) & (get(src, form, 0.2, m, "reached") > 0) & (get(src, form, 0.2, m, "collisions") == 0)
            s.append(np.mean(ok))
        row.append(f"{m} {100 * np.mean(s):.0f}%")
    out[f"succ_thr_{thr}"] = row

json.dump(out, open("results/stats_tests.json", "w"), indent=1)
print(json.dumps({k: v for k, v in out.items() if k.startswith("succ")}, indent=1))
