"""Ground-truth error vs compass bias (p = 0.4) -> results/e8b_compass.json"""
import json
import numpy as np
import core
from runner import run
from experiments import pack
DEG = np.pi / 180
out = []
for b in [0.5, 1.0, 2.0, 4.0, 6.0]:
    core.Cfg.compass_bias_std = b * DEG
    for form in ["triangle", "Y"]:
        for m in ["asa", "lpsi_pred"]:
            o, _, _ = run(m, E=100, N=4, form=form, p_loss=0.4, burst=4.0, seed=11, n_unknown=0)
            out.append(dict(bias=b, form=form, method=m, **pack(o)))
            print(b, form, m, f"gt {np.median(o['rmse_gt']):.3f} succ {o['success'].mean():.2f}", flush=True)
core.Cfg.compass_bias_std = 2.0 * DEG
json.dump(out, open("results/e8b_compass.json", "w"))
print("done")
