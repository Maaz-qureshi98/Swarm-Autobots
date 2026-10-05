"""Robustness to simulator mismatch: rerun the loss experiment (p = 0.4, L = 4) with
pessimistic robot and sensor models. -> results/e8_mismatch.json"""
import json, time
import numpy as np
import core
from runner import run
from experiments import pack

DEG = np.pi / 180
BASE = dict(slip_v=core.Cfg.slip_v, slip_w=core.Cfg.slip_w, compass_bias_std=core.Cfg.compass_bias_std,
            tau_m=core.Cfg.tau_m, us_noise=core.Cfg.us_noise, ir_noise=core.Cfg.ir_noise, slip_noise=core.Cfg.slip_noise)
COND = {
    "nominal": {},
    "slip": dict(slip_v=(0.90, 1.00), slip_w=(0.60, 0.95)),
    "compass": dict(compass_bias_std=6.0 * DEG),
    "lag": dict(tau_m=0.30),
    "noise": dict(us_noise=(0.015, 0.03), ir_noise=(0.03, 0.09), slip_noise=(0.09, 0.15)),
}
COND["all"] = {k: v for d in list(COND.values())[1:] for k, v in d.items()}

out = []
for cname, over in COND.items():
    for k, v in BASE.items():
        setattr(core.Cfg, k, v)
    for k, v in over.items():
        setattr(core.Cfg, k, v)
    for form in ["triangle", "Y"]:
        for m in ["asa", "lpsi_zoh", "consensus", "lpsi_pred"]:
            t0 = time.time()
            o, _, _ = run(m, E=100, N=4, form=form, p_loss=0.4, burst=4.0, seed=11, n_unknown=0)
            out.append(dict(cond=cname, form=form, method=m, **pack(o)))
            print(cname, form, m, f"gt {np.median(o['rmse_gt']):.3f} est {np.median(o['rmse_est']):.3f} "
                  f"succ {o['success'].mean():.2f} coll {o['collisions'].mean():.2f} ({time.time()-t0:.0f}s)", flush=True)
    json.dump(out, open("results/e8_mismatch.json", "w"))
for k, v in BASE.items():
    setattr(core.Cfg, k, v)
print("done")
