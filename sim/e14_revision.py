"""Revision experiments (E14). Results -> results/e14_<part>.json
  calib  : prediction and stronger predictor baselines (constant velocity, constant
           acceleration) under the nominal, hardware-calibrated, and ideal localization models
  safety : contact duration, and a range-based potential-field baseline, 0.142 m footprint
  elect  : time with two coexisting leaders and contacts during it, 0.142 m footprint
  table1cal: calibrated-model column of Table I
Usage: python3 e14_revision.py calib [model ...] | table1cal | safety | elect"""
import json, sys
import numpy as np
import core
from core import Cfg, DEG
from runner import run


def summ(o):
    f = lambda x: [float(v) for v in np.asarray(x, float)]
    return dict(gt=float(np.median(o["rmse_gt"])), est=float(np.median(o["rmse_est"])),
                succ=float(o["success"].mean()), coll=float(o["collisions"].mean()),
                contact_free=float((o["collisions"] == 0).mean()),
                contact_time=float(o["contact_time"].mean()),
                contact_time_hit=float(np.median(o["contact_time"][o["collisions"] > 0])) if (o["collisions"] > 0).any() else 0.0,
                reach=float(o["reached"].mean()),
                gt_all=f(o["rmse_gt"]), est_all=f(o["rmse_est"]))


part = sys.argv[1]
OUT = {}
if part == "calib":
    models = {
        "nominal": Cfg,
        "cal025": type("C", (Cfg,), dict(compass_bias_std=0.25 * DEG, slip_v=(0.98, 1.0))),
        "cal05": type("C", (Cfg,), dict(compass_bias_std=0.5 * DEG, slip_v=(0.98, 1.0))),
        "ideal": type("C", (Cfg,), dict(compass_bias_std=0.0, compass_noise_std=0.0, slip_v=(1.0, 1.0),
                                       slip_w=(1.0, 1.0), slip_noise=(0.0, 0.0))),
    }
    sel = sys.argv[2:] or list(models)
    for mname in sel:
        C = models[mname]
        for form in ["triangle", "Y"]:
            for p in [0.0, 0.2, 0.4, 0.6]:
                for m in ["lpsi_zoh", "lpsi_pred", "lpsi_cv", "lpsi_ca", "consensus"]:
                    o, _, _ = run(m, E=100, N=4, form=form, p_loss=p, burst=4.0, seed=11, n_unknown=0, cfg=C)
                    OUT[f"{mname}|{form}|{p}|{m}"] = summ(o)
                    print(mname, form, p, m, f"gt {np.median(o['rmse_gt']):.4f} est {np.median(o['rmse_est']):.4f} "
                          f"succ {o['success'].mean():.2f}", flush=True)
elif part == "table1cal":
    # calibrated-model column of Table I (0.142 m disc, p = 0.6)
    core.Cfg.R = 0.142
    C = type("C", (Cfg,), dict(compass_bias_std=0.25 * DEG, slip_v=(0.98, 1.0)))
    for form in ["triangle", "Y"]:
        for m in ["asa", "lpsi_zoh", "lpsi_pred", "consensus"]:
            o, _, _ = run(m, E=100, N=4, form=form, p_loss=0.6, burst=4.0, seed=11, n_unknown=0, cfg=C)
            OUT[f"{form}|{m}"] = summ(o)
            print(form, m, f"gt {np.median(o['rmse_gt']):.4f} coll {o['collisions'].mean():.2f} succ {o['success'].mean():.2f}", flush=True)
elif part == "safety":
    core.Cfg.R = 0.142
    for form in ["triangle", "Y"]:
        for nu in [1, 2, 3]:
            for m, kw in [("lpsi_pred", {}), ("pred_stop", {}), ("pred_nobyp", {}), ("pred_nocbf", {}),
                          ("pred_apf", dict(k=0.01)), ("pred_apf", dict(k=0.03)), ("pred_apf", dict(k=0.06))]:
                if m == "pred_apf":
                    core.apf_correction.__defaults__ = (None, kw["k"], 0.35)
                o, _, _ = run(m, E=100, N=4, form=form, p_loss=0.2, burst=4.0, seed=13, n_unknown=nu)
                key = m + (f"_k{kw['k']}" if kw else "")
                OUT[f"{form}|{nu}|{key}"] = summ(o)
                print(form, nu, key, f"free {(o['collisions'] == 0).mean():.2f} coll {o['collisions'].mean():.2f} "
                      f"ctime {o['contact_time'].mean():.2f} succ {o['success'].mean():.2f}", flush=True)
elif part == "elect":
    core.Cfg.R = 0.142
    for form in ["triangle", "Y"]:
        for p in [0.0, 0.2, 0.4, 0.6]:
            o, _, _ = run("lpsi_pred", E=100, N=4, form=form, p_loss=p, burst=4.0, seed=14,
                          n_unknown=0, fail_time=12.0, election=True, max_time=90.0)
            r = summ(o)
            r.update(spurious=float(o["false_elections"].mean()),
                     multi_time=float(o["multi_lead_time"].mean()),
                     multi_time_max=float(o["multi_lead_time"].max()),
                     multi_coll=float(o["multi_lead_coll"].sum()),
                     recover=float(np.nanmedian(o["recover_t"])))
            OUT[f"{form}|{p}"] = r
            print(form, p, {k: round(v, 3) for k, v in r.items() if not isinstance(v, list)}, flush=True)
tag = "_" + "_".join(sys.argv[2:]) if len(sys.argv) > 2 else ""
json.dump(OUT, open(f"results/e14_{part}{tag}.json", "w"))
