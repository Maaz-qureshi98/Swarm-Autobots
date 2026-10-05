"""Footprint check (E13): the safety and collision experiments repeated with the
robot modelled as its 0.14 m circumscribed disc instead of the 0.12 m nominal disc.
R sets contact detection, the robot discs seen by the range sensors, and the A*
inflation. Same seeds as E1, E2, E9, and the lambda sweep, so results are paired.
Results -> results/e13_footprint.json"""
import json
import numpy as np
import core
from runner import run
from experiments import pack

core.Cfg.R = 0.14
OUT = {"R": core.Cfg.R}

# E2 at 0.14 m: unmapped obstacles near the path (Fig. 5)
e2 = []
for form in ["triangle", "Y"]:
    for nu in [0, 1, 2, 3]:
        for m in ["asa", "pred_nocbf", "pred_stop", "pred_nobyp", "lpsi_pred", "consensus"]:
            o, _, _ = run(m, E=100, N=4, form=form, p_loss=0.2, burst=4.0, seed=13, n_unknown=nu)
            e2.append(dict(form=form, n_unknown=nu, method=m, **pack(o)))
            print("E13-e2", form, nu, m, f"coll {o['collisions'].mean():.2f} succ {o['success'].mean():.2f}", flush=True)
OUT["e2"] = e2

# E9 at 0.14 m: component ablation (Table II)
VARIANTS = [("full", "lpsi_pred", {}), ("no_prediction", "lpsi_zoh", {}),
            ("no_pacing", "lpsi_pred", {"pacing": False}),
            ("no_formation_aware", "lpsi_pred", {"form_aware": False}),
            ("no_recovery", "pred_nobyp", {}), ("no_filter", "pred_nocbf", {})]
e9 = {}
for name, m, kw in VARIANTS:
    agg = {}
    for form in ["triangle", "Y"]:
        o, _, _ = run(m, E=100, N=4, form=form, p_loss=0.4, burst=4.0, seed=21, n_unknown=1, **kw)
        agg[form] = dict(gt=float(np.median(o["rmse_gt"])), est=float(np.median(o["rmse_est"])),
                         coll=float(o["collisions"].mean()), succ=float(o["success"].mean()),
                         reach=float(o["reached"].mean()), gtime=float(np.nanmedian(o["goal_time"])))
    e9[name] = {k: float(np.mean([agg[f][k] for f in agg])) for k in agg["triangle"]}
    print("E13-e9", name, {k: round(v, 3) for k, v in e9[name].items()}, flush=True)
OUT["e9"] = e9

# lambda sweep at 0.14 m
lam_rows = []
for lam in [0.1, 0.25, 0.5, 1.0]:
    core.Cfg.cbf_lam = lam
    coll, succ = [], []
    for form in ["triangle", "Y"]:
        o, _, _ = run("lpsi_pred", E=100, N=4, form=form, p_loss=0.2, burst=4.0, seed=13, n_unknown=2)
        coll.append(np.mean(o["collisions"])); succ.append(np.mean(o["success"]))
    lam_rows.append(dict(lam=lam, coll=float(np.mean(coll)), succ=float(np.mean(succ))))
    print("E13-lam", lam_rows[-1], flush=True)
core.Cfg.cbf_lam = 0.25
OUT["lam"] = lam_rows

# Table I conditions at 0.14 m (collisions and success, no unmapped obstacles)
e1 = []
for form in ["triangle", "Y"]:
    for p in [0.2, 0.6]:
        for m in ["asa", "lpsi_zoh", "lpsi_pred", "consensus"]:
            o, _, _ = run(m, E=100, N=4, form=form, p_loss=p, burst=4.0, seed=11, n_unknown=0)
            e1.append(dict(form=form, p=p, method=m, **pack(o)))
            print("E13-e1", form, p, m, f"gt {np.median(o['rmse_gt']):.3f} coll {o['collisions'].mean():.2f} "
                  f"succ {o['success'].mean():.2f}", flush=True)
OUT["e1"] = e1

json.dump(OUT, open("results/e13_footprint.json", "w"))
