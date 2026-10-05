"""Parameter sensitivity: election timeout tau_e and barrier rate lambda."""
import json, numpy as np, core
from runner import run
a = lambda o, k: np.asarray(o[k], float)
out = {"tau_e": [], "lam": []}
for te in [1.0, 1.5, 2.0, 3.0]:
    core.Cfg.tau_elect = te
    row = dict(tau_e=te)
    rec, conf = [], []
    for form in ["triangle", "Y"]:
        o, _, _ = run("lpsi_pred", E=100, N=4, form=form, p_loss=0.4, burst=4.0, seed=14, n_unknown=0,
                      fail_time=12.0, election=True, max_time=90.0)
        rec.append(np.nanmedian(a(o, "recover_t"))); 
        o2, _, _ = run("lpsi_pred", E=100, N=4, form=form, p_loss=0.4, burst=4.0, seed=11, n_unknown=0)
        conf.append(np.mean(a(o2, "false_elections")))
    row.update(recover_med=float(np.mean(rec)), spurious=float(np.mean(conf)))
    out["tau_e"].append(row); print(row, flush=True)
core.Cfg.tau_elect = 1.5
for lam in [0.1, 0.25, 0.5, 1.0]:
    core.Cfg.cbf_lam = lam
    coll, succ = [], []
    for form in ["triangle", "Y"]:
        o, _, _ = run("lpsi_pred", E=100, N=4, form=form, p_loss=0.2, burst=4.0, seed=13, n_unknown=2)
        coll.append(np.mean(a(o, "collisions"))); succ.append(np.mean(a(o, "success")))
    row = dict(lam=lam, coll=float(np.mean(coll)), succ=float(np.mean(succ)))
    out["lam"].append(row); print(row, flush=True)
core.Cfg.cbf_lam = 0.25
# formation-aware planning feasibility in the main study
E1 = json.load(open("results/e1_loss.json"))
out["fa_ok"] = float(np.mean([np.mean(r["fa_ok"]) for r in E1]))
print("fa_ok", out["fa_ok"])
json.dump(out, open("results/sensitivity.json", "w"), indent=1)
