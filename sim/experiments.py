"""All simulation experiments for the paper. Results -> results/*.json"""
import json, sys, time
import numpy as np
from runner import run

KEYS = ["switch_lat", "rmse_gt", "rmse_est", "max_gt", "collisions", "min_sep", "interv", "reached",
        "goal_time", "final_gt", "success", "false_elections", "recover_t", "fa_ok"]


def pack(o):
    return {k: [None if (isinstance(x, float) and np.isnan(x)) else float(x) for x in o[k]] for k in KEYS}


def save(name, data):
    json.dump(data, open(f"results/{name}.json", "w"))


def e1_loss_sweep(E=100):
    out = []
    for form in ["triangle", "Y"]:
        for p in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]:
            for m in ["asa", "lpsi_zoh", "lpsi_pred"]:
                o, _, _ = run(m, E=E, N=4, form=form, p_loss=p, burst=4.0, seed=11, n_unknown=0)
                out.append(dict(form=form, p=p, method=m, **pack(o)))
                print("E1", form, p, m, f"est {np.median(o['rmse_est']):.3f} succ {o['success'].mean():.2f}", flush=True)
    save("e1_loss", out)


def e1b_burst(E=100):
    out = []
    for form in ["triangle", "Y"]:
        for L in [1.0, 4.0, 8.0, 12.0]:
            for m in ["lpsi_zoh", "lpsi_pred"]:
                o, _, _ = run(m, E=E, N=4, form=form, p_loss=0.4, burst=L, seed=12, n_unknown=0)
                out.append(dict(form=form, burst=L, method=m, **pack(o)))
                print("E1b", form, L, m, f"est {np.median(o['rmse_est']):.3f}", flush=True)
    save("e1b_burst", out)


def e2_safety(E=100):
    out = []
    for form in ["triangle", "Y"]:
        for nu in [0, 1, 2, 3]:
            for m in ["asa", "pred_nocbf", "pred_stop", "pred_nobyp", "lpsi_pred"]:
                o, _, _ = run(m, E=E, N=4, form=form, p_loss=0.2, burst=4.0, seed=13, n_unknown=nu)
                out.append(dict(form=form, n_unknown=nu, method=m, **pack(o)))
                print("E2", form, nu, m, f"coll {o['collisions'].mean():.2f} succ {o['success'].mean():.2f}", flush=True)
    save("e2_safety", out)


def e3_election(E=100):
    out = []
    for form in ["triangle", "Y"]:
        for p in [0.0, 0.2, 0.4, 0.6]:
            for el in [True, False]:
                o, _, _ = run("lpsi_pred", E=E, N=4, form=form, p_loss=p, burst=4.0, seed=14,
                              n_unknown=0, fail_time=12.0, election=el, max_time=90.0)
                out.append(dict(form=form, p=p, election=el, **pack(o)))
                print("E3", form, p, el, f"reach {o['reached'].mean():.2f} rec {np.nanmedian(o['recover_t']) if np.isfinite(o['recover_t']).any() else np.nan:.2f}", flush=True)
    save("e3_election", out)


def e4_scale(E=40):
    out = []
    for form in ["triangle", "Y"]:
        for N in [4, 8, 12, 16, 20]:
            for m in ["asa", "lpsi_pred"]:
                o, _, _ = run(m, E=E, N=N, form=form, p_loss=0.2, burst=4.0, seed=15, n_obs=0, n_unknown=0)
                out.append(dict(form=form, N=N, method=m, **pack(o)))
                print("E4", form, N, m, f"est {np.median(o['rmse_est']):.3f} succ {o['success'].mean():.2f}", flush=True)
    save("e4_scale", out)


def e5_records(E=100):
    """Ground-truth vs dead-reckoning drift and example trajectories."""
    res = {}
    for form in ["triangle", "Y"]:
        o, sim, rec = run("lpsi_pred", E=E, N=4, form=form, p_loss=0.2, burst=4.0, seed=16,
                          n_unknown=0, record=True)
        pos = np.array([r["pos"] for r in rec]); est = np.array([r["est"] for r in rec])
        err = np.hypot(*(est[..., :2] - pos).transpose(3, 0, 1, 2))           # T,E,N
        dist = np.concatenate([np.zeros((1, E, 4)), np.cumsum(np.hypot(*np.diff(pos, axis=0).transpose(3, 0, 1, 2)), 0)])
        bins = np.arange(0, 9.01, 0.5)
        idx = np.digitize(dist, bins) - 1
        med, q25, q75 = [], [], []
        for b in range(len(bins) - 1):
            v = err[idx == b]
            med.append(float(np.median(v)) if v.size else None)
            q25.append(float(np.percentile(v, 25)) if v.size else None)
            q75.append(float(np.percentile(v, 75)) if v.size else None)
        # one example episode with full traces
        e = int(np.argsort(np.abs(o["rmse_est"] - np.median(o["rmse_est"])))[0])
        ex = dict(pos=pos[:, e].tolist(), est=est[:, e, :, :2].tolist(), th=np.array([r["th"][e] for r in rec]).tolist(),
                  t=[float(r["t"][e]) for r in rec], e_gt=np.array([r["e_gt"][e] for r in rec]).tolist(),
                  obs=sim.obs_list[e], path=sim.paths[e].tolist(), W=sim.W, H=sim.H,
                  offsets=sim.offsets[e].tolist(), rmse_est=float(o["rmse_est"][e]), rmse_gt=float(o["rmse_gt"][e]))
        res[form] = dict(bins=bins.tolist(), med=med, q25=q25, q75=q75, example=ex)
        print("E5", form, "done", flush=True)
    save("e5_drift", res)


def e6_switch(E=100):
    """On-line pattern switching commanded through the leader broadcast."""
    out = []
    for a, b in [("triangle", "Y"), ("Y", "triangle")]:
        for p in [0.0, 0.2, 0.4, 0.6]:
            o, _, _ = run("lpsi_pred", E=E, N=4, form=a, switch_to=b, switch_time=12.0, p_loss=p,
                          burst=4.0, seed=18, n_unknown=0)
            out.append(dict(src=a, dst=b, p=p, **pack(o)))
            print("E6", a, b, p, f"lat {np.nanmedian(o['switch_lat']):.2f}", flush=True)
    save("e6_switch", out)


def e7_consensus(part="a", E=100):
    """Published baseline (consensus formation control) through E1, E2 and E4."""
    out = []
    if part == "a":
        for form in ["triangle", "Y"]:
            for p in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]:
                o, _, _ = run("consensus", E=E, N=4, form=form, p_loss=p, burst=4.0, seed=11, n_unknown=0)
                out.append(dict(exp="e1", form=form, p=p, method="consensus", **pack(o)))
                print("E7a", form, p, f"gt {np.median(o['rmse_gt']):.3f}", flush=True)
            for nu in [0, 1, 2, 3]:
                o, _, _ = run("consensus", E=E, N=4, form=form, p_loss=0.2, burst=4.0, seed=13, n_unknown=nu)
                out.append(dict(exp="e2", form=form, n_unknown=nu, method="consensus", **pack(o)))
                print("E7a-e2", form, nu, f"coll {o['collisions'].mean():.2f}", flush=True)
    else:
        for form in ["triangle", "Y"]:
            for N in [4, 8, 12, 16, 20]:
                o, _, _ = run("consensus", E=40, N=N, form=form, p_loss=0.2, burst=4.0, seed=15, n_obs=0, n_unknown=0)
                out.append(dict(exp="e4", form=form, N=N, method="consensus", **pack(o)))
                print("E7b", form, N, f"gt {np.median(o['rmse_gt']):.3f}", flush=True)
    save(f"e7_consensus_{part}", out)


def e3_example():
    """Single leader-failure episode trace for the timeline figure."""
    o, sim, rec = run("lpsi_pred", E=10, N=4, form="Y", p_loss=0.2, burst=4.0, seed=17,
                      n_unknown=0, fail_time=12.0, election=True, record=True, max_time=90.0)
    e = int(np.nanargmin(np.abs(o["recover_t"] - np.nanmedian(o["recover_t"]))))
    ex = dict(t=[float(r["t"][e]) for r in rec], e_gt=np.array([r["e_gt"][e] for r in rec]).tolist(),
              fol=np.array([r["fol"][e] for r in rec]).tolist(), leader=np.array([r["leader"][e] for r in rec]).tolist(),
              is_leader=np.array([r["is_leader"][e] for r in rec]).tolist(), alive=np.array([r["alive"][e] for r in rec]).tolist(),
              pos=np.array([r["pos"][e] for r in rec]).tolist(), obs=sim.obs_list[e], path=sim.paths[e].tolist(),
              W=sim.W, H=sim.H, recover_t=float(o["recover_t"][e]))
    save("e3_example", ex)


if __name__ == "__main__":
    which = sys.argv[1:] or ["e1", "e1b", "e2", "e3", "e3x", "e4", "e5"]
    F = dict(e7a=lambda: e7_consensus("a"), e7b=lambda: e7_consensus("b"), e6=e6_switch, e1=e1_loss_sweep, e1b=e1b_burst, e2=e2_safety, e3=e3_election, e3x=e3_example, e4=e4_scale, e5=e5_records)
    for w in which:
        t0 = time.time(); F[w](); print(w, "took", round(time.time() - t0), "s", flush=True)
