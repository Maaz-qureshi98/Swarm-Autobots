"""Additional experiments: component ablation (E9), empirical check of Proposition 1 (E10),
and simulation at the hardware scale of 2-3 m paths (E11). Results -> results/e9_review.json"""
import json
import numpy as np
from runner import run
from core import predict_leader, formation, wrap

OUT = {}


def med_iqr(x):
    x = np.asarray(x, float); x = x[np.isfinite(x)]
    return [float(np.median(x)), float(np.percentile(x, 25)), float(np.percentile(x, 75))]


# ---------------------------------------------------------------- E9: component ablation
VARIANTS = [
    ("full", "lpsi_pred", {}),
    ("no_prediction", "lpsi_zoh", {}),
    ("no_pacing", "lpsi_pred", {"pacing": False}),
    ("no_formation_aware", "lpsi_pred", {"form_aware": False}),
    ("no_recovery", "pred_nobyp", {}),
    ("no_filter", "pred_nocbf", {}),
]
e9 = {}
for name, m, kw in VARIANTS:
    agg = {}
    for form in ["triangle", "Y"]:
        o, _, _ = run(m, E=100, N=4, form=form, p_loss=0.4, burst=4.0, seed=21, n_unknown=1, **kw)
        agg[form] = dict(gt=float(np.median(o["rmse_gt"])), est=float(np.median(o["rmse_est"])),
                         coll=float(o["collisions"].mean()), succ=float(o["success"].mean()),
                         reach=float(o["reached"].mean()),
                         gtime=float(np.nanmedian(o["goal_time"])), minsep=float(np.median(o["min_sep"])))
    e9[name] = {k: float(np.mean([agg[f][k] for f in agg])) for k in agg["triangle"]}
    e9[name]["per_form"] = agg
    print("E9", name, {k: round(v, 3) for k, v in e9[name].items() if k != "per_form"}, flush=True)
OUT["e9_ablation"] = e9

# ---------------------------------------------------------------- E10: Proposition 1 check
# Reference error of the predicted vs held leader packet as a function of packet age, measured
# on recorded leader trajectories (leader estimate = reference that followers track).
taus = np.round(np.arange(0.1, 1.01, 0.1), 2)
q_by_form = {f: formation(f, 4)[1:] for f in ["triangle", "Y"]}
err = {"zoh": {t: [] for t in taus}, "pred": {t: [] for t in taus}}
for form in ["triangle", "Y"]:
    _, sim, rec = run("lpsi_pred", E=40, N=4, form=form, p_loss=0.0, burst=4.0, seed=22,
                      n_unknown=0, record=True)
    dt = sim.c.dt
    est = np.array([r["est"][:, 0] for r in rec])      # T,E,3 leader estimate
    u = np.array([r["uact"][:, 0] for r in rec])       # T,E,2 transmitted twist
    done = np.array([r["done"] for r in rec])          # T,E
    T = len(rec)
    Q = q_by_form[form]
    for k0 in range(int(3.0 / dt), T, 2):              # packet every broadcast period after settling
        pkt = np.concatenate([est[k0], u[k0]], -1)     # E,5
        for t in taus:
            k1 = k0 + int(round(t / dt))
            if k1 >= T:
                continue
            ok = ~done[k1] & (np.abs(u[k0, :, 0]) > 0.02)
            if not ok.any():
                continue
            pr, _, _ = predict_leader(pkt[ok], t)
            tr = est[k1, ok]
            for name, P in (("zoh", pkt[ok, :3]), ("pred", pr)):
                for qq in Q:
                    def slot(p):
                        c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
                        return p[:, :2] + np.stack([c * qq[0] - s * qq[1], s * qq[0] + c * qq[1]], -1)
                    err[name][t].extend(np.hypot(*(slot(P) - slot(tr)).T).tolist())
e10 = {n: {str(t): [float(np.median(err[n][t])), float(np.percentile(err[n][t], 95))] for t in taus} for n in err}
lt = np.log(taus)
for n in err:
    md = np.array([np.median(err[n][t]) for t in taus])
    e10[n + "_slope"] = float(np.polyfit(lt, np.log(md), 1)[0])
print("E10", {n: e10[n + "_slope"] for n in err},
      {t: (round(np.median(err['zoh'][t]) * 100, 2), round(np.median(err['pred'][t]) * 100, 2)) for t in taus}, flush=True)
OUT["e10_prop1"] = e10

# ---------------------------------------------------------------- E11: hardware-scale comparison
# Ground-truth slot error and centre-spacing deviation when the leader has driven 2-3 m,
# the path length of the hardware runs (no unmapped obstacles, p = 0.2).
pairs = [(0, 1), (1, 2), (1, 3), (2, 3)]
e11 = {}
for form in ["triangle", "Y"]:
    _, sim, rec = run("lpsi_pred", E=100, N=4, form=form, p_loss=0.2, burst=4.0, seed=23,
                      n_unknown=0, record=True)
    pos = np.array([r["pos"] for r in rec])            # T,E,N,2
    done = np.array([r["done"] for r in rec])
    off = formation(form, 4)
    nom = {pq: float(np.hypot(*(off[pq[0]] - off[pq[1]]))) for pq in pairs}
    step = np.hypot(*np.diff(pos[:, :, 0], axis=0).transpose(2, 0, 1))
    dist = np.vstack([np.zeros((1, pos.shape[1])), np.cumsum(step, 0)])
    th = np.array([r["th"][:, 0] for r in rec])
    res = {}
    for D in [0.0, 1.0, 2.0, 2.5, 3.0]:
        slot_max, sp_max = [], []
        for e in range(pos.shape[1]):
            k = int(np.argmax(dist[:, e] >= D)) if D > 0 else int(3.0 / sim.c.dt)
            if dist[k, e] < D or done[k, e]:
                continue
            pl, c_, s_ = pos[k, e, 0], np.cos(th[k, e]), np.sin(th[k, e])
            sl = [pl + np.array([c_ * o[0] - s_ * o[1], s_ * o[0] + c_ * o[1]]) for o in off]
            slot_max.append(max(np.hypot(*(pos[k, e, i] - sl[i])) for i in range(1, 4)))
            sp_max.append(max(abs(np.hypot(*(pos[k, e, a] - pos[k, e, b])) - nom[(a, b)]) for a, b in pairs))
        res[str(D)] = dict(slot=med_iqr(slot_max), spacing=med_iqr(sp_max), n=len(slot_max))
    e11[form] = res
    print("E11", form, {D: (round(v["slot"][0] * 100, 1), round(v["spacing"][0] * 100, 1), v["n"]) for D, v in res.items()}, flush=True)
OUT["e11_hwscale"] = e11

json.dump(OUT, open("results/e9_review.json", "w"), indent=1)
