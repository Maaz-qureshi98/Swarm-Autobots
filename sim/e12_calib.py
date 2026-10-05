"""E10b: Proposition 1 under its own assumptions (no slip, no compass error).
E12: which compass-bias / slip levels reproduce the ground-truth hardware errors at 2-3 m.
Results -> results/e12_calib.json"""
import json
import numpy as np
from runner import run
from core import Cfg, predict_leader, formation, DEG

OUT = {}
taus = np.round(np.arange(0.1, 1.01, 0.1), 2)


class Ideal(Cfg):
    slip_v = (1.0, 1.0); slip_w = (1.0, 1.0); slip_noise = (0.0, 0.0)
    compass_bias_std = 0.0; compass_noise_std = 0.0


def prop1(cfg, seed=24):
    err = {"zoh": {t: [] for t in taus}, "pred": {t: [] for t in taus}}
    for form in ["triangle", "Y"]:
        _, sim, rec = run("lpsi_pred", E=40, N=4, form=form, p_loss=0.0, burst=4.0, seed=seed,
                          n_unknown=0, record=True, cfg=cfg)
        dt = sim.c.dt
        est = np.array([r["est"][:, 0] for r in rec]); u = np.array([r["uact"][:, 0] for r in rec])
        done = np.array([r["done"] for r in rec]); T = len(rec)
        for k0 in range(int(3.0 / dt), T, 2):
            pkt = np.concatenate([est[k0], u[k0]], -1)
            for t in taus:
                k1 = k0 + int(round(t / dt))
                if k1 >= T:
                    continue
                ok = ~done[k1] & (np.abs(u[k0, :, 0]) > 0.02)
                if not ok.any():
                    continue
                pr, _, _ = predict_leader(pkt[ok], t); tr = est[k1, ok]
                for name, P in (("zoh", pkt[ok, :3]), ("pred", pr)):
                    for qq in formation(form, 4)[1:]:
                        def slot(p):
                            c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
                            return p[:, :2] + np.stack([c * qq[0] - s * qq[1], s * qq[0] + c * qq[1]], -1)
                        err[name][t].extend(np.hypot(*(slot(P) - slot(tr)).T).tolist())
    res = {n: {str(t): float(np.median(err[n][t])) for t in taus} for n in err}
    for n in err:
        md = np.array([np.median(err[n][t]) for t in taus])
        res[n + "_slope"] = float(np.polyfit(np.log(taus), np.log(md), 1)[0])
    return res


OUT["prop1_ideal"] = prop1(Ideal)
print("E10b ideal", {k: round(v, 2) for k, v in OUT["prop1_ideal"].items() if "slope" in k},
      {t: (round(OUT['prop1_ideal']['zoh'][str(t)] * 100, 3), round(OUT['prop1_ideal']['pred'][str(t)] * 100, 3)) for t in taus}, flush=True)

pairs = [(0, 1), (1, 2), (1, 3), (2, 3)]


def hwscale(cfg, D_list=(0.0, 2.5), seed=23):
    out = {}
    for form in ["triangle", "Y"]:
        _, sim, rec = run("lpsi_pred", E=100, N=4, form=form, p_loss=0.2, burst=4.0, seed=seed,
                          n_unknown=0, record=True, cfg=cfg)
        pos = np.array([r["pos"] for r in rec]); th = np.array([r["th"][:, 0] for r in rec])
        done = np.array([r["done"] for r in rec]); off = formation(form, 4)
        nom = {pq: float(np.hypot(*(off[pq[0]] - off[pq[1]]))) for pq in pairs}
        step = np.hypot(*np.diff(pos[:, :, 0], axis=0).transpose(2, 0, 1))
        dist = np.vstack([np.zeros((1, pos.shape[1])), np.cumsum(step, 0)])
        for D in D_list:
            sl_, sp_ = [], []
            for e in range(pos.shape[1]):
                k = int(np.argmax(dist[:, e] >= D)) if D > 0 else int(3.0 / sim.c.dt)
                if dist[k, e] < D or done[k, e]:
                    continue
                pl, c_, s_ = pos[k, e, 0], np.cos(th[k, e]), np.sin(th[k, e])
                sl = [pl + np.array([c_ * o[0] - s_ * o[1], s_ * o[0] + c_ * o[1]]) for o in off]
                sl_.append(max(np.hypot(*(pos[k, e, i] - sl[i])) for i in range(1, 4)))
                sp_.append(max(abs(np.hypot(*(pos[k, e, a] - pos[k, e, b])) - nom[(a, b)]) for a, b in pairs))
            out.setdefault(str(D), {"slot": [], "spacing": []})
            out[str(D)]["slot"] += sl_; out[str(D)]["spacing"] += sp_
    return {D: {k: [float(np.median(v)), float(np.percentile(v, 25)), float(np.percentile(v, 75))]
                for k, v in d.items()} for D, d in out.items()}


grid = {}
for bias in [0.25, 0.5, 1.0, 2.0]:
    for slip_lo in [0.99, 0.98, 0.96]:
        C = type("C", (Cfg,), dict(compass_bias_std=bias * DEG, slip_v=(slip_lo, 1.0)))
        r = hwscale(C)
        grid[f"{bias}_{slip_lo}"] = r
        print("E12", bias, slip_lo, {D: (round(v['slot'][0] * 100, 1), round(v['spacing'][0] * 100, 1)) for D, v in r.items()}, flush=True)
OUT["hwscale_grid"] = grid
json.dump(OUT, open("results/e12_calib.json", "w"), indent=1)
