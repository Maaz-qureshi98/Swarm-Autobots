"""Hardware metrics from an aruco_tracker.py CSV: ground-truth slot error per follower,
formation RMSE, minimum separation, and (optionally) dead-reckoning drift against a robot log.

    python3 tools/eval_hardware.py trial.csv --formation Y [--leader 0] [--odom robot1_log.csv --robot 1]

Slot error uses the same definition as the paper: the follower's distance to its slot
p_L + R(theta_L)(o_i - o_L), with the true leader pose from the camera.
"""
import argparse, csv
import numpy as np

FORM = {"triangle": [(0, 0), (-0.5, 0), (-1.0, -0.5), (-1.0, 0.5)],
        "Y": [(0, 0), (-0.5, 0), (-1.3, -0.5), (-1.3, 0.5)]}


def load(fn):
    d = {}
    for r in csv.DictReader(open(fn)):
        d.setdefault(int(r["robot"]), []).append((float(r["t"]), float(r["x"]), float(r["y"]), float(r["theta"])))
    return {k: np.array(v) for k, v in d.items()}


def at(track, t):
    return np.array([np.interp(t, track[:, 0], track[:, j]) for j in (1, 2)] +
                    [np.arctan2(np.interp(t, track[:, 0], np.sin(track[:, 3])), np.interp(t, track[:, 0], np.cos(track[:, 3])))])


def evaluate(tr, form, leader=0, settle=3.0):
    F = np.array(FORM[form]); L = tr[leader]
    t0, t1 = L[0, 0] + settle, L[-1, 0]
    ts = np.arange(t0, t1, 0.1)
    out = {}
    for i, T in tr.items():
        if i == leader:
            continue
        e = []
        for t in ts:
            if not (T[0, 0] <= t <= T[-1, 0]):
                continue
            pl = at(L, t); pi = at(T, t)
            o = F[i % 4] - F[leader % 4]
            c, s = np.cos(pl[2]), np.sin(pl[2])
            slot = pl[:2] + np.array([c * o[0] - s * o[1], s * o[0] + c * o[1]])
            e.append(np.hypot(*(pi[:2] - slot)))
        out[i] = np.array(e)
    allr = np.concatenate([v for v in out.values() if len(v)])
    seps = []
    ids = sorted(tr)
    for t in ts:
        P = [at(tr[i], t)[:2] for i in ids if tr[i][0, 0] <= t <= tr[i][-1, 0]]
        for a in range(len(P)):
            for b in range(a + 1, len(P)):
                seps.append(np.hypot(*(P[a] - P[b])))
    return dict(per_robot_rmse={i: float(np.sqrt(np.mean(v ** 2))) for i, v in out.items() if len(v)},
                rmse=float(np.sqrt(np.mean(allr ** 2))), max_err=float(allr.max()),
                min_center_sep=float(min(seps)) if seps else None)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv"); ap.add_argument("--formation", default="Y", choices=list(FORM))
    ap.add_argument("--leader", type=int, default=0)
    a = ap.parse_args()
    r = evaluate(load(a.csv), a.formation, a.leader)
    print(r)
