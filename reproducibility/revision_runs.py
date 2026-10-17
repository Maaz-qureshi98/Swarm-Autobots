"""Revision experiments.
(a) Prediction in the low-error settings: CT vs ZOH at p=0 and p=0.6.
(b) Conservative planar weighting of the range-restriction filter.
(c) Potential-field gains beyond the original sweep, with shared recovery.
Run from this directory:  python3 revision_runs.py
Outputs: results_revision/*.json and revision_summary.json
"""
from pathlib import Path
import sys, json, time
import numpy as np
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'sim'))
import core
from runner import run

DEG = np.pi / 180
OUT = ROOT / 'results_revision'; OUT.mkdir(exist_ok=True)
core.METHODS['pred_apf_recovery'] = dict(base='lpsi', pred=True, cbf=False,
                                         apf=True, shared_recovery=True)
NOMINAL = dict(R=core.Cfg.R, compass_bias_std=core.Cfg.compass_bias_std,
               slip_v=core.Cfg.slip_v, cbf_plane=core.Cfg.cbf_plane)


def restore():
    for k, v in NOMINAL.items():
        setattr(core.Cfg, k, v)


def name_of(tag, form, method, n_unknown, p_loss, gain, bias, plane):
    return (f"{tag}_{form}_{method}_{n_unknown}_{p_loss:g}_"
            f"{gain if gain else 0:g}_{bias if bias else 0:g}_{int(plane)}.json")


def execute(tag, method, form, *, radius, p_loss, n_unknown, bias=None, slip_lo=None,
            plane=False, gain=None, episodes=100, seed=13):
    nm = name_of(tag, form, method, n_unknown, p_loss, gain, bias, plane)
    if (OUT / nm).exists():
        return json.loads((OUT / nm).read_text())
    restore()
    core.Cfg.R = radius
    core.Cfg.cbf_plane = plane
    if bias is not None:
        core.Cfg.compass_bias_std = bias * DEG
    if slip_lo is not None:
        core.Cfg.slip_v = (slip_lo, 1.0)
    if gain is not None:
        core.apf_correction.__defaults__ = (None, gain, 0.35)
    t = time.perf_counter()
    o, _, _ = run(method, E=episodes, N=4, form=form, p_loss=p_loss, burst=4.0,
                  seed=seed, n_unknown=n_unknown)
    res = dict(tag=tag, method=method, formation=form, radius=radius, p_loss=p_loss,
               n_unknown=n_unknown, bias_deg=bias, slip_lo=slip_lo, plane=plane,
               gain=gain, episodes=episodes, seed=seed,
               rmse_gt_median=float(np.median(o['rmse_gt'])),
               rmse_est_median=float(np.median(o['rmse_est'])),
               contact_free_pct=100 * float(np.mean(o['collisions'] == 0)),
               held_pct=100 * float(np.mean(o['success'])),
               contact_time_s=float(np.mean(o['contact_time'])),
               reach_pct=100 * float(np.mean(o['reached'])),
               metrics={k: np.asarray(o[k]).tolist() for k in
                        ['rmse_gt', 'rmse_est', 'success', 'collisions',
                         'contact_time', 'reached']},
               elapsed_s=time.perf_counter() - t)
    (OUT / nm).write_text(json.dumps(res, indent=1))
    print(f"{nm:70s} gt {res['rmse_gt_median']:.3f} free {res['contact_free_pct']:5.1f} "
          f"held {res['held_pct']:5.1f} ({res['elapsed_s']:.0f}s)", flush=True)
    restore()
    return res


JOBS = (
    [dict(tag='hwregime', method=m, form=f, radius=0.12, p_loss=p, n_unknown=0,
          bias=b, slip_lo=0.99)
     for b in [0.25, 0.5] for p in [0.0, 0.6]
     for m in ['lpsi_zoh', 'lpsi_pred'] for f in ['triangle', 'Y']]
    + [dict(tag='planefilter', method='lpsi_pred', form=f, radius=0.142, p_loss=0.2,
            n_unknown=n, plane=True, gain=0.06)
       for f in ['triangle', 'Y'] for n in [1, 2, 3]]
    + [dict(tag='apfsweep', method='pred_apf_recovery', form=f, radius=0.142,
            p_loss=0.2, n_unknown=n, gain=k)
       for f in ['triangle', 'Y'] for n in [1, 2, 3] for k in [0.10, 0.15, 0.20]]
)


def gain_ci(ct, zoh, rng, draws=4000):
    est = 1 - np.mean([np.median(c) for c in ct]) / np.mean([np.median(z) for z in zoh])
    boots = []
    for _ in range(draws):
        num = den = 0.0
        for c, z in zip(ct, zoh):
            idx = rng.integers(0, len(c), len(c))
            num += np.median(np.asarray(c)[idx]); den += np.median(np.asarray(z)[idx])
        boots.append(1 - num / den)
    return float(est), [float(q) for q in np.quantile(boots, [.025, .975])]


def summarize():
    r = {}
    for p in OUT.glob('*.json'):
        x = json.loads(p.read_text())
        r[(x['tag'], x['formation'], x['n_unknown'], x['gain'], x['bias_deg'],
           x['p_loss'], x['method'])] = x
    rng = np.random.default_rng(20261017)
    s = {'low_error_regime': {}}
    for b in [0.25, 0.5]:
        for p in [0.0, 0.6]:
            ct = [r[('hwregime', f, 0, None, b, p, 'lpsi_pred')]['metrics']['rmse_gt'] for f in ['triangle', 'Y']]
            zoh = [r[('hwregime', f, 0, None, b, p, 'lpsi_zoh')]['metrics']['rmse_gt'] for f in ['triangle', 'Y']]
            est, ci = gain_ci(ct, zoh, rng)
            s['low_error_regime'][f'bias{b}_p{p}'] = dict(
                ct_rmse=float(np.mean([np.median(c) for c in ct])),
                zoh_rmse=float(np.mean([np.median(z) for z in zoh])), gain=est, ci95=ci)
    s['plane_filter'] = {n: {k: float(np.mean([r[('planefilter', f, n, 0.06, None, 0.2, 'lpsi_pred')][k]
                                               for f in ['triangle', 'Y']]))
                             for k in ['contact_free_pct', 'held_pct', 'contact_time_s', 'reach_pct']}
                         for n in [1, 2, 3]}
    s['apf_extended_gains'] = {f'{k:g}': {n: {m: float(np.mean(
        [r[('apfsweep', f, n, k, None, 0.2, 'pred_apf_recovery')][m] for f in ['triangle', 'Y']]))
        for m in ['contact_free_pct', 'held_pct', 'contact_time_s']} for n in [1, 2, 3]}
        for k in [0.10, 0.15, 0.20]}
    (ROOT / 'revision_summary.json').write_text(json.dumps(s, indent=2))
    return s


if __name__ == '__main__':
    todo = [j for j in JOBS if not (OUT / name_of(j['tag'], j['form'], j['method'],
            j['n_unknown'], j['p_loss'], j.get('gain'), j.get('bias'), j.get('plane', False))).exists()]
    print(f'{len(todo)} of {len(JOBS)} remaining', flush=True)
    for j in todo:
        execute(**j)
    if not todo:
        print(json.dumps(summarize(), indent=2))
