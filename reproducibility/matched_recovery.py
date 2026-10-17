"""Paired APF/filter obstacle-response comparison with a shared recovery layer.
Run from this directory:  python3 matched_recovery.py
Outputs: results/<formation>_<obstacles>_<method>_<gain>_<episodes>.json
"""
from pathlib import Path
import sys, json, time, argparse
import numpy as np
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'sim'))
import core
from runner import run


def execute(form, obstacles, method, gain, episodes=100):
    core.Cfg.R = 0.142
    core.METHODS['pred_apf_recovery'] = dict(base='lpsi', pred=True, cbf=False,
                                             apf=True, shared_recovery=True)
    core.apf_correction.__defaults__ = (None, gain, 0.35)
    t = time.perf_counter()
    out, _, _ = run(method, E=episodes, N=4, form=form, p_loss=0.2, burst=4.,
                    seed=13, n_unknown=obstacles)
    arrays = {k: np.where(np.isfinite(np.asarray(v)), np.asarray(v), None).tolist()
              for k, v in out.items()}
    result = dict(formation=form, unmapped_obstacles=obstacles, method=method, gain=gain,
                  episodes=episodes, seed=13, loss=0.2, burst=4., radius=0.142,
                  contact_free_pct=100 * float(np.mean(out['collisions'] == 0)),
                  held_pct=100 * float(np.mean(out['success'])),
                  contact_time_s=float(np.mean(out['contact_time'])),
                  reach_pct=100 * float(np.mean(out['reached'])),
                  metrics=arrays, elapsed_s=time.perf_counter() - t)
    dest = ROOT / 'results'; dest.mkdir(exist_ok=True)
    name = f'{form}_{obstacles}_{method}_{gain:g}_{episodes}.json'
    (dest / name).write_text(json.dumps(result, indent=2))
    print(name, 'free', result['contact_free_pct'], 'held', result['held_pct'],
          'seconds', round(result['elapsed_s'], 1), flush=True)
    return result


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', type=str, default=None, help='comma-separated formations')
    a = ap.parse_args()
    forms = a.only.split(',') if a.only else ['triangle', 'Y']
    for f in forms:
        for n in [1, 2, 3]:
            for m, k in [('lpsi_pred', .06), ('pred_apf', .06),
                         ('pred_apf_recovery', .01), ('pred_apf_recovery', .03),
                         ('pred_apf_recovery', .06)]:
                if (ROOT / 'results' / f'{f}_{n}_{m}_{k:g}_100.json').exists():
                    continue
                execute(f, n, m, k)
