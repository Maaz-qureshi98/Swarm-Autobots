"""Aggregate the matched-recovery batch and the revision batches.
Run after matched_recovery.py and revision_runs.py."""
from pathlib import Path
import json
import numpy as np
ROOT = Path(__file__).resolve().parent
FORMS = ['triangle', 'Y']
rng = np.random.default_rng(20261016)

base = {}
for p in (ROOT / 'results').glob('*_100.json'):
    x = json.loads(p.read_text())
    base[(x['formation'], x['unmapped_obstacles'], x['method'], x['gain'])] = x

out = {}
for n in [1, 2, 3]:
    out[n] = {}
    for method, gain in [('lpsi_pred', .06), ('pred_apf', .06), ('pred_apf_recovery', .01),
                         ('pred_apf_recovery', .03), ('pred_apf_recovery', .06)]:
        rows = [base[f, n, method, gain] for f in FORMS]
        out[n][f'{method}_{gain}'] = {k: float(np.mean([v[k] for v in rows]))
                                      for k in ['contact_free_pct', 'held_pct',
                                                'contact_time_s', 'reach_pct']}
    # stratified paired mission bootstrap, equal weight per formation
    d = [np.array(base[f, n, 'lpsi_pred', .06]['metrics']['success'])
         - np.array(base[f, n, 'pred_apf_recovery', .06]['metrics']['success']) for f in FORMS]
    boots = np.mean([x[rng.integers(0, 100, (4000, 100))].mean(1) for x in d], axis=0) * 100
    out[n]['held_difference_filter_minus_apf006_pp'] = {
        'estimate': float(np.mean(d) * 100), 'ci95': np.quantile(boots, [.025, .975]).tolist()}

(ROOT / 'recovery_summary.json').write_text(json.dumps(out, indent=2))
print(json.dumps(out, indent=2))
