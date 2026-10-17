# Reproduction package

```
SWARM AUTOBOTS - TMECH REPRODUCTION PACKAGE
Regenerated 17 October 2026 (the earlier copy was lost; this rebuild clones
the same upstream commit and reruns every batch from its recorded seed).

SOURCE
https://github.com/Maaz-qureshi98/Swarm-Autobots  commit d17530b
The scripts import the simulator from ../sim in this repository. The two
changes recorded in revision_core.patch are already applied to sim/core.py:
  1. control_step honours a shared_recovery method flag, so the potential-field
     configurations can use the SAME recovery layer as the range filter
     (leader rotate-to-clear and follower rotate-and-advance, identical
     thresholds, turn selection, hysteresis, and guard on committed advance).
  2. Cfg.cbf_plane selects the conservative planar weighting on the oblique
     rays, lam*h*cos(beta)/T instead of lam*h/(T cos beta). Default False
     reproduces the shipped rule, including its association order, so every
     archived number is bit-identical.
Legacy identifiers such as cbf_filter are names, not claims of safety
invariance. The filter is a speed restriction, not a control barrier function.

REQUIREMENTS
Python 3 and NumPy. Nothing else.

BATCHES
  python3 matched_recovery.py     30 conditions: triangle/Y, 1-3 unmapped
                                  obstacles, 100 missions each, seed 13,
                                  loss 0.2, burst 4, 0.142 m footprint.
                                  Configurations: the range-filter controller;
                                  APF at k=0.06 without recovery; APF with the
                                  shared recovery layer at k=0.01, 0.03, 0.06.
  python3 revision_runs.py        (a) prediction in the low-error settings,
                                  CT vs ZOH at p=0 and p=0.6, 0.25 and 0.5 deg
                                  bias, slip U(0.99,1), 0.12 m footprint,
                                  paired bootstrap on the reduction of the
                                  median ground-truth RMSE, 4000 draws,
                                  seed 20261017;
                                  (b) the conservative cos(beta) weighting;
                                  (c) APF gains 0.10, 0.15, 0.20 with shared
                                  recovery.
  python3 summarize_recovery.py   aggregates results/ into recovery_summary.json
  python3 ../sim/e12_calib.py
                                  the compass-bias and slip sweep compared with
                                  the floor-grid checkpoints (manuscript
                                  Table III). This is a sensitivity analysis:
                                  no single setting reproduces both the spacing
                                  and the slot checkpoints, the simulated and
                                  measured statistics use different sampling
                                  protocols, and bias and slip are not
                                  separately identifiable from two values.

VERIFICATION OF THIS REBUILD
Rerunning matched_recovery.py and revision_runs.py from a clean clone, on a
different machine and a different NumPy version, reproduced every published
value exactly: all ten Table VI rows, the low-error prediction gains
(34.6% [31.5,36.7] at 0.25 deg and 25.0% [21.8,27.6] at 0.5 deg under 60%
loss; 2.0% [-2.7,3.4] and 0.5% [-0.6,2.1] without loss), the conservative
cos(beta) rows (95.5/92.0/91.0 contact-free, 72.0/57.0/40.5 held), the
extended gain sweep, and the best-gain margins (+17.0/+25.0/+30.5 points,
intervals [11.5,22.5], [17.0,33.0], [22.0,38.5]).
e12_calib.py (manuscript Table III) was not rerun in this rebuild; it is
included unmodified and takes a few minutes.

OUTPUTS
results/ and results_revision/ keep every mission's metrics, so the summaries
and intervals can be recomputed without rerunning the simulator.

SCOPE
No new hardware trial was performed. The hardware records in the manuscript
are the original ones. Gains are reported as a descriptive sweep; where a gain
was selected on the same missions used to evaluate it, the manuscript says so.

```
