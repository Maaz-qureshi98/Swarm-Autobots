# Hardware revision trials: compass bias, controlled packet loss, cycle time

Three measurements answer the remaining reviewer requests. All three use the trial firmware with
small additions, the floor grid of the 2.4 × 2.4 m arena, and a laptop logging serial output.
Total time is about one afternoon.

| # | Measurement | Answers | Time |
|---|---|---|---|
| A | Per-robot compass bias at 8 grid headings, motors off and on | R2: the bias spread is inferred, not measured | ~45 min |
| B | Case 3 (dynamic Y, board obstacle) with injected Gilbert–Elliott loss (p = 0.6, L = 4): 5 paired ZOH / prediction runs | R2/AE: no hardware loss experiment | ~2 h |
| C | `micros()` cycle time, logged during B | R2: cycle time not measured | free with B |

Files:

* `firmware/swarm_agent/HwExperiment.h`: `hw::GeDropper` (loss injection), `hw::CycleStats`, `hw::logLine`
* `firmware/swarm_agent/SwarmAgent.h`: `Params::predict` / `setPredict(false)` switches the follower to ZOH
* `firmware/compass_cal/compass_cal.ino`: compass measurement sketch
* `tools/analyze_compass.py`, `tools/analyze_hw_trials.py`: analysis; both print the LaTeX macros for `main.tex`

The C++ agent with the ZOH switch is checked against the Python reference in both modes
(`firmware/test/make_trace.py` + `test_equivalence.cpp`, 5 scenarios, 0 mismatches), and the loss
injector against the simulator's channel (`firmware/test/test_ge_dropper.cpp`: p and L reproduced
for p = 0.2–0.6 and L = 1–12).

---

## A. Compass bias (about 45 min for four robots)

1. In `compass_cal.ino`, paste the trial firmware's heading code and its calibration offsets into
   `readHeadingDeg()` (marked `TRIAL FIRMWARE`), and set `MY_RANK` and the motor pins. The point is
   to measure the heading the robots actually used, not a raw sensor.
2. Arena as in the trials: same place, LiPo fitted, deck closed.
3. For each robot: align a chassis edge with a grid line, type the true heading (0, 45, …, 315)
   and press Enter. The sketch averages 200 samples over 2 s.
4. Type `m` (motors on at cruise PWM). Put the robot on a block so the tracks run free, and repeat
   the 8 headings. This is the driving condition, with the motors' and the pack's current on.
5. Save each robot's serial output as `compass_<rank>.csv`, then run

```bash
python3 tools/analyze_compass.py compass_0.csv compass_1.csv compass_2.csv compass_3.csv
```

The script reports, per robot:

* **bias**: the constant heading error;
* **ripple**: the heading-dependent hard- and soft-iron error;
* **noise**.

Across robots it reports the **between-robot bias spread**. That is the number to compare with the
0.25–0.5° that the calibrated simulator needs. A bias common to all robots rotates the whole
formation and does not distort it, so only the spread matters.

Outcomes:

* **Spread within about 0.2–0.7°.** The calibration is independently confirmed, and limitation (iii) becomes evidence.
* **Spread much larger (for example 2°).** Report it honestly. The text then says that the grid results imply a smaller effective spread than the static measurement, for example because the heading-dependent ripple averages out on straight paths. Rerun `sim/e14_revision.py calib` with the measured value as a sensitivity check.

## B. Controlled packet loss on Case 3 (about 2 h)

### Firmware additions (followers; about 20 lines)

```cpp
#include "SwarmAgent.h"
#include "HwExperiment.h"

#define PREDICT 1                  // 1 = prediction, 0 = ZOH  (flash or toggle over serial per run)
#define LOSS_P  0.6
#define LOSS_L  4.0
#define RUN_SEED 1                 // same seed for zohK and predK -> identical drop pattern

swarm::SwarmAgent agent(MY_RANK, 4, 1);                         // Y
hw::GeDropper dropper(LOSS_P, LOSS_L, 0xC0FFEEu * RUN_SEED + MY_RANK);
hw::CycleStats cyc;

void onRecv(/* ESP-NOW receive callback */) {
  // ... unpack the 20-byte LeaderState into m ...
  if (dropper.drop(m.seq)) return;   // injected Gilbert-Elliott loss
  agent.onLeaderState(m, now_s());
}

void setup() { /* ... */ agent.setPredict(PREDICT); }

void loop() {                         // every 50 ms
  uint32_t t0 = micros();
  swarm::Output o = agent.step(now_s(), x, y, th, us, irl, irr);
  cyc.add(micros() - t0);             // add your sensor read and motor write inside the bracket
                                      // if you want the whole cycle, not just the agent
  setMotors(o.v, o.w);
  hw::logLine(Serial, millis(), MY_RANK, agent, now_s(), dropper, cyc.last(), o.v, o.w, PREDICT);
}
```

Alpha needs only the `micros()` bracket and `logLine` (no dropper; it does not receive leader frames).
Only leader frames are dropped. Status frames to the leader are left alone, so pacing behaves as in the trials.

### Runs

* Case 3 setup: dynamic Y, same board obstacle, same start cells, batteries charged.
* Five pairs: `zoh1/pred1`, …, `zoh5/pred5`. In pair K both runs use `RUN_SEED K`, so the injected
  loss pattern is identical. Alternate the order: zoh first in odd pairs, pred first in even pairs.
* Log all four robots over serial, as `<run>_<rank>.csv`, for example `zoh1_0.csv … pred5_3.csv`.
* At the end of each run, take the same grid measurements as in Sec. VI: the largest Beta slot
  deviation relative to Alpha, and the largest centre-spacing deviation. Write them to `grid.csv`:

```
run,slot_dev_cm,spacing_dev_cm
zoh1,9.5,3.0
pred1,5.0,2.0
...
```

### Analysis

```bash
python3 tools/analyze_hw_trials.py logs/*.csv --grid grid.csv
```

The script prints:

* per run: injected loss, packet age (median, 95th percentile, max), and followers' controller-level RMSE;
* the grid deviations;
* the paired comparison (in how many pairs prediction is lower, mean paired reduction, Wilcoxon signed-rank test);
* cycle-time statistics.

At the end it prints the LaTeX macros for `main.tex`.

### What to expect (pre-registered in simulation)

`sim/e15_revision2.py hwplan` simulates this exact trial: the calibrated model, Y with one obstacle,
p = 0.6, L = 4, and the first 2.5 m of leader travel. Paired over 200 missions:

* **Controller-level RMSE:** 6.9 cm with ZOH vs 3.9 cm with prediction (median). The paired median reduction is 44%, and prediction is lower in 92% of missions.
* **End-of-run slot deviation:** 10.5 cm vs 5.3 cm (median). The paired reduction is 42%, and prediction is lower in 87% of missions.
* **How many pairs you need:**
  * With 5 pairs, the mean paired difference favours prediction with probability 0.998.
  * With 3 unpaired runs per mode, this is only 0.80, which is why the design is paired.
  * Expect one pair in five to go the other way. Report it as measured.

## C. Cycle time

It is logged in every line of B (`cycle_us`), and `analyze_hw_trials.py` reports the mean, 99th
percentile and maximum over all robots against the 50 ms budget. If the bracket also includes
the sensor reads (the HC-SR04 echo wait can take several ms), say so in the paper.

---

## Putting the results into the paper

`main.tex` has a switch `\hwrevtrue` / `\hwrevfalse` near the top and a block of `\HW…` macros.
With `\hwrevfalse` (the default) the paper reads as it does now, with the limitations stated.

After the trials:

1. Paste the macros printed by the two scripts over the defaults.
2. Set `\hwrevtrue`.

The new Sec. VI-B paragraphs, the abstract sentence, and the shortened limitations then switch on
automatically. Read the new paragraphs once against your numbers. If a result goes against
prediction, the text must say so; do not just change the macros.
