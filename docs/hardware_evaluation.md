# Hardware evaluation protocol (ground truth on the real robots)

This is the protocol for continuous ground-truth tracking of hardware runs with an overhead
camera, extending the floor-grid measurements reported so far. It needs one overhead camera (a phone is enough), printed ArUco markers, and the
scripts in `tools/` and `firmware/espnow_loss_test/`.

## 1. Setup

1. Generate and print ArUco markers (`DICT_4X4_50`) at true size: ids 40-43 for the floor (12 cm), ids 0-3 for
   the robots (10 cm), e.g. `python3 -c "import cv2; cv2.imwrite('m0.png', cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 600))"`
2. Tape markers 40, 41, 42, 43 flat at the four arena corners, in that order, counter-clockwise
   from (0, 0). Measure their centres (for the 8 x 8 ft arena: (0,0), (2.44,0),
   (2.44,2.44), (0,2.44)).
3. Fix one marker flat on top of each robot, centred, with the marker's x-axis pointing
   forward (Alpha = 0, Beta-1 = 1, Beta-2 = 2, Beta-3 = 3). Measure the marker height above
   the floor.
4. Mount the camera as high as possible above the arena centre, so all six markers are in
   view. Record at 30 fps or more, 1080p.

The tracker is checked by `python3 tools/test_aruco_synthetic.py` (tilted camera, rendered
markers): mean position error 0.5 mm, maximum 1.4 mm, heading error under 1.4 degrees.

## 2. Trials

Run at least 10 trials per condition (gives a useful confidence interval):

| Condition | Firmware | Notes |
|---|---|---|
| Triangle (teleoperated) | ASA, swarm layer | operator gestures over Alpha's rangers |
| Static Y | ASA, swarm layer | 60 s hold |
| Dynamic Y + board obstacle | ASA, swarm layer | same board, same placement |
| Leader failure | swarm layer | switch Alpha off at about 10 s |
| Pattern switch | swarm layer | triangle to Y command mid-run |

Start every robot at its grid slot. Log each robot's dead-reckoned pose over serial if
possible (for drift).

## 3. Processing

```bash
python3 tools/aruco_tracker.py trial.mp4 --corners 0,0 2.44,0 2.44,2.44 0,2.44 \
        --camera-height 2.5 --marker-height 0.18 --out trial.csv
python3 tools/eval_hardware.py trial.csv --formation Y
```

`eval_hardware.py` reports per-follower and overall ground-truth slot RMSE (same definition
as the simulation study), maximum slot error, and minimum centre separation.

## 4. Radio channel

Flash `firmware/espnow_loss_test/espnow_loss_test.ino` to two robots (one sender, one
receiver), log the receiver for 10 minutes at several distances and with robots moving, and
run `python3 firmware/espnow_loss_test/analyze_loss.py log.csv`. This gives the measured loss
rate p and burst length L that the simulator uses (the analyzer recovers p = 0.30, L = 4.06
from a synthetic channel with p = 0.3, L = 4).

## 5. What to report

- Table: per condition, trials, success, ground-truth slot RMSE (median, IQR), collisions,
  for ASA and the swarm layer.
- Leader failure: time from switch-off to all survivors following the new leader.
- Measured p and L, and the simulator rerun at those values.
