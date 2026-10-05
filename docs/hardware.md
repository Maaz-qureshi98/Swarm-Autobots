# Hardware

Four identical robots. Any robot can act as Alpha (leader); the Alpha used in
the original trials also carried an ESP32-CAM.

## Bill of materials (per robot)

| Part | Model | Notes |
|---|---|---|
| Chassis | T101 tracked chassis | 20 x 20 cm, payload up to 5 kg |
| Motors | 2 x GM25-370 with Hall encoders | |
| Motor driver | L298N | |
| Controller | ESP32, 38-pin | dual core, 240 MHz, 520 KB SRAM |
| Front ranger | HC-SR04 ultrasonic | 0.02 to 2 m, on the heading |
| Side rangers | 2 x Sharp 2Y0A21 IR | 10 to 80 cm, at +/-60 deg |
| Heading | GY-271 magnetometer | |
| Fire payload | flame sensor, relay, 5 V pump, 350 ml tank | detection up to 80 cm |
| Power | 12 V 4200 mAh LiPo; L298N fed directly, 5 V logic from two LM2596 buck converters in parallel | over 2 h empty, over 1 h loaded |

Mass: 1.26 kg, or 1.65 kg with a full tank.

## Sensor geometry

All three rangers are mounted radially, 0.10 m from the robot center. The
barrier filter in `sim/core.py: cbf_filter` assumes this geometry. The Sharp
sensor output folds back below 10 cm, so a very close object can read as far;
only the ultrasonic ranger covers that zone, and only on the heading.

## Formations used in the trials

Spacings below are clearances between 20 cm chassis. Slot offsets in the
leader frame (x forward, y left), in meters, are the center positions used in
`sim/core.py: formation`.

| Pattern | Clearances | Slot offsets |
|---|---|---|
| Triangle | Hardware: teleoperated, Alpha on a table commanded by hand gestures, the three Betas 30 cm apart on the floor. Simulation: leader 0.5 m ahead of the Beta triangle | (0,0), (-0.5,0), (-1,-0.5), (-1,0.5) |
| Y | Beta-2/3 60 cm behind Beta-1, 30 cm to each side | (0,0), (-0.5,0), (-1.3,-0.5), (-1.3,0.5) |

CAD renders are in `media/`. The thermal camera and servo shown in the CAD
model were never fitted.

## Results

Ground-truth measurements of the swarm layer on the arena's floor grid: every
centre spacing within 2.3 cm of nominal, slot deviation 2–4 cm at rest and
4–8 cm after 2–3 m, leader takeover 1.7–1.8 s after Alpha was switched off,
and pattern switches in 1.5–2.0 s. Repeated runs agreed within about ±5 cm.
`sim/figs/fig_cases.png` shows the four cases; `sim/e12_calib.py` compares
these values with the simulator.
