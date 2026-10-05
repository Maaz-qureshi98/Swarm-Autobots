# Firmware

The ESP32 firmware handles the sensors, dead reckoning, motor PWM, the pump,
and the 10 Hz ESP-NOW frames. The swarm layer on the robots follows
`ros2_ws/src/swarm_autobots/swarm_autobots/agent.py` line by line.

## ESP-NOW frames

Both frames are little-endian and sent as broadcast (no acknowledgement).

**LeaderState, 20 bytes**, sent by the robot that believes it leads. It is
also the election heartbeat.

| Offset | Type | Field |
|---|---|---|
| 0 | uint8 | leader id (rank, from MAC address) |
| 1 | uint16 | sequence number |
| 3 | uint8 | formation (pattern) id |
| 4 | float32 | x [m] |
| 8 | float32 | y [m] |
| 12 | float32 | heading [rad] |
| 16 | int16 | v [mm/s] |
| 18 | int16 | w [mrad/s] |

**FollowerStatus, 4 bytes**, sent by each follower.

| Offset | Type | Field |
|---|---|---|
| 0 | uint8 | robot id |
| 1 | uint8 | leader id it follows |
| 2 | uint16 | slot error [mm] |

Reference packing in Python: `ros2_ws/src/swarm_autobots/swarm_autobots/frames.py`.

## C++ swarm agent (`swarm_agent/SwarmAgent.h`)

A header-only C++ port of the reference agent (`ros2_ws/.../agent.py`): no heap, no STL,
double precision. It contains the election with slot persistence, leader pursuit and pacing,
leader prediction, look-ahead tracking, the range-based speed filter, and deadlock recovery.

**Equivalence test.** `test/make_trace.py` records the Python agent in closed loop with the
simulator (triangle, Y with two unmapped obstacles, leader failure at 40% loss, pattern
switch at 60% loss); `test/test_equivalence.cpp` replays every input through the C++ agent:

```bash
python3 firmware/test/make_trace.py
cd firmware/test && g++ -O2 -std=c++17 -I../swarm_agent test_equivalence.cpp -o test_equivalence
./test_equivalence trace.txt
# 4 scenarios, 11688 agent steps, 0 mismatches, max |dv| 0.00e+00, |dw| 1.11e-16
```

**ESP32 build check.** The agent cross-compiles warning-free for the ESP32 (Xtensa LX6,
`xtensa-esp32-elf-g++` 13.2, `-Os`): 5.9 kB code and 4.8 kB static RAM, of 520 kB SRAM.

```bash
xtensa-esp32-elf-g++ -mlongcalls -Os -std=gnu++17 -fno-exceptions -fno-rtti \
    -c -Iswarm_agent test/esp32_build_check.cpp -o agent.o && xtensa-esp32-elf-size agent.o
```

**Using it in a sketch.**

```cpp
#include "SwarmAgent.h"
swarm::SwarmAgent agent(MY_RANK, 4, 0);          // rank from the MAC address, triangle
// ESP-NOW receive callback: unpack the 20-byte frame, then
//   agent.onLeaderState(frame, now_s);   or   agent.onStatus(id, err, now_s);
// every 50 ms:
swarm::Output o = agent.step(now_s, x, y, theta, us_m, ir_left_m, ir_right_m);
setMotors(o.v, o.w);
if (o.kind == 1) broadcastLeaderState(o.ls);      // 20 B, see frames.py
if (o.kind == 2) broadcastStatus(o.status_id, o.status_leader, o.status_err);
```

## ESP-NOW loss test (`espnow_loss_test/`)

Sender/receiver sketch and analyzer that measure the broadcast loss rate and burst length on
the real robots. See `docs/hardware_evaluation.md`. Not yet run on the robots.

## Trial firmware

The Arduino sketches used in the hardware trials are to be added to
`firmware/esp32_swarm/`.
