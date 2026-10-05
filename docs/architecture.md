# Architecture

Every robot carries the same three layers. Robots interact only through ESP-NOW
broadcast at the bottom layer.

```
+--------------------------------------------------------------------------+
| Swarm layer (one agent per robot, no ROS dependency)                     |
|   election + slot persistence | leader: A*, pursuit, pacing |           |
|   follower: predictor, look-ahead tracking | speed filter, recovery      |
+--------------------------------------------------------------------------+
| Middleware (ROS 2)                                                       |
|   /robot_i/odom  /robot_i/ranges  /robot_i/cmd_vel                       |
|   /espnow/tx/*   /robot_i/espnow/rx/*   /swarm/path  /swarm/formation_cmd|
+--------------------------------------------------------------------------+
| Embedded (ESP32)                                                         |
|   encoders, compass, US, 2x IR, flame | dead reckoning, PWM, pump |      |
|   ESP-NOW radio: 20 B state / 4 B status at 10 Hz                        |
+--------------------------------------------------------------------------+
```

## Per-robot control cycle (20 Hz)

1. Read encoders, compass, and the three ranges; update the dead-reckoned pose.
2. If a follower has heard no heartbeat for `tau_e = 1.5 s`, mark the leader
   failed and claim leadership after a backoff of `b_i * 0.2 s`, where `b_i`
   counts lower-ranked robots still believed alive.
3. On the broadcast slot, handle heartbeats (yield to a lower rank) and send
   the state frame (leader) or status frame (follower).
4. Leader: pure pursuit on the A* path, speed scaled by the pacing factor.
5. Follower: if the last packet is older than `tau_stop = 1 s`, stop.
   Otherwise predict the leader along a constant-twist arc and track the slot
   `o_i - o_L` with the look-ahead point controller.
6. Clip forward speed with the range-based speed filter; run deadlock recovery
   if the robot is blocked; send `(v, w)` to the motors.

## Key equations

Leader prediction after packet age `tau`:

```
th~ = th + w * tau
p~  = p + (v / w) * [sin(th~) - sin(th),  cos(th) - cos(th~)]
```

Range-based speed filter on ray `j` (bearing `beta_j`, safety distance `r_s`):

```
v_safe = max(v_min, min(v_nom, min_j  lambda * (r_j - r_s) / (T * cos(beta_j))))
```

with `lambda = 0.25`, `T = 50 ms`, `v_min = -0.05 m/s`.

Pacing: the leader multiplies its cruise speed by
`clip(1 - (e_max - 0.25) / 0.5, 0.2, 1)`, where `e_max` is the largest
follower slot error reported in the last second.

## Where each piece lives

| Piece | Simulator (`sim/core.py`) | Portable agent (`ros2_ws/.../agent.py`) |
|---|---|---|
| Leader prediction | `predict_leader` | `SwarmAgent._predict` |
| Look-ahead tracking | `lpsi_control` | `SwarmAgent._track` |
| Range-based speed filter | `cbf_filter` | `SwarmAgent._barrier` |
| Election | `SwarmSim` (heartbeat bookkeeping) | `SwarmAgent._election` |
| Leader pursuit and pacing | `control_step` | `SwarmAgent._pursuit` |
| Formation-aware A* | `astar_path` | `planner_node.py` |
| Consensus baseline | `consensus_control` | not ported |
