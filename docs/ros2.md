# ROS 2 middleware

ROS 2 gives the swarm layer one interface to the simulated or real embedded
layer and to the operator. It is a middle layer only: robots coordinate through
ESP-NOW, not through ROS.

## Packages

* `swarm_autobots_msgs`: `LeaderState` (same fields as the 20 B ESP-NOW state
  frame), `FollowerStatus` (4 B status frame), `Ranges` (front, left, right).
* `swarm_autobots`:
  * `agent.py`: the portable per-robot agent (plain Python, no ROS import).
  * `agent_node.py`: ROS 2 wrapper, one node per robot (`swarm_agent`).
  * `sim_world_node.py`: simulated embedded layer (`sim_world`).
  * `espnow_channel_node.py`: per-receiver Gilbert-Elliott loss (`espnow_channel`).
  * `planner_node.py`: formation-aware A* for a mapped arena.
  * `espnow_gateway_node.py`: USB ESP32 gateway that mirrors the radio channel
    onto ROS topics for logging and operator commands.
  * `frames.py`: binary packing of the two ESP-NOW frames.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/robot_i/odom` | `nav_msgs/Odometry` | embedded -> agent |
| `/robot_i/ranges` | `swarm_autobots_msgs/Ranges` | embedded -> agent |
| `/robot_i/cmd_vel` | `geometry_msgs/Twist` | agent -> embedded |
| `/espnow/tx/leader_state`, `/espnow/tx/status` | frames | agent -> radio |
| `/robot_i/espnow/rx/leader_state`, `/robot_i/espnow/rx/status` | frames | radio -> agent |
| `/swarm/path` | `nav_msgs/Path` (latched) | planner -> agents |
| `/swarm/formation_cmd` | `std_msgs/String` | operator -> leader |

## Running

```bash
cd ros2_ws && colcon build && source install/setup.bash
ros2 launch swarm_autobots swarm_sim.launch.py num_robots:=4 formation:=Y loss:=0.2
ros2 topic pub --once /swarm/formation_cmd std_msgs/String "data: triangle"
```

## Status

The nodes were exercised with an in-process stand-in for `rclpy`
(`test/fake_ros.py`, `test/test_ros_graph.py`), and the agent was checked in
closed loop against the simulator (`test/test_agent_closed_loop.py`). Build
and launch once on a ROS 2 Humble or Jazzy machine before relying on them.
