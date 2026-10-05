"""Smoke test of the ROS 2 node graph (sim_world + espnow_channel + 4 swarm_agent nodes)
with the fake_ros stand-in. Checks topic wiring, message fields and mission completion."""
import os, sys, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import fake_ros
fake_ros.install()
from swarm_autobots import frames
from swarm_autobots.sim_world_node import SimWorld
from swarm_autobots.espnow_channel_node import ChannelNode
from swarm_autobots.agent_node import AgentNode
from std_msgs.msg import String

# frame round trip
m = dict(leader_id=2, seq=513, formation_id=1, x=1.25, y=-0.5, theta=0.3, v=0.2, w=-0.75)
r = frames.unpack_leader(frames.pack_leader(m))
assert r["leader_id"] == 2 and abs(r["v"] - 0.2) < 1e-3 and abs(r["w"] + 0.75) < 1e-3
assert frames.unpack_status(frames.pack_status(3, 0, 0.123)) == (3, 0, 0.123)

def mission(form, loss, switch=None, t_end=60.0):
    fake_ros.BUS.__init__(); fake_ros._PUBS.clear()
    fake_ros.Node.PARAMS = {"sim_world": dict(num_robots=4, formation=form, seed=3),
                            "espnow_channel": dict(num_robots=4, loss=loss, seed=3)}
    world = SimWorld(); chan = ChannelNode()
    agents = []
    for i in range(4):
        fake_ros.Node.PARAMS["swarm_agent"] = dict(robot_id=i, num_robots=4, formation=form)
        agents.append(AgentNode())
    cmd = world.create_publisher(String, "/swarm/formation_cmd", None)
    t = 0.0
    while t < t_end:
        t += 1.0
        fake_ros.run_until(t)
        if switch and abs(t - switch[0]) < 0.5:
            cmd.publish(String(data=switch[1]))
        if any(a.agent.goal_reached for a in agents):
            break
    return world, agents, t

for form in ["triangle", "Y"]:
    w, ag, t = mission(form, 0.2)
    print(form, "goal reached:", any(a.agent.goal_reached for a in ag), "t=%.0fs" % t,
          "collisions:", int(w.sim.collisions[0]), "followers' leader:", [a.agent.leader for a in ag])
    assert any(a.agent.goal_reached for a in ag) and w.sim.collisions[0] == 0
w, ag, t = mission("triangle", 0.2, switch=(10.0, "Y"))
print("pattern switch -> Y: formation ids", [a.agent.formation_id for a in ag], "goal:", any(a.agent.goal_reached for a in ag))
assert all(a.agent.formation_id == 1 for a in ag)
print("ROS graph smoke test PASS")
