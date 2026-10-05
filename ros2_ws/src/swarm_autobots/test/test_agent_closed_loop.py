"""Closed-loop test of the ROS-free swarm agent against the reference simulator.
Run from the repo root:  python3 ros2_ws/src/swarm_autobots/test/test_agent_closed_loop.py
"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))                       # swarm_autobots package
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "sim"))  # sim/core.py
from swarm_autobots.agent import SwarmAgent
from core import SwarmSim


def mission(form="triangle", p=0.2, seed=0, fail_time=None, switch=None, max_t=70.0):
    sim = SwarmSim(1, 4, form=form, p_loss=p, burst=4.0, seed=seed, n_unknown=0,
                   switch_to=(switch[1] if switch else None))
    agents = [SwarmAgent(i, 4, form) for i in range(4)]
    for a in agents:
        a.set_path(sim.paths[0].tolist())
    rng = np.random.default_rng(seed)
    bad = np.zeros(4, bool)
    q = 1 / 4.0; s = p * q / (1 - p) if p > 0 else 0.0
    alive = [True] * 4
    t, step, coll0 = 0.0, 0, 0
    lat = None
    switched = False
    while t < max_t:
        ranges = sim.sense()[0]
        if fail_time is not None and t >= fail_time and alive[0]:
            alive[0] = False; sim.alive[0, 0] = False; sim.pos[0, 0] = -50.0
        if switch and not switched and t >= switch[0] - 1e-6:
            switched = True
            for a in agents:
                a.command_formation(switch[1])
        cmds = np.zeros((1, 4, 2))
        frames, statuses = [], []
        for i, a in enumerate(agents):
            if not alive[i]:
                continue
            v, w, out = a.step(t, tuple(sim.est[0, i]), tuple(ranges[i]))
            cmds[0, i] = (v, w)
            if isinstance(out, dict):
                frames.append(out)
            elif out is not None:
                statuses.append(out)
        # latency: every follower uses the new pattern and its (fresh) slot error < 0.1 m
        if switched and lat is None:
            fol = [a for i, a in enumerate(agents) if alive[i] and not a.is_leader]
            if all(a.formation_id == agents[[i for i in range(4) if alive[i] and agents[i].is_leader][0]].formation_id
                   and a.my_err < 0.10 for a in fol):
                lat = t - switch[0]
        if step % 2 == 0:                          # 10 Hz ESP-NOW slot, Gilbert-Elliott per receiver
            u = rng.random(4)
            bad = np.where(bad, u > q, u < s)
            for i, a in enumerate(agents):
                if not alive[i] or bad[i]:
                    continue
                for f in frames:
                    a.on_leader_state(f, t)
                for (rid, lid, err) in statuses:
                    if rid != i:
                        a.on_status(rid, err, t)
        sim.physics(cmds)
        t += sim.c.dt; step += 1
        if any(a.goal_reached for i, a in enumerate(agents) if alive[i] and a.is_leader):
            break
    leaders = [i for i in range(4) if alive[i] and agents[i].is_leader]
    return dict(reached=any(a.goal_reached for a in agents), time=t, collisions=int(sim.collisions[0]),
                leaders=leaders, switch_lat=lat)


if __name__ == "__main__":
    ok = True
    for form in ["triangle", "Y"]:
        rs = [mission(form, 0.2, seed=s) for s in range(10)]
        reach = np.mean([r["reached"] for r in rs]); coll = sum(r["collisions"] for r in rs)
        print(f"{form}: reached {reach:.0%}, collisions {coll}")
        ok &= reach >= 0.9 and coll == 0
    rs = [mission("Y", 0.2, seed=s, fail_time=12.0, max_t=90.0) for s in range(10)]
    print("leader failure: reached", np.mean([r["reached"] for r in rs]), "final leaders", [r["leaders"] for r in rs][:4])
    ok &= np.mean([r["reached"] for r in rs]) >= 0.9 and all(r["leaders"] == [1] for r in rs)
    rs = [mission("triangle", 0.2, seed=s, switch=(12.0, "Y")) for s in range(10)]
    lats = [r["switch_lat"] for r in rs if r["switch_lat"] is not None]
    print("pattern switch: latency median", np.median(lats) if lats else None, "n", len(lats))
    ok &= len(lats) >= 9
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
