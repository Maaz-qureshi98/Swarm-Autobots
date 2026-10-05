"""Record closed-loop traces of the Python reference agent for the C++ equivalence test.
Run from the repo root:  python3 firmware/test/make_trace.py  ->  firmware/test/trace.txt"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "swarm_autobots"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
from swarm_autobots.agent import SwarmAgent, FORMATION_IDS, Params
from core import SwarmSim

LOG = []


class Logged(SwarmAgent):
    def on_leader_state(self, m, t):
        LOG.append(f"L {self.id} {t!r} {m['leader_id']} {m['seq']} {m['formation_id']} {m['x']!r} {m['y']!r} "
                   f"{m['theta']!r} {m['v']!r} {m['w']!r}")
        super().on_leader_state(m, t)

    def on_status(self, rid, err, t):
        LOG.append(f"T {self.id} {t!r} {rid} {err!r}")
        super().on_status(rid, err, t)

    def set_path(self, path):
        LOG.append(f"P {self.id} {len(path)} " + " ".join(f"{x!r} {y!r}" for x, y in path))
        super().set_path(path)

    def command_formation(self, name):
        LOG.append(f"F {self.id} {FORMATION_IDS[name]}")
        super().command_formation(name)

    def step(self, t, pose, ranges):
        v, w, out = super().step(t, pose, ranges)
        if isinstance(out, dict):
            o = f"1 {out['seq']} {out['formation_id']}"
        elif out is not None:
            o = f"2 {out[1]} {out[2]!r}"
        else:
            o = "0"
        LOG.append(f"S {self.id} {t!r} {pose[0]!r} {pose[1]!r} {pose[2]!r} {ranges[0]!r} {ranges[1]!r} {ranges[2]!r} "
                   f"{v!r} {w!r} {o}")
        return v, w, out


class ZohParams(Params):
    predict = False


def mission(name, form, p, seed, n_unknown=0, fail_time=None, switch=None, max_t=70.0, predict=True):
    LOG.append(f"N {name} 4 {FORMATION_IDS[form]} {int(predict)}")
    sim = SwarmSim(1, 4, form=form, p_loss=p, burst=4.0, seed=seed, n_unknown=n_unknown,
                   switch_to=(switch[1] if switch else None))
    agents = [Logged(i, 4, form, params=Params if predict else ZohParams) for i in range(4)]
    for a in agents:
        a.set_path(sim.paths[0].tolist())
    rng = np.random.default_rng(seed)
    bad = np.zeros(4, bool); q = 0.25; s = p * q / (1 - p) if p > 0 else 0.0
    alive = [True] * 4; t, step, switched = 0.0, 0, False
    while t < max_t:
        ranges = sim.sense()[0]
        if fail_time is not None and t >= fail_time and alive[0]:
            alive[0] = False; sim.alive[0, 0] = False; sim.pos[0, 0] = -50.0
        if switch and not switched and t >= switch[0] - 1e-6:
            switched = True
            for a in agents:
                a.command_formation(switch[1])
        cmds = np.zeros((1, 4, 2)); frames, statuses = [], []
        for i, a in enumerate(agents):
            if not alive[i]:
                continue
            v, w, out = a.step(t, tuple(float(z) for z in sim.est[0, i]), tuple(float(z) for z in ranges[i]))
            cmds[0, i] = (v, w)
            if isinstance(out, dict):
                frames.append(out)
            elif out is not None:
                statuses.append(out)
        if step % 2 == 0:
            u = rng.random(4); bad = np.where(bad, u > q, u < s)
            for i, a in enumerate(agents):
                if alive[i] and not bad[i]:
                    for f in frames:
                        a.on_leader_state(f, t)
                    for (rid, lid, err) in statuses:
                        if rid != i:
                            a.on_status(rid, err, t)
        sim.physics(cmds); t += sim.c.dt; step += 1
        if any(a.goal_reached for i, a in enumerate(agents) if alive[i] and a.is_leader):
            break


if __name__ == "__main__":
    mission("triangle_p02", "triangle", 0.2, 0)
    mission("Y_unmapped2_p02", "Y", 0.2, 3, n_unknown=2)
    mission("Y_leaderfail_p04", "Y", 0.4, 1, fail_time=12.0, max_t=90.0)
    mission("switch_tri_to_Y_p06", "triangle", 0.6, 2, switch=(12.0, "Y"))
    mission("Y_unmapped1_p06_zoh", "Y", 0.6, 4, n_unknown=1, predict=False)   # hardware ZOH mode
    open(os.path.join(HERE, "trace.txt"), "w").write("\n".join(LOG) + "\n")
    print(len(LOG), "events,", sum(l.startswith("S") for l in LOG), "agent steps")
