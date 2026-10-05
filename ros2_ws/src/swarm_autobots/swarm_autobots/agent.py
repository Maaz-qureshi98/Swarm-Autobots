"""Swarm layer of Swarm Autobots: the per-robot algorithm, free of ROS and numpy-light,
so the same logic runs inside a ROS 2 node, in tests, and (ported) in ESP32 firmware.

One SwarmAgent instance = one robot. Every robot runs identical code; the role
(leader / follower) is decided by the heartbeat election at run time.
"""
import math

FORMATIONS = {
    # centre-to-centre slots in the leader frame (x forward), hardware geometry
    "triangle": [(0.0, 0.0), (-0.5, 0.0), (-1.0, -0.5), (-1.0, 0.5)],
    "Y":        [(0.0, 0.0), (-0.5, 0.0), (-1.3, -0.5), (-1.3, 0.5)],
}
FORMATION_IDS = {"triangle": 0, "Y": 1}
FORMATION_NAMES = {v: k for k, v in FORMATION_IDS.items()}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Params:
    dt = 0.05
    d = 0.10                 # off-axis control point
    K = 1.0
    v_max, v_min, w_max = 0.40, -0.05, 2.0
    v_cruise = 0.20
    tau_stop, tau_elect, T_backoff = 1.0, 1.5, 0.2
    lam, r_safe_us, r_safe_ir, ir_bearing = 0.25, 0.10, 0.10, math.radians(60)
    pace_e0, pace_e1, pace_min, status_timeout = 0.25, 0.50, 0.2, 1.0
    lookahead = 0.5
    predict = True           # False: zero-order hold on the last leader packet (hardware ZOH runs)


class SwarmAgent:
    def __init__(self, robot_id, n_robots, formation="triangle", params=Params):
        self.id, self.N, self.P = robot_id, n_robots, params
        self.formation_id = FORMATION_IDS[formation]
        self.is_leader = robot_id == 0
        self.leader = 0
        self.failed = set()
        self.cand_deadline = None
        self.pkt = None                       # last LeaderState from believed leader (dict)
        self.t_rx = 0.0
        self.status = {}                      # robot_id -> (err, t)
        self.path, self.prog = None, 0
        self.goal_reached = False
        self.seq = 0
        self.byp_dir, self.byp_last, self.byp_adv = 0.0, -1e9, 0.0
        self.rot_dir = 0.0
        self.my_err = 0.0

    # ------------------------------------------------------------ messages in
    def on_leader_state(self, m, t):
        """m: dict with leader_id, formation_id, x, y, theta, v, w (one ESP-NOW frame)."""
        j = m["leader_id"]
        if j == self.id:
            return
        if self.is_leader:
            if j < self.id:                       # yield to lower rank
                self.is_leader = False
            else:
                return
        if j <= self.leader or self.leader in self.failed or self.cand_deadline is not None:
            self.leader = j
            self.failed.discard(j)
            self.cand_deadline = None
        if j == self.leader:
            self.pkt = dict(m)
            self.t_rx = t
            self.formation_id = m["formation_id"]

    def on_status(self, robot_id, err, t):
        if self.is_leader:
            self.status[robot_id] = (err, t)

    def set_path(self, path):
        self.path, self.prog = list(path), 0

    def command_formation(self, name):
        """Operator pattern switch; only the leader applies it, followers learn it from heartbeats."""
        if self.is_leader:
            self.formation_id = FORMATION_IDS[name]

    # ------------------------------------------------------------ election
    def _election(self, t):
        P = self.P
        if self.is_leader:
            return
        if self.cand_deadline is None and t - self.t_rx > P.tau_elect:
            self.failed.add(self.leader)
            b = sum(1 for j in range(self.id) if j not in self.failed)
            self.cand_deadline = t + b * P.T_backoff
        if self.cand_deadline is not None and t >= self.cand_deadline:
            self.is_leader, self.leader, self.cand_deadline = True, self.id, None
            if self.path:
                self.prog = 0   # rejoin at nearest point (done in _pursuit)

    # ------------------------------------------------------------ slots
    def slot(self):
        F = FORMATIONS[FORMATION_NAMES[self.formation_id]]
        oi, oL = F[self.id % len(F)], F[self.leader % len(F)]
        return (oi[0] - oL[0], oi[1] - oL[1])

    # ------------------------------------------------------------ leader
    def _pursuit(self, pose):
        P = self.P
        x, y, th = pose
        path = self.path
        if not path:
            return 0.0, 0.0, 0.0
        if self.prog == 0:
            self.prog = min(range(len(path)), key=lambda k: (path[k][0] - x) ** 2 + (path[k][1] - y) ** 2)
        k = self.prog
        dist = lambda q: math.hypot(q[0] - x, q[1] - y)
        while k < len(path) - 1 and dist(path[k]) > dist(path[k + 1]):
            k += 1
        self.prog = k
        j = k
        while j < len(path) - 1 and dist(path[j]) < P.lookahead:
            j += 1
        tgt, goal = path[j], path[-1]
        dg = dist(goal)
        if dg < 0.15 and k >= len(path) - 3:
            self.goal_reached = True
            return 0.0, 0.0, 0.0
        a = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - th)
        v = P.v_cruise * max(0.0, math.cos(a)) if abs(a) < math.pi / 2 else 0.0
        v = min(v, 0.5 * dg + 0.05)
        fresh = [e for (e, ts) in self.status.values() if self._t - ts < P.status_timeout]
        if fresh:
            v *= min(1.0, max(P.pace_min, 1 - (max(fresh) - P.pace_e0) / P.pace_e1))
        Lh = max(dist(tgt), 0.2)
        w = 2.0 * max(v, 0.08) * math.sin(a) / Lh
        if abs(a) > math.pi / 3:
            w = 1.2 * math.copysign(1, a)
        return v, w, a

    # ------------------------------------------------------------ follower
    def _predict(self, tau):
        m = self.pkt
        x, y, th, v, w = m["x"], m["y"], m["theta"], m["v"], m["w"]
        if not self.P.predict:                      # zero-order hold
            return x, y, th, v, w
        if abs(w) < 1e-3:
            return x + v * tau * math.cos(th), y + v * tau * math.sin(th), th, v, w
        thn = th + w * tau
        return (x + v / w * (math.sin(thn) - math.sin(th)),
                y - v / w * (math.cos(thn) - math.cos(th)), wrap(thn), v, w)

    def _track(self, pose, Lx, Ly, Lth, vL, wL):
        P = self.P
        ox, oy = self.slot()
        ox += P.d
        c, s = math.cos(Lth), math.sin(Lth)
        px, py = Lx + c * ox - s * oy, Ly + s * ox + c * oy
        dpx = vL * c + wL * (-s * ox - c * oy)
        dpy = vL * s + wL * (c * ox - s * oy)
        x, y, th = pose
        ci, si = math.cos(th), math.sin(th)
        ex, ey = px - (x + P.d * ci), py - (y + P.d * si)
        ux, uy = dpx + P.K * ex, dpy + P.K * ey
        v = ci * ux + si * uy
        w = (-si * ux + ci * uy) / P.d
        slot = (px - P.d * c, py - P.d * s)
        self.my_err = math.hypot(slot[0] - x, slot[1] - y)
        eyb = -si * ex + ci * ey
        return v, w, eyb

    # ------------------------------------------------------------ safety
    def _barrier(self, v, ranges):
        P = self.P
        f, l, r = ranges
        lim = min(P.lam * (f - P.r_safe_us) / P.dt,
                  P.lam * (min(l, r) - P.r_safe_ir) / (P.dt * math.cos(P.ir_bearing)))
        return min(P.v_max, max(P.v_min, min(v, lim)))

    # ------------------------------------------------------------ main cycle
    def step(self, t, pose, ranges):
        """One 20 Hz cycle. pose = dead-reckoned (x, y, theta); ranges = (front, left, right).
        Returns (v, w, outgoing) where outgoing is a LeaderState dict, a status tuple, or None."""
        P = self.P
        self._t = t
        self._election(t)
        out = None
        if self.is_leader:
            vn, wn, a = self._pursuit(pose)
            v = self._barrier(vn, ranges)
            w = wn
            if vn > 0.05 and v < 0.03:            # rotate-to-clear
                if self.rot_dir == 0:
                    self.rot_dir = math.copysign(1, a) if abs(a) > 0.05 else (1.0 if ranges[1] >= ranges[2] else -1.0)
                w = 0.8 * self.rot_dir
            elif ranges[0] >= 0.25:
                self.rot_dir = 0.0
            self.seq = (self.seq + 1) % 65536
            out = dict(leader_id=self.id, seq=self.seq, formation_id=self.formation_id,
                       x=pose[0], y=pose[1], theta=pose[2], v=v, w=w)
            return v, w, out
        if self.pkt is None or t - self.t_rx > P.tau_stop:
            return 0.0, 0.0, None
        Lx, Ly, Lth, vL, wL = self._predict(min(t - self.t_rx, P.tau_stop))
        vn, wn, eyb = self._track(pose, Lx, Ly, Lth, vL, wL)
        vn = min(P.v_max, max(P.v_min, vn)); wn = min(P.w_max, max(-P.w_max, wn))
        v = self._barrier(vn, ranges)
        w = wn
        # rotate-and-advance deadlock breaker
        blocked = vn > 0.05 and v < 0.03
        recent = t - self.byp_last < 2.0
        if blocked:
            if not recent:
                self.byp_dir = math.copysign(1, eyb) if abs(eyb) > 0.05 else (1.0 if ranges[1] >= ranges[2] else -1.0)
            self.byp_last = t
            w = 0.8 * self.byp_dir
        elif recent and self.byp_adv <= 0 and t - self.byp_last < 1.5 * P.dt:
            self.byp_adv = 0.8
        if not blocked and self.byp_adv > 0:
            v, w = self._barrier(0.15, ranges), 0.0
        self.byp_adv = max(0.0, self.byp_adv - P.dt)
        out = (self.id, self.leader, self.my_err)
        return v, w, out
