"""
Swarm Autobots simulator and controllers.

Vectorized over E independent episodes (envs) with N robots each.
Physical and sensor parameters follow the Swarm Autobots hardware
(BSc thesis, Bahria University 2021) and component datasheets:
  - tracked chassis ~8x8 in  -> circumscribed radius 0.15 m
  - GM25-370 Hall-encoder motors, PWM limited -> v_max 0.40 m/s
  - HC-SR04 front ultrasonic (2 cm - 2 m used, ~15 deg cone)
  - 2x Sharp 2Y0A21 IR (10 - 80 cm) mounted at +-60 deg
  - GY-271 magnetometer heading
  - ESP-NOW broadcast from leader at 10 Hz, 250 B frames
"""
import heapq
import numpy as np

DEG = np.pi / 180.0


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------
class Cfg:
    dt = 0.05            # control period (20 Hz)
    comm_every = 2       # leader broadcast every 2 steps (10 Hz)
    R = 0.12             # robot radius (0.20 m chassis + tracks)
    mount = 0.10         # sensor mount radius (radially mounted)
    v_max = 0.40
    v_min = -0.05        # limited blind reverse
    w_max = 2.0
    tau_m = 0.15         # motor time constant
    v_cruise = 0.20      # leader cruise speed
    # sensors: 3 sub-rays for the US cone, then left IR, right IR
    ray_ang = np.array([-12.0, 0.0, 12.0, 60.0, -60.0]) * DEG
    us_rng = (0.02, 2.0)
    ir_rng = (0.10, 0.80)
    # controller
    d_ctrl = 0.10
    K = 1.0
    tau_stop = 1.0
    tau_elect = 1.5
    T_backoff = 0.2
    # barrier filter
    cbf_lam = 0.25
    r_safe_us = 0.10
    r_safe_ir = 0.10
    # noise
    slip_v = (0.96, 1.00)
    slip_w = (0.75, 0.95)
    compass_bias_std = 2.0 * DEG
    compass_noise_std = 1.0 * DEG
    slip_noise = (0.03, 0.05)       # per-step relative noise on v and w slip
    us_noise = (0.005, 0.01)        # US noise std: a + b * range
    ir_noise = (0.01, 0.03)         # IR noise std: a + b * range
    # pacing
    pace_e0 = 0.25
    pace_e1 = 0.50
    pace_min = 0.2
    status_timeout = 1.0


# --------------------------------------------------------------------------
# Formations (leader frame offsets, slot 0 = leader at origin)
# --------------------------------------------------------------------------
def formation(name, N, s=0.5):
    """Formations from the hardware trials. Offsets (leader frame, x forward) are
    centre-to-centre spacings: a 30 cm clearance between 20 cm chassis -> 0.5 m.
      triangle: Beta-1 0.5 m behind Alpha; Beta-2/3 a further 0.5 m back, +-0.5 m lateral
      Y:        Beta-1 0.5 m behind Alpha; Beta-2/3 a further 0.8 m back (60 cm clearance), +-0.5 m lateral
    Larger swarms extend the same pattern (used only for the scalability study)."""
    off = [(0.0, 0.0)]
    if name == "triangle":
        k = 1
        while len(off) < N:
            for j in range(k):
                if len(off) < N:
                    off.append((-k * s, (j - (k - 1) / 2.0) * 2 * s))
            k += 1
    elif name == "Y":
        stem = max(1, int(round((N - 1) / 3)))
        for k in range(1, stem + 1):
            off.append((-k * s, 0.0))
        i = 1
        while len(off) < N:
            for sgn in (-1, 1):
                if len(off) < N:
                    off.append((-stem * s - i * 0.8, sgn * i * s))
            i += 1
    else:
        raise ValueError(name)
    return np.array(off[:N])


# --------------------------------------------------------------------------
# Planning: A* on an inflated occupancy grid + line-of-sight shortcutting
# --------------------------------------------------------------------------
def astar_path(W, H, obs, start, goal, infl, res=0.1):
    nx, ny = int(np.ceil(W / res)), int(np.ceil(H / res))
    xs = (np.arange(nx) + 0.5) * res
    ys = (np.arange(ny) + 0.5) * res
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    occ = (X < infl) | (X > W - infl) | (Y < infl) | (Y > H - infl)
    for (ox, oy, r) in obs:
        occ |= (X - ox) ** 2 + (Y - oy) ** 2 < (r + infl) ** 2

    def cell(p):
        return (min(nx - 1, max(0, int(p[0] / res))), min(ny - 1, max(0, int(p[1] / res))))

    s, g = cell(start), cell(goal)
    # clear a small disc around start/goal so a robot that starts inside an
    # inflated zone can still leave it
    occ &= ~((X - start[0]) ** 2 + (Y - start[1]) ** 2 < (infl * 0.8) ** 2)
    occ[g] = False
    nb = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
          (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    gs = {s: 0.0}
    par = {}
    pq = [(0.0, s)]
    closed = set()
    while pq:
        _, c = heapq.heappop(pq)
        if c in closed:
            continue
        if c == g:
            break
        closed.add(c)
        for dx, dy, w in nb:
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or occ[n]:
                continue
            ng = gs[c] + w
            if ng < gs.get(n, 1e18):
                gs[n] = ng
                par[n] = c
                h = np.hypot(g[0] - n[0], g[1] - n[1])
                heapq.heappush(pq, (ng + h, n))
    if g not in par and g != s:
        return None
    cells = [g]
    while cells[-1] != s:
        cells.append(par[cells[-1]])
    cells = cells[::-1]
    pts = np.array([((c[0] + 0.5) * res, (c[1] + 0.5) * res) for c in cells])
    pts[0] = start
    pts[-1] = goal

    def free(a, b):
        n = max(2, int(np.hypot(*(b - a)) / (res * 0.5)))
        for t in np.linspace(0, 1, n):
            c = cell(a + t * (b - a))
            if occ[c]:
                return False
        return True

    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not free(pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    # densify for pure pursuit
    dense = [out[0]]
    for a, b in zip(out[:-1], out[1:]):
        n = max(1, int(np.hypot(*(b - a)) / 0.05))
        for t in np.linspace(0, 1, n + 1)[1:]:
            dense.append(a + t * (b - a))
    return np.array(dense)


# --------------------------------------------------------------------------
# Simulator
# --------------------------------------------------------------------------
class SwarmSim:
    def __init__(self, E, N, form="triangle", p_loss=0.0, burst=3.0, n_obs=None,
                 obs_density=0.15, seed=0, cfg=Cfg, form_list=None,
                 p_range=None, burst_range=None, fail_time=None, election=True,
                 max_time=None, n_unknown=2, pacing=True, form_aware=True, switch_to=None):
        self.E, self.N, self.c = E, N, cfg
        self.rng = np.random.default_rng(seed)
        self.form_name = form
        self.form_list = form_list
        self.p_loss_fixed, self.burst_fixed = p_loss, burst
        self.p_range, self.burst_range = p_range, burst_range
        self.obs_density = obs_density
        self.n_obs_fixed = n_obs
        self.fail_time = fail_time
        self.election = election
        self.max_time_cfg = max_time
        self.n_unknown = n_unknown
        self.switch_to = switch_to
        self.FORMS = ["triangle", "Y"]
        self.pacing = pacing
        self.form_aware = form_aware
        # geometry from the largest formation used
        names = list(form_list if form_list else [form]) + ([switch_to] if switch_to else [])
        ext = np.concatenate([formation(f, N) for f in names])
        self.ext_min = ext.min(0)
        self.ext_max = ext.max(0)
        span_x = self.ext_max[0] - self.ext_min[0]
        span_y = self.ext_max[1] - self.ext_min[1]
        self.W = max(7.0, 6.0 + 2 * span_x)
        self.H = max(6.0, span_y + 3.5)
        self.M_obs = 0
        self.reset_all()

    # ---------------------------------------------------------------- reset
    def reset_all(self):
        E, N = self.E, self.N
        self.t = np.zeros(E)
        self.done = np.zeros(E, bool)
        self.pos = np.zeros((E, N, 2))
        self.th = np.zeros((E, N))
        self.est = np.zeros((E, N, 3))
        self.uact = np.zeros((E, N, 2))
        self.slipv = np.ones((E, N))
        self.slipw = np.ones((E, N))
        self.cbias = np.zeros((E, N))
        self.offsets = np.zeros((E, N, 2))
        self.form_off = np.stack([formation(f, N) for f in ["triangle", "Y"]])   # F,N,2
        self.form_cmd = np.zeros(E, int)
        self.form_rx = np.zeros((E, N), int)
        self.form_idx = np.zeros(E, int)
        self.forms = []
        self.p = np.zeros(E)
        self.burst = np.ones(E)
        self.ch_bad = np.zeros((E, N), bool)
        self.paths = [None] * E
        self.goal = np.zeros((E, 2))
        self.obs_list = [None] * E
        self.alive = np.ones((E, N), bool)
        self.is_leader = np.zeros((E, N), bool)
        self.bel_leader = np.zeros((E, N), int)
        self.last_rx = np.zeros((E, N))
        self.pkt = np.zeros((E, N, 5))
        self.dead_known = np.zeros((E, N, N), bool)
        self.cand_deadline = np.full((E, N), np.inf)
        self.prog = np.zeros((E, N), int)
        self.contact = np.zeros((E, N), bool)
        self.collisions = np.zeros(E, int)
        self.goal_time = np.full(E, np.inf)
        self.leader_ever_failed = np.zeros(E, bool)
        self.false_elections = np.zeros(E, int)
        self.max_time = np.zeros(E)
        self.fa_ok = np.zeros(E, bool)
        self.lead_a = np.zeros((E, N))
        self.rot_dir = np.zeros((E, N))
        self.byp_dir = np.zeros((E, N))
        self.byp_last = np.full((E, N), -1e9)
        self.byp_adv = np.zeros((E, N))
        self.status_err = np.zeros((E, N))
        self.nb_pkt = np.zeros((E, N, N, 3))
        self.nb_rx = np.full((E, N, N), -1e9)
        self.status_t = np.full((E, N), -1e9)
        self.my_err = np.zeros((E, N))
        for e in range(E):
            self._reset_env(e)
        self.ranges = self.sense()

    def _reset_env(self, e):
        c, N, rng = self.c, self.N, self.rng
        W, H = self.W, self.H
        fname = self.form_list[rng.integers(len(self.form_list))] if self.form_list else self.form_name
        off = formation(fname, N)
        self.offsets[e] = off
        self.form_cmd[e] = self.FORMS.index(fname)
        self.form_rx[e] = self.form_cmd[e]
        self.p[e] = rng.uniform(*self.p_range) if self.p_range else self.p_loss_fixed
        self.burst[e] = rng.uniform(*self.burst_range) if self.burst_range else self.burst_fixed
        # start and goal
        x0 = 0.6 + max(0.0, -self.ext_min[0]) + 0.2
        xg = W - 0.6 - max(0.0, self.ext_max[0]) - 0.2
        y0 = H / 2 + rng.uniform(-0.3, 0.3)
        yg = H / 2 + rng.uniform(-0.6, 0.6)
        start = np.array([x0, y0])
        goal = np.array([xg, yg])
        # obstacles in the middle band
        bx0 = x0 + max(0.0, self.ext_max[0]) + 0.7
        bx1 = xg + min(0.0, self.ext_min[0]) - 0.7
        area = max(0.0, bx1 - bx0) * (H - 1.0)
        n_obs = self.n_obs_fixed if self.n_obs_fixed is not None else int(round(self.obs_density * area))
        half_w = np.abs(off[:, 1]).max()
        for _ in range(200):
            obs = []
            tries = 0
            while len(obs) < n_obs and tries < 2000:
                tries += 1
                r = rng.uniform(0.10, 0.22)
                ox, oy = rng.uniform(bx0, bx1), rng.uniform(0.5 + r, H - 0.5 - r)
                if all(np.hypot(ox - a, oy - b) > r + rb + 0.50 for a, b, rb in obs):
                    obs.append((ox, oy, r))
            path = None
            if self.form_aware:
                # formation-aware A*: inflate by robot radius + formation half-width
                path = astar_path(W, H, obs, start, goal, infl=c.R + 0.10 + half_w)
            fa_ok = path is not None
            if path is None:
                path = astar_path(W, H, obs, start, goal, infl=c.R + 0.15)
            if path is not None:
                break
        # unknown (unmapped) obstacles placed near the planned path, inside the band
        unk = []
        tries = 0
        cand = path[(path[:, 0] > bx0) & (path[:, 0] < bx1)]
        n_unk = self.n_unknown if not isinstance(self.n_unknown, tuple) else rng.integers(self.n_unknown[0], self.n_unknown[1] + 1)
        while len(unk) < n_unk and tries < 500 and len(cand) > 0:
            tries += 1
            k = rng.integers(len(cand))
            r = rng.uniform(0.10, 0.18)
            lat = rng.uniform(-1, 1) * (half_w + 0.2)
            # perpendicular offset from the path direction
            k2 = min(k + 2, len(cand) - 1); k1 = max(k - 2, 0)
            dvec = cand[k2] - cand[k1]
            nrm = np.hypot(*dvec) + 1e-9
            nvec = np.array([-dvec[1], dvec[0]]) / nrm
            ox, oy = cand[k] + lat * nvec
            if abs(lat) < 0.35:  # do not block the leader's own path
                continue
            if all(np.hypot(ox - a, oy - b) > r + rb + 0.45 for a, b, rb in obs + unk):
                unk.append((ox, oy, r))
        self.unknown_list = getattr(self, "unknown_list", [None] * self.E)
        self.unknown_list[e] = unk
        obs = obs + unk
        self.obs_list[e] = obs
        self.fa_ok[e] = fa_ok if self.form_aware else False
        self.paths[e] = path
        self.goal[e] = goal
        th0 = 0.0
        Rm = np.array([[np.cos(th0), -np.sin(th0)], [np.sin(th0), np.cos(th0)]])
        self.pos[e] = start + off @ Rm.T
        self.th[e] = th0
        self.uact[e] = 0
        self.slipv[e] = rng.uniform(*c.slip_v, N)
        self.slipw[e] = rng.uniform(*c.slip_w, N)
        self.cbias[e] = rng.normal(0, c.compass_bias_std, N)
        # dead reckoning initialized from known grid start poses
        self.est[e, :, :2] = self.pos[e]
        self.est[e, :, 2] = th0
        self.t[e] = 0.0
        self.done[e] = False
        self.ch_bad[e] = False
        self.alive[e] = True
        self.is_leader[e] = False
        self.is_leader[e, 0] = True
        self.bel_leader[e] = 0
        self.last_rx[e] = 0.0
        self.pkt[e] = np.array([*self.est[e, 0], 0.0, 0.0])
        self.dead_known[e] = False
        self.cand_deadline[e] = np.inf
        self.prog[e] = 0
        self.contact[e] = False
        self.collisions[e] = 0
        self.goal_time[e] = np.inf
        self.leader_ever_failed[e] = False
        self.false_elections[e] = 0
        if hasattr(self, 'status_err'):
            self.status_err[e] = 0.0
            self.status_t[e] = -1e9
            self.nb_rx[e] = -1e9
            self.my_err[e] = 0.0
        L = len(path) * 0.05
        self.max_time[e] = self.max_time_cfg if self.max_time_cfg else L / c.v_cruise * 2.0 + 15.0
        self._build_obs_arrays()

    def _build_obs_arrays(self):
        M = max(1, max(len(o) if o is not None else 0 for o in self.obs_list))
        self.M_obs = M
        self.oc = np.full((self.E, M, 2), -100.0)
        self.orad = np.zeros((self.E, M))
        for e, ol in enumerate(self.obs_list):
            if ol is None:
                continue
            for k, (x, y, r) in enumerate(ol):
                self.oc[e, k] = (x, y)
                self.orad[e, k] = r

    def reset_envs(self, mask):
        for e in np.where(mask)[0]:
            self._reset_env(e)

    # ---------------------------------------------------------------- sensing
    def raycast(self):
        c = self.c
        E, N = self.E, self.N
        ang = self.th[:, :, None] + c.ray_ang[None, None, :]       # E,N,K
        d = np.stack([np.cos(ang), np.sin(ang)], -1)                # E,N,K,2
        o = self.pos[:, :, None, :] + c.mount * d                   # E,N,K,2
        centers = np.concatenate([self.oc, self.pos], 1)           # E,M+N,2
        radii = np.concatenate([self.orad, np.full((E, N), c.R)], 1)
        oc = o[:, :, :, None, :] - centers[:, None, None, :, :]     # E,N,K,M',2
        b = (oc * d[:, :, :, None, :]).sum(-1)
        cc = (oc ** 2).sum(-1) - radii[:, None, None, :] ** 2
        disc = b * b - cc
        sq = np.sqrt(np.maximum(disc, 0))
        t = -b - sq
        t = np.where(cc < 0, 0.0, t)
        valid = (disc > 0) & (t >= 0) & (radii[:, None, None, :] > 0)
        # exclude self
        M = self.M_obs
        selfmask = np.zeros((N, M + N), bool)
        selfmask[np.arange(N), M + np.arange(N)] = True
        valid &= ~selfmask[None, :, None, :]
        t = np.where(valid, t, np.inf).min(-1)                      # E,N,K
        # walls
        W, H = self.W, self.H
        with np.errstate(divide="ignore", invalid="ignore"):
            tx = np.where(d[..., 0] > 1e-9, (W - o[..., 0]) / d[..., 0],
                          np.where(d[..., 0] < -1e-9, -o[..., 0] / d[..., 0], np.inf))
            ty = np.where(d[..., 1] > 1e-9, (H - o[..., 1]) / d[..., 1],
                          np.where(d[..., 1] < -1e-9, -o[..., 1] / d[..., 1], np.inf))
        t = np.minimum(t, np.minimum(tx, ty))
        return np.maximum(t, 0.0)

    def sense(self):
        """Returns noisy [front_US, left_IR, right_IR] readings (E,N,3)."""
        c = self.c
        t = self.raycast()
        rng = self.rng
        us = t[..., :3].min(-1)
        us = np.clip(us + rng.normal(0, 1, us.shape) * (c.us_noise[0] + c.us_noise[1] * us), *c.us_rng)
        ir = t[..., 3:5]
        ir = np.clip(ir + rng.normal(0, 1, ir.shape) * (c.ir_noise[0] + c.ir_noise[1] * ir), *c.ir_rng)
        self.true_rays = t
        return np.concatenate([us[..., None], ir], -1)

    # ---------------------------------------------------------------- comms
    def comm_step(self):
        """Leader heartbeat/state broadcast over a receiver-side Gilbert-Elliott channel,
        plus heartbeat-timeout leader election ordered by rank (MAC order = index)."""
        c, E, N = self.c, self.E, self.N
        rng = self.rng
        # channel state transitions (per receiver)
        p = np.clip(self.p, 0, 0.95)[:, None]
        q = 1.0 / self.burst[:, None]
        s = np.where(p > 0, p * q / np.maximum(1 - p, 1e-6), 0.0)
        u = rng.random((E, N))
        self.ch_bad = np.where(self.ch_bad, u > q, u < s)
        rx_ok = ~self.ch_bad
        bc = self.alive & self.is_leader                            # E,N broadcasters
        for i in range(N):
            heard = bc & rx_ok[:, i:i + 1]
            heard[:, i] = False
            any_h = heard.any(1)
            jstar = np.where(any_h, heard.argmax(1), N)             # lowest rank heard
            act = any_h & self.alive[:, i] & ~self.done
            # leader yields to lower-rank heartbeat
            yld = act & self.is_leader[:, i] & (jstar < i)
            self.is_leader[yld, i] = False
            if np.any(yld):
                self.false_elections[yld] += 1
            fol = act & ~self.is_leader[:, i]
            bl = self.bel_leader[:, i]
            bl_dead = self.dead_known[np.arange(E), i, bl]
            adopt = fol & ((jstar <= bl) | bl_dead | (self.cand_deadline[:, i] < np.inf))
            ee = np.where(adopt)[0]
            if len(ee):
                js = jstar[ee]
                self.bel_leader[ee, i] = js
                self.dead_known[ee, i, js] = False
                self.cand_deadline[ee, i] = np.inf
            upd = fol & (jstar == self.bel_leader[:, i])
            ee = np.where(upd)[0]
            if len(ee):
                js = jstar[ee]
                self.pkt[ee, i, :3] = self.est[ee, js]
                self.form_rx[ee, i] = self.form_cmd[ee]
                self.pkt[ee, i, 3:] = self.uact[ee, js]
                self.last_rx[ee, i] = self.t[ee]
        # all-to-all state broadcast (consensus baseline only)
        heardj = self.alive[:, None, :] & rx_ok[:, :, None] & ~np.eye(N, dtype=bool)[None]
        self.nb_pkt = np.where(heardj[..., None], self.est[:, None, :, :], self.nb_pkt)
        self.nb_rx = np.where(heardj, self.t[:, None, None], self.nb_rx)
        # follower status uplink (error magnitude) to the believed leader
        for j in range(N):
            snd = self.alive[:, j] & ~self.is_leader[:, j] & ~self.done
            L = self.bel_leader[:, j]
            ok = snd & rx_ok[np.arange(E), L] & self.is_leader[np.arange(E), L]
            ee = np.where(ok)[0]
            if len(ee):
                self.status_err[ee, j] = self.my_err[ee, j]
                self.status_t[ee, j] = self.t[ee]

    def election_step(self):
        c, E, N = self.c, self.E, self.N
        if not self.election:
            return
        for i in range(N):
            fol = self.alive[:, i] & ~self.is_leader[:, i] & ~self.done
            age = self.t - self.last_rx[:, i]
            start = fol & (age > c.tau_elect) & (self.cand_deadline[:, i] == np.inf)
            ee = np.where(start)[0]
            for e in ee:
                self.dead_known[e, i, self.bel_leader[e, i]] = True
                pos = sum(1 for j in range(i) if not self.dead_known[e, i, j])
                self.cand_deadline[e, i] = self.t[e] + pos * c.T_backoff
            win = fol & (self.t >= self.cand_deadline[:, i])
            ee = np.where(win)[0]
            if len(ee):
                self.is_leader[ee, i] = True
                self.bel_leader[ee, i] = i
                self.cand_deadline[ee, i] = np.inf
                # rejoin the shared mission path at the nearest point ahead
                for e in ee:
                    P = self.paths[e]
                    self.prog[e, i] = int(np.argmin(((P - self.est[e, i, :2]) ** 2).sum(1)))

    # ---------------------------------------------------------------- slots
    def slot_offsets(self):
        """Slot persistence: robot i always keeps its own formation slot o_i.
        If it follows leader L, its offset in L's frame is o_i - o_L, so a change
        of leader moves the formation frame, not the followers."""
        E, N = self.E, self.N
        L = self.bel_leader
        fo = self.form_off[self.form_rx]                          # E,N,N,2: table each robot uses
        own = fo[:, np.arange(N), np.arange(N)]                    # E,N,2   o_i
        oL = np.take_along_axis(fo, L[:, :, None, None].repeat(2, -1), 2)[:, :, 0]   # E,N,2  o_L
        off = own - oL
        off[self.is_leader] = 0.0
        return off

    # ---------------------------------------------------------------- physics
    def physics(self, cmd):
        c, E, N, rng = self.c, self.E, self.N, self.rng
        dt = c.dt
        cmd = cmd.copy()
        cmd[~self.alive] = 0.0
        cmd[self.done] = 0.0
        cmd[..., 0] = np.clip(cmd[..., 0], c.v_min, c.v_max)
        cmd[..., 1] = np.clip(cmd[..., 1], -c.w_max, c.w_max)
        self.uact += (cmd - self.uact) * (dt / c.tau_m)
        v = self.uact[..., 0] * self.slipv * (1 + c.slip_noise[0] * rng.standard_normal((E, N)))
        w = self.uact[..., 1] * self.slipw * (1 + c.slip_noise[1] * rng.standard_normal((E, N)))
        thm = self.th + 0.5 * w * dt
        newpos = self.pos + (v * dt)[..., None] * np.stack([np.cos(thm), np.sin(thm)], -1)
        newth = wrap(self.th + w * dt)
        # collision check at new position
        R = c.R
        dpp = newpos[:, :, None, :] - newpos[:, None, :, :]
        dist = np.sqrt((dpp ** 2).sum(-1)) + np.eye(N)[None] * 1e9
        rr = (dist < 2 * R).any(-1)
        dpo = newpos[:, :, None, :] - self.oc[:, None, :, :]
        ro = (np.sqrt((dpo ** 2).sum(-1)) < R + self.orad[:, None, :]).any(-1)
        rw = (newpos[..., 0] < R) | (newpos[..., 0] > self.W - R) | \
             (newpos[..., 1] < R) | (newpos[..., 1] > self.H - R)
        hit = (rr | ro | rw) & self.alive & ~self.done[:, None]
        # count new contact events (rising edge per robot)
        new_ev = hit & ~self.contact
        self.collisions += new_ev.sum(1)
        self.contact = hit
        moved = ~hit
        self.pos = np.where(moved[..., None], newpos, self.pos)
        self.th = np.where(moved, newth, self.th)
        # odometry: encoders (stall -> 0 when blocked) + compass heading
        v_enc = np.where(moved, self.uact[..., 0], 0.0)
        cm = wrap(self.th + self.cbias + rng.normal(0, c.compass_noise_std, (E, N)))
        thmid = self.est[..., 2] + 0.5 * wrap(cm - self.est[..., 2])
        self.est[..., 0] += v_enc * dt * np.cos(thmid)
        self.est[..., 1] += v_enc * dt * np.sin(thmid)
        self.est[..., 2] = cm
        self.t += dt * (~self.done)

    # ---------------------------------------------------------------- leader
    def leader_cmd(self, e, i):
        c = self.c
        P = self.paths[e]
        x, y, th = self.est[e, i]
        k = self.prog[e, i]
        # advance progress pointer
        while k < len(P) - 1 and np.hypot(*(P[k] - (x, y))) > np.hypot(*(P[k + 1] - (x, y))):
            k += 1
        self.prog[e, i] = k
        Ld = 0.5
        j = k
        while j < len(P) - 1 and np.hypot(*(P[j] - (x, y))) < Ld:
            j += 1
        tgt = P[j]
        dg = np.hypot(*(self.goal[e] - (x, y)))
        if dg < 0.15 and k >= len(P) - 3:
            return 0.0, 0.0, True
        a = wrap(np.arctan2(tgt[1] - y, tgt[0] - x) - th)
        self.lead_a[e, i] = a
        v = c.v_cruise * max(0.0, np.cos(a)) if abs(a) < np.pi / 2 else 0.0
        v = min(v, 0.5 * dg + 0.05)
        if self.pacing:
            fresh = (self.t[e] - self.status_t[e]) < c.status_timeout
            fresh[i] = False
            if fresh.any():
                emax = self.status_err[e][fresh].max()
                scale = np.clip(1 - (emax - c.pace_e0) / c.pace_e1, c.pace_min, 1.0)
                v *= scale
        Lh = max(np.hypot(*(tgt - (x, y))), 0.2)
        w = 2.0 * max(v, 0.08) * np.sin(a) / Lh
        if abs(a) > np.pi / 3:
            w = 1.2 * np.sign(a)
        return v, w, False


# --------------------------------------------------------------------------
# Controllers
# --------------------------------------------------------------------------
def predict_leader(pkt, tau):
    """Constant-twist (unicycle arc) prediction of the leader state."""
    x, y, th, v, w = [pkt[..., k] for k in range(5)]
    small = np.abs(w) < 1e-3
    ws = np.where(small, 1.0, w)
    xn = np.where(small, x + v * tau * np.cos(th), x + v / ws * (np.sin(th + w * tau) - np.sin(th)))
    yn = np.where(small, y + v * tau * np.sin(th), y - v / ws * (np.cos(th + w * tau) - np.cos(th)))
    return np.stack([xn, yn, wrap(th + w * tau)], -1), v, w


def lpsi_control(est, Lpose, vL, wL, off, cfg=Cfg, feedforward=True):
    """Leader-follower tracking of an off-axis control point (equivalent to l-psi
    separation-bearing control). Returns (v, w) and the control-point error."""
    c = cfg
    xL, yL, thL = Lpose[..., 0], Lpose[..., 1], Lpose[..., 2]
    cL, sL = np.cos(thL), np.sin(thL)
    # control-point target is shifted by d along the leader heading so that the
    # robot centre (not the control point) converges to the slot
    ox, oy = off[..., 0] + c.d_ctrl, off[..., 1]
    px = xL + cL * ox - sL * oy
    py = yL + sL * ox + cL * oy
    if feedforward:
        dpx = vL * cL + wL * (-sL * ox - cL * oy)
        dpy = vL * sL + wL * (cL * ox - sL * oy)
    else:
        dpx = dpy = 0.0
    x, y, th = est[..., 0], est[..., 1], est[..., 2]
    ci, si = np.cos(th), np.sin(th)
    ex = px - (x + c.d_ctrl * ci)
    ey = py - (y + c.d_ctrl * si)
    ux = dpx + c.K * ex
    uy = dpy + c.K * ey
    v = ci * ux + si * uy
    w = (-si * ux + ci * uy) / c.d_ctrl
    slot = np.stack([px - c.d_ctrl * cL, py - c.d_ctrl * sL], -1)
    return np.stack([v, w], -1), np.stack([ex, ey], -1), slot


def cbf_filter(cmd, ranges, cfg=Cfg):
    """Closed-form sampled-data barrier on raw range readings.
    For each ray j at body bearing beta_j: h_j = r_j - r_s,j.  For a radially
    mounted sensor and a point-like obstacle on the ray, dr_j/dt = -v cos(beta_j)
    (independent of the turn rate).  Enforcing h_j(k+1) >= (1-lam) h_j(k) gives
    the linear constraint  v cos(beta_j) <= lam (r_j - r_s,j) / T,
    and the minimum-deviation solution of the resulting 1-D QP is a clip on v."""
    c = cfg
    cb = np.cos(c.ray_ang[3])                                   # IR bearing (60 deg)
    lim_us = c.cbf_lam * (ranges[..., 0] - c.r_safe_us) / c.dt   # cos(0)=1 (cone axis)
    lim_ir = c.cbf_lam * (np.minimum(ranges[..., 1], ranges[..., 2]) - c.r_safe_ir) / (c.dt * cb)
    vmax = np.minimum(lim_us, lim_ir)
    out = cmd.copy()
    out[..., 0] = np.maximum(np.minimum(cmd[..., 0], vmax), c.v_min)
    out[..., 0] = np.minimum(out[..., 0], c.v_max)
    out[..., 1] = np.clip(cmd[..., 1], -c.w_max, c.w_max)
    return out, (np.abs(out[..., 0] - np.clip(cmd[..., 0], c.v_min, c.v_max)) > 1e-3)


# --------------------------------------------------------------------------
# One control step for all robots under a given method
# --------------------------------------------------------------------------
METHODS = {
    # name: base controller, leader predictor, barrier filter
    "asa":        dict(base="copy", pred=False, cbf=False),   # original firmware
    "lpsi_zoh":   dict(base="lpsi", pred=False, cbf=True),
    "lpsi_pred":  dict(base="lpsi", pred=True, cbf=True),     # proposed
    "pred_nobyp": dict(base="lpsi", pred=True, cbf=True, bypass=False),
    "pred_nocbf": dict(base="lpsi", pred=True, cbf=False),
    "pred_stop":  dict(base="lpsi", pred=True, cbf=False, stop=True),
    # published baseline: consensus-based formation control with a pinned leader
    # (Ren 2007; Olfati-Saber et al. 2007), all-to-all broadcast, held neighbour states,
    # same planner, pacing, and safety layer as the proposed method
    "consensus":  dict(base="cons", pred=False, cbf=True),
}


CONS_PIN = 2.0   # extra pinning weight on the leader


def consensus_control(sim, off, cfg=Cfg):
    """Consensus formation control (Ren 2007) with a pinned leader. Follower i forms
    the formation reference implied by each received neighbour state,
    xi_j = p_j - R(theta_ref) (o_j - o_L), and tracks the weighted average
    xi* = (sum_j a_ij xi_j + b xi_L) / (sum_j a_ij + b) with its own slot offset.
    Neighbour states are held between packets; theta_ref and the feed-forward twist
    come from the leader's last packet."""
    c = cfg
    E, N = sim.E, sim.N
    L = sim.bel_leader
    thr = sim.pkt[..., 2]
    vL, wL = sim.pkt[..., 3], sim.pkt[..., 4]
    fo = sim.form_off[sim.form_rx]                                   # E,N(i),N(j),2
    oL = np.take_along_axis(fo, L[:, :, None, None].repeat(2, -1), 2)  # E,N,1,2
    oj = fo - oL                                                      # o_j - o_L as seen by i
    cr, sr = np.cos(thr)[..., None], np.sin(thr)[..., None]
    rx = cr * oj[..., 0] - sr * oj[..., 1]
    ry = sr * oj[..., 0] + cr * oj[..., 1]
    P = sim.nb_pkt[..., :2].copy()
    own = np.arange(N)
    P[:, own, own] = sim.est[:, :, :2]                                # own state is always fresh
    fresh = (sim.t[:, None, None] - sim.nb_rx) < c.tau_stop
    fresh[:, own, own] = True
    w = fresh.astype(float)
    w += CONS_PIN * (np.arange(N)[None, None, :] == L[:, :, None]) * fresh
    xi_x = P[..., 0] - rx
    xi_y = P[..., 1] - ry
    sw = np.maximum(w.sum(-1), 1e-9)
    Lv = np.stack([(w * xi_x).sum(-1) / sw, (w * xi_y).sum(-1) / sw, thr], -1)
    return lpsi_control(sim.est, Lv, vL, wL, off, c)


def control_step(sim, method):
    """Computes commands for every robot. Returns cmd and info dict."""
    c = sim.c
    m = METHODS[method]
    E, N = sim.E, sim.N
    ranges = sim.ranges
    off = sim.slot_offsets()
    age = sim.t[:, None] - sim.last_rx
    tau = np.minimum(age, c.tau_stop)
    if m["pred"]:
        Lhat, vL, wL = predict_leader(sim.pkt, tau)
    else:
        Lhat, vL, wL = sim.pkt[..., :3], sim.pkt[..., 3], sim.pkt[..., 4]
    if m["base"] == "lpsi":
        nom, err, tgt = lpsi_control(sim.est, Lhat, vL, wL, off, c)
    elif m["base"] == "cons":
        nom, err, tgt = consensus_control(sim, off, c)
    else:
        # original ASA: followers replay the leader's last broadcast twist
        nom = np.stack([sim.pkt[..., 3], sim.pkt[..., 4]], -1)
        _, err, tgt = lpsi_control(sim.est, sim.pkt[..., :3], vL, wL, off, c)
    nom[..., 0] = np.clip(nom[..., 0], c.v_min, c.v_max)
    nom[..., 1] = np.clip(nom[..., 1], -c.w_max, c.w_max)
    stale = age > c.tau_stop
    nom[stale] = 0.0
    sim.my_err = np.hypot(tgt[..., 0] - sim.est[..., 0], tgt[..., 1] - sim.est[..., 1])
    cmd = nom.copy()
    cmd[stale] = 0.0
    if m["cbf"]:
        filt, interv = cbf_filter(cmd, ranges, c)
    else:
        filt = cmd.copy()
        interv = np.zeros((E, N), bool)
        if method == "asa" or m.get("stop"):
            # original firmware rule: stop when front ultrasonic < 20 cm
            filt[..., 0] = np.where(ranges[..., 0] < 0.20, 0.0, filt[..., 0])
    # leaders: pure pursuit on the planned path (+ same barrier filter)
    lead_done = np.zeros(E, bool)
    for e in range(E):
        for i in range(N):
            if sim.is_leader[e, i] and sim.alive[e, i]:
                v, w, dn = sim.leader_cmd(e, i)
                lead_done[e] |= dn
                filt[e, i] = (v, w)
                cmd[e, i] = (v, w)
    if m["cbf"] or method == "asa":
        lf, li = cbf_filter(filt, ranges, c)
        L = sim.is_leader & sim.alive
        filt[L] = lf[L] if m["cbf"] else filt[L]
        if m["cbf"]:
            # rotate-to-clear: if the barrier blocks the leader, turn in place toward the path side
            blocked = L & (filt[..., 0] < 0.03) & (cmd[..., 0] > 0.05)
            side = np.where(np.abs(sim.lead_a) > 0.05, np.sign(sim.lead_a),
                            np.where(ranges[..., 1] >= ranges[..., 2], 1.0, -1.0))
            # commit to a turn direction while blocked (hysteresis)
            sim.rot_dir = np.where(blocked & (sim.rot_dir == 0), side, sim.rot_dir)
            sim.rot_dir = np.where(blocked | (ranges[..., 0] < 0.25), sim.rot_dir, 0.0)
            filt[..., 1] = np.where(blocked, sim.rot_dir * 0.8, filt[..., 1])
            # followers: rotate-and-advance deadlock breaker (same rule, plus a short
            # committed advance once the cone is clear)
            if m.get("bypass", True):
                F = ~sim.is_leader & sim.alive & ~stale
                th = sim.est[..., 2]
                eyb = -np.sin(th) * err[..., 0] + np.cos(th) * err[..., 1]
                fblk = F & (filt[..., 0] < 0.03) & (cmd[..., 0] > 0.05)
                fside = np.where(np.abs(eyb) > 0.05, np.sign(eyb),
                                 np.where(ranges[..., 1] >= ranges[..., 2], 1.0, -1.0))
                recent = (sim.t[:, None] - sim.byp_last) < 2.0
                sim.byp_dir = np.where(fblk & ~recent, fside, sim.byp_dir)
                sim.byp_last = np.where(fblk, sim.t[:, None], sim.byp_last)
                # rotate while blocked
                filt[..., 1] = np.where(fblk, sim.byp_dir * 0.8, filt[..., 1])
                # when it just cleared, start a committed advance of 0.8 s
                cleared = F & ~fblk & recent & (sim.byp_adv <= 0) & ((sim.t[:, None] - sim.byp_last) < c.dt * 1.5)
                sim.byp_adv = np.where(cleared, 0.8, sim.byp_adv)
                adv = F & (sim.byp_adv > 0) & ~fblk
                v_adv = np.minimum(0.15, filt[..., 0] * 0 + cbf_filter(np.stack([np.full((E, N), 0.15), np.zeros((E, N))], -1), ranges, c)[0][..., 0])
                filt[..., 0] = np.where(adv, v_adv, filt[..., 0])
                filt[..., 1] = np.where(adv, 0.0, filt[..., 1])
                sim.byp_adv = np.maximum(sim.byp_adv - c.dt, 0.0)
    info = dict(err=err, tgt=tgt, off=off, nom=nom, interv=interv,
                lead_done=lead_done, stale=stale, cmd_prefilter=cmd)
    return filt, info


def gt_formation_error(sim):
    """Ground-truth formation error of each follower w.r.t. the true pose of the
    robot it believes to be leader, at the slot it believes it holds."""
    E, N = sim.E, sim.N
    off = sim.slot_offsets()
    L = sim.bel_leader
    Lp = sim.pos[np.arange(E)[:, None], L]
    Lt = sim.th[np.arange(E)[:, None], L]
    c, s = np.cos(Lt), np.sin(Lt)
    tx = Lp[..., 0] + c * off[..., 0] - s * off[..., 1]
    ty = Lp[..., 1] + s * off[..., 0] + c * off[..., 1]
    e = np.hypot(sim.pos[..., 0] - tx, sim.pos[..., 1] - ty)
    fol = sim.alive & ~sim.is_leader
    return e, fol
