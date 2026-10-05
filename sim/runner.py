import time
import numpy as np
from core import SwarmSim, control_step, gt_formation_error, Cfg


def run(method, E=50, N=4, seed=0, record=False, settle=3.0, switch_time=None, **simkw):
    sim = SwarmSim(E, N, seed=seed, **simkw)
    c = sim.c
    sq_gt = np.zeros(E); sq_est = np.zeros(E); cnt = np.zeros(E)
    max_gt = np.zeros(E)
    interv = np.zeros(E); icnt = np.zeros(E)
    min_sep = np.full(E, np.inf)
    final_gt = np.full(E, np.inf)
    step = 0
    rec = [] if record else None
    fail_t = simkw.get("fail_time", None)
    sw_t = simkw.pop("switch_time", None) if False else switch_time
    sw_lat = np.full(E, np.nan)
    sw_done = np.zeros(E, bool)
    recover_t = np.full(E, np.nan)
    while not sim.done.all():
        if step % c.comm_every == 0:
            sim.comm_step()
        sim.election_step()
        sim.ranges = sim.sense()
        cmd, info = control_step(sim, method)
        if fail_t is not None:
            f = (~sim.leader_ever_failed) & (sim.t >= fail_t)
            if f.any():
                sim.alive[f, 0] = False
                sim.is_leader[f, 0] = False
                # protocol: failed leader is removed from the arena (retrieved)
                sim.pos[f, 0] = -50.0
                sim.leader_ever_failed[f] = True
        if sw_t is not None:
            s_now = (~sw_done) & (sim.t >= sw_t)
            if s_now.any():
                sim.form_cmd[s_now] = sim.FORMS.index(sim.switch_to)
                L = sim.is_leader & sim.alive
                sim.form_rx[s_now[:, None] & L] = sim.form_cmd[np.where(s_now[:, None] & L)[0]]
                sw_done |= s_now
        sim.physics(cmd)
        step += 1
        # goal bookkeeping
        newgoal = info["lead_done"] & np.isinf(sim.goal_time)
        sim.goal_time[newgoal] = sim.t[newgoal]
        # metrics
        e_gt, fol = gt_formation_error(sim)
        e_est = np.hypot(info["tgt"][..., 0] - sim.est[..., 0], info["tgt"][..., 1] - sim.est[..., 1])
        act = (~sim.done) & (sim.t > settle)
        m = fol & act[:, None]
        sq_gt += (e_gt ** 2 * m).sum(1)
        sq_est += (e_est ** 2 * m).sum(1)
        cnt += m.sum(1)
        max_gt = np.maximum(max_gt, np.where(m, e_gt, 0).max(1))
        interv += (info["interv"] & m).sum(1); icnt += m.sum(1)
        P = sim.pos
        d = np.sqrt(((P[:, :, None] - P[:, None]) ** 2).sum(-1)) + np.eye(N) * 1e9
        min_sep = np.where(~sim.done, np.minimum(min_sep, d.min((1, 2))), min_sep)
        if fail_t is not None:
            ok = sim.leader_ever_failed & np.isnan(recover_t) & (sim.t > fail_t + 0.2)
            alive_fol = fol
            allin = np.where(alive_fol, e_gt < 0.30, True).all(1) & (sim.is_leader & sim.alive).any(1)
            # require a single agreed leader among alive robots
            agree = np.array([len(set(sim.bel_leader[e][sim.alive[e]])) == 1 for e in range(E)])
            r = ok & allin & agree
            recover_t[r] = sim.t[r] - fail_t
        if sw_t is not None:
            folm = sim.alive & ~sim.is_leader
            ok = sw_done & np.isnan(sw_lat) & np.where(folm, (sim.form_rx == sim.form_cmd[:, None]) & (sim.my_err < 0.10), True).all(1)
            sw_lat[ok] = sim.t[ok] - sw_t
        if record:
            rec.append(dict(t=sim.t.copy(), pos=sim.pos.copy(), th=sim.th.copy(), est=sim.est.copy(),
                            uact=sim.uact.copy(), done=sim.done.copy(),
                            e_gt=e_gt.copy(), fol=fol.copy(), leader=sim.bel_leader.copy(),
                            is_leader=sim.is_leader.copy(), alive=sim.alive.copy(),
                            interv=info["interv"].copy(), stale=info["stale"].copy()))
        fin = (sim.t >= sim.goal_time + 15.0) | (sim.t >= sim.max_time)
        newly = fin & ~sim.done
        if newly.any():
            final_gt[newly] = np.where(fol[newly], e_gt[newly], 0).max(1)
        sim.done |= fin
    reached = np.isfinite(sim.goal_time)
    out = dict(
        rmse_gt=np.sqrt(sq_gt / np.maximum(cnt, 1)),
        rmse_est=np.sqrt(sq_est / np.maximum(cnt, 1)),
        max_gt=max_gt,
        collisions=sim.collisions.astype(float),
        min_sep=min_sep - 2 * c.R,
        interv=interv / np.maximum(icnt, 1),
        reached=reached.astype(float),
        goal_time=np.where(reached, sim.goal_time, np.nan),
        final_gt=np.where(final_gt > 20, np.nan, final_gt),
        success=(reached & (final_gt < 0.50) & (sim.collisions == 0)).astype(float),
        false_elections=sim.false_elections.astype(float),
        recover_t=recover_t,
        fa_ok=sim.fa_ok.astype(float),
        switch_lat=sw_lat,
    )
    return out, sim, rec


if __name__ == "__main__":
    for m in ["asa", "lpsi_zoh", "lpsi_pred", "lpsi_apf", "pred_nocbf"]:
        t0 = time.time()
        o, sim, _ = run(m, E=40, N=4, form="triangle", p_loss=0.2, seed=1)
        print(f"{m:11s} rmse_gt {o['rmse_gt'].mean():.3f} est {o['rmse_est'].mean():.3f} "
              f"coll {o['collisions'].mean():.2f} succ {o['success'].mean():.2f} "
              f"reach {o['reached'].mean():.2f} int {o['interv'].mean():.3f} "
              f"minsep {o['min_sep'].mean():.3f} t={time.time()-t0:.1f}s")
