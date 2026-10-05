// SwarmAgent.h - C++ port of ros2_ws/src/swarm_autobots/swarm_autobots/agent.py
// Header-only, no heap allocation, no STL; builds for ESP32 (Arduino) and on a PC.
// One instance per robot; call step() at 20 Hz and feed received ESP-NOW frames
// through onLeaderState() / onStatus(). Units: m, rad, s.
#pragma once
#include <math.h>
#include <stdint.h>

namespace swarm {

static const int MAX_ROBOTS = 20;      // ESP-NOW unicast peer limit; broadcast has none
static const int MAX_PATH = 256;

struct Params {
  double dt = 0.05, d = 0.10, K = 1.0;
  double v_max = 0.40, v_min = -0.05, w_max = 2.0, v_cruise = 0.20;
  double tau_stop = 1.0, tau_elect = 1.5, T_backoff = 0.2;
  double lam = 0.25, r_safe_us = 0.10, r_safe_ir = 0.10, ir_bearing = M_PI / 3;  // 60 deg
  double pace_e0 = 0.25, pace_e1 = 0.50, pace_min = 0.2, status_timeout = 1.0;
  double lookahead = 0.5;
  bool predict = true;   // false: zero-order hold on the last leader packet (hardware ZOH runs)
};

// Leader state frame (20 B on air, see frames.py)
struct LeaderState { uint8_t leader_id; uint16_t seq; uint8_t formation_id; double x, y, theta, v, w; };
struct Output { double v = 0, w = 0; int kind = 0;   // 0 none, 1 LeaderState, 2 status
                LeaderState ls{}; int status_id = 0, status_leader = 0; double status_err = 0; };

// centre-to-centre slots in the leader frame, hardware geometry
static const double FORM[2][4][2] = {
  {{0.0, 0.0}, {-0.5, 0.0}, {-1.0, -0.5}, {-1.0, 0.5}},   // triangle
  {{0.0, 0.0}, {-0.5, 0.0}, {-1.3, -0.5}, {-1.3, 0.5}}};  // Y

inline double wrapAngle(double a) {
  double r = fmod(a + M_PI, 2 * M_PI);
  if (r < 0) r += 2 * M_PI;
  return r - M_PI;
}
inline double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double signd(double x) { return x < 0 ? -1.0 : 1.0; }

class SwarmAgent {
 public:
  SwarmAgent(int robot_id, int n_robots, int formation_id = 0, Params p = Params())
      : id_(robot_id), N_(n_robots), P_(p), formation_id_(formation_id) {
    is_leader_ = (robot_id == 0);
    for (int j = 0; j < MAX_ROBOTS; ++j) { failed_[j] = false; status_t_[j] = -1e18; status_e_[j] = 0; }
  }

  // ------------------------------------------------------------ messages in
  void onLeaderState(const LeaderState& m, double t) {
    int j = m.leader_id;
    if (j == id_) return;
    if (is_leader_) {
      if (j < id_) is_leader_ = false;   // yield to lower rank
      else return;
    }
    if (j < 0 || j >= MAX_ROBOTS) return;
    if (j <= leader_ || isFailed(leader_) || has_cand_) {
      leader_ = j; failed_[j] = false; has_cand_ = false;
    }
    if (j == leader_) { pkt_ = m; has_pkt_ = true; t_rx_ = t; formation_id_ = m.formation_id; }
  }
  void onStatus(int robot_id, double err, double t) {
    if (is_leader_ && robot_id >= 0 && robot_id < MAX_ROBOTS) { status_e_[robot_id] = err; status_t_[robot_id] = t; }
  }
  void setPath(const double (*xy)[2], int n) {
    n_path_ = n > MAX_PATH ? MAX_PATH : n;
    for (int k = 0; k < n_path_; ++k) { path_[k][0] = xy[k][0]; path_[k][1] = xy[k][1]; }
    prog_ = 0;
  }
  void commandFormation(int fid) { if (is_leader_) formation_id_ = fid; }

  bool isLeader() const { return is_leader_; }
  int leader() const { return leader_; }
  int formation() const { return formation_id_; }
  double slotError() const { return my_err_; }
  bool hasPacket() const { return has_pkt_; }
  double packetAge(double t) const { return has_pkt_ ? t - t_rx_ : -1.0; }   // s, -1 before the first packet
  void setPredict(bool on) { P_.predict = on; }

  // ------------------------------------------------------------ main cycle (20 Hz)
  Output step(double t, double x, double y, double th, double rf, double rl, double rr) {
    Output out;
    t_ = t;
    election(t);
    if (is_leader_) {
      double a; double vn, wn; pursuit(x, y, th, vn, wn, a);
      double v = barrier(vn, rf, rl, rr), w = wn;
      if (vn > 0.05 && v < 0.03) {                       // rotate-to-clear
        if (rot_dir_ == 0) rot_dir_ = fabs(a) > 0.05 ? signd(a) : (rl >= rr ? 1.0 : -1.0);
        w = 0.8 * rot_dir_;
      } else if (rf >= 0.25) {
        rot_dir_ = 0;
      }
      seq_ = (seq_ + 1) % 65536;
      out.v = v; out.w = w; out.kind = 1;
      out.ls = LeaderState{(uint8_t)id_, (uint16_t)seq_, (uint8_t)formation_id_, x, y, th, v, w};
      return out;
    }
    if (!has_pkt_ || t - t_rx_ > P_.tau_stop) return out;
    double tau = t - t_rx_; if (tau > P_.tau_stop) tau = P_.tau_stop;
    double Lx, Ly, Lth, vL, wL; predict(tau, Lx, Ly, Lth, vL, wL);
    double vn, wn, eyb; track(x, y, th, Lx, Ly, Lth, vL, wL, vn, wn, eyb);
    vn = clampd(vn, P_.v_min, P_.v_max); wn = clampd(wn, -P_.w_max, P_.w_max);
    double v = barrier(vn, rf, rl, rr), w = wn;
    bool blocked = vn > 0.05 && v < 0.03;                // rotate-and-advance deadlock breaker
    bool recent = t - byp_last_ < 2.0;
    if (blocked) {
      if (!recent) byp_dir_ = fabs(eyb) > 0.05 ? signd(eyb) : (rl >= rr ? 1.0 : -1.0);
      byp_last_ = t;
      w = 0.8 * byp_dir_;
    } else if (recent && byp_adv_ <= 0 && t - byp_last_ < 1.5 * P_.dt) {
      byp_adv_ = 0.8;
    }
    if (!blocked && byp_adv_ > 0) { v = barrier(0.15, rf, rl, rr); w = 0.0; }
    byp_adv_ = byp_adv_ - P_.dt; if (byp_adv_ < 0) byp_adv_ = 0;
    out.v = v; out.w = w; out.kind = 2;
    out.status_id = id_; out.status_leader = leader_; out.status_err = my_err_;
    return out;
  }

 private:
  int id_, N_; Params P_;
  int formation_id_; bool is_leader_; int leader_ = 0;
  bool failed_[MAX_ROBOTS]; bool has_cand_ = false; double cand_deadline_ = 0;
  LeaderState pkt_{}; bool has_pkt_ = false; double t_rx_ = 0;
  double status_e_[MAX_ROBOTS], status_t_[MAX_ROBOTS];
  double path_[MAX_PATH][2]; int n_path_ = 0, prog_ = 0;
  bool goal_reached_ = false; int seq_ = 0;
  double byp_dir_ = 0, byp_last_ = -1e9, byp_adv_ = 0, rot_dir_ = 0, my_err_ = 0, t_ = 0;

  bool isFailed(int j) const { return j >= 0 && j < MAX_ROBOTS && failed_[j]; }
  void election(double t) {
    if (is_leader_) return;
    if (!has_cand_ && t - t_rx_ > P_.tau_elect) {
      if (leader_ >= 0 && leader_ < MAX_ROBOTS) failed_[leader_] = true;
      int b = 0; for (int j = 0; j < id_; ++j) if (!failed_[j]) ++b;
      cand_deadline_ = t + b * P_.T_backoff; has_cand_ = true;
    }
    if (has_cand_ && t >= cand_deadline_) {
      is_leader_ = true; leader_ = id_; has_cand_ = false;
      if (n_path_ > 0) prog_ = 0;
    }
  }
  void slot(double& ox, double& oy) const {
    const double (*F)[2] = FORM[formation_id_];
    ox = F[id_ % 4][0] - F[leader_ % 4][0]; oy = F[id_ % 4][1] - F[leader_ % 4][1];
  }
  double dist(int k, double x, double y) const { return hypot(path_[k][0] - x, path_[k][1] - y); }
  void pursuit(double x, double y, double th, double& v, double& w, double& a) {
    v = w = a = 0;
    if (n_path_ == 0) return;
    if (prog_ == 0) {
      int best = 0; double bd = 1e300;
      for (int k = 0; k < n_path_; ++k) {
        double d2 = (path_[k][0] - x) * (path_[k][0] - x) + (path_[k][1] - y) * (path_[k][1] - y);
        if (d2 < bd) { bd = d2; best = k; }
      }
      prog_ = best;
    }
    int k = prog_;
    while (k < n_path_ - 1 && dist(k, x, y) > dist(k + 1, x, y)) ++k;
    prog_ = k;
    int j = k;
    while (j < n_path_ - 1 && dist(j, x, y) < P_.lookahead) ++j;
    double dg = dist(n_path_ - 1, x, y);
    if (dg < 0.15 && k >= n_path_ - 3) { goal_reached_ = true; return; }
    a = wrapAngle(atan2(path_[j][1] - y, path_[j][0] - x) - th);
    v = fabs(a) < M_PI / 2 ? P_.v_cruise * fmax(0.0, cos(a)) : 0.0;
    v = fmin(v, 0.5 * dg + 0.05);
    double emax = -1e300; bool fresh = false;
    for (int r = 0; r < MAX_ROBOTS; ++r)
      if (t_ - status_t_[r] < P_.status_timeout) { fresh = true; if (status_e_[r] > emax) emax = status_e_[r]; }
    if (fresh) v *= fmin(1.0, fmax(P_.pace_min, 1 - (emax - P_.pace_e0) / P_.pace_e1));
    double Lh = fmax(dist(j, x, y), 0.2);
    w = 2.0 * fmax(v, 0.08) * sin(a) / Lh;
    if (fabs(a) > M_PI / 3) w = 1.2 * signd(a);
  }
  void predict(double tau, double& x, double& y, double& th, double& v, double& w) const {
    x = pkt_.x; y = pkt_.y; th = pkt_.theta; v = pkt_.v; w = pkt_.w;
    if (!P_.predict) return;                             // zero-order hold
    if (fabs(w) < 1e-3) { x += v * tau * cos(th); y += v * tau * sin(th); return; }
    double thn = th + w * tau;
    x += v / w * (sin(thn) - sin(th)); y -= v / w * (cos(thn) - cos(th)); th = wrapAngle(thn);
  }
  void track(double x, double y, double th, double Lx, double Ly, double Lth, double vL, double wL,
             double& v, double& w, double& eyb) {
    double ox, oy; slot(ox, oy); ox += P_.d;
    double c = cos(Lth), s = sin(Lth);
    double px = Lx + c * ox - s * oy, py = Ly + s * ox + c * oy;
    double dpx = vL * c + wL * (-s * ox - c * oy), dpy = vL * s + wL * (c * ox - s * oy);
    double ci = cos(th), si = sin(th);
    double ex = px - (x + P_.d * ci), ey = py - (y + P_.d * si);
    double ux = dpx + P_.K * ex, uy = dpy + P_.K * ey;
    v = ci * ux + si * uy; w = (-si * ux + ci * uy) / P_.d;
    my_err_ = hypot(px - P_.d * c - x, py - P_.d * s - y);
    eyb = -si * ex + ci * ey;
  }
  double barrier(double v, double rf, double rl, double rr) const {
    double lim = fmin(P_.lam * (rf - P_.r_safe_us) / P_.dt,
                      P_.lam * (fmin(rl, rr) - P_.r_safe_ir) / (P_.dt * cos(P_.ir_bearing)));
    return fmin(P_.v_max, fmax(P_.v_min, fmin(v, lim)));
  }
};

}  // namespace swarm
