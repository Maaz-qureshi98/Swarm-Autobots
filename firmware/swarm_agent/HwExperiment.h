// HwExperiment.h - instrumentation for the hardware revision trials (header-only, no heap).
//
//  * GeDropper   Gilbert-Elliott packet-drop injection on the receive side. Call drop(seq) in the
//                ESP-NOW receive callback for every LeaderState frame BEFORE handing it to the agent;
//                if it returns true, discard the frame. The chain advances once per broadcast period,
//                using the frame's sequence number, so frames lost on the air also advance it.
//                Same model as the simulator: loss rate p, mean burst length L, bad->good 1/L,
//                good->bad p/(L(1-p)), start in the good state.
//  * CycleStats  micros() timing of the control cycle (agent.step() plus whatever you bracket).
//  * logLine     one CSV line per control cycle for tools/analyze_hw_trials.py.
//
// Typical follower sketch (see docs/hardware_revision_protocol.md):
//   swarm::SwarmAgent agent(MY_RANK, 4, 1);             // Y formation
//   hw::GeDropper drop(0.6, 4.0, 0xC0FFEE + MY_RANK);   // p = 0.6, L = 4, per-robot seed
//   hw::CycleStats cyc;
//   void onRecv(...) { LeaderState m = unpack(data);
//                      if (drop.drop(m.seq)) return;    // injected loss
//                      agent.onLeaderState(m, now_s()); }
//   void loop() {   // every 50 ms
//     uint32_t t0 = micros();
//     swarm::Output o = agent.step(now_s(), x, y, th, us, irl, irr);
//     cyc.add(micros() - t0);
//     hw::logLine(Serial, millis(), MY_RANK, agent, now_s(), drop, cyc.last(), o.v, o.w);
//   }
#pragma once
#include <stdint.h>

namespace hw {

class GeDropper {
 public:
  // p in [0, 0.95]; L >= 1; p = 0 disables injection. The model needs p <= L / (L + 1)
  // (e.g. p <= 0.5 for L = 1); larger p is reached with a longer mean burst only.
  GeDropper(double p = 0.0, double L = 1.0, uint32_t seed = 1) { configure(p, L, seed); }
  void configure(double p, double L, uint32_t seed) {
    if (p < 0) p = 0; if (p > 0.95) p = 0.95; if (L < 1) L = 1;
    p_ = p; q_ = 1.0 / L; s_ = p > 0 ? p * q_ / (1.0 - p) : 0.0; if (s_ > 1.0) s_ = 1.0;
    rng_ = seed ? seed : 1; bad_ = false; have_seq_ = false; n_rx_ = n_drop_ = 0;
  }
  // Returns true if this frame must be dropped. seq is the 16-bit LeaderState sequence number.
  bool drop(uint16_t seq) {
    if (p_ <= 0) { ++n_rx_; return false; }
    int steps = 1;
    if (have_seq_) {
      steps = (uint16_t)(seq - last_seq_);             // broadcast periods since the last frame
      if (steps <= 0) return true;                     // duplicate or reordered: drop
      if (steps > 200) steps = 200;                    // long gap: chain has mixed anyway
    }
    last_seq_ = seq; have_seq_ = true;
    for (int k = 0; k < steps; ++k) bad_ = bad_ ? (uni() >= q_) : (uni() < s_);
    ++n_rx_;
    if (bad_) { ++n_drop_; return true; }
    return false;
  }
  double p() const { return p_; }
  double L() const { return 1.0 / q_; }
  uint32_t received() const { return n_rx_; }     // frames that reached the callback
  uint32_t dropped() const { return n_drop_; }    // of those, dropped by injection

 private:
  double p_ = 0, q_ = 1, s_ = 0; uint32_t rng_ = 1; bool bad_ = false;
  bool have_seq_ = false; uint16_t last_seq_ = 0; uint32_t n_rx_ = 0, n_drop_ = 0;
  double uni() {                                   // xorshift32, uniform in [0, 1)
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    return (rng_ >> 8) * (1.0 / 16777216.0);
  }
};

class CycleStats {
 public:
  void add(uint32_t us) {
    last_ = us; ++n_; sum_ += us; if (us > max_) max_ = us; if (us < min_) min_ = us;
    int b = us / 50; if (b >= NB) b = NB - 1; ++hist_[b];   // 50 us bins up to 10 ms
  }
  uint32_t last() const { return last_; }
  uint32_t count() const { return n_; }
  double mean() const { return n_ ? (double)sum_ / n_ : 0.0; }
  uint32_t max() const { return max_; }
  uint32_t min() const { return n_ ? min_ : 0; }
  uint32_t percentile(double q) const {            // upper edge of the bin holding quantile q
    uint32_t target = (uint32_t)(q * n_), c = 0;
    for (int b = 0; b < NB; ++b) { c += hist_[b]; if (c > target) return (b + 1) * 50; }
    return NB * 50;
  }
  template <class Out> void print(Out& out) const {
    out.printf("# cycle_us n=%lu mean=%.1f min=%lu p99=%lu max=%lu\n", (unsigned long)n_, mean(),
               (unsigned long)min(), (unsigned long)percentile(0.99), (unsigned long)max_);
  }

 private:
  static const int NB = 200;
  uint32_t hist_[NB] = {0}; uint32_t last_ = 0, n_ = 0, max_ = 0, min_ = 0xFFFFFFFF; uint64_t sum_ = 0;
};

// CSV, one line per cycle:
// ms,rank,is_leader,leader,pkt_age_ms,rx,injected_drops,slot_err_mm,v_mm_s,w_mrad_s,cycle_us,predict
template <class Out, class Agent>
void logLine(Out& out, uint32_t ms, int rank, const Agent& a, double now_s, const GeDropper& d,
             uint32_t cycle_us, double v, double w, bool predict) {
  double age = a.packetAge(now_s);
  out.printf("%lu,%d,%d,%d,%ld,%lu,%lu,%ld,%ld,%ld,%lu,%d\n", (unsigned long)ms, rank, a.isLeader() ? 1 : 0,
             a.leader(), age < 0 ? -1L : (long)(age * 1000.0 + 0.5), (unsigned long)d.received(),
             (unsigned long)d.dropped(), (long)(a.slotError() * 1000.0 + 0.5), (long)(v * 1000.0),
             (long)(w * 1000.0), (unsigned long)cycle_us, predict ? 1 : 0);
}

}  // namespace hw
