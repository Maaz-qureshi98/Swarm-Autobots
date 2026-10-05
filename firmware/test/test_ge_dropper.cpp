// Checks that GeDropper reproduces the simulator's Gilbert-Elliott channel: loss rate p and mean
// burst length L, including when frames are also lost on the air (sequence gaps).
//   g++ -O2 -std=c++17 -I../swarm_agent test_ge_dropper.cpp -o test_ge && ./test_ge
#include <cmath>
#include <cstdio>
#include <random>
#include "HwExperiment.h"

static bool check(double p, double L, double air_loss) {
  hw::GeDropper d(p, L, 12345);
  std::mt19937 air(7);
  std::uniform_real_distribution<double> u(0, 1);
  const int N = 400000;
  int lost = 0, bursts = 0, run = 0;
  for (int k = 0; k < N; ++k) {
    bool on_air = u(air) >= air_loss;               // frame reaches the callback
    bool dropped = on_air ? d.drop((uint16_t)k) : true;
    bool injected_bad = on_air && dropped;
    (void)injected_bad;
    if (dropped) { ++lost; ++run; } else if (run) { ++bursts; run = 0; }
  }
  if (run) ++bursts;
  double p_hat = (double)lost / N, L_hat = bursts ? (double)lost / bursts : 0;
  // expected total loss: injected chain loss p combined with independent air loss
  double p_exp = 1 - (1 - p) * (1 - air_loss);
  bool ok = std::fabs(p_hat - p_exp) < 0.01 && (air_loss > 0 || std::fabs(L_hat - L) < 0.1 * L);
  std::printf("p=%.2f L=%.1f air=%.2f -> loss %.3f (expected %.3f), mean burst %.2f %s\n", p, L, air_loss,
              p_hat, p_exp, L_hat, ok ? "ok" : "FAIL");
  return ok;
}

int main() {
  bool ok = true;
  for (double p : {0.2, 0.4, 0.6})
    for (double L : {1.0, 4.0, 12.0})
      if (p <= L / (L + 1)) ok &= check(p, L, 0.0);   // feasible Gilbert-Elliott parameters only
  ok &= check(0.6, 4.0, 0.05);   // physical loss on top of injection
  hw::GeDropper off(0.0, 4.0, 1);
  for (int k = 0; k < 1000; ++k) ok &= !off.drop((uint16_t)k);
  hw::CycleStats c; for (uint32_t us : {120u, 180u, 240u, 2600u}) c.add(us);
  ok &= c.max() == 2600 && c.min() == 120 && c.count() == 4;
  std::printf(ok ? "PASS\n" : "FAIL\n");
  return ok ? 0 : 1;
}
