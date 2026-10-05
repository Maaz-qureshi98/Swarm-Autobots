// Cross-compile check: instantiates the agent and runs one cycle, so the compiler emits the full
// control path for the ESP32 (Xtensa LX6). Build: see firmware/README.md.
#include "SwarmAgent.h"
extern "C" double esp32_cycle(double t, double x, double y, double th, double rf, double rl, double rr) {
  static swarm::SwarmAgent agent(1, 4, 0);
  swarm::LeaderState m{0, 1, 0, 0.0, 0.0, 0.0, 0.2, 0.0};
  agent.onLeaderState(m, t);
  swarm::Output o = agent.step(t, x, y, th, rf, rl, rr);
  return o.v + o.w;
}
