// Replays trace.txt (recorded from the Python reference agent) through the C++ SwarmAgent
// and checks that every command and outgoing frame matches.
//   g++ -O2 -std=c++17 -I../swarm_agent test_equivalence.cpp -o test_equivalence && ./test_equivalence trace.txt
#include <cstdio>
#include <cstring>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "SwarmAgent.h"
using namespace swarm;

int main(int argc, char** argv) {
  std::ifstream in(argc > 1 ? argv[1] : "trace.txt");
  if (!in) { std::printf("cannot open trace\n"); return 2; }
  std::vector<SwarmAgent> ag; std::string line, scen;
  long steps = 0, bad = 0, scenarios = 0; double maxdv = 0, maxdw = 0, maxde = 0;
  while (std::getline(in, line)) {
    std::istringstream s(line); char c; s >> c;
    if (c == 'N') {
      int n, f; s >> scen >> n >> f; ag.clear();
      for (int i = 0; i < n; ++i) ag.emplace_back(i, n, f);
      ++scenarios;
    } else if (c == 'P') {
      int id, n; s >> id >> n; std::vector<double> xy(2 * n);
      for (int k = 0; k < 2 * n; ++k) s >> xy[k];
      ag[id].setPath(reinterpret_cast<const double (*)[2]>(xy.data()), n);
    } else if (c == 'F') {
      int id, f; s >> id >> f; ag[id].commandFormation(f);
    } else if (c == 'L') {
      int id, lid, seq, fid; double t; LeaderState m{};
      s >> id >> t >> lid >> seq >> fid >> m.x >> m.y >> m.theta >> m.v >> m.w;
      m.leader_id = lid; m.seq = seq; m.formation_id = fid; ag[id].onLeaderState(m, t);
    } else if (c == 'T') {
      int id, rid; double t, e; s >> id >> t >> rid >> e; ag[id].onStatus(rid, e, t);
    } else if (c == 'S') {
      int id, kind; double t, x, y, th, rf, rl, rr, v, w; s >> id >> t >> x >> y >> th >> rf >> rl >> rr >> v >> w >> kind;
      Output o = ag[id].step(t, x, y, th, rf, rl, rr);
      double dv = std::fabs(o.v - v), dw = std::fabs(o.w - w), de = 0; bool ok = o.kind == kind;
      if (kind == 1) { int seq, fid; s >> seq >> fid; ok = ok && o.ls.seq == seq && o.ls.formation_id == fid; }
      if (kind == 2) { int lid; double e; s >> lid >> e; de = std::fabs(o.status_err - e); ok = ok && o.status_leader == lid; }
      maxdv = std::fmax(maxdv, dv); maxdw = std::fmax(maxdw, dw); maxde = std::fmax(maxde, de);
      if (!ok || dv > 1e-9 || dw > 1e-9 || de > 1e-9) {
        if (bad < 5) std::printf("mismatch %s robot %d t=%.2f: kind %d/%d v %.12f/%.12f w %.12f/%.12f\n",
                                 scen.c_str(), id, t, o.kind, kind, o.v, v, o.w, w);
        ++bad;
      }
      ++steps;
    }
  }
  std::printf("%ld scenarios, %ld agent steps, %ld mismatches, max |dv| %.2e, |dw| %.2e, |de| %.2e\n",
              scenarios, steps, bad, maxdv, maxdw, maxde);
  std::printf(bad == 0 ? "PASS\n" : "FAIL\n");
  return bad == 0 ? 0 : 1;
}
