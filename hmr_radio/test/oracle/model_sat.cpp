// The analytic model on the oracle's scenarios.
#include "hmr_radio/radio_mac.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
using namespace hmr_radio;
int main(int argc, char ** argv)
{
  std::string sc = argv[1];
  int n = std::atoi(argv[2]);
  std::vector<int> mcs;
  for (char * t = std::strtok(argv[3], ","); t; t = std::strtok(nullptr, ",")) mcs.push_back(std::atoi(t));
  double strong = argc > 4 ? std::atof(argv[4]) : 60.0, hint = argc > 5 ? std::atof(argv[5]) : 60.0;
  MacParams mp;
  if (argc > 6) mp.slot_s = std::atof(argv[6]) * 1e-6;
  ShareInput in;
  int nodes = sc == "sat" ? n + 1 : sc == "hidden" ? 3 : 4;
  in.n = nodes;
  in.noise_dbm = -174 + 10 * std::log10(20e6) + 7;
  in.hear.assign(nodes * nodes, 0); in.active.assign(nodes * nodes, 0);
  in.rx_dbm.assign(nodes * nodes, -200); in.snr_db.assign(nodes * nodes, -100); in.mcs.assign(nodes * nodes, 0);
  auto link = [&](int a, int b, double loss) {
    for (auto [x, y] : {std::pair{a, b}, std::pair{b, a}}) {
      in.rx_dbm[x * nodes + y] = 20 - loss;
      in.snr_db[x * nodes + y] = 20 - loss - in.noise_dbm;
      in.hear[x * nodes + y] = in.snr_db[x * nodes + y] >= mp.detect_snr_db;
    }
  };
  auto m_of = [&](int k) { return mcs[std::min<size_t>(k, mcs.size() - 1)]; };
  std::vector<std::pair<int, int>> flows;
  if (sc == "sat") {
    for (int a = 0; a < nodes; ++a) for (int b = a + 1; b < nodes; ++b) link(a, b, strong);
    for (int k = 0; k < n; ++k) flows.push_back({k, n});
  } else if (sc == "hidden") {
    link(0, 1, strong); link(2, 1, hint); link(0, 2, 200);
    flows = {{0, 1}, {2, 1}};
  } else {
    for (int a = 0; a < nodes; ++a) for (int b = a + 1; b < nodes; ++b) link(a, b, 200);
    link(0, 1, strong); link(2, 3, strong);
    flows = {{0, 1}, {2, 3}};
  }
  for (size_t k = 0; k < flows.size(); ++k) {
    auto [s, d] = flows[k];
    in.active[s * nodes + d] = 1;
    in.mcs[s * nodes + d] = m_of(k);
  }
  if (argc > 7 && std::string(argv[7]) == "noagg") {
    in.backlog_bytes.assign(nodes * nodes, 1e9);
    in.backlog_bytes[2 * nodes + 1] = mp.mpdu_payload_bytes;
  }
  auto out = ShareChannel(mp, in);
  double tot = 0;
  for (auto [s, d] : flows) {
    double g = out.goodput_bps[s * nodes + d] / 1e6; tot += g;
    std::printf("flow %d->%d goodput %.3f Mbps (mcs %d, hidden_loss %.3f, duty %.3f, dom %d, p %.3f, agg %d)\n",
      s, d, g, in.mcs[s * nodes + d], out.hidden_loss[s * nodes + d], out.duty[s], out.domain_size[s], out.collision_p[s],
      FullAggregate(mp, in.mcs[s * nodes + d]).mpdus);
  }
  std::printf("total %.3f Mbps\n", tot);
}
