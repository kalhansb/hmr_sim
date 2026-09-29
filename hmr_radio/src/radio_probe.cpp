// radio_probe: the hmr_radio core from the command line. The known-answer
// tests (test/comms_v2_calib.py) and offline replays call this, so they run
// the node's own physics rather than a copy of it.
//
//   radio_probe PROFILE.yaml [WORLD.yaml|-] gamma
//       prints gamma_explicit_db_m and gamma_bg_db_m.
//   radio_probe PROFILE.yaml [WORLD.yaml|-] eval
//       reads "ax ay ha bx by hb" per line on stdin (ha, hb: antenna heights;
//       fade 0) and prints one CSV row of the loss breakdown per link.
//
// Options before the command: --gamma-seed N --gamma-links N (defaults as the node).
#include "hmr_radio/radio_core.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  std::vector<std::string> args(argv + 1, argv + argc);
  uint64_t seed = 20260927;
  int links = 10000;
  std::vector<std::string> pos;
  for (size_t k = 0; k < args.size(); ++k) {
    if (args[k] == "--gamma-seed" && k + 1 < args.size()) {
      seed = std::strtoull(args[++k].c_str(), nullptr, 10);
    } else if (args[k] == "--gamma-links" && k + 1 < args.size()) {
      links = std::atoi(args[++k].c_str());
    } else {
      pos.push_back(args[k]);
    }
  }
  if (pos.size() != 3 || (pos[2] != "gamma" && pos[2] != "eval")) {
    std::cerr << "usage: radio_probe [--gamma-seed N] [--gamma-links N] "
      "PROFILE.yaml WORLD.yaml|- gamma|eval\n";
    return 2;
  }
  try {
    const auto profile = hmr_radio::LoadProfileYaml(pos[0]);
    hmr_radio::World world;
    if (pos[1] != "-") {
      world = hmr_radio::LoadWorldYaml(pos[1]);
    }
    const hmr_radio::LinkModel model(profile, world, seed, links);
    if (pos[2] == "gamma") {
      std::printf("gamma_explicit_db_m=%.6f\ngamma_bg_db_m=%.6f\nnoise_dbm=%.4f\n",
        model.gamma_explicit(), model.gamma_bg(), hmr_radio::NoiseDbm(profile));
      return 0;
    }
    std::printf("distance_m,pl_ground_db,l_background_db,l_trunks_db,l_bushes_db,l_veg_db,"
      "l_ridges_db,l_range_db,path_loss_db,rx_dbm,snr_db,trunks_on_link\n");
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::istringstream ss(line);
      hmr_radio::Endpoint a, b;
      if (!(ss >> a.x >> a.y >> a.antenna_h >> b.x >> b.y >> b.antenna_h)) {
        std::cerr << "bad line: " << line << "\n";
        return 2;
      }
      a.z = a.antenna_h;
      b.z = b.antenna_h;
      const auto o = model.Evaluate(a, b, 0.0);
      std::printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.1f,%.6f,%.6f,%.6f,%d\n",
        o.distance_m, o.pl_ground_db, o.l_background_db, o.l_trunks_db, o.l_bushes_db,
        o.l_veg_db, o.l_ridges_db, o.l_range_db, o.path_loss_db, o.rx_dbm, o.snr_db,
        o.trunks_on_link);
    }
  } catch (const std::exception & e) {
    std::cerr << "radio_probe: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
