// E6 at the v2_phy link-up threshold (4 dB preamble detection) plus the
// single-link saturated goodput a lone pair gets at each distance.
#include "hmr_radio/radio_core.hpp"
#include "hmr_radio/radio_mac.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
using namespace hmr_radio;
static Endpoint At(double x, double y) { Endpoint e; e.x = x; e.y = y; e.antenna_h = 0.4; return e; }
int main(int argc, char ** argv)
{
  const std::string src = argv[1];
  MacParams mp;
  // best single-link goodput vs SNR, 0.05 dB grid from -10 to 50
  const double g0 = -10, gs = 0.05; const int gn = 1201;
  std::vector<double> G(gn); std::vector<int> B(gn);
  for (int k = 0; k < gn; ++k) {
    const double snr = g0 + gs * k;
    double best = 0; int bm = -1;
    if (snr >= mp.detect_snr_db) for (int m = 0; m < 8; ++m) {
      ShareInput in; in.n = 2; in.noise_dbm = -93.99;
      in.hear = {0, 1, 1, 0}; in.active = {0, 1, 0, 0};
      in.rx_dbm = {-200, snr - 93.99, snr - 93.99, -200}; in.snr_db = {-100, snr, snr, -100};
      in.mcs = {0, m, 0, 0};
      const double g = ShareChannel(mp, in).goodput_bps[1];
      if (g > best) { best = g; bm = m; }
    }
    G[k] = best; B[k] = bm;
  }
  World w; const double half = 150.0;
  w.xmin = w.ymin = -half; w.xmax = w.ymax = half;
  std::mt19937_64 rng(1);
  std::uniform_real_distribution<double> u(-half, half);
  std::poisson_distribution<int> pois(0.0278 * 4 * half * half);
  for (int t = pois(rng); t > 0; --t) w.trunks.push_back({u(rng), u(rng), 0.45});
  const double D[] = {10, 20, 30, 40, 50, 60};
  for (const char * name : {"neutral", "pessimistic"}) {
    const Profile p = LoadProfileYaml(src + "/config/profiles/" + name + ".yaml");
    const LinkModel m(p, w);
    std::printf("%s\n  d   P(up)@2  P(up)@4  E[goodput|up] Mbps  E[goodput] Mbps  modal MCS at median SNR\n", name);
    std::mt19937_64 r2(7);
    std::uniform_real_distribution<double> v(-half + 70, half - 70), th(0, 2 * M_PI);
    for (double d : D) {
      double p2 = 0, p4 = 0, eg = 0; int n = 0;
      std::vector<double> snrs;
      while (n < 3000) {
        const double x = v(r2), y = v(r2), a = th(r2);
        const Endpoint e0 = At(x, y), e1 = At(x + d * std::cos(a), y + d * std::sin(a));
        bool clear = true;
        for (const auto & t : w.trunks)
          if (std::hypot(t.x - e0.x, t.y - e0.y) < 0.9 || std::hypot(t.x - e1.x, t.y - e1.y) < 0.9) { clear = false; break; }
        if (!clear) continue;
        const double snr = m.Evaluate(e0, e1, 0.0).snr_db;
        snrs.push_back(snr);
        const double s = p.fade_sigma_db;
        p2 += 0.5 * std::erfc(-(snr - 2.0) / (s * std::sqrt(2.0)));
        p4 += 0.5 * std::erfc(-(snr - 4.0) / (s * std::sqrt(2.0)));
        // integrate goodput over the log-normal fade, 81 points over +-4 sigma
        double acc = 0, wsum = 0;
        for (int q = -40; q <= 40; ++q) {
          const double z = q / 10.0, wt = std::exp(-0.5 * z * z);
          const double sv = snr + z * s;
          int k = (int)std::lround((sv - g0) / gs); k = std::max(0, std::min(gn - 1, k));
          acc += wt * G[k]; wsum += wt;
        }
        eg += acc / wsum;
        ++n;
      }
      std::sort(snrs.begin(), snrs.end());
      const double med = snrs[snrs.size() / 2];
      int k = (int)std::lround((med - g0) / gs); k = std::max(0, std::min(gn - 1, k));
      std::printf("  %2.0f  %6.1f%%  %6.1f%%  %8.2f  %8.2f   median SNR %.1f dB -> MCS %d\n", d, 100 * p2 / n, 100 * p4 / n,
        eg / p4 / 1e6, eg / n / 1e6, med, B[k]);
    }
    Profile q = p; q.gamma_itu_db_m = m.gamma_bg();
    World none; none.xmin = none.ymin = -1000; none.xmax = none.ymax = 1000;
    const LinkModel fr(q, none);
    double r2d = 0, r4 = 0;
    for (double d = 1; d < 120; d += 0.1) {
      const double s = fr.Evaluate(At(0, 0), At(d, 0), 0.0).snr_db;
      if (s >= 2.0) r2d = d;
      if (s >= 4.0) r4 = d;
    }
    std::printf("  trunk-free range: %.1f m at 2 dB, %.1f m at 4 dB\n", r2d, r4);
  }
}
