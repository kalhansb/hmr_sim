// Known-answer tests for hmr_radio's core (DESIGN_radio_v2.md §2.14 E0–E7, §12).
// They call the same library the node runs, never a copy of the physics; the
// only reference formulas written out here are v1's (to prove the v1 stack is
// v1) and the P.526 compositions the design states in closed form.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "hmr_radio/radio_core.hpp"

using hmr_radio::Endpoint;
using hmr_radio::LinkModel;
using hmr_radio::Profile;
using hmr_radio::World;

namespace
{

const std::string kSrc = HMR_RADIO_SRC;

Profile Neutral() {return hmr_radio::LoadProfileYaml(kSrc + "/config/profiles/neutral.yaml");}
Profile Pessimistic()
{
  return hmr_radio::LoadProfileYaml(kSrc + "/config/profiles/pessimistic.yaml");
}

Endpoint At(double x, double y, double h = 0.4)
{
  Endpoint e;
  e.x = x;
  e.y = y;
  e.antenna_h = h;
  return e;
}

double Lin(double db) {return std::pow(10.0, -db / 10.0);}

// A LinkModel over `w` whose gamma sample is trivial; for tests of one term.
LinkModel Bare(const Profile & p, const World & w) {return LinkModel(p, w, 1, 1);}

}  // namespace

TEST(Profiles, LoadAsWritten)
{
  const Profile n = Neutral(), s = Pessimistic();
  EXPECT_EQ(n.model, hmr_radio::Model::kV2);
  EXPECT_DOUBLE_EQ(n.p0_db, 47.1);
  EXPECT_DOUBLE_EQ(n.gamma_itu_db_m, 0.44);
  EXPECT_DOUBLE_EQ(n.veg_max_atten_db, 34.4);
  EXPECT_DOUBLE_EQ(s.p0_db, 54.17);
  EXPECT_DOUBLE_EQ(s.gamma_itu_db_m, 0.49);
  EXPECT_DOUBLE_EQ(s.veg_max_atten_db, 36.0);
  for (const Profile & p : {n, s}) {
    EXPECT_DOUBLE_EQ(p.tx_power_dbm, 20.0);
    EXPECT_DOUBLE_EQ(p.tx_gain_dbi + p.rx_gain_dbi, 4.0);
    EXPECT_DOUBLE_EQ(p.fade_sigma_db, 4.4);
    EXPECT_DOUBLE_EQ(p.antenna_height_m, 0.4);
    EXPECT_LE(p.max_range_m, 0.0);
    EXPECT_TRUE(p.ground_two_ray);
    // MCS0 budget 116 dB (§2.14): P_tx + G - noise - 2 dB.
    EXPECT_NEAR(p.tx_power_dbm + 4.0 - hmr_radio::NoiseDbm(p) - 2.0, 116.0, 0.05);
  }
  const Profile v1 = hmr_radio::LoadProfileYaml(kSrc + "/config/profiles/v1.yaml");
  EXPECT_EQ(v1.model, hmr_radio::Model::kV1);
  EXPECT_DOUBLE_EQ(v1.tx_power_dbm, 30.0);
  EXPECT_DOUBLE_EQ(v1.tree_attenuation_db, 70.0);
  EXPECT_DOUBLE_EQ(v1.max_range_m, 30.0);
}

// E1: a 0.9 m trunk on a 20 m link, the §2.14 table to 0.05 dB.
TEST(E1, SingleTrunk)
{
  const struct {double x, y, want;} cases[] = {
    {10.0, 0.0, 9.59}, {10.0, 0.3, 7.66}, {10.0, 0.6, 3.63}, {2.0, 0.0, 12.94},
    {0.5, 0.0, 18.13}};
  for (const auto & c : cases) {
    World w;
    w.trunks.push_back({c.x, c.y, 0.45});
    const LinkModel m = Bare(Neutral(), w);
    int n = 0;
    EXPECT_NEAR(m.TrunksDb(At(0, 0), At(20, 0), &n), c.want, 0.05) << c.x << "," << c.y;
    EXPECT_EQ(n, 1);
    // Reciprocal, and invariant under rotating the link.
    EXPECT_NEAR(m.TrunksDb(At(20, 0), At(0, 0)), c.want, 0.05);
  }
  World w;
  const double th = 0.7;
  w.trunks.push_back({10.0 * std::cos(th) - 0.3 * std::sin(th),
      10.0 * std::sin(th) + 0.3 * std::cos(th), 0.45});
  EXPECT_NEAR(Bare(Neutral(), w).TrunksDb(At(0, 0), At(20 * std::cos(th), 20 * std::sin(th))),
    7.66, 0.05);
}

// A trunk never adds gain: the clamp removes eq. 72's -0.2 dB just off a narrow trunk.
TEST(E1, TrunkLossNeverNegative)
{
  for (double r : {0.05, 0.1, 0.45, 1.0}) {
    for (double y = -3.0; y <= 3.0; y += 0.01) {
      World w;
      w.trunks.push_back({7.0, y, r});
      EXPECT_GE(Bare(Neutral(), w).TrunksDb(At(0, 0), At(30, 0)), 0.0) << r << " " << y;
    }
  }
}

// E0: Gay-Fernández's forest (0.03 trunks/m², r 13.9–20.4 cm, 2.45 GHz), set up
// as the design's own check (2026-09-27): d ~ U[10, 60], trunks in a ±6 m band
// and 1 m clear of either end along the link, k = trunks within ±1 m, slope
// fitted through the origin. It must be 1.0–2.0 dB/trunk ([33] eq. 9: 1.52), and
// the mean at k = 8 at most 32 dB ([33]: "up to 30 dB").
TEST(E0, GayFernandezSlope)
{
  Profile p = Neutral();
  p.frequency_hz = 2.45e9;
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  double skk = 0, skl = 0, sum8 = 0;
  int n8 = 0;
  const double band = 6.0;
  for (int layout = 0; layout < 10000; ++layout) {
    const double d = 10.0 + 50.0 * u(rng);
    std::poisson_distribution<int> pois(0.03 * (d - 2) * 2 * band);
    World w;
    int k = 0;
    for (int t = pois(rng); t > 0; --t) {
      const double x = 1.0 + (d - 2.0) * u(rng), y = band * (2 * u(rng) - 1);
      w.trunks.push_back({x, y, 0.139 + (0.204 - 0.139) * u(rng)});
      k += std::abs(y) < 1.0;
    }
    const double l = Bare(p, w).TrunksDb(At(0, 0, 1.5), At(d, 0, 1.5));
    skk += double(k) * k;
    skl += k * l;
    if (k == 8) {
      sum8 += l;
      ++n8;
    }
  }
  const double slope = skl / skk;
  std::printf("[E0] slope %.3f dB/trunk (design 1.95, [33] 1.52), mean at k=8: %.1f dB (n=%d)\n",
    slope, n8 ? sum8 / n8 : NAN, n8);
  EXPECT_GE(slope, 1.0);
  EXPECT_LE(slope, 2.0);
  ASSERT_GT(n8, 20);
  EXPECT_LE(sum8 / n8, 32.0);
}

// E2: no double count. gamma_bg + gamma_explicit == gamma_ITU, or gamma_bg == 0.
TEST(E2, NoDoubleCount)
{
  const World w = hmr_radio::LoadWorldYaml(kSrc + "/worlds/flatforest_dense.radio.yaml");
  ASSERT_EQ(w.trunks.size(), 278u);
  for (const Profile & p : {Neutral(), Pessimistic()}) {
    const LinkModel m(p, w);
    std::printf("[E2] %s: gamma_explicit %.4f gamma_bg %.4f dB/m\n", p.name.c_str(),
      m.gamma_explicit(), m.gamma_bg());
    EXPECT_GT(m.gamma_explicit(), 0.0);
    if (m.gamma_explicit() < p.gamma_itu_db_m) {
      EXPECT_NEAR(m.gamma_bg() + m.gamma_explicit(), p.gamma_itu_db_m, 0.01);
    } else {
      EXPECT_EQ(m.gamma_bg(), 0.0);
    }
    // Measured from a fixed sample: the same every time.
    EXPECT_EQ(LinkModel(p, w).gamma_explicit(), m.gamma_explicit());
  }
  Profile p = Neutral();
  p.gamma_itu_db_m = 0.01;
  EXPECT_EQ(LinkModel(p, w).gamma_bg(), 0.0);
}

// E3: a 1.2 m bush, top 0.85 m, centred on a 20 m link: top + 2 sides + through,
// power-summed; beside the link, its two side edges only.
TEST(E3, SingleBush)
{
  const Profile p = Neutral();
  World w;
  w.bushes.push_back({10.0, 0.0, 0.6, 0.6, 0.0, 0.85});
  const LinkModel m = Bare(p, w);
  const double s = hmr_radio::NuScale(20.0, 10.0, 3.0e8 / p.frequency_hz);
  const double want = -10.0 * std::log10(
    Lin(hmr_radio::J(0.45 * s)) + 2 * Lin(hmr_radio::J(0.6 * s)) + Lin(1.5 * 1.2));
  EXPECT_NEAR(m.BushesDb(At(0, 0), At(20, 0)), want, 1e-9);
  std::printf("[E3] bush on link: %.2f dB\n", want);
  // Top below the antenna line: gamma_u low, and the top edge clears.
  World lo = w;
  lo.bushes[0].height = 0.3;
  // The paths sum past unity here (the top clears, the through path costs
  // 0.6 dB), and a bush never adds gain: clamped at 0.
  const double want_lo = std::max(0.0, -10.0 * std::log10(
    Lin(hmr_radio::J(-0.1 * s)) + 2 * Lin(hmr_radio::J(0.6 * s)) + Lin(0.5 * 1.2)));
  EXPECT_NEAR(Bare(p, lo).BushesDb(At(0, 0), At(20, 0)), want_lo, 1e-9);
  // Rotated 45°: the chord is the diagonal.
  World rot = w;
  rot.bushes[0].yaw = M_PI / 4;
  const double diag = 1.2 * std::sqrt(2.0);
  const double want_rot = -10.0 * std::log10(
    Lin(hmr_radio::J(0.45 * s)) + 2 * Lin(hmr_radio::J(diag / 2 * s)) + Lin(1.5 * diag));
  EXPECT_NEAR(Bare(p, rot).BushesDb(At(0, 0), At(20, 0)), want_rot, 1e-9);
  // Beside the link: the screen's sides only, never negative.
  for (double y = 0.61; y < 4.0; y += 0.05) {
    World b = w;
    b.bushes[0].y = y;
    const double got = Bare(p, b).BushesDb(At(0, 0), At(20, 0));
    EXPECT_NEAR(got, hmr_radio::ScreenSidesDb(y - 0.6, y + 0.6, s), 1e-9) << y;
    EXPECT_GE(got, 0.0);
  }
}

// E4: ridges, §2.12's table (H 3 m, 0.4 m antennas, sharp crest, endless ridge).
TEST(E4, SingleRidge)
{
  const Profile p = Neutral();
  const struct {double d, xc, want;} cases[] = {
    {10, 5, 29.2}, {20, 10, 26.2}, {40, 20, 23.2}, {60, 30, 21.5}, {40, 5, 26.8},
    {40, 2, 30.4}};
  for (const auto & c : cases) {
    World w;
    w.ridges.push_back({{{c.xc, -1000.0}, {c.xc, 1000.0}}, 3.0});
    const LinkModel m = Bare(p, w);
    EXPECT_NEAR(m.RidgesDb(At(0, 0), At(c.d, 0)), c.want, 0.06) << c.d << " " << c.xc;
    // Outside the vegetation power sum: A_m never reduces it.
    const auto bd = m.Evaluate(At(0, 0), At(c.d, 0), 0.0);
    EXPECT_NEAR(bd.l_ridges_db, m.RidgesDb(At(0, 0), At(c.d, 0)), 1e-12);
    EXPECT_NEAR(bd.path_loss_db, bd.pl_ground_db + bd.l_veg_db + bd.l_ridges_db, 1e-9);
  }
}

TEST(E4, RidgeEndsAndGrazing)
{
  const Profile p = Neutral();
  // A 20 m ridge x = 20, y in [-10, 10]; the link y = c crosses it for |c| < 10.
  auto loss = [&](double c) {
      World w;
      w.ridges.push_back({{{20.0, -10.0}, {20.0, 10.0}}, 3.0});
      return Bare(p, w).RidgesDb(At(0, c), At(40, c));
    };
  // Mid-ridge: the crest (23.2 dB alone) power-summed with both ends, 10 m off.
  const double s20 = hmr_radio::NuScale(40.0, 20.0, 3.0e8 / p.frequency_hz);
  const double mid = loss(0.0);
  EXPECT_NEAR(mid, -10.0 * std::log10(Lin(hmr_radio::J(2.6 * s20)) +
    2 * Lin(hmr_radio::J(10.0 * s20))), 1e-9);
  EXPECT_LT(mid, 23.2);
  EXPECT_LT(loss(9.0), mid);              // near an end: the path around it helps
  EXPECT_GT(loss(9.9), 0.0);
  EXPECT_LT(loss(12.0), loss(10.1));      // outside: grazing edge only, falling off
  EXPECT_LT(loss(14.0), 0.5);
  for (double c = -14; c <= 14; c += 0.1) {
    EXPECT_GE(loss(c), 0.0);
  }
}

// Two ridges by Deygout: the principal on the full path plus the other on its
// sub-path to the principal's crest, never less than either ridge alone.
TEST(E4, TwoRidgesDeygout)
{
  const Profile p = Neutral();
  World w1, w2, both;
  w1.ridges.push_back({{{15.0, -1000.0}, {15.0, 1000.0}}, 3.0});
  w2.ridges.push_back({{{30.0, -1000.0}, {30.0, 1000.0}}, 2.0});
  both.ridges = {w1.ridges[0], w2.ridges[0]};
  const auto a = At(0, 0), b = At(40, 0);
  const double l1 = Bare(p, w1).RidgesDb(a, b), l2 = Bare(p, w2).RidgesDb(a, b);
  const double l12 = Bare(p, both).RidgesDb(a, b);
  // Principal = ridge 1 (the larger loss); ridge 2 lies on the crest->b sub-path.
  ASSERT_GT(l1, l2);
  Endpoint crest = At(15, 0, 3.0);
  const double sub = Bare(p, w2).RidgesDb(crest, b);
  EXPECT_NEAR(l12, l1 + sub, 1e-9);
  EXPECT_GE(l12, std::max(l1, l2));
  std::printf("[E4] Deygout: %.2f + %.2f = %.2f dB (alone %.2f, %.2f)\n", l1, sub, l12, l1, l2);
}

// E5: no world, so gamma_explicit = 0 and PL = powersum(PL_ground + gamma_ITU d,
// PL_ground + A_m).
TEST(E5, NoWorld)
{
  for (const Profile & p : {Neutral(), Pessimistic()}) {
    const LinkModel m(p, World{});
    EXPECT_EQ(m.gamma_explicit(), 0.0);
    EXPECT_DOUBLE_EQ(m.gamma_bg(), p.gamma_itu_db_m);
    const double db = 4 * 0.4 * 0.4 / (3.0e8 / p.frequency_hz);
    for (double d : {1.0, 3.0, 5.0, 10.0, 30.0, 60.0, 120.0}) {
      const double g = p.p0_db + 20 * std::log10(d) + (d > db ? 20 * std::log10(d / db) : 0);
      const double want = hmr_radio::PowerSumDb(g + p.gamma_itu_db_m * d, g + p.veg_max_atten_db);
      const auto bd = m.Evaluate(At(0, 0), At(d, 0), 0.0);
      EXPECT_NEAR(bd.pl_ground_db, g, 1e-9);
      EXPECT_NEAR(bd.path_loss_db, want, 1e-9) << d;
      EXPECT_EQ(bd.l_range_db, 0.0);
      EXPECT_NEAR(bd.snr_db, 24.0 - want - hmr_radio::NoiseDbm(p), 1e-9);
    }
  }
}

// E6: P(link up) on flatforest_dense's density (0.9 m trunks, 0.0278/m²,
// centres >= 0.9 m from a robot), 3000 links per distance, sigma 4.4, to ±3 points.
TEST(E6, ProfileTable)
{
  World w;
  const double half = 150.0;
  w.xmin = w.ymin = -half;
  w.xmax = w.ymax = half;
  std::mt19937_64 rng(1);
  std::uniform_real_distribution<double> u(-half, half);
  std::poisson_distribution<int> pois(0.0278 * 4 * half * half);
  for (int t = pois(rng); t > 0; --t) {
    w.trunks.push_back({u(rng), u(rng), 0.45});
  }
  const double D[] = {10, 20, 30, 40, 50, 60};
  const double want_n[] = {100, 99, 89, 59, 26, 7};
  const double want_p[] = {100, 94, 64, 20, 2, 0};
  for (const Profile & p : {Neutral(), Pessimistic()}) {
    const LinkModel m(p, w);
    std::printf("[E6] %s gamma_explicit %.3f gamma_bg %.3f; P(up):", p.name.c_str(),
      m.gamma_explicit(), m.gamma_bg());
    std::mt19937_64 r2(7);
    std::uniform_real_distribution<double> v(-half + 70, half - 70), th(0, 2 * M_PI);
    for (int k = 0; k < 6; ++k) {
      double pu = 0;
      int n = 0;
      while (n < 3000) {
        const double x = v(r2), y = v(r2), a = th(r2);
        const Endpoint e0 = At(x, y), e1 = At(x + D[k] * std::cos(a), y + D[k] * std::sin(a));
        bool clear = true;
        for (const auto & t : w.trunks) {
          if (std::hypot(t.x - e0.x, t.y - e0.y) < 0.9 || std::hypot(t.x - e1.x, t.y - e1.y) < 0.9) {
            clear = false;
            break;
          }
        }
        if (!clear) {
          continue;
        }
        const double snr = m.Evaluate(e0, e1, 0.0).snr_db;
        pu += 0.5 * std::erfc(-(snr - 2.0) / (p.fade_sigma_db * std::sqrt(2.0)));
        ++n;
      }
      pu = 100 * pu / n;
      std::printf(" %g m %.1f%%", D[k], pu);
      EXPECT_NEAR(pu, (p.p0_db < 50 ? want_n : want_p)[k], 3.0) << p.name << " d=" << D[k];
    }
    std::printf("\n");
    // Trunk-free MCS0 range (§2.14: 61 m neutral, 44 m pessimistic): a link in
    // this forest that no trunk touches, so PL_ground + gamma_bg d, capped by A_m.
    World none;
    none.xmin = none.ymin = -1000;
    none.xmax = none.ymax = 1000;
    Profile q = p;
    q.gamma_itu_db_m = m.gamma_bg();
    const LinkModel free(q, none);
    double r = 0;
    for (double d = 1; d < 120; d += 0.1) {
      if (free.Evaluate(At(0, 0), At(d, 0), 0.0).snr_db >= 2.0) {
        r = d;
      }
    }
    std::printf("[E6] %s trunk-free MCS0 range %.1f m\n", p.name.c_str(), r);
    EXPECT_NEAR(r, p.p0_db < 50 ? 61.0 : 44.0, 1.0);
  }
}

// E7: p0 from [36]: pessimistic, 17 dBm, 1.5 + 5 dBi, 1 m link, 1 m antennas, no
// obstacle and no woodland (the measurement was in the open) -> -30.67 dBm.
TEST(E7, BotellaCamposP0)
{
  Profile p = Pessimistic();
  p.tx_power_dbm = 17.0;
  p.tx_gain_dbi = 1.5;
  p.rx_gain_dbi = 5.0;
  p.gamma_itu_db_m = 0.0;
  const LinkModel m(p, World{});
  EXPECT_NEAR(m.Evaluate(At(0, 0, 1.0), At(1, 0, 1.0), 0.0).rx_dbm, -30.67, 0.01);
}

// The v1 stack is v1: the deterministic part of v1's ComputePair, copied
// verbatim from hmr_comms_sim_node.cpp, must agree bit for bit.
namespace v1ref
{
struct Vec3 {double x = 0.0, y = 0.0, z = 0.0;};
double Dist3(const Vec3 & a, const Vec3 & b)
{
  const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
double PointToSegmentDistSq2D(const Vec3 & a, const Vec3 & b, const Vec3 & c)
{
  const double abx = b.x - a.x, aby = b.y - a.y;
  const double acx = c.x - a.x, acy = c.y - a.y;
  const double e = acx * abx + acy * aby;
  if (e <= 0.0) {
    return acx * acx + acy * acy;
  }
  const double f = abx * abx + aby * aby;
  if (e >= f) {
    const double bcx = c.x - b.x, bcy = c.y - b.y;
    return bcx * bcx + bcy * bcy;
  }
  return acx * acx + acy * acy - e * e / f;
}
double FresnelZoneRadius(double distance_m, double frequency_hz)
{
  return 0.5 * std::sqrt(3.0e8 / frequency_hz * distance_m);
}
}  // namespace v1ref

TEST(V1, BitForBit)
{
  const Profile p = hmr_radio::LoadProfileYaml(kSrc + "/config/profiles/v1.yaml");
  const World w = hmr_radio::LoadWorldYaml(kSrc + "/worlds/flatforest_dense.radio.yaml");
  const LinkModel m(p, w);
  std::vector<v1ref::Vec3> trees;
  for (const auto & t : w.trunks) {
    trees.push_back({t.x, t.y, 0.0});
  }
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> u(-50, 50), z(0.0, 0.6), f(-12, 12);
  int with_trees = 0, beyond = 0;
  for (int k = 0; k < 20000; ++k) {
    const v1ref::Vec3 a{u(rng), u(rng), z(rng)};
    const v1ref::Vec3 b{k % 7 ? u(rng) : a.x + 1e-3, k % 7 ? u(rng) : a.y, z(rng)};
    const double fade = f(rng);
    const double distance = std::max(v1ref::Dist3(a, b), 0.1);
    const double link_width = p.tree_radius_m + v1ref::FresnelZoneRadius(distance, p.frequency_hz);
    int trees_on_link = 0;
    for (const auto & tree : trees) {
      if (v1ref::PointToSegmentDistSq2D(a, b, tree) < link_width * link_width) {
        ++trees_on_link;
      }
    }
    double path_loss = p.p0_db + 20.0 * std::log10(distance) +
      trees_on_link * p.tree_attenuation_db + fade;
    if (p.max_range_m > 0.0 && distance > p.max_range_m) {
      path_loss += 200.0;
    }
    const double rx_dbm = p.tx_power_dbm - path_loss;
    const double snr_db = rx_dbm - p.noise_floor_dbm;

    Endpoint ea, eb;
    ea.x = a.x; ea.y = a.y; ea.z = a.z;
    eb.x = b.x; eb.y = b.y; eb.z = b.z;
    const auto bd = m.Evaluate(ea, eb, fade);
    ASSERT_EQ(bd.distance_m, distance);
    ASSERT_EQ(bd.trunks_on_link, trees_on_link);
    ASSERT_EQ(bd.path_loss_db, path_loss) << k;
    ASSERT_EQ(bd.snr_db, snr_db);
    with_trees += trees_on_link > 0;
    beyond += distance > p.max_range_m;
  }
  EXPECT_GT(with_trees, 1000);
  EXPECT_GT(beyond, 1000);
}

// Counter fading: a pure function of (seed, stream, tick), stationary sigma and
// lag-1 correlation alpha, and independent of any other link's draws.
TEST(Fade, CounterDeterministicAndIndependent)
{
  hmr_radio::CounterFade a(42, 7, 4.4, 0.9), b(42, 7, 4.4, 0.9), other(42, 8, 4.4, 0.9);
  hmr_radio::CounterFade c(42, 7, 4.4, 0.9);
  std::vector<double> xs;
  for (uint64_t t = 1; t <= 200000; ++t) {
    const double va = a.Step(t);
    other.Step(t);            // interleaved draws on another link
    other.Step(t);
    ASSERT_EQ(va, b.Step(t));
    xs.push_back(va);
  }
  for (uint64_t t = 1; t <= 200000; ++t) {
    ASSERT_EQ(c.Step(t), xs[t - 1]);
  }
  double m = 0, v = 0, c1 = 0;
  for (double x : xs) {m += x;}
  m /= xs.size();
  for (size_t i = 0; i < xs.size(); ++i) {
    v += (xs[i] - m) * (xs[i] - m);
    if (i) {c1 += (xs[i] - m) * (xs[i - 1] - m);}
  }
  EXPECT_NEAR(std::sqrt(v / xs.size()), 4.4, 0.15);
  EXPECT_NEAR(c1 / v, 0.9, 0.01);
  EXPECT_NEAR(m, 0.0, 0.3);
  // Different seed or stream, different trace.
  hmr_radio::CounterFade d(43, 7, 4.4, 0.9);
  EXPECT_NE(d.Step(1), xs[0]);
  EXPECT_NE(hmr_radio::CounterGaussian(42, 7, 1), hmr_radio::CounterGaussian(42, 8, 1));
}

TEST(Fade, StationaryStart)
{
  // The first value is already N(0, sigma), not 0.
  double s2 = 0;
  const int n = 20000;
  for (int k = 0; k < n; ++k) {
    hmr_radio::CounterFade f(static_cast<uint64_t>(k), 3, 4.4, 0.9);
    const double x = f.Step(1);
    s2 += x * x;
  }
  EXPECT_NEAR(std::sqrt(s2 / n), 4.4, 0.1);
}

TEST(Fade, SkippedTicksDoNotShiftTheTrace)
{
  hmr_radio::CounterFade every(5, 9, 4.4, 0.9), sparse(5, 9, 4.4, 0.9), late(5, 9, 4.4, 0.9);
  std::vector<double> xs(1001);
  for (uint64_t t = 1; t <= 1000; ++t) {
    xs[t] = every.Step(t);
  }
  for (uint64_t t : {3u, 4u, 50u, 51u, 400u, 1000u}) {
    ASSERT_EQ(sparse.Step(t), xs[t]) << t;
  }
  ASSERT_EQ(late.Step(777), xs[777]);
  ASSERT_EQ(late.Step(777), xs[777]);   // the same tick twice: no advance
}
