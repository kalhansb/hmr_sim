// Radio emulator v2 core. Spec: DESIGN_radio_v2.md §2.11 (bushes), §2.12
// (ridges), §2.14 (loss stack and profiles). Every formula here is checked by
// test/test_radio_core.cpp against the design's published values.
#include "hmr_radio/radio_core.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hmr_radio
{

namespace
{

constexpr double kC = 3.0e8;
constexpr double kPi = 3.14159265358979323846;
// Robot half-width used to keep sampled γ_explicit endpoints out of obstacles:
// a trunk centre is never closer than r + this to a robot (0.9 m for the oak).
constexpr double kRobotHalfWidth = 0.45;

uint64_t SplitMix64(uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

double ToUnit(uint64_t h)
{
  // (0, 1): never 0, so log() is safe.
  return (static_cast<double>(h >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}

uint64_t CounterHash(uint64_t seed, uint64_t stream, uint64_t counter)
{
  return SplitMix64(SplitMix64(SplitMix64(seed) ^ stream) ^ counter);
}

// Link frame: along-track x from a, cross-track y (left of a->b positive).
struct Frame
{
  double ax, ay, ux, uy, d;
  Frame(const Endpoint & a, const Endpoint & b)
  : ax(a.x), ay(a.y)
  {
    const double dx = b.x - a.x, dy = b.y - a.y;
    d = std::max(std::hypot(dx, dy), 0.1);
    ux = dx / d;
    uy = dy / d;
  }
  double Along(double x, double y) const { return (x - ax) * ux + (y - ay) * uy; }
  double Cross(double x, double y) const { return ux * (y - ay) - uy * (x - ax); }
};

double Db2Lin(double db) { return std::pow(10.0, -db / 10.0); }

double PointSegDist(double px, double py, const std::array<double, 2> & p,
  const std::array<double, 2> & q)
{
  const double vx = q[0] - p[0], vy = q[1] - p[1];
  const double wx = px - p[0], wy = py - p[1];
  const double vv = vx * vx + vy * vy;
  const double t = vv > 0.0 ? std::clamp((wx * vx + wy * vy) / vv, 0.0, 1.0) : 0.0;
  return std::hypot(wx - t * vx, wy - t * vy);
}

// 2D point-to-segment distance squared, as v1's PointToSegmentDistSq2D.
double V1PointToSegmentDistSq2D(const Endpoint & a, const Endpoint & b, double cx, double cy)
{
  const double abx = b.x - a.x, aby = b.y - a.y;
  const double acx = cx - a.x, acy = cy - a.y;
  const double e = acx * abx + acy * aby;
  if (e <= 0.0) {
    return acx * acx + acy * acy;
  }
  const double f = abx * abx + aby * aby;
  if (e >= f) {
    const double bcx = cx - b.x, bcy = cy - b.y;
    return bcx * bcx + bcy * bcy;
  }
  return acx * acx + acy * acy - e * e / f;
}

}  // namespace

// ---- profile ------------------------------------------------------------------

double NoiseDbm(const Profile & p)
{
  if (p.model == Model::kV1) {
    return p.noise_floor_dbm;
  }
  return -174.0 + 10.0 * std::log10(p.bandwidth_hz) + p.noise_figure_db;
}

Profile LoadProfileYaml(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("radio profile '" + path + "': " + e.what());
  }
  YAML::Node m = root;
  if (root["hmr_comms_sim"] && root["hmr_comms_sim"]["ros__parameters"]) {
    m = root["hmr_comms_sim"]["ros__parameters"];
  }
  Profile p;
  if (m["radio_profile"]) {p.name = m["radio_profile"].as<std::string>();}
  const std::string stack = m["radio_stack"] ? m["radio_stack"].as<std::string>() : "v1";
  if (stack == "v1") {
    p.model = Model::kV1;
  } else if (stack == "v2_prop") {
    p.model = Model::kV2;
  } else {
    throw std::runtime_error("radio profile '" + path + "': unknown radio_stack " + stack);
  }
  if (m["ground_model"]) {p.ground_two_ray = m["ground_model"].as<std::string>() == "two_ray";}
  auto get = [&](const char * k, double & v) {
      if (m[k]) {v = m[k].as<double>();}
    };
  get("frequency_hz", p.frequency_hz);
  get("tx_power_dbm", p.tx_power_dbm);
  get("p0_db", p.p0_db);
  get("max_range_m", p.max_range_m);
  get("noise_floor_dbm", p.noise_floor_dbm);
  get("tree_attenuation_db", p.tree_attenuation_db);
  get("tree_radius_m", p.tree_radius_m);
  get("tx_gain_dbi", p.tx_gain_dbi);
  get("rx_gain_dbi", p.rx_gain_dbi);
  get("noise_figure_db", p.noise_figure_db);
  get("bandwidth_hz", p.bandwidth_hz);
  get("gamma_itu_db_m", p.gamma_itu_db_m);
  get("veg_max_atten_db", p.veg_max_atten_db);
  get("bush_gamma_low_db_m", p.bush_gamma_low_db_m);
  get("bush_gamma_high_db_m", p.bush_gamma_high_db_m);
  get("antenna_height_m", p.antenna_height_m);
  get("fade_sigma_db", p.fade_sigma_db);
  get("fade_alpha", p.fade_alpha);
  return p;
}

// ---- P.526 --------------------------------------------------------------------

double J(double nu)
{
  if (nu <= -0.78) {
    return 0.0;
  }
  const double v = nu - 0.1;
  return 6.9 + 20.0 * std::log10(std::sqrt(v * v + 1.0) + v);
}

double NuScale(double d, double d1, double lambda)
{
  const double d2 = d - d1;
  return std::sqrt(2.0 * d / (lambda * d1 * d2));
}

double PowerSumDb(double a, double b)
{
  return -10.0 * std::log10(Db2Lin(a) + Db2Lin(b));
}

double ScreenSidesDb(double y_lo, double y_hi, double s)
{
  // Edge clearances: positive when the edge reaches past the link line.
  const double j_lo = J(-y_lo * s);
  const double j_hi = J(y_hi * s);
  return std::max(0.0, -10.0 * std::log10(Db2Lin(j_lo) + Db2Lin(j_hi)));
}

// ---- world file ---------------------------------------------------------------

World LoadWorldYaml(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("radio world '" + path + "': " + e.what());
  }
  World w;
  auto need = [&](const YAML::Node & n, size_t k, const char * what) {
      if (!n.IsSequence() || n.size() != k) {
        throw std::runtime_error("radio world '" + path + "': malformed " + what);
      }
    };
  if (root["source"]) {w.source = root["source"].as<std::string>();}
  if (root["source_sha256"]) {w.source_sha256 = root["source_sha256"].as<std::string>();}
  if (root["mission_area"]) {
    const auto a = root["mission_area"];
    need(a, 4, "mission_area");
    w.xmin = a[0].as<double>();
    w.xmax = a[1].as<double>();
    w.ymin = a[2].as<double>();
    w.ymax = a[3].as<double>();
  }
  for (const auto & t : root["trunks"]) {
    need(t, 3, "trunk [x, y, r]");
    w.trunks.push_back({t[0].as<double>(), t[1].as<double>(), t[2].as<double>()});
  }
  for (const auto & b : root["bushes"]) {
    need(b, 6, "bush [x, y, half_x, half_y, yaw, height]");
    w.bushes.push_back({b[0].as<double>(), b[1].as<double>(), b[2].as<double>(),
        b[3].as<double>(), b[4].as<double>(), b[5].as<double>()});
  }
  for (const auto & r : root["ridges"]) {
    Ridge rg;
    rg.height = r["height"].as<double>();
    for (const auto & p : r["points"]) {
      need(p, 2, "ridge point [x, y]");
      rg.pts.push_back({p[0].as<double>(), p[1].as<double>()});
    }
    if (rg.pts.size() < 2) {
      throw std::runtime_error("radio world '" + path + "': ridge with < 2 points");
    }
    w.ridges.push_back(rg);
  }
  return w;
}

// ---- link model -----------------------------------------------------------------

LinkModel::LinkModel(Profile profile, World world, uint64_t seed, int links)
: p_(std::move(profile)), w_(std::move(world)), lambda_(kC / p_.frequency_hz)
{
  if (p_.model == Model::kV2) {
    gamma_explicit_ = MeasureGammaExplicit(seed, links);
    gamma_bg_ = std::max(0.0, p_.gamma_itu_db_m - gamma_explicit_);
  }
}

double LinkModel::PlGroundDb(double d, double ha, double hb) const
{
  double pl = p_.p0_db + 20.0 * std::log10(d);
  if (p_.ground_two_ray) {
    const double d_b = 4.0 * ha * hb / lambda_;
    if (d > d_b) {
      pl += 20.0 * std::log10(d / d_b);
    }
  }
  return pl;
}

double LinkModel::TrunksDb(const Endpoint & a, const Endpoint & b, int * n_counted) const
{
  const Frame f(a, b);
  double sum = 0.0;
  int n = 0;
  for (const auto & t : w_.trunks) {
    const double x = f.Along(t.x, t.y);
    if (x <= 0.0 || x >= f.d) {
      continue;
    }
    const double y = f.Cross(t.x, t.y);
    const double s = NuScale(f.d, x, lambda_);
    // Near edge at nu <= -0.78 contributes nothing and the far edge only
    // pushes the sum below 0, which the clamp removes.
    if (std::abs(y) - t.r >= 0.78 / s) {
      continue;
    }
    const double l = ScreenSidesDb(y - t.r, y + t.r, s);
    sum += l;
    n += l >= 1.0;
  }
  if (n_counted) {
    *n_counted = n;
  }
  return sum;
}

double LinkModel::BushesDb(const Endpoint & a, const Endpoint & b) const
{
  const Frame f(a, b);
  double sum = 0.0;
  for (const auto & bu : w_.bushes) {
    const double xc = f.Along(bu.x, bu.y);
    if (xc <= 0.0 || xc >= f.d) {
      continue;
    }
    const double c = std::cos(bu.yaw), sn = std::sin(bu.yaw);
    double y_lo = 1e18, y_hi = -1e18;
    for (int sx : {-1, 1}) {
      for (int sy : {-1, 1}) {
        const double px = bu.x + c * sx * bu.half_x - sn * sy * bu.half_y;
        const double py = bu.y + sn * sx * bu.half_x + c * sy * bu.half_y;
        const double y = f.Cross(px, py);
        y_lo = std::min(y_lo, y);
        y_hi = std::max(y_hi, y);
      }
    }
    const double s = NuScale(f.d, xc, lambda_);
    // Chord of the link through the footprint (Liang-Barsky in the bush frame).
    auto to_local = [&](const Endpoint & e, double & lx, double & ly) {
        const double dx = e.x - bu.x, dy = e.y - bu.y;
        lx = c * dx + sn * dy;
        ly = -sn * dx + c * dy;
      };
    double ax, ay, bx, by;
    to_local(a, ax, ay);
    to_local(b, bx, by);
    double t0 = 0.0, t1 = 1.0;
    const double dxl = bx - ax, dyl = by - ay;
    const double pp[4] = {-dxl, dxl, -dyl, dyl};
    const double qq[4] = {ax + bu.half_x, bu.half_x - ax, ay + bu.half_y, bu.half_y - ay};
    bool inside = true;
    for (int k = 0; k < 4 && inside; ++k) {
      if (pp[k] == 0.0) {
        inside = qq[k] >= 0.0;
      } else {
        const double r = qq[k] / pp[k];
        if (pp[k] < 0.0) {
          t0 = std::max(t0, r);
        } else {
          t1 = std::min(t1, r);
        }
      }
    }
    const double chord = (inside && t1 > t0) ? (t1 - t0) * f.d : 0.0;
    if (chord <= 0.0) {
      // Beside the link: its two side edges, as a trunk.
      if (std::min(std::abs(y_lo), std::abs(y_hi)) < 0.78 / s || (y_lo < 0 && y_hi > 0)) {
        sum += ScreenSidesDb(y_lo, y_hi, s);
      }
      continue;
    }
    // Through it: top + two sides + the through-vegetation path (§2.11).
    const double h_los = a.antenna_h + (b.antenna_h - a.antenna_h) * xc / f.d;
    const double h_top = bu.height - h_los;
    const double gamma_u = h_top > 0.0 ? p_.bush_gamma_high_db_m : p_.bush_gamma_low_db_m;
    const double lin = Db2Lin(J(h_top * s)) + Db2Lin(J(-y_lo * s)) + Db2Lin(J(y_hi * s)) +
      Db2Lin(gamma_u * chord);
    sum += std::max(0.0, -10.0 * std::log10(lin));
  }
  return sum;
}

double LinkModel::RidgeDb(const Ridge & r, const Endpoint & a, const Endpoint & b,
  double * cross_x, double * cross_y) const
{
  const Frame f(a, b);
  // Crossings of the link with the centre-line polyline.
  double best = -1.0;
  for (size_t k = 0; k + 1 < r.pts.size(); ++k) {
    const auto & p = r.pts[k];
    const auto & q = r.pts[k + 1];
    const double yp = f.Cross(p[0], p[1]), yq = f.Cross(q[0], q[1]);
    if ((yp > 0) == (yq > 0) || yp == yq) {
      continue;
    }
    const double u = yp / (yp - yq);
    const double ix = p[0] + u * (q[0] - p[0]), iy = p[1] + u * (q[1] - p[1]);
    const double xc = f.Along(ix, iy);
    if (xc <= 0.0 || xc >= f.d) {
      continue;
    }
    const double s = NuScale(f.d, xc, lambda_);
    const double h_los = a.antenna_h + (b.antenna_h - a.antenna_h) * xc / f.d;
    double lin = Db2Lin(J((r.height - h_los) * s));
    for (const auto * e : {&r.pts.front(), &r.pts.back()}) {
      const double xe = std::clamp(f.Along((*e)[0], (*e)[1]), 0.01 * f.d, 0.99 * f.d);
      const double he = std::abs(f.Cross((*e)[0], (*e)[1]));
      lin += Db2Lin(J(he * NuScale(f.d, xe, lambda_)));
    }
    const double l = std::max(0.0, -10.0 * std::log10(lin));
    if (l > best) {
      best = l;
      if (cross_x) {*cross_x = ix;}
      if (cross_y) {*cross_y = iy;}
    }
  }
  if (best >= 0.0) {
    return best;
  }
  // No crossing: the nearest vertex (usually an end) as a grazing edge (§2.12 step 6).
  double l = 0.0;
  for (const auto & v : r.pts) {
    const double xv = f.Along(v[0], v[1]);
    if (xv <= 0.0 || xv >= f.d) {
      continue;
    }
    l = std::max(l, J(-std::abs(f.Cross(v[0], v[1])) * NuScale(f.d, xv, lambda_)));
  }
  if (cross_x) {*cross_x = std::numeric_limits<double>::quiet_NaN();}
  return l;
}

double LinkModel::RidgesDb(const Endpoint & a, const Endpoint & b) const
{
  if (w_.ridges.empty()) {
    return 0.0;
  }
  // Deygout (P.526 §4.5.1) over ridges, each scored by its §2.12 loss: the
  // principal ridge on the full path, then the worst ridge on each sub-path to
  // its crest. Up to three edges.
  size_t principal = 0;
  double lp = -1.0, px = 0.0, py = 0.0;
  for (size_t k = 0; k < w_.ridges.size(); ++k) {
    double cx, cy;
    const double l = RidgeDb(w_.ridges[k], a, b, &cx, &cy);
    if (l > lp) {
      lp = l;
      principal = k;
      px = cx;
      py = cy;
    }
  }
  if (lp <= 0.0 || std::isnan(px) || w_.ridges.size() == 1) {
    return std::max(lp, 0.0);
  }
  Endpoint crest;
  crest.x = px;
  crest.y = py;
  crest.antenna_h = w_.ridges[principal].height;
  double sub = 0.0;
  for (const auto & seg : {std::make_pair(a, crest), std::make_pair(crest, b)}) {
    double worst = 0.0;
    for (size_t k = 0; k < w_.ridges.size(); ++k) {
      if (k != principal) {
        worst = std::max(worst, RidgeDb(w_.ridges[k], seg.first, seg.second, nullptr, nullptr));
      }
    }
    sub += worst;
  }
  return lp + sub;
}

double LinkModel::MeasureGammaExplicit(uint64_t seed, int links) const
{
  if (w_.trunks.empty() && w_.bushes.empty()) {
    return 0.0;
  }
  const double h = p_.antenna_height_m;
  auto blocked = [&](double x, double y) {
      if (x < w_.xmin || x > w_.xmax || y < w_.ymin || y > w_.ymax) {
        return true;
      }
      for (const auto & t : w_.trunks) {
        if (std::hypot(x - t.x, y - t.y) < t.r + kRobotHalfWidth) {
          return true;
        }
      }
      for (const auto & bu : w_.bushes) {
        const double c = std::cos(bu.yaw), sn = std::sin(bu.yaw);
        const double lx = c * (x - bu.x) + sn * (y - bu.y);
        const double ly = -sn * (x - bu.x) + c * (y - bu.y);
        if (std::abs(lx) < bu.half_x + kRobotHalfWidth &&
          std::abs(ly) < bu.half_y + kRobotHalfWidth)
        {
          return true;
        }
      }
      for (const auto & r : w_.ridges) {
        for (size_t k = 0; k + 1 < r.pts.size(); ++k) {
          if (PointSegDist(x, y, r.pts[k], r.pts[k + 1]) < 1.0 + kRobotHalfWidth) {
            return true;
          }
        }
      }
      return false;
    };
  const uint64_t stream = Fnv1a64("gamma_explicit");
  uint64_t k = 0;
  double sum = 0.0;
  int n = 0;
  for (uint64_t tries = 0; n < links && tries < 100ULL * links; ++tries) {
    const double d = 10.0 + 50.0 * ToUnit(CounterHash(seed, stream, k++));
    const double x0 = w_.xmin + (w_.xmax - w_.xmin) * ToUnit(CounterHash(seed, stream, k++));
    const double y0 = w_.ymin + (w_.ymax - w_.ymin) * ToUnit(CounterHash(seed, stream, k++));
    const double th = 2.0 * kPi * ToUnit(CounterHash(seed, stream, k++));
    const double x1 = x0 + d * std::cos(th), y1 = y0 + d * std::sin(th);
    if (blocked(x0, y0) || blocked(x1, y1)) {
      continue;
    }
    Endpoint a, b;
    a.x = x0;
    a.y = y0;
    b.x = x1;
    b.y = y1;
    a.antenna_h = b.antenna_h = h;
    sum += (TrunksDb(a, b) + BushesDb(a, b)) / d;
    ++n;
  }
  if (n < links) {
    throw std::runtime_error("gamma_explicit: could only place " + std::to_string(n) +
            " of " + std::to_string(links) + " sample links in the mission area");
  }
  return sum / n;
}

Breakdown LinkModel::Evaluate(const Endpoint & a, const Endpoint & b, double fade_db) const
{
  Breakdown o;
  o.fade_db = fade_db;
  if (p_.model == Model::kV1) {
    // v1's ComputePair, term for term and in the same order.
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    const double distance = std::max(std::sqrt(dx * dx + dy * dy + dz * dz), 0.1);
    const double link_width = p_.tree_radius_m + 0.5 * std::sqrt(kC / p_.frequency_hz * distance);
    int trees = 0;
    for (const auto & t : w_.trunks) {
      if (V1PointToSegmentDistSq2D(a, b, t.x, t.y) < link_width * link_width) {
        ++trees;
      }
    }
    double path_loss = p_.p0_db + 20.0 * std::log10(distance) +
      trees * p_.tree_attenuation_db + fade_db;
    if (p_.max_range_m > 0.0 && distance > p_.max_range_m) {
      path_loss += 200.0;
      o.l_range_db = 200.0;
    }
    o.distance_m = distance;
    o.pl_ground_db = p_.p0_db + 20.0 * std::log10(distance);
    o.l_trunks_db = trees * p_.tree_attenuation_db;
    o.l_veg_db = o.l_trunks_db;
    o.trunks_on_link = trees;
    o.path_loss_db = path_loss;
    o.rx_dbm = p_.tx_power_dbm - path_loss;
    o.snr_db = o.rx_dbm - p_.noise_floor_dbm;
    return o;
  }
  const Frame f(a, b);
  const double d = f.d;
  o.distance_m = d;
  o.pl_ground_db = PlGroundDb(d, a.antenna_h, b.antenna_h);
  o.l_background_db = gamma_bg_ * d;
  o.l_trunks_db = TrunksDb(a, b, &o.trunks_on_link);
  o.l_bushes_db = BushesDb(a, b);
  const double direct = o.pl_ground_db + o.l_background_db + o.l_trunks_db + o.l_bushes_db;
  const double veg = p_.veg_max_atten_db > 0.0 ?
    PowerSumDb(direct, o.pl_ground_db + p_.veg_max_atten_db) : direct;
  o.l_veg_db = veg - o.pl_ground_db;
  o.l_ridges_db = RidgesDb(a, b);
  if (p_.max_range_m > 0.0 && d > p_.max_range_m) {
    o.l_range_db = 200.0;
  }
  o.path_loss_db = veg + o.l_ridges_db + o.l_range_db + fade_db;
  o.rx_dbm = p_.tx_power_dbm + p_.tx_gain_dbi + p_.rx_gain_dbi - o.path_loss_db;
  o.snr_db = o.rx_dbm - NoiseDbm(p_);
  return o;
}

// ---- randomness -------------------------------------------------------------------

uint64_t Fnv1a64(const std::string & s, uint64_t h)
{
  for (unsigned char ch : s) {
    h ^= ch;
    h *= 1099511628211ULL;
  }
  return h;
}

double CounterGaussian(uint64_t seed, uint64_t stream, uint64_t counter)
{
  const uint64_t h = CounterHash(seed, stream, counter);
  const double u1 = ToUnit(h);
  const double u2 = ToUnit(SplitMix64(h));
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

CounterFade::CounterFade(uint64_t seed, uint64_t stream, double sigma_db, double alpha)
: seed_(seed), stream_(stream), sigma_(sigma_db), alpha_(alpha), x_(0.0)
{
}

double CounterFade::Step(uint64_t tick)
{
  // Walk every tick since the last call, so x_t is the same whichever ticks
  // the caller skipped (a pair without fresh poses is not evaluated).
  for (uint64_t k = last_ + 1; k <= tick; ++k) {
    const double z = CounterGaussian(seed_, stream_, k);
    x_ = k == 1 ? sigma_ * z :
      alpha_ * x_ + std::sqrt(1.0 - alpha_ * alpha_) * sigma_ * z;
  }
  last_ = std::max(last_, tick);
  return x_;
}

}  // namespace hmr_radio
