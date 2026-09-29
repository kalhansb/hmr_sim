// Radio emulator v2 core: the §2.14 propagation stack (DESIGN_radio_v2.md) and
// the v1 link equation, with no ROS dependency. The node, the radio_probe CLI
// and the known-answer tests all call this one implementation, so a test never
// checks a copy of the physics.
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace hmr_radio
{

// ---- world ------------------------------------------------------------------

struct Trunk
{
  double x = 0.0, y = 0.0, r = 0.0;   // centre and radius at antenna height (m)
};

// Oriented rectangular footprint with a flat top.
struct Bush
{
  double x = 0.0, y = 0.0;             // footprint centre (m)
  double half_x = 0.0, half_y = 0.0;   // half extents in the bush frame (m)
  double yaw = 0.0;                    // rad
  double height = 0.0;                 // top above ground (m)
};

// Steep-sided ridge along a centre-line polyline, crest height above ground.
struct Ridge
{
  std::vector<std::array<double, 2>> pts;
  double height = 0.0;
};

struct World
{
  std::vector<Trunk> trunks;
  std::vector<Bush> bushes;
  std::vector<Ridge> ridges;
  // Mission area, used only to sample the γ_explicit links.
  double xmin = -50.0, xmax = 50.0, ymin = -50.0, ymax = 50.0;
  std::string source;          // SDF the file was generated from
  std::string source_sha256;   // its sha256, checked by the launch file
};

// Reads a radio_world.yaml written by extract_radio_world.py. Throws
// std::runtime_error on a malformed file.
World LoadWorldYaml(const std::string & path);

// ---- profile ----------------------------------------------------------------

enum class Model { kV1, kV2 };

struct Profile
{
  std::string name = "v1";
  Model model = Model::kV1;
  double frequency_hz = 2.4e9;
  double tx_power_dbm = 30.0;
  double p0_db = 49.17;          // loss at 1 m, antenna gains excluded (v2)
  double max_range_m = 30.0;     // <= 0: off. Beyond it +200 dB
  // v1 only
  double noise_floor_dbm = -101.0;
  double tree_attenuation_db = 70.0;
  double tree_radius_m = 0.3;
  // v2 only
  double tx_gain_dbi = 2.0;
  double rx_gain_dbi = 2.0;
  double noise_figure_db = 7.0;
  double bandwidth_hz = 20e6;
  bool ground_two_ray = true;
  double gamma_itu_db_m = 0.44;  // P.833 woodland, per metre
  double veg_max_atten_db = 34.4;   // A_m; <= 0 means no cap
  double bush_gamma_low_db_m = 0.5;   // shrub top at or below the link line
  double bush_gamma_high_db_m = 1.5;  // shrub top above it
  double antenna_height_m = 0.4;      // default for robots without their own
  // fading, both models: AR(1) per link tick with stationary std fade_sigma_db
  double fade_sigma_db = 4.8;
  double fade_alpha = 0.9;
};

// Noise power at the receiver (dBm): v1 uses noise_floor_dbm, v2 kTB + NF.
double NoiseDbm(const Profile & p);

// Reads a profile yaml (config/profiles/*.yaml): the node's own parameter file,
// so the CLI and the tests load exactly what the node loads. Accepts the
// hmr_comms_sim/ros__parameters nesting or a flat map. Keys it does not know
// are ignored; radio_stack decides the model (v1 or v2_prop).
Profile LoadProfileYaml(const std::string & path);

// ---- P.526 building blocks ----------------------------------------------------

// P.526-16 eq. 31: knife-edge loss J(ν) in dB, 0 for ν <= -0.78.
double J(double nu);
// ν scale for an edge at along-track d1 on a link of horizontal length d:
// ν = h · s, s = sqrt(2 d / (λ d1 (d - d1))).
double NuScale(double d, double d1, double lambda);
// Power sum of parallel paths, each given as a loss in dB (−10·log10 Σ 10^(−L/10)).
double PowerSumDb(double a, double b);
// P.526 §5.1 eq. 72 for a screen spanning cross-track [y_lo, y_hi] with the link
// on y = 0: the two side edges, power-summed. Clamped at 0 (a screen never adds
// gain; the unclamped form dips to about −0.2 dB just off a narrow trunk).
double ScreenSidesDb(double y_lo, double y_hi, double s);

// ---- link evaluation -----------------------------------------------------------

struct Endpoint
{
  double x = 0.0, y = 0.0, z = 0.0;   // z: odom z, used only by the v1 distance
  double antenna_h = 0.4;             // antenna height above ground (v2)
};

struct Breakdown
{
  double distance_m = 0.0;     // v1: 3D, v2: horizontal
  double pl_ground_db = 0.0;   // v1: p0 + 20 log d; v2: plus the two-ray term
  double l_background_db = 0.0;   // γ_bg · d (v2)
  double l_trunks_db = 0.0;    // v1: n · tree_attenuation_db; v2: Σ P.526 trunks
  double l_bushes_db = 0.0;
  double l_veg_db = 0.0;       // after the A_m power sum, relative to pl_ground
  double l_ridges_db = 0.0;
  double l_range_db = 0.0;     // 200 dB past max_range_m, else 0
  double fade_db = 0.0;
  double path_loss_db = 0.0;   // pl_ground + l_veg + l_ridges + l_range + fade
  double rx_dbm = 0.0;
  double snr_db = 0.0;
  int trunks_on_link = 0;      // v1: corridor count; v2: trunks costing >= 1 dB
};

class LinkModel
{
public:
  // gamma_explicit_seed / links: the fixed sample that measures the mean
  // explicit vegetation loss per metre (§2.14). It depends on the world and the
  // profile only, never on a cell's seed.
  LinkModel(Profile profile, World world,
    uint64_t gamma_explicit_seed = 20260927, int gamma_explicit_links = 10000);

  const Profile & profile() const { return p_; }
  const World & world() const { return w_; }
  double lambda() const { return lambda_; }
  double gamma_explicit() const { return gamma_explicit_; }   // dB/m
  double gamma_bg() const { return gamma_bg_; }               // dB/m

  // Deterministic: the fade is passed in, so the caller owns the randomness.
  Breakdown Evaluate(const Endpoint & a, const Endpoint & b, double fade_db) const;

  // Parts, exposed for the tests and radio_probe (v2 geometry; horizontal).
  double PlGroundDb(double d, double ha, double hb) const;
  double TrunksDb(const Endpoint & a, const Endpoint & b, int * n_counted = nullptr) const;
  double BushesDb(const Endpoint & a, const Endpoint & b) const;
  double RidgesDb(const Endpoint & a, const Endpoint & b) const;

private:
  double RidgeDb(const Ridge & r, const Endpoint & a, const Endpoint & b,
    double * cross_x, double * cross_y) const;
  double MeasureGammaExplicit(uint64_t seed, int links) const;

  Profile p_;
  World w_;
  double lambda_;
  double gamma_explicit_ = 0.0;
  double gamma_bg_ = 0.0;
};

// ---- randomness ----------------------------------------------------------------

// Counter-based standard normal: a pure function of (seed, stream, counter), so
// a link's fade never depends on how many other draws happened before it.
double CounterGaussian(uint64_t seed, uint64_t stream, uint64_t counter);
// Stable 64-bit FNV-1a, used for stream ids and the regime hash.
uint64_t Fnv1a64(const std::string & s, uint64_t h = 1469598103934665603ULL);

// AR(1) shadow fade for one link, driven by CounterGaussian: x_1 = sigma z_1,
// x_t = alpha x_(t-1) + sqrt(1 - alpha^2) sigma z_t, so it starts stationary.
// Step(tick) returns x_tick (ticks from 1) and walks any ticks skipped since
// the last call, so the value never depends on which ticks were evaluated.
class CounterFade
{
public:
  CounterFade(uint64_t seed, uint64_t stream, double sigma_db, double alpha);
  double Step(uint64_t tick);
  double value() const { return x_; }

private:
  uint64_t seed_, stream_;
  double sigma_, alpha_, x_;
  uint64_t last_ = 0;
};

}  // namespace hmr_radio
