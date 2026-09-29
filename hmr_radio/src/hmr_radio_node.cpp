// hmr_radio_node — radio emulator v2 (DESIGN_radio_v2.md §2.14, §12 and §13).
//
// A copy of hmr_sim's hmr_comms_sim_node (v1, pinned and left untouched) whose
// relay, airtime, transmission models, stats and diagnostics are unchanged
// line for line under radio_stack v1 and v2_prop. Only the link computation
// differs there: it calls hmr_radio's core library, the same code radio_probe
// and the known-answer tests run. v2_phy replaces the rate tiers and the
// airtime bucket with radio_mac (below); the relay queues are v1's.
//
// It runs under the node name hmr_comms_sim, so every consumer (link_logger,
// comms_gates, the planners' link gate) reads the same topics as with v1.
//
// Link model, per robot pair at link_rate_hz on the node clock (sim time):
//   radio_stack v1       v1's equation term for term (corridor trunk count x
//                        tree_attenuation_db, 3D distance, 200 dB past
//                        max_range_m); with fade_mode legacy it reproduces a
//                        v1 run's fading trace too.
//   radio_stack v2_prop  §2.14: two-ray ground + P.526 per trunk, bush and
//                        ridge + the ITU background gamma_bg = gamma_itu -
//                        gamma_explicit, capped by A_m; v1's rate tiers and
//                        shared airtime. gamma_explicit is measured once at
//                        startup from a fixed sample, never from the cell seed.
//   radio_stack v2_phy   v2_prop's propagation, then §13's 802.11n PHY and MAC
//                        (radio_mac): HT MCS0-7 under the NIST error model, a
//                        Minstrel-like MCS per pair, and the air shared by an
//                        analytic EDCA model (Bianchi per carrier-sense domain,
//                        hidden robots, spatial reuse) in place of v1's single
//                        airtime bucket. Needs transmission_model progressive.
//   Fading: AR(1). fade_mode counter draws each link's fade as a pure function
//   of (seed, link, tick), so it never depends on traffic or on other links.
//
// Obstacles come from a radio world yaml (scripts/extract_radio_world.py),
// never from name matching here; the launch file checks it against the SDF.
//
// Diagnostics, in addition to v1's link_states / robot_index / stats:
//   ~/link_breakdown  Float64MultiArray, one row per pair, columns in
//                     ~/radio_regime: where each link's loss comes from.
//   ~/radio_regime    latched JSON: every effective radio parameter, the world
//                     hash, gamma_explicit and the regime hash over all of it.
//   ~/link_mac        v2_phy only. Float64MultiArray, one row per directed
//                     link, columns in ~/radio_regime: MCS, PER, and the MAC's
//                     busy time, goodput and losses over the last link period.
// Moved comments (the relay code's "(notes: ...)" tags): hmr_sim/docs/hmr_sim_code_notes.md

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "hmr_radio/radio_core.hpp"
#include "hmr_radio/radio_mac.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace
{

struct Vec3
{
  double x = 0.0, y = 0.0, z = 0.0;
};

// The one channel width the whole rate ladder is quoted in: the 72.0 / 28.9 /
// 7.2 Mbps tiers are 802.11n 20 MHz MCS7 / MCS3 / MCS0 rates. Changing this
// without changing the ladder makes every Eb/N0 in this file wrong.
constexpr double kChannelBandwidthHz = 20e6;

double DbmToPow(double dbm)
{
  return 0.001 * std::pow(10.0, dbm / 10.0);
}

// 64-QAM BER over AWGN (line of sight). spectral_efficiency is bits/s/Hz at the
// tier the link is in, not the top tier. 64-QAM at every tier makes this a
// pessimistic bound; ber gates nothing. (notes: comms-sim-awgn-ber-tier)
double AwgnQam64Ber(double power_w, double noise_w, double spectral_efficiency)
{
  const double M = 64.0;
  const int k = 6;
  const double ebno = (power_w / noise_w) / spectral_efficiency;
  const double factor = (4.0 / k) * (1.0 - 1.0 / std::sqrt(M)) * 0.5;
  const double x = std::sqrt((3.0 * k * ebno) / (M - 1.0));
  return factor * std::erfc(x / std::sqrt(2.0));
}

// 64-QAM BER over a Rayleigh channel (trees on the link). Same
// spectral_efficiency contract and the same pessimistic-bound caveat as above.
double RayleighQam64Ber(double power_w, double noise_w, double spectral_efficiency)
{
  const double ebno = (power_w / noise_w) / spectral_efficiency;
  auto term = [ebno](double c) {
    return 1.0 - std::sqrt((c / 7.0) * ebno / (1.0 + (c / 7.0) * ebno));
  };
  return (7.0 / 24.0) * term(1.0) + (1.0 / 4.0) * term(9.0) -
         (1.0 / 24.0) * term(25.0) + (1.0 / 24.0) * term(81.0) -
         (1.0 / 24.0) * term(169.0);
}

// MessageSuccessProbability was removed on purpose. Do not gate delivery on
// BER: the bandwidth state machine is the only authority on whether a message
// gets through. (notes: comms-sim-no-ber-gating)

}  // namespace

class HmrRadioNode : public rclcpp::Node
{
public:
  HmrRadioNode()
  : Node("hmr_comms_sim")
  {
    // ---- parameters -------------------------------------------------------
    robot_names_ = declare_parameter<std::vector<std::string>>(
      "robot_names", std::vector<std::string>{});
    pose_topic_pattern_ = declare_parameter<std::string>(
      "pose_topic_pattern", "/{robot}/odom_ground_truth");
    auto reliable_topics = declare_parameter<std::vector<std::string>>(
      "reliable_topics", std::vector<std::string>{});
    auto best_effort_topics = declare_parameter<std::vector<std::string>>(
      "best_effort_topics", std::vector<std::string>{});
    auto latest_topics = declare_parameter<std::vector<std::string>>(
      "latest_topics", std::vector<std::string>{});
    // Empty entries are dropped, so a list can be emptied from a launch file
    // (a parameter override cannot carry an empty list): [""] means none.
    for (auto * list : {&reliable_topics, &best_effort_topics, &latest_topics}) {
      list->erase(std::remove(list->begin(), list->end(), std::string{}), list->end());
    }
    // Seconds a topic may stay pending after the last discovery (or the first
    // poll) before a one-time warning names it. <= 0: never warn.
    pending_warn_sec_ = declare_parameter<double>("pending_warn_sec", 120.0);

    // ---- radio (hmr_radio core; the profile yaml sets these) --------------
    // radio_stack is the one switch between models, and only listed stacks
    // run: a layer combination nothing tested is refused at startup.
    radio_stack_ = declare_parameter<std::string>("radio_stack", "v1");
    radio_profile_ = declare_parameter<std::string>("radio_profile", "v1");
    radio_world_file_ = declare_parameter<std::string>("radio_world_file", "");
    fade_mode_ = declare_parameter<std::string>("fade_mode", "legacy");
    ground_model_ = declare_parameter<std::string>("ground_model", "none");
    {
      hmr_radio::Profile d;   // defaults are v1's
      auto & p = profile_;
      p.frequency_hz = declare_parameter<double>("frequency_hz", d.frequency_hz);
      p.tx_power_dbm = declare_parameter<double>("tx_power_dbm", d.tx_power_dbm);
      p.p0_db = declare_parameter<double>("p0_db", d.p0_db);
      p.max_range_m = declare_parameter<double>("max_range_m", d.max_range_m);
      p.noise_floor_dbm = declare_parameter<double>("noise_floor_dbm", d.noise_floor_dbm);
      p.tree_attenuation_db = declare_parameter<double>("tree_attenuation_db", d.tree_attenuation_db);
      p.tree_radius_m = declare_parameter<double>("tree_radius_m", d.tree_radius_m);
      p.tx_gain_dbi = declare_parameter<double>("tx_gain_dbi", d.tx_gain_dbi);
      p.rx_gain_dbi = declare_parameter<double>("rx_gain_dbi", d.rx_gain_dbi);
      p.noise_figure_db = declare_parameter<double>("noise_figure_db", d.noise_figure_db);
      p.bandwidth_hz = declare_parameter<double>("bandwidth_hz", d.bandwidth_hz);
      p.gamma_itu_db_m = declare_parameter<double>("gamma_itu_db_m", d.gamma_itu_db_m);
      p.veg_max_atten_db = declare_parameter<double>("veg_max_atten_db", d.veg_max_atten_db);
      p.bush_gamma_low_db_m = declare_parameter<double>("bush_gamma_low_db_m", d.bush_gamma_low_db_m);
      p.bush_gamma_high_db_m =
        declare_parameter<double>("bush_gamma_high_db_m", d.bush_gamma_high_db_m);
      p.antenna_height_m = declare_parameter<double>("antenna_height_m", d.antenna_height_m);
      p.fade_sigma_db = declare_parameter<double>("fade_sigma_db", d.fade_sigma_db);
      p.fade_alpha = declare_parameter<double>("fade_alpha", d.fade_alpha);
    }
    // 802.11n MAC, radio_stack v2_phy only (DESIGN_radio_v2.md §13).
    {
      hmr_radio::MacParams d;
      auto & m = mac_;
      m.slot_s = 1e-6 * declare_parameter<double>("mac_slot_us", 1e6 * d.slot_s);
      m.sifs_s = 1e-6 * declare_parameter<double>("mac_sifs_us", 1e6 * d.sifs_s);
      m.aifsn = static_cast<int>(declare_parameter<int64_t>("mac_aifsn", d.aifsn));
      m.cw_min = static_cast<int>(declare_parameter<int64_t>("mac_cw_min", d.cw_min));
      m.cw_max = static_cast<int>(declare_parameter<int64_t>("mac_cw_max", d.cw_max));
      m.retry_limit = static_cast<int>(declare_parameter<int64_t>("mac_retry_limit", d.retry_limit));
      m.mpdu_payload_bytes = static_cast<int>(
        declare_parameter<int64_t>("mac_mpdu_payload_bytes", d.mpdu_payload_bytes));
      m.mpdu_overhead_bytes = static_cast<int>(
        declare_parameter<int64_t>("mac_mpdu_overhead_bytes", d.mpdu_overhead_bytes));
      m.max_ampdu_bytes = static_cast<int>(
        declare_parameter<int64_t>("mac_max_ampdu_bytes", d.max_ampdu_bytes));
      m.max_ampdu_mpdus = static_cast<int>(
        declare_parameter<int64_t>("mac_max_ampdu_mpdus", d.max_ampdu_mpdus));
      m.ppdu_max_s = 1e-6 * declare_parameter<double>("mac_ppdu_max_us", 1e6 * d.ppdu_max_s);
      m.detect_snr_db = declare_parameter<double>("mac_detect_snr_db", d.detect_snr_db);
      m.minstrel_keep_per_100ms =
        declare_parameter<double>("minstrel_keep_per_100ms", d.minstrel_keep_per_100ms);
      m.minstrel_min_success =
        declare_parameter<double>("minstrel_min_success", d.minstrel_min_success);
    }
    // Per robot, in robot_names order; empty = antenna_height_m for all.
    antenna_heights_m_ = declare_parameter<std::vector<double>>(
      "antenna_heights_m", std::vector<double>{});
    gamma_explicit_seed_ = declare_parameter<int64_t>("gamma_explicit_seed", 20260927);
    gamma_explicit_links_ = declare_parameter<int64_t>("gamma_explicit_links", 10000);
    link_rate_hz_ = declare_parameter<double>("link_rate_hz", 5.0);
    delay_ms_ = declare_parameter<double>("delay_ms", 2.5);
    airtime_capacity_ = declare_parameter<double>("airtime_capacity", 1.0);
    airtime_burst_s_ = declare_parameter<double>("airtime_burst_s", 0.1);
    // Best-effort messages bypass the airtime admission check (still charged).
    best_effort_priority_ = declare_parameter<bool>("best_effort_priority", false);
    // Residual loss on a link the state machine reports as up, matching
    // HMRNetSim.cc's PDR of 1e-8 above the SNR>=2 dB boundary.
    residual_pdr_ = declare_parameter<double>("residual_pdr", 1e-8);
    // Seconds (node clock, so sim time under use_sim_time) after which a pose
    // is stale; links touching a stale endpoint go invalid: no delivery and a
    // zeroed diagnostic row. <= 0 trusts a pose forever.
    // (notes: comms-sim-pose-timeout)
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 2.0);
    reliable_queue_max_bytes_ = static_cast<size_t>(
      declare_parameter<int64_t>("reliable_queue_max_bytes", 64LL * 1024 * 1024));
    rx_qos_depth_ = static_cast<size_t>(declare_parameter<int64_t>("rx_qos_depth", 100));
    // Latest topics carry whole states (~10 MB map frames): a deep history
    // buys nothing and could hold gigabytes.
    latest_qos_depth_ = static_cast<size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("latest_qos_depth", 2)));
    const auto tx_model = declare_parameter<std::string>("transmission_model", "admission");
    if (tx_model == "progressive") {
      progressive_ = true;
    } else if (tx_model != "admission") {
      RCLCPP_FATAL(get_logger(),
        "transmission_model must be 'admission' or 'progressive', got '%s'.", tx_model.c_str());
      throw std::runtime_error("bad transmission_model");
    }
    const double stats_period_s = declare_parameter<double>("stats_period_s", 10.0);
    const int64_t seed = declare_parameter<int64_t>("seed", 42);

    if (robot_names_.size() < 2) {
      RCLCPP_FATAL(get_logger(), "robot_names needs at least 2 robots.");
      throw std::runtime_error("robot_names needs at least 2 robots");
    }
    {
      std::map<std::string, int> listed;
      for (const auto * list : {&reliable_topics, &best_effort_topics, &latest_topics}) {
        for (const auto & t : *list) {
          if (++listed[t] > 1) {
            RCLCPP_FATAL(get_logger(),
              "Topic '%s' listed in more than one of reliable_topics, "
              "best_effort_topics and latest_topics.", t.c_str());
            throw std::runtime_error("topic in more than one policy list");
          }
        }
      }
    }
    if (reliable_topics.empty() && best_effort_topics.empty() && latest_topics.empty()) {
      RCLCPP_WARN(get_logger(),
        "No reliable_topics/best_effort_topics/latest_topics configured — only link "
        "states will be published.");
    }

    // Physics RNG and drop RNG are separate on purpose: fades then depend only
    // on (seed, tick), never on traffic, so link traces are repeatable even
    // when the relayed workload differs.
    fade_rng_.seed(static_cast<uint64_t>(seed));
    drop_rng_.seed(static_cast<uint64_t>(seed) + 1);

    const size_t n = robot_names_.size();
    poses_.resize(n);
    has_pose_.assign(n, false);
    pose_last_s_.assign(n, 0.0);
    pose_topics_.assign(n, std::string());
    for (size_t i = 0; i < n; ++i) {
      name_to_idx_[robot_names_[i]] = i;
    }

    SetUpRadio(seed);

    // ---- pose subscriptions ----------------------------------------------
    for (size_t i = 0; i < n; ++i) {
      std::string topic = pose_topic_pattern_;
      const auto marker = topic.find("{robot}");
      if (marker == std::string::npos) {
        RCLCPP_FATAL(get_logger(), "pose_topic_pattern must contain '{robot}'.");
        throw std::runtime_error("bad pose_topic_pattern");
      }
      topic.replace(marker, std::string("{robot}").size(), robot_names_[i]);
      auto sub = create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS(),
        [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
          poses_[i] = {msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z};
          has_pose_[i] = true;
          // Receipt time, not msg->header.stamp: the freshness question is
          // "did this stream stall", which a publisher stamping stale headers
          // would hide. The two agree to within a transport hop in the sim.
          pose_last_s_[i] = get_clock()->now().seconds();
        });
      pose_subs_.push_back(sub);
      pose_topics_[i] = topic;
      RCLCPP_INFO(get_logger(), "Pose source for '%s': %s",
        robot_names_[i].c_str(), topic.c_str());
    }

    // ---- diagnostics publishers ------------------------------------------
    link_states_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/link_states", 10);
    link_breakdown_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/link_breakdown", 10);
    stats_pub_ = create_publisher<std_msgs::msg::String>("~/stats", 10);
    robot_index_pub_ = create_publisher<std_msgs::msg::String>(
      "~/robot_index", rclcpp::QoS(1).transient_local());
    PublishRobotIndex();

    // ---- relay topic discovery -------------------------------------------
    for (size_t s = 0; s < n; ++s) {
      for (const auto & t : reliable_topics) {
        pending_topics_["/" + robot_names_[s] + "/" + t] = {s, t, Policy::kReliable};
      }
      for (const auto & t : best_effort_topics) {
        pending_topics_["/" + robot_names_[s] + "/" + t] = {s, t, Policy::kBestEffort};
      }
      for (const auto & t : latest_topics) {
        pending_topics_["/" + robot_names_[s] + "/" + t] = {s, t, Policy::kLatest};
      }
    }

    // ---- timers (node clock, so use_sim_time works) ----------------------
    link_timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(1.0 / std::max(link_rate_hz_, 0.1)),
      std::bind(&HmrRadioNode::OnLinkTimer, this));
    delivery_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.005),
      std::bind(&HmrRadioNode::OnDeliveryTimer, this));
    if (!pending_topics_.empty()) {
      discovery_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(1.0),
        std::bind(&HmrRadioNode::DiscoverAndSetup, this));
    }
    stats_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(std::max(stats_period_s, 1.0)),
      std::bind(&HmrRadioNode::PublishStats, this));

    RCLCPP_INFO(get_logger(),
      "hmr_comms_sim up: %zu robots, %zu trees, %zu relay topics pending, seed=%ld, "
      "transmission_model=%s.",
      n, model_->world().trunks.size(), pending_topics_.size(), static_cast<long>(seed),
      progressive_ ? "progressive" : "admission");
  }

private:
  enum class Policy { kReliable, kBestEffort, kLatest };

  struct PairState
  {
    // hysteresis state
    std::deque<double> snr_history;
    double fade_db = 0.0;
    // published snapshot
    bool valid = false;      // both endpoint poses have been seen
    bool connected = false;
    double distance_m = 0.0;
    double trees_on_link = 0.0;
    double path_loss_db = 0.0;
    double snr_db = 0.0;
    double ber = 1.0;
    double bandwidth_mbps = 72.0;
    hmr_radio::Breakdown bd;
    std::shared_ptr<hmr_radio::CounterFade> fade;   // fade_mode counter
    hmr_radio::RateControl rate;                    // v2_phy
  };

  struct PendingTopic
  {
    size_t sender;
    std::string comms_topic;
    Policy policy;
  };

  struct QueuedMsg
  {
    std::shared_ptr<rclcpp::SerializedMessage> msg;
    rclcpp::GenericPublisher::SharedPtr pub;
    size_t bytes;
    bool latest = false;
  };

  // progressive model: the message on the air on one directional link
  struct InFlight
  {
    QueuedMsg m;
    double bits_left = 0.0;
    bool active = false;
  };

  struct Delivery
  {
    int64_t t_ns;
    uint64_t seq;
    rclcpp::GenericPublisher::SharedPtr pub;
    std::shared_ptr<rclcpp::SerializedMessage> msg;
    bool operator>(const Delivery & o) const
    {
      return t_ns != o.t_ns ? t_ns > o.t_ns : seq > o.seq;
    }
  };

  struct LinkStats
  {
    uint64_t relayed = 0;
    uint64_t bytes_relayed = 0;
    uint64_t drop_ber = 0;
    uint64_t drop_airtime = 0;
    uint64_t drop_disconnected = 0;
    uint64_t drop_overflow = 0;
    uint64_t drop_superseded = 0;   // latest topics only; not a loss
    uint64_t drop_aborted = 0;      // progressive: latest message cut by a link drop
    uint64_t latest_relayed = 0;    // latest messages delivered (in relayed too)
  };

  // v2_phy, per directed link: cumulative for ~/stats, p_* since the last
  // ~/link_mac row.
  struct MacStats
  {
    double busy_s = 0.0;     // time with a backlog on a connected link
    double txops = 0.0;      // expected successful TXOPs
    double hidden_s = 0.0;   // busy_s weighted by hidden_loss
    std::array<double, hmr_radio::kNumMcs> mcs_s{};   // connected time at each MCS
    double p_busy_s = 0.0, p_bits = 0.0, p_hidden_s = 0.0, p_coll_s = 0.0;
  };

  size_t PairKey(size_t i, size_t j) const
  {
    return std::min(i, j) * robot_names_.size() + std::max(i, j);
  }
  size_t DirKey(size_t s, size_t r) const { return s * robot_names_.size() + r; }

  // ---- radio model --------------------------------------------------------
  void SetUpRadio(int64_t seed)
  {
    const size_t n = robot_names_.size();
    auto & p = profile_;
    p.name = radio_profile_;
    if (radio_stack_ == "v1") {
      p.model = hmr_radio::Model::kV1;
    } else if (radio_stack_ == "v2_prop") {
      p.model = hmr_radio::Model::kV2;
    } else if (radio_stack_ == "v2_phy") {
      p.model = hmr_radio::Model::kV2;
      phy_ = true;
    } else {
      RCLCPP_FATAL(get_logger(),
        "radio_stack '%s' is not an allowed stack. Allowed: v1 (v1's link "
        "equation), v2_prop (DESIGN_radio_v2 §2.14 propagation on v1 tiers and "
        "airtime), v2_phy (§2.14 propagation, §13 PHY and MAC).", radio_stack_.c_str());
      throw std::runtime_error("bad radio_stack");
    }
    if (ground_model_ != "none" && ground_model_ != "two_ray") {
      RCLCPP_FATAL(get_logger(), "ground_model must be none or two_ray, got '%s'.",
        ground_model_.c_str());
      throw std::runtime_error("bad ground_model");
    }
    p.ground_two_ray = ground_model_ == "two_ray";
    if (fade_mode_ != "legacy" && fade_mode_ != "counter") {
      RCLCPP_FATAL(get_logger(), "fade_mode must be legacy or counter, got '%s'.",
        fade_mode_.c_str());
      throw std::runtime_error("bad fade_mode");
    }
    legacy_fade_ = fade_mode_ == "legacy";
    if (p.model == hmr_radio::Model::kV2 && legacy_fade_) {
      RCLCPP_FATAL(get_logger(), "radio_stack %s needs fade_mode counter: the "
        "legacy shared generator makes every link's fade depend on traffic.",
        radio_stack_.c_str());
      throw std::runtime_error("v2 stack with legacy fading");
    }
    if (phy_ && !progressive_) {
      RCLCPP_FATAL(get_logger(), "radio_stack v2_phy needs transmission_model "
        "progressive: its MAC shares the air on the delivery tick, which admission "
        "does not have.");
      throw std::runtime_error("v2_phy without progressive transmission");
    }
    if (!antenna_heights_m_.empty() && antenna_heights_m_.size() != n) {
      RCLCPP_FATAL(get_logger(), "antenna_heights_m has %zu entries for %zu robots.",
        antenna_heights_m_.size(), n);
      throw std::runtime_error("bad antenna_heights_m");
    }
    antenna_h_.assign(n, p.antenna_height_m);
    if (!antenna_heights_m_.empty()) {
      antenna_h_ = antenna_heights_m_;
    }

    hmr_radio::World world;
    if (radio_world_file_.empty()) {
      RCLCPP_WARN(get_logger(),
        "radio_world_file not set — no obstacles: distance, ground and background only.");
    } else {
      try {
        world = hmr_radio::LoadWorldYaml(radio_world_file_);
      } catch (const std::exception & e) {
        RCLCPP_FATAL(get_logger(), "%s", e.what());
        throw;
      }
    }
    try {
      model_ = std::make_unique<hmr_radio::LinkModel>(p, world,
          static_cast<uint64_t>(gamma_explicit_seed_), static_cast<int>(gamma_explicit_links_));
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "radio model: %s", e.what());
      throw;
    }
    noise_dbm_ = hmr_radio::NoiseDbm(p);
    fade_seed_ = static_cast<uint64_t>(seed);

    // The same phrase as v1, so the harness's comms_trees_loaded still parses.
    RCLCPP_INFO(get_logger(),
      "Loaded %zu tree positions (radio world %s; %zu bushes, %zu ridges; source %s)",
      world.trunks.size(), radio_world_file_.empty() ? "(none)" : radio_world_file_.c_str(),
      world.bushes.size(), world.ridges.size(), world.source.c_str());

    // Every effective value, canonical order, then the hash over the string.
    std::ostringstream canon, json;
    auto num = [](double v) {
        std::ostringstream s;
        s.precision(10);
        s << v;
        return s.str();
      };
    std::vector<std::pair<std::string, std::string>> kv = {
      {"radio_stack", radio_stack_}, {"radio_profile", radio_profile_},
      {"fade_mode", fade_mode_}, {"ground_model", ground_model_},
      {"frequency_hz", num(p.frequency_hz)}, {"tx_power_dbm", num(p.tx_power_dbm)},
      {"p0_db", num(p.p0_db)}, {"max_range_m", num(p.max_range_m)},
      {"noise_floor_dbm", num(p.noise_floor_dbm)},
      {"tree_attenuation_db", num(p.tree_attenuation_db)},
      {"tree_radius_m", num(p.tree_radius_m)}, {"tx_gain_dbi", num(p.tx_gain_dbi)},
      {"rx_gain_dbi", num(p.rx_gain_dbi)}, {"noise_figure_db", num(p.noise_figure_db)},
      {"bandwidth_hz", num(p.bandwidth_hz)}, {"gamma_itu_db_m", num(p.gamma_itu_db_m)},
      {"veg_max_atten_db", num(p.veg_max_atten_db)},
      {"bush_gamma_low_db_m", num(p.bush_gamma_low_db_m)},
      {"bush_gamma_high_db_m", num(p.bush_gamma_high_db_m)},
      {"fade_sigma_db", num(p.fade_sigma_db)}, {"fade_alpha", num(p.fade_alpha)},
      {"link_rate_hz", num(link_rate_hz_)},
      {"gamma_explicit_seed", std::to_string(gamma_explicit_seed_)},
      {"gamma_explicit_links", std::to_string(gamma_explicit_links_)},
      {"gamma_explicit_db_m", num(model_->gamma_explicit())},
      {"gamma_bg_db_m", num(model_->gamma_bg())},
      {"noise_dbm", num(noise_dbm_)},
      {"world_source", world.source}, {"world_sha256", world.source_sha256},
      {"world_trunks", std::to_string(world.trunks.size())},
      {"world_bushes", std::to_string(world.bushes.size())},
      {"world_ridges", std::to_string(world.ridges.size())},
    };
    for (size_t i = 0; i < n; ++i) {
      kv.push_back({"antenna_height_m." + robot_names_[i], num(antenna_h_[i])});
    }
    if (phy_) {
      const auto & m = mac_;
      const std::vector<std::pair<std::string, std::string>> mac_kv = {
        {"mac_slot_us", num(1e6 * m.slot_s)}, {"mac_sifs_us", num(1e6 * m.sifs_s)},
        {"mac_aifsn", std::to_string(m.aifsn)}, {"mac_cw_min", std::to_string(m.cw_min)},
        {"mac_cw_max", std::to_string(m.cw_max)},
        {"mac_retry_limit", std::to_string(m.retry_limit)},
        {"mac_mpdu_payload_bytes", std::to_string(m.mpdu_payload_bytes)},
        {"mac_mpdu_overhead_bytes", std::to_string(m.mpdu_overhead_bytes)},
        {"mac_max_ampdu_bytes", std::to_string(m.max_ampdu_bytes)},
        {"mac_max_ampdu_mpdus", std::to_string(m.max_ampdu_mpdus)},
        {"mac_ppdu_max_us", num(1e6 * m.ppdu_max_s)},
        {"mac_detect_snr_db", num(m.detect_snr_db)},
        {"minstrel_keep_per_100ms", num(m.minstrel_keep_per_100ms)},
        {"minstrel_min_success", num(m.minstrel_min_success)},
      };
      kv.insert(kv.end(), mac_kv.begin(), mac_kv.end());
    }
    json << "{";
    for (size_t k = 0; k < kv.size(); ++k) {
      canon << kv[k].first << "=" << kv[k].second << "\n";
      json << (k ? "," : "") << "\"" << kv[k].first << "\":\"" << kv[k].second << "\"";
    }
    char hex[17];
    std::snprintf(hex, sizeof(hex), "%016llx",
      static_cast<unsigned long long>(hmr_radio::Fnv1a64(canon.str())));
    radio_regime_ = hex;
    json << ",\"radio_regime\":\"" << radio_regime_ << "\",\"link_breakdown_columns\":[";
    for (size_t k = 0; k < kBreakdownCols; ++k) {
      json << (k ? "," : "") << "\"" << kBreakdownNames[k] << "\"";
    }
    json << "]";
    if (phy_) {
      json << ",\"link_mac_columns\":[";
      for (size_t k = 0; k < kLinkMacCols; ++k) {
        json << (k ? "," : "") << "\"" << kLinkMacNames[k] << "\"";
      }
      json << "]";
      link_mac_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/link_mac", 10);
    }
    json << "}";
    radio_regime_pub_ = create_publisher<std_msgs::msg::String>(
      "~/radio_regime", rclcpp::QoS(1).transient_local());
    std_msgs::msg::String msg;
    msg.data = json.str();
    radio_regime_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
      "radio_regime=%s radio_stack=%s radio_profile=%s gamma_explicit_db_m=%.4f "
      "gamma_bg_db_m=%.4f noise_dbm=%.2f world_sha256=%s",
      radio_regime_.c_str(), radio_stack_.c_str(), radio_profile_.c_str(),
      model_->gamma_explicit(), model_->gamma_bg(), noise_dbm_,
      world.source_sha256.empty() ? "(none)" : world.source_sha256.c_str());
    RCLCPP_INFO(get_logger(), "radio effective: %s", msg.data.c_str());
  }

  // ---- link model tick ----------------------------------------------------
  void OnLinkTimer()
  {
    const size_t n = robot_names_.size();
    std_msgs::msg::Float64MultiArray out;
    const size_t n_pairs = n * (n - 1) / 2;
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "pair";
    out.layout.dim[0].size = n_pairs;
    out.layout.dim[0].stride = n_pairs * kLinkStateCols;
    out.layout.dim[1].label = "field";
    out.layout.dim[1].size = kLinkStateCols;
    out.layout.dim[1].stride = kLinkStateCols;
    out.data.reserve(n_pairs * kLinkStateCols);
    std_msgs::msg::Float64MultiArray bd_out;
    bd_out.layout.dim.resize(2);
    bd_out.layout.dim[0].label = "pair";
    bd_out.layout.dim[0].size = n_pairs;
    bd_out.layout.dim[0].stride = n_pairs * kBreakdownCols;
    bd_out.layout.dim[1].label = "field";
    bd_out.layout.dim[1].size = kBreakdownCols;
    bd_out.layout.dim[1].stride = kBreakdownCols;
    bd_out.data.reserve(n_pairs * kBreakdownCols);
    ++tick_;

    // Freshness is a property of a robot, evaluated once per tick, so that all
    // n-1 links touching a stalled robot agree and so the warning fires once
    // per robot rather than once per pair.
    const double now_s = get_clock()->now().seconds();
    std::vector<char> fresh(n, 0);
    for (size_t i = 0; i < n; ++i) {
      if (!has_pose_[i]) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "No pose EVER received for '%s' on %s — every link touching it is "
          "invalid and carries nothing. This is a bring-up fault, not a radio "
          "result: check the odom bridge before trusting this run.",
          robot_names_[i].c_str(), pose_topics_[i].c_str());
        continue;
      }
      const double age = now_s - pose_last_s_[i];
      if (pose_timeout_s_ > 0.0 && age > pose_timeout_s_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Pose for '%s' is %.1f s stale (> pose_timeout_s=%.1f) on %s — links "
          "touching it are invalid until it resumes.",
          robot_names_[i].c_str(), age, pose_timeout_s_, pose_topics_[i].c_str());
        continue;
      }
      fresh[i] = 1;
    }

    for (size_t i = 0; i < n; ++i) {
      for (size_t j = i + 1; j < n; ++j) {
        PairState & ps = pair_states_[PairKey(i, j)];
        if (!fresh[i] || !fresh[j]) {
          ps.valid = false;
          ps.connected = false;
        } else {
          ComputePair(i, j, ps);
        }
        // An invalid row publishes zeros, never the last good computation: a
        // frozen pose otherwise keeps path_loss_db > 0 forever, which is
        // exactly what the downstream "path_loss_db <= 0" startup mask reads as
        // a real link. Readers that predate the valid column still see zeros.
        const std::array<double, kLinkStateCols> row = ps.valid
          ? std::array<double, kLinkStateCols>{
              static_cast<double>(i), static_cast<double>(j),
              ps.distance_m, ps.trees_on_link, ps.path_loss_db, ps.snr_db,
              ps.ber, ps.bandwidth_mbps, ps.connected ? 1.0 : 0.0, 1.0}
          : std::array<double, kLinkStateCols>{
              static_cast<double>(i), static_cast<double>(j),
              0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
        out.data.insert(out.data.end(), row.begin(), row.end());
        const auto & o = ps.bd;
        const std::array<double, kBreakdownCols> bd_row = ps.valid
          ? std::array<double, kBreakdownCols>{
              static_cast<double>(i), static_cast<double>(j), o.distance_m,
              o.pl_ground_db, o.l_background_db, o.l_trunks_db, o.l_bushes_db,
              o.l_veg_db, o.l_ridges_db, o.l_range_db, o.fade_db, o.path_loss_db,
              o.snr_db, static_cast<double>(o.trunks_on_link), 1.0}
          : std::array<double, kBreakdownCols>{
              static_cast<double>(i), static_cast<double>(j)};
        bd_out.data.insert(bd_out.data.end(), bd_row.begin(), bd_row.end());
      }
    }
    link_states_pub_->publish(out);
    link_breakdown_pub_->publish(bd_out);
    if (phy_) {
      PublishLinkMac();
    }
  }

  void ComputePair(size_t i, size_t j, PairState & ps)
  {
    hmr_radio::Endpoint a, b;
    a.x = poses_[i].x;
    a.y = poses_[i].y;
    a.z = poses_[i].z;
    a.antenna_h = antenna_h_[i];
    b.x = poses_[j].x;
    b.y = poses_[j].y;
    b.z = poses_[j].z;
    b.antenna_h = antenna_h_[j];

    const auto & prof = model_->profile();
    if (legacy_fade_) {
      // v1's shared generator in v1's call order: with radio_stack v1 this
      // reproduces a v1 run's fading trace.
      std::normal_distribution<double> gauss(0.0, prof.fade_sigma_db);
      ps.fade_db = prof.fade_alpha * ps.fade_db +
        std::sqrt(1.0 - prof.fade_alpha * prof.fade_alpha) * gauss(fade_rng_);
    } else {
      if (!ps.fade) {
        const auto & lo = std::min(robot_names_[i], robot_names_[j]);
        const auto & hi = std::max(robot_names_[i], robot_names_[j]);
        ps.fade = std::make_shared<hmr_radio::CounterFade>(fade_seed_,
            hmr_radio::Fnv1a64("fade|" + lo + "|" + hi), prof.fade_sigma_db, prof.fade_alpha);
      }
      ps.fade_db = ps.fade->Step(tick_);
    }

    ps.bd = model_->Evaluate(a, b, ps.fade_db);
    const double rx_dbm = ps.bd.rx_dbm;
    const double snr_db = ps.bd.snr_db;
    const int trees_on_link = ps.bd.trunks_on_link;
    const double distance = ps.bd.distance_m;
    const double path_loss = ps.bd.path_loss_db;

    double ber;
    if (phy_) {
      // v2_phy: up and down on v1's lowest-tier hysteresis at the detection
      // threshold; the rate of the MCS the rate control holds, and the coded
      // bit error probability at that MCS.
      ps.connected = NextConnected(ps, snr_db);
      const int mcs = ps.rate.Update(mac_, snr_db, 1.0 / std::max(link_rate_hz_, 0.1));
      ps.bandwidth_mbps = ps.connected ? hmr_radio::McsRateBps(mcs) / 1e6 : 0.0;
      ber = ps.connected ?
        hmr_radio::CodedBitErrorProb(mcs, std::pow(10.0, snr_db / 10.0)) : 1.0;
    } else {
      // Tier first, then BER at that tier. NextBandwidth reads only snr_history
      // and ps.bandwidth_mbps and writes only snr_history, so the order leaves
      // every delivery decision unchanged. (notes: comms-sim-tier-before-ber)
      ps.bandwidth_mbps = NextBandwidth(ps, snr_db);
      ps.connected = ps.bandwidth_mbps > 0.0;

      if (ps.bandwidth_mbps <= 0.0) {
        // No gear engaged: BER is undefined and nothing gets through, so publish
        // 1.0, as an invalid row does. Consumers should mask on connected; this
        // keeps an unmasked read wrong in the safe direction.
        // (notes: comms-sim-ber-when-no-gear)
        ber = 1.0;
      } else {
        const double spectral_efficiency = ps.bandwidth_mbps * 1e6 / kChannelBandwidthHz;
        ber = trees_on_link == 0 ?
          AwgnQam64Ber(DbmToPow(rx_dbm), DbmToPow(noise_dbm_), spectral_efficiency) :
          RayleighQam64Ber(DbmToPow(rx_dbm), DbmToPow(noise_dbm_), spectral_efficiency);
      }
    }
    ber = std::clamp(ber, 0.0, 1.0);

    ps.distance_m = distance;
    ps.trees_on_link = trees_on_link;
    ps.path_loss_db = path_loss;
    ps.snr_db = snr_db;
    ps.ber = ber;
    ps.valid = true;
  }

  // Tiered rate adaptation with hysteresis; history now genuinely spans
  // 8 / link_rate_hz seconds because it is sampled here, not per sim step.
  double NextBandwidth(PairState & ps, double snr_db)
  {
    ps.snr_history.push_back(snr_db);
    if (ps.snr_history.size() > 8) {
      ps.snr_history.pop_front();
    }
    if (ps.snr_history.size() < 8) {
      if (snr_db > 25.0) {return 72.0;}
      if (snr_db > 11.0) {return 28.9;}
      if (snr_db > 2.0) {return 7.2;}
      return 0.0;
    }
    const double cur = ps.bandwidth_mbps;
    const double threshold = cur >= 72.0 ? 25.0 : cur >= 28.9 ? 11.0 : 2.0;
    int high = 0, low = 0;
    for (const double s : ps.snr_history) {
      if (s > threshold) {++high;}
      if (s < threshold) {++low;}
    }
    if (cur >= 72.0) {
      return low >= 3 ? 28.9 : 72.0;
    }
    if (cur >= 28.9) {
      if (high == 8) {return 72.0;}
      return low >= 3 ? 7.2 : 28.9;
    }
    if (cur >= 7.2) {
      if (high == 8) {return 28.9;}
      return low >= 3 ? 0.0 : 7.2;
    }
    return high == 8 ? 7.2 : 0.0;
  }

  // ---- relay topic discovery (same approach as the old relay node) --------
  void DiscoverAndSetup()
  {
    if (pending_topics_.empty()) {
      discovery_timer_->cancel();
      RCLCPP_INFO(get_logger(), "All relay topics discovered and connected.");
      return;
    }
    const auto names_and_types = get_topic_names_and_types();
    std::vector<std::string> discovered;

    for (const auto & [src_topic, info] : pending_topics_) {
      const auto it = names_and_types.find(src_topic);
      if (it == names_and_types.end() || it->second.empty()) {
        continue;
      }
      const std::string & msg_type = it->second[0];

      // Mirror the live publisher's reliability/durability so the relay matches
      // whatever QoS the source stack chose. Depth must survive a reconnect
      // backlog burst: a reliable KeepLast(10) publisher would silently replace
      // unacked samples when the queue drains ~all at once.
      rclcpp::QoS qos{rclcpp::KeepLast(
          info.policy == Policy::kLatest ? latest_qos_depth_ : rx_qos_depth_)};
      const auto pubs_info = get_publishers_info_by_topic(src_topic);
      if (!pubs_info.empty()) {
        const auto rmw_qos = pubs_info.front().qos_profile().get_rmw_qos_profile();
        if (rmw_qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
          qos.best_effort();
        }
        if (rmw_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
          qos.transient_local();
        }
      }

      const size_t sender = info.sender;
      const std::string comms_topic = info.comms_topic;
      const Policy policy = info.policy;

      for (size_t r = 0; r < robot_names_.size(); ++r) {
        if (r == sender) {
          continue;
        }
        const std::string rx_topic =
          "/" + robot_names_[r] + "/rx/" + robot_names_[sender] + "/" + comms_topic;
        rx_pubs_[DirKey(sender, r)][comms_topic] =
          create_generic_publisher(rx_topic, msg_type, qos);
        RCLCPP_INFO(get_logger(), "  relay [%s]: %s -> %s [%s]",
          policy == Policy::kReliable ? "reliable" :
          policy == Policy::kLatest ? "latest" : "best-effort",
          src_topic.c_str(), rx_topic.c_str(), msg_type.c_str());
      }

      tx_subs_.push_back(create_generic_subscription(
        src_topic, msg_type, qos,
        [this, sender, comms_topic, policy](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          OnTxMessage(sender, comms_topic, policy, msg);
        }));
      discovered.push_back(src_topic);
    }
    for (const auto & t : discovered) {
      pending_topics_.erase(t);
    }
    if (pending_topics_.empty()) {
      return;
    }
    // Timed from the last discovery, not the start: the planners come up a
    // minute or more after the emulator. With none yet, from the first poll,
    // which runs on the node clock and so, under use_sim_time, only once
    // /clock does.
    const rclcpp::Time now = get_clock()->now();
    if (!discovered.empty() || !pending_anchor_set_) {
      pending_anchor_ = now;
      pending_anchor_set_ = true;
    }
    std::string names;
    for (const auto & [src_topic, info] : pending_topics_) {
      names += (names.empty() ? "" : ", ") + src_topic;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Waiting for %zu relay topics to appear: %s", pending_topics_.size(), names.c_str());
    // Once only; discovery keeps polling, so a late publisher is still relayed.
    if (!pending_warned_ && pending_warn_sec_ > 0.0 &&
      (now - pending_anchor_).seconds() >= pending_warn_sec_)
    {
      pending_warned_ = true;
      RCLCPP_WARN(get_logger(),
        "%zu relay topic(s) still unpublished %.0f s after the last discovery: %s. "
        "Nothing relays them; if the running node never publishes them, drop "
        "them from reliable_topics/best_effort_topics/latest_topics.",
        pending_topics_.size(), (now - pending_anchor_).seconds(), names.c_str());
    }
  }

  // ---- the "cable" --------------------------------------------------------
  void OnTxMessage(
    size_t sender, const std::string & comms_topic, Policy policy,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg)
  {
    const size_t bytes = msg->size();
    for (size_t r = 0; r < robot_names_.size(); ++r) {
      if (r == sender) {
        continue;
      }
      const size_t dir = DirKey(sender, r);
      auto pub_topics = rx_pubs_.find(dir);
      if (pub_topics == rx_pubs_.end()) {
        continue;
      }
      auto pub_it = pub_topics->second.find(comms_topic);
      if (pub_it == pub_topics->second.end()) {
        continue;
      }
      PairState & ps = pair_states_[PairKey(sender, r)];
      LinkStats & stats = stats_[dir];

      if (policy == Policy::kReliable || policy == Policy::kLatest) {
        auto & queue = reliable_queues_[dir];
        if (policy == Policy::kLatest) {
          // Every queued entry is still waiting for the air (DrainReliable
          // pops what it schedules), so all of this publisher's are stale.
          for (auto q = queue.begin(); q != queue.end(); ) {
            if (q->pub == pub_it->second) {
              reliable_queue_bytes_[dir] -= q->bytes;
              ++stats.drop_superseded;
              q = queue.erase(q);
            } else {
              ++q;
            }
          }
        }
        queue.push_back({msg, pub_it->second, bytes, policy == Policy::kLatest});
        reliable_queue_bytes_[dir] += bytes;
        while (reliable_queue_bytes_[dir] > reliable_queue_max_bytes_ && queue.size() > 1) {
          reliable_queue_bytes_[dir] -= queue.front().bytes;
          queue.pop_front();
          ++stats.drop_overflow;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
            "Reliable backlog %s->%s exceeded %zu bytes; dropping oldest. Deltas lost "
            "on this link — the receiver's merged map will be missing voxels.",
            robot_names_[sender].c_str(), robot_names_[r].c_str(),
            reliable_queue_max_bytes_);
        }
        if (!progressive_) {
          DrainReliable(dir, ps, stats);
        }
        continue;
      }

      // best-effort
      if (!ps.valid || !ps.connected) {
        ++stats.drop_disconnected;
        continue;
      }
      if (phy_) {
        // Queued on its link and sent ahead of the reliable queue; lost only
        // when an MPDU runs out of retries (SendBits).
        auto & be = be_queues_[dir];
        be.push_back({msg, pub_it->second, bytes, false});
        be_queue_bytes_[dir] += bytes;
        while (be_queue_bytes_[dir] > reliable_queue_max_bytes_ && be.size() > 1) {
          be_queue_bytes_[dir] -= be.front().bytes;
          be.pop_front();
          ++stats.drop_overflow;
        }
        continue;
      }
      const size_t bits = bytes * 8;
      // Delivery is the bandwidth state machine's call (ps.connected, checked
      // above). Link quality reaches the experiment through the tier, as
      // airtime pressure in cost_s below, not as vanished messages.
      // (notes: comms-sim-best-effort-delivery)
      if (uniform_(drop_rng_) < residual_pdr_) {
        ++stats.drop_ber;
        continue;
      }
      RefillAirtime();
      if (!best_effort_priority_ && airtime_tokens_ <= 0.0) {
        ++stats.drop_airtime;
        continue;
      }
      const double cost_s = bits / (ps.bandwidth_mbps * 1e6);
      airtime_tokens_ -= cost_s;
      ScheduleDelivery(pub_it->second, msg, cost_s);
      ++stats.relayed;
      stats.bytes_relayed += bytes;
    }
  }

  void DrainReliable(size_t dir, PairState & ps, LinkStats & stats)
  {
    auto queue_it = reliable_queues_.find(dir);
    if (queue_it == reliable_queues_.end()) {
      return;
    }
    auto & queue = queue_it->second;
    while (!queue.empty() && ps.valid && ps.connected) {
      RefillAirtime();
      if (airtime_tokens_ <= 0.0) {
        break;
      }
      QueuedMsg & front = queue.front();
      const size_t bits = front.bytes * 8;
      // The tier is the whole quality model: a byte costs bits/tier of airtime
      // and nothing more, with no BER-derived retransmission multiplier.
      // (notes: comms-sim-reliable-airtime-cost)
      const double cost_s = bits / (ps.bandwidth_mbps * 1e6);
      airtime_tokens_ -= cost_s;
      ScheduleDelivery(front.pub, front.msg, cost_s);
      ++stats.relayed;
      stats.latest_relayed += front.latest;
      stats.bytes_relayed += front.bytes;
      reliable_queue_bytes_[dir] -= front.bytes;
      queue.pop_front();
    }
  }

  // Progressive model, once per delivery tick: every link with something to
  // send gets up to dt of air, the airtime balance split evenly among them.
  void AdvanceTransmissions()
  {
    const rclcpp::Time now = get_clock()->now();
    if (!tx_last_init_ || now < tx_last_) {
      tx_last_ = now;
      tx_last_init_ = true;
      return;
    }
    const double dt = (now - tx_last_).seconds();
    tx_last_ = now;
    if (dt <= 0.0) {
      return;
    }
    if (phy_) {
      AdvancePhy(dt);
      return;
    }
    const size_t n = robot_names_.size();
    std::vector<size_t> sending;
    for (auto & [dir, queue] : reliable_queues_) {
      InFlight & f = inflight_[dir];
      const PairState & ps = pair_states_[PairKey(dir / n, dir % n)];
      if (!ps.valid || !ps.connected) {
        if (f.active && f.m.latest) {
          // A stale partial frame is worth less than the newest whole one.
          f.active = false;
          ++stats_[dir].drop_aborted;
        }
        continue;
      }
      if (f.active || !queue.empty()) {
        sending.push_back(dir);
      }
    }
    if (sending.empty()) {
      return;
    }
    RefillAirtime();
    const size_t start = drain_offset_++ % sending.size();
    for (size_t k = 0; k < sending.size() && airtime_tokens_ > 0.0; ++k) {
      const size_t dir = sending[(start + k) % sending.size()];
      const double air = std::min(dt, airtime_tokens_ / double(sending.size() - k));
      airtime_tokens_ -= Transmit(dir, air);
    }
  }

  // Sends up to air seconds on one link at its current tier; returns the air
  // used. Starts queued messages as earlier ones finish.
  double Transmit(size_t dir, double air)
  {
    const size_t n = robot_names_.size();
    const double bps = pair_states_[PairKey(dir / n, dir % n)].bandwidth_mbps * 1e6;
    auto & queue = reliable_queues_[dir];
    InFlight & f = inflight_[dir];
    LinkStats & stats = stats_[dir];
    double used = 0.0;
    while (used < air) {
      if (!f.active) {
        if (queue.empty()) {
          break;
        }
        f.m = queue.front();
        queue.pop_front();
        reliable_queue_bytes_[dir] -= f.m.bytes;
        f.bits_left = f.m.bytes * 8.0;
        f.active = true;
      }
      const double need = f.bits_left / bps;
      if (need > air - used) {
        f.bits_left -= (air - used) * bps;
        used = air;
        break;
      }
      used += need;
      f.active = false;
      ScheduleDelivery(f.m.pub, f.m.msg, 0.0);
      ++stats.relayed;
      stats.latest_relayed += f.m.latest;
      stats.bytes_relayed += f.m.bytes;
      f.m = QueuedMsg{};
    }
    return used;
  }

  // ---- v2_phy: the air shared by radio_mac (DESIGN_radio_v2.md §13) --------
  // Every directed link with a backlog on a connected pair contends.
  // ShareChannel gives each its goodput for the current set, and the tick
  // advances in steps that end when a link runs dry, so the air it leaves goes
  // to the others.
  void AdvancePhy(double dt)
  {
    const size_t n = robot_names_.size();
    hmr_radio::ShareInput in;
    in.n = static_cast<int>(n);
    in.noise_dbm = noise_dbm_;
    in.hear.assign(n * n, 0);
    in.rx_dbm.assign(n * n, -300.0);
    in.snr_db.assign(n * n, -300.0);
    in.mcs.assign(n * n, 0);
    std::vector<size_t> dirs;
    for (size_t s = 0; s < n; ++s) {
      for (size_t r = 0; r < n; ++r) {
        if (s == r) {
          continue;
        }
        const size_t dir = DirKey(s, r);
        const PairState & ps = pair_states_[PairKey(s, r)];
        if (ps.valid) {
          in.rx_dbm[dir] = ps.bd.rx_dbm;
          in.snr_db[dir] = ps.snr_db;
          in.hear[dir] = ps.snr_db >= mac_.detect_snr_db;
          in.mcs[dir] = ps.rate.mcs();
        }
        if (!ps.valid || !ps.connected) {
          const auto f = inflight_.find(dir);
          if (f != inflight_.end() && f->second.active && f->second.m.latest) {
            // A stale partial frame is worth less than the newest whole one.
            f->second.active = false;
            ++stats_[dir].drop_aborted;
          }
          // Best-effort frames on a dead link run out of retries.
          const auto bf = be_inflight_.find(dir);
          if (bf != be_inflight_.end() && bf->second.active) {
            bf->second.active = false;
            ++stats_[dir].drop_disconnected;
          }
          const auto bq = be_queues_.find(dir);
          if (bq != be_queues_.end() && !bq->second.empty()) {
            stats_[dir].drop_disconnected += bq->second.size();
            bq->second.clear();
            be_queue_bytes_[dir] = 0;
          }
          continue;
        }
        if (BacklogBits(dir) > 0.0) {
          dirs.push_back(dir);
        }
      }
    }
    double left = dt;
    for (size_t step = 0; left > 0.0 && !dirs.empty() && step <= n * n; ++step) {
      in.active.assign(n * n, 0);
      in.backlog_bytes.assign(n * n, 0.0);
      for (const size_t dir : dirs) {
        in.active[dir] = 1;
        in.backlog_bytes[dir] = BacklogBits(dir) / 8.0;
      }
      const hmr_radio::ShareOutput out = hmr_radio::ShareChannel(mac_, in);
      double t = left;
      for (const size_t dir : dirs) {
        if (out.goodput_bps[dir] > 0.0) {
          t = std::min(t, BacklogBits(dir) / out.goodput_bps[dir]);
        }
      }
      std::vector<size_t> still;
      for (const size_t dir : dirs) {
        const double per = hmr_radio::MpduPer(in.mcs[dir], in.snr_db[dir],
            hmr_radio::MpduBytes(mac_));
        const double coll = out.collision_p[dir / n];
        MacStats & ms = mac_stats_[dir];
        ms.busy_s += t;
        ms.txops += t * out.txop_rate[dir];
        ms.hidden_s += t * out.hidden_loss[dir];
        ms.p_busy_s += t;
        ms.p_hidden_s += t * out.hidden_loss[dir];
        ms.p_coll_s += t * coll;
        ms.p_bits += SendBits(dir, out.goodput_bps[dir] * t,
            1.0 - (1.0 - coll) * (1.0 - out.hidden_loss[dir]) * (1.0 - per));
        if (BacklogBits(dir) > 0.0) {
          still.push_back(dir);
        }
      }
      dirs.swap(still);
      left -= t;
    }
  }

  // v2_phy: sends up to bits on one link, best-effort first (fq_codel serves
  // sparse flows ahead of bulk), then the reliable/latest queue in order. A
  // best-effort message is lost when one of its MPDUs fails all retry_limit + 1
  // attempts, each failing with probability q. Returns the bits used.
  double SendBits(size_t dir, double bits, double q)
  {
    LinkStats & stats = stats_[dir];
    double used = 0.0;
    for (const bool be : {true, false}) {
      auto & queue = be ? be_queues_[dir] : reliable_queues_[dir];
      size_t & queue_bytes = be ? be_queue_bytes_[dir] : reliable_queue_bytes_[dir];
      InFlight & f = be ? be_inflight_[dir] : inflight_[dir];
      while (used < bits) {
        if (!f.active) {
          if (queue.empty()) {
            break;
          }
          f.m = queue.front();
          queue.pop_front();
          queue_bytes -= f.m.bytes;
          f.bits_left = f.m.bytes * 8.0;
          f.active = true;
        }
        const double take = std::min(f.bits_left, bits - used);
        f.bits_left -= take;
        used += take;
        if (f.bits_left > 1e-6) {
          break;
        }
        f.active = false;
        const double mpdus = std::ceil(f.m.bytes / double(mac_.mpdu_payload_bytes));
        if (be && uniform_(drop_rng_) <
          1.0 - std::pow(1.0 - std::pow(q, mac_.retry_limit + 1), mpdus))
        {
          ++stats.drop_ber;
        } else {
          ScheduleDelivery(f.m.pub, f.m.msg, 0.0);
          ++stats.relayed;
          stats.latest_relayed += f.m.latest;
          stats.bytes_relayed += f.m.bytes;
        }
        f.m = QueuedMsg{};
      }
    }
    return used;
  }

  // v2_phy: bits waiting on one link, in flight and queued, both policies.
  double BacklogBits(size_t dir) const
  {
    double bits = 0.0;
    for (const auto * fl : {&inflight_, &be_inflight_}) {
      const auto f = fl->find(dir);
      if (f != fl->end() && f->second.active) {
        bits += f->second.bits_left;
      }
    }
    for (const auto * qb : {&reliable_queue_bytes_, &be_queue_bytes_}) {
      const auto b = qb->find(dir);
      if (b != qb->end()) {
        bits += 8.0 * b->second;
      }
    }
    return bits;
  }

  // v2_phy: v1's lowest-tier hysteresis at the MAC's detection threshold: up
  // after 8 of 8 samples above it, down once 3 of 8 fall below.
  bool NextConnected(PairState & ps, double snr_db)
  {
    ps.snr_history.push_back(snr_db);
    if (ps.snr_history.size() > 8) {
      ps.snr_history.pop_front();
    }
    const double th = mac_.detect_snr_db;
    if (ps.snr_history.size() < 8) {
      return snr_db > th;
    }
    int high = 0, low = 0;
    for (const double s : ps.snr_history) {
      high += s > th;
      low += s < th;
    }
    return ps.connected ? low < 3 : high == 8;
  }

  // v2_phy: one row per directed link for the link period just ended. The MAC
  // columns are means over the time the link had a backlog (zero if none).
  void PublishLinkMac()
  {
    const size_t n = robot_names_.size();
    const double period = 1.0 / std::max(link_rate_hz_, 0.1);
    std_msgs::msg::Float64MultiArray out;
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "link";
    out.layout.dim[0].size = n * (n - 1);
    out.layout.dim[0].stride = n * (n - 1) * kLinkMacCols;
    out.layout.dim[1].label = "field";
    out.layout.dim[1].size = kLinkMacCols;
    out.layout.dim[1].stride = kLinkMacCols;
    out.data.reserve(n * (n - 1) * kLinkMacCols);
    for (size_t s = 0; s < n; ++s) {
      for (size_t r = 0; r < n; ++r) {
        if (s == r) {
          continue;
        }
        const size_t dir = DirKey(s, r);
        const PairState & ps = pair_states_[PairKey(s, r)];
        MacStats & ms = mac_stats_[dir];
        const int mcs = ps.rate.mcs();
        if (ps.valid && ps.connected) {
          ms.mcs_s[mcs] += period;
        }
        const double busy = ms.p_busy_s;
        const std::array<double, kLinkMacCols> row = {
          static_cast<double>(s), static_cast<double>(r), static_cast<double>(mcs),
          hmr_radio::McsRateBps(mcs) / 1e6,
          ps.valid ? hmr_radio::MpduPer(mcs, ps.snr_db, hmr_radio::MpduBytes(mac_)) : 1.0,
          busy / period,
          busy > 0.0 ? ms.p_bits / busy / 1e6 : 0.0,
          busy > 0.0 ? ms.p_hidden_s / busy : 0.0,
          busy > 0.0 ? ms.p_coll_s / busy : 0.0,
          std::ceil(BacklogBits(dir) / 8.0),
          ps.connected ? 1.0 : 0.0, ps.valid ? 1.0 : 0.0};
        out.data.insert(out.data.end(), row.begin(), row.end());
        ms.p_busy_s = ms.p_bits = ms.p_hidden_s = ms.p_coll_s = 0.0;
      }
    }
    link_mac_pub_->publish(out);
  }

  void RefillAirtime()
  {
    const rclcpp::Time now = get_clock()->now();
    if (!airtime_init_ || now < airtime_last_refill_) {
      airtime_last_refill_ = now;
      airtime_tokens_ = airtime_burst_s_;
      airtime_init_ = true;
      return;
    }
    airtime_tokens_ = std::min(
      airtime_tokens_ + (now - airtime_last_refill_).seconds() * airtime_capacity_,
      airtime_burst_s_);
    airtime_last_refill_ = now;
  }

  void ScheduleDelivery(
    const rclcpp::GenericPublisher::SharedPtr & pub,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg,
    double transmit_s)
  {
    const rclcpp::Time now = get_clock()->now();
    const int64_t t_ns = now.nanoseconds() +
      static_cast<int64_t>((transmit_s + delay_ms_ * 1e-3) * 1e9);
    deliveries_.push({t_ns, next_seq_++, pub, msg});
  }

  void OnDeliveryTimer()
  {
    // Drain backlogs first (rotating start so no link starves the others)...
    if (progressive_) {
      AdvanceTransmissions();
    } else if (!reliable_queues_.empty()) {
      std::vector<size_t> keys;
      keys.reserve(reliable_queues_.size());
      for (const auto & [dir, q] : reliable_queues_) {
        if (!q.empty()) {
          keys.push_back(dir);
        }
      }
      if (!keys.empty()) {
        const size_t start = drain_offset_++ % keys.size();
        const size_t n = robot_names_.size();
        for (size_t k = 0; k < keys.size(); ++k) {
          const size_t dir = keys[(start + k) % keys.size()];
          DrainReliable(dir, pair_states_[PairKey(dir / n, dir % n)], stats_[dir]);
        }
      }
    }
    // ...then release messages whose transmit+delay time has passed.
    const int64_t now_ns = get_clock()->now().nanoseconds();
    while (!deliveries_.empty() && deliveries_.top().t_ns <= now_ns) {
      const Delivery & d = deliveries_.top();
      d.pub->publish(*d.msg);
      deliveries_.pop();
    }
  }

  // ---- diagnostics --------------------------------------------------------
  void PublishRobotIndex()
  {
    std::ostringstream ss;
    ss << "{\"robots\":[";
    for (size_t i = 0; i < robot_names_.size(); ++i) {
      ss << (i ? "," : "") << "\"" << robot_names_[i] << "\"";
    }
    ss << "],\"link_state_columns\":[\"i\",\"j\",\"distance_m\",\"trees_on_link\","
          "\"path_loss_db\",\"snr_db\",\"ber\",\"bandwidth_mbps\",\"connected\","
          "\"valid\"]}";
    std_msgs::msg::String msg;
    msg.data = ss.str();
    robot_index_pub_->publish(msg);
  }

  void PublishStats()
  {
    const size_t n = robot_names_.size();
    std::ostringstream ss;
    ss << "{\"t\":" << std::fixed << get_clock()->now().seconds() << ",\"links\":[";
    bool first = true;
    uint64_t total_relayed = 0, total_dropped = 0;
    for (const auto & [dir, s] : stats_) {
      const uint64_t dropped =
        s.drop_ber + s.drop_airtime + s.drop_disconnected + s.drop_overflow;
      total_relayed += s.relayed;
      total_dropped += dropped;
      size_t queue_bytes = 0;
      const auto qb = reliable_queue_bytes_.find(dir);
      if (qb != reliable_queue_bytes_.end()) {
        queue_bytes = qb->second;
      }
      const auto fl = inflight_.find(dir);
      if (fl != inflight_.end() && fl->second.active) {
        queue_bytes += static_cast<size_t>(std::ceil(fl->second.bits_left / 8.0));
      }
      if (phy_) {
        queue_bytes = static_cast<size_t>(std::ceil(BacklogBits(dir) / 8.0));
      }
      ss << (first ? "" : ",")
         << "{\"from\":\"" << robot_names_[dir / n] << "\",\"to\":\"" << robot_names_[dir % n]
         << "\",\"relayed\":" << s.relayed
         << ",\"bytes\":" << s.bytes_relayed
         << ",\"drop_ber\":" << s.drop_ber
         << ",\"drop_airtime\":" << s.drop_airtime
         << ",\"drop_disconnected\":" << s.drop_disconnected
         << ",\"drop_overflow\":" << s.drop_overflow
         << ",\"drop_superseded\":" << s.drop_superseded
         << ",\"drop_aborted\":" << s.drop_aborted
         << ",\"latest_relayed\":" << s.latest_relayed
         << ",\"backlog_bytes\":" << queue_bytes;
      if (phy_) {
        // v2_phy only, so v1 and v2_prop lines stay byte-identical.
        const MacStats & ms = mac_stats_[dir];
        ss << ",\"busy_s\":" << ms.busy_s << ",\"txops\":" << ms.txops
           << ",\"hidden_s\":" << ms.hidden_s << ",\"mcs_s\":[";
        for (int m = 0; m < hmr_radio::kNumMcs; ++m) {
          ss << (m ? "," : "") << ms.mcs_s[m];
        }
        ss << "]";
      }
      ss << "}";
      first = false;
    }
    ss << "]}";
    std_msgs::msg::String msg;
    msg.data = ss.str();
    stats_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "relay totals: %lu delivered, %lu dropped, airtime=%.3fs",
      static_cast<unsigned long>(total_relayed), static_cast<unsigned long>(total_dropped),
      airtime_tokens_);
    // The same per-link, per-reason cumulative counters as ~/stats, into comms.log.
    // Campaigns run --record 0, so the topic is never bagged and the totals line
    // above was the only trace: no drop could be pinned to a link or a gate.
    // Separate line so the totals line (manoeuvre_events.py RE_RELAY) is unchanged.
    RCLCPP_INFO(get_logger(), "link counters: %s", msg.data.c_str());
  }

  static constexpr size_t kLinkStateCols = 10;
  static constexpr size_t kLinkMacCols = 12;
  static constexpr const char * kLinkMacNames[kLinkMacCols] = {
    "i", "j", "mcs", "phy_rate_mbps", "mpdu_per", "busy_frac", "goodput_mbps",
    "hidden_loss", "collision_p", "backlog_bytes", "connected", "valid"};
  static constexpr size_t kBreakdownCols = 15;
  static constexpr const char * kBreakdownNames[kBreakdownCols] = {
    "i", "j", "distance_m", "pl_ground_db", "l_background_db", "l_trunks_db",
    "l_bushes_db", "l_veg_db", "l_ridges_db", "l_range_db", "fade_db",
    "path_loss_db", "snr_db", "trunks_on_link", "valid"};

  // parameters
  std::vector<std::string> robot_names_;
  std::string pose_topic_pattern_;
  std::string radio_stack_, radio_profile_, radio_world_file_, fade_mode_, ground_model_;
  hmr_radio::Profile profile_;
  std::vector<double> antenna_heights_m_;
  int64_t gamma_explicit_seed_ = 20260927, gamma_explicit_links_ = 10000;
  double link_rate_hz_, delay_ms_, airtime_capacity_, airtime_burst_s_;
  double residual_pdr_;
  bool best_effort_priority_ = false;
  double pose_timeout_s_;
  size_t reliable_queue_max_bytes_;
  size_t rx_qos_depth_;
  size_t latest_qos_depth_ = 2;
  bool progressive_ = false;

  // world + poses
  std::map<std::string, size_t> name_to_idx_;
  std::vector<Vec3> poses_;
  std::vector<bool> has_pose_;        // a pose was received at some point
  std::vector<double> pose_last_s_;   // node-clock seconds of the last one
  std::vector<std::string> pose_topics_;
  std::unique_ptr<hmr_radio::LinkModel> model_;
  std::vector<double> antenna_h_;
  double noise_dbm_ = -101.0;
  bool legacy_fade_ = true;
  uint64_t fade_seed_ = 42;
  uint64_t tick_ = 0;
  std::string radio_regime_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> pose_subs_;

  // link model state, keyed by PairKey
  std::map<size_t, PairState> pair_states_;
  std::mt19937_64 fade_rng_, drop_rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};

  // relay plumbing
  std::map<std::string, PendingTopic> pending_topics_;
  double pending_warn_sec_ = 120.0;
  rclcpp::Time pending_anchor_;
  bool pending_anchor_set_ = false;
  bool pending_warned_ = false;
  std::vector<rclcpp::GenericSubscription::SharedPtr> tx_subs_;
  // rx_pubs_[DirKey(sender, receiver)][comms_topic]
  std::map<size_t, std::map<std::string, rclcpp::GenericPublisher::SharedPtr>> rx_pubs_;
  std::map<size_t, std::deque<QueuedMsg>> reliable_queues_;
  std::map<size_t, size_t> reliable_queue_bytes_;
  std::map<size_t, LinkStats> stats_;

  // shared channel
  double airtime_tokens_ = 0.0;
  rclcpp::Time airtime_last_refill_;
  bool airtime_init_ = false;

  // deferred delivery
  std::priority_queue<Delivery, std::vector<Delivery>, std::greater<Delivery>> deliveries_;
  uint64_t next_seq_ = 0;
  size_t drain_offset_ = 0;

  // progressive model
  std::map<size_t, InFlight> inflight_;
  rclcpp::Time tx_last_;
  bool tx_last_init_ = false;

  // v2_phy: MAC parameters, best-effort queues (v1 sends those at once) and
  // MAC diagnostics, keyed by DirKey
  hmr_radio::MacParams mac_;
  bool phy_ = false;
  std::map<size_t, std::deque<QueuedMsg>> be_queues_;
  std::map<size_t, size_t> be_queue_bytes_;
  std::map<size_t, InFlight> be_inflight_;
  std::map<size_t, MacStats> mac_stats_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr link_states_pub_,
    link_breakdown_pub_, link_mac_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_, robot_index_pub_,
    radio_regime_pub_;
  rclcpp::TimerBase::SharedPtr link_timer_, delivery_timer_, discovery_timer_, stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<HmrRadioNode> node;
  try {
    node = std::make_shared<HmrRadioNode>();
  } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
    // Raised from the Node base constructor while ingesting overrides, so no
    // catch in the node body sees it. Usually an empty list, which has no
    // inferable element type and arrives NOT_SET.
    // (notes: comms-sim-empty-list-abort)
    RCLCPP_FATAL(rclcpp::get_logger("hmr_comms_sim"),
      "Bad parameter override: %s. If you wrote an empty list `[]`, that has no "
      "inferable element type — write [\"\"] for an empty list of strings.",
      e.what());
    rclcpp::shutdown();
    return 1;
  }
  // Single-threaded spin is load-bearing: all state is unlocked because every
  // callback (poses, relayed messages, timers) runs on this one executor thread.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
