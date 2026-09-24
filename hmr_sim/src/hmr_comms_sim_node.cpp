// HMR Comms Sim Node — message-level wireless-link emulator for multi-robot runs.
//
// Link physics and message relay live in one ROS2 node, so it runs against the
// live sim, a bag replay or any pose source without Gazebo. Tree positions are
// parsed from the world SDF once at startup. (notes: comms-sim-origin)
//
// Link model, per robot pair at link_rate_hz on the node clock (sim time):
//   distance + trees crossing the Fresnel zone
//     -> path loss  P0 + 20 log10(d) + N_trees * Lv + shadow fade
//        (forest model from https://ieeexplore.ieee.org/document/9260568),
//        plus a 200 dB cliff past max_range_m — a hard radio horizon, since
//        free-space loss alone never drops a 30 dBm link inside the ROI
//     -> SNR -> BER (64-QAM: AWGN when line-of-sight, Rayleigh through trees)
//     -> bandwidth tier {72, 28.9, 7.2, 0} Mbps via 8-sample SNR hysteresis.
//   Shadow fade is an AR(1) process (stationary std = fade_sigma_db), not white
//   noise per call: fading must evolve as robots move, and a seeded RNG makes a
//   run repeatable. connected == (bandwidth > 0).
//
// Relay, per (sender, receiver, topic):   /<robot>/<topic>  ->  /<receiver>/rx/<sender>/<topic>
//   All links share ONE airtime budget (airtime_capacity seconds of channel per
//   second) — a message of B bytes on a link running at R Mbps consumes 8B/R
//   seconds of it, so pairs contend for the channel like real radios do.
//   Admission requires a positive token balance; the balance may then go
//   negative (debt), which keeps arbitrarily large messages sendable while
//   still enforcing the long-run rate.
//   Three per-topic policies:
//     reliable    — never dropped. FIFO queue per directional link, drained only
//                   while the link is up and airtime is available; a down link
//                   grows a backlog that is delivered late, never lost (bounded
//                   by reliable_queue_max_bytes, oldest dropped + warned).
//     latest      — reliable, for streams whose every message is a whole
//                   state (full-map frames). A new message removes any older
//                   one from the same sender still queued for that receiver
//                   (counted as drop_superseded, never as a loss); a message
//                   already on the air is never cancelled.
//                   Airtime cost is bits / the CURRENT TIER: a bad link is slow.
//     best_effort — dropped when the link is down or when the shared channel has
//                   no airtime; otherwise delivered, less residual_pdr.
//                   With best_effort_priority=true the airtime check is skipped
//                   (the message is still charged, so it slows the reliable
//                   queues): small control traffic such as the gen-34 team
//                   beacon then reaches a connected peer even while a map
//                   backlog holds the channel in debt. Default false keeps the
//                   earlier behaviour.
//   Two transmission models for reliable and latest (transmission_model):
//     admission   — (default; every run before gen-34 §12) a queued message
//                   is admitted whole at the tier in force when airtime turns
//                   positive, charged bits/tier, and delivered at now + cost
//                   whatever the link does next. Messages admitted in one
//                   drain are delivered by size, not in order.
//     progressive — one message on the air per directional link. Each
//                   delivery tick sends dt of air at the CURRENT tier, the
//                   shared airtime split evenly across the links sending;
//                   a message is delivered when its last bit is sent, so
//                   delivery is FIFO per link. A link that drops mid-message
//                   resumes a reliable one on reconnect and aborts a latest
//                   one (drop_aborted; the newest frame goes next contact).
//   Both policies take delivery from the bandwidth state machine alone, as in
//   HMRNetSim.cc: gear 0 means nothing gets through, any other gear means
//   essentially everything gets through at that gear's speed. ber/per are
//   published diagnostics and MUST NOT gate delivery — see NextBandwidth.
//   Delivery is deferred by transmission time + delay_ms via a timer heap.
//
// Diagnostics:
//   ~/link_states  std_msgs/Float64MultiArray, one row per unordered pair:
//                  [i, j, distance_m, trees_on_link, path_loss_db, snr_db,
//                   ber, bandwidth_mbps, connected, valid]
//                  valid == 0 means THE MODEL HAS NOTHING TO SAY about this
//                  pair on this tick, because at least one endpoint's pose is
//                  missing or older than pose_timeout_s. Every physical field
//                  in an invalid row is published as a hard zero (ber 1.0,
//                  bandwidth 0.0) rather than as the last value computed from
//                  a pose that has since gone stale. Consumers must drop
//                  invalid rows, not average them in.
//   ~/robot_index  latched std_msgs/String JSON: row/column key for the above
//   ~/stats        std_msgs/String JSON, per-link relay/drop counters
// Moved comments: docs/hmr_sim_code_notes.md

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <tinyxml2.h>

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

double Dist3(const Vec3 & a, const Vec3 & b)
{
  const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 2D distance from point C to segment AB (trees block by trunk position; height
// is irrelevant at these link geometries).
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

// The one channel width the whole rate ladder is quoted in: the 72.0 / 28.9 /
// 7.2 Mbps tiers are 802.11n 20 MHz MCS7 / MCS3 / MCS0 rates. Changing this
// without changing the ladder makes every Eb/N0 in this file wrong.
constexpr double kChannelBandwidthHz = 20e6;

double FresnelZoneRadius(double distance_m, double frequency_hz)
{
  return 0.5 * std::sqrt(3.0e8 / frequency_hz * distance_m);
}

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

std::string ToLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return std::tolower(c); });
  return s;
}

}  // namespace

class HmrCommsSimNode : public rclcpp::Node
{
public:
  HmrCommsSimNode()
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

    world_sdf_ = declare_parameter<std::string>("world_sdf", "");
    // A tree matches if its name contains ANY substring here; an unlisted
    // species is transparent to the radio (pine does not match pinus_pinaster).
    // To match nothing pass one empty string: an empty list aborts the node.
    // (notes: comms-sim-tree-name-substrings)
    tree_name_substrings_ = declare_parameter<std::vector<std::string>>(
      "tree_name_substrings", std::vector<std::string>{
        "tree", "pine", "pinus", "oak", "euca", "ulex"});
    for (auto & s : tree_name_substrings_) {
      s = ToLower(s);
    }
    tree_name_substrings_.erase(
      std::remove(tree_name_substrings_.begin(), tree_name_substrings_.end(), std::string{}),
      tree_name_substrings_.end());
    // Deprecated scalar tree_name_substring: if set, it replaces (does not
    // extend) the tree_name_substrings list.
    // (notes: comms-sim-legacy-tree-substring)
    const auto legacy_substring = declare_parameter<std::string>("tree_name_substring", "");
    if (!legacy_substring.empty()) {
      RCLCPP_WARN(get_logger(),
        "tree_name_substring is deprecated — use tree_name_substrings (a list). "
        "Overriding the list with the single legacy value '%s'.", legacy_substring.c_str());
      tree_name_substrings_ = {ToLower(legacy_substring)};
    }
    if (tree_name_substrings_.empty()) {
      RCLCPP_WARN(get_logger(),
        "tree_name_substrings is empty — no model will match, so the link model "
        "degenerates to free space with fading and no obstacle attenuation.");
    }
    tree_radius_m_ = declare_parameter<double>("tree_radius_m", 0.3);
    frequency_hz_ = declare_parameter<double>("frequency_hz", 2.4e9);
    tx_power_dbm_ = declare_parameter<double>("tx_power_dbm", 30.0);
    noise_floor_dbm_ = declare_parameter<double>("noise_floor_dbm", -101.0);
    p0_db_ = declare_parameter<double>("p0_db", 49.17);
    // 70 dB per trunk makes ONE tree in the Fresnel zone fatal to the link.
    // Runs made with a different value are a different radio regime; never pool
    // across them. (notes: comms-sim-tree-attenuation-70db)
    tree_attenuation_db_ = declare_parameter<double>("tree_attenuation_db", 70.0);
    // Hard radio horizon (3D). With trunks fatal, tree-free lanes are the
    // links left over, and free-space loss alone keeps a 30 dBm link decodable
    // for kilometres — so range itself is capped. <= 0 disables the cap.
    max_range_m_ = declare_parameter<double>("max_range_m", 30.0);
    fade_sigma_db_ = declare_parameter<double>("fade_sigma_db", 4.8);
    fade_alpha_ = declare_parameter<double>("fade_alpha", 0.9);
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

    LoadTreePositions();

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
      std::bind(&HmrCommsSimNode::OnLinkTimer, this));
    delivery_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.005),
      std::bind(&HmrCommsSimNode::OnDeliveryTimer, this));
    if (!pending_topics_.empty()) {
      discovery_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(1.0),
        std::bind(&HmrCommsSimNode::DiscoverAndSetup, this));
    }
    stats_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(std::max(stats_period_s, 1.0)),
      std::bind(&HmrCommsSimNode::PublishStats, this));

    RCLCPP_INFO(get_logger(),
      "hmr_comms_sim up: %zu robots, %zu trees, %zu relay topics pending, seed=%ld, "
      "transmission_model=%s.",
      n, trees_.size(), pending_topics_.size(), static_cast<long>(seed),
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

  size_t PairKey(size_t i, size_t j) const
  {
    return std::min(i, j) * robot_names_.size() + std::max(i, j);
  }
  size_t DirKey(size_t s, size_t r) const { return s * robot_names_.size() + r; }

  // ---- tree extraction ----------------------------------------------------
  bool LooksLikeTree(const char * text) const
  {
    if (!text) {
      return false;
    }
    const std::string low = ToLower(text);
    for (const auto & sub : tree_name_substrings_) {
      if (low.find(sub) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  void LoadTreePositions()
  {
    if (world_sdf_.empty()) {
      RCLCPP_WARN(get_logger(),
        "world_sdf not set — no obstacle attenuation, pure distance model.");
      return;
    }
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(world_sdf_.c_str()) != tinyxml2::XML_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Could not parse world SDF '%s' — continuing with 0 trees.",
        world_sdf_.c_str());
      return;
    }
    auto * sdf = doc.FirstChildElement("sdf");
    auto * world = sdf ? sdf->FirstChildElement("world") : nullptr;
    if (!world) {
      RCLCPP_ERROR(get_logger(), "No <sdf><world> in '%s' — continuing with 0 trees.",
        world_sdf_.c_str());
      return;
    }
    auto read_pose = [this](tinyxml2::XMLElement * elem, const char * what, Vec3 & out) {
      auto * pose = elem->FirstChildElement("pose");
      if (!pose || !pose->GetText()) {
        // Legal SDF — but a tree silently placed at the origin sits on the spawn
        // point and attenuates every link, so say so rather than absorbing it.
        RCLCPP_WARN(get_logger(),
          "Tree '%s' has no <pose>; placing it at the origin per SDF defaults. "
          "If that is not where it belongs, the world needs an explicit pose.",
          what ? what : "(unnamed)");
        return true;
      }
      std::istringstream ss(pose->GetText());
      return static_cast<bool>(ss >> out.x >> out.y >> out.z);
    };
    // Distinct names that did NOT match, with instance counts, so a world whose
    // species this build cannot see is visible at startup instead of silently
    // costing attenuation. Trailing instance digits are folded away.
    std::map<std::string, int> rejected;
    auto note_rejected = [&rejected](const char * raw) {
      std::string n = raw ? raw : "(unnamed)";
      while (!n.empty() && (std::isdigit(static_cast<unsigned char>(n.back())) ||
        n.back() == '_' || n.back() == ' '))
      {
        n.pop_back();
      }
      ++rejected[n.empty() ? "(unnamed)" : n];
    };
    for (auto * model = world->FirstChildElement("model"); model;
         model = model->NextSiblingElement("model"))
    {
      const char * name = model->Attribute("name");
      if (!LooksLikeTree(name)) {
        note_rejected(name);
        continue;
      }
      Vec3 p;
      if (read_pose(model, name, p)) {
        trees_.push_back(p);
      }
    }
    // Worlds that <include> tree models with the pose in the include tag. The
    // instance name and the model URI are BOTH tested — flatforestv2's pines are
    // <name>pine_N</name> under <uri>model://cmu_pine_tree</uri>, and a world
    // that leaves an include unnamed still identifies its species via the URI.
    for (auto * inc = world->FirstChildElement("include"); inc;
         inc = inc->NextSiblingElement("include"))
    {
      auto * name_elem = inc->FirstChildElement("name");
      auto * uri_elem = inc->FirstChildElement("uri");
      const char * iname = name_elem ? name_elem->GetText() : nullptr;
      const char * iuri = uri_elem ? uri_elem->GetText() : nullptr;
      if (!LooksLikeTree(iname) && !LooksLikeTree(iuri)) {
        note_rejected(iname ? iname : iuri);
        continue;
      }
      Vec3 p;
      if (read_pose(inc, iname ? iname : iuri, p)) {
        trees_.push_back(p);
      }
    }
    std::string subs;
    for (const auto & s : tree_name_substrings_) {
      subs += (subs.empty() ? "'" : ", '") + s + "'";
    }
    RCLCPP_INFO(get_logger(),
      "Loaded %zu tree positions (name or include URI contains any of [%s]) from %s",
      trees_.size(), subs.c_str(), world_sdf_.c_str());
    std::string rej;
    int shown = 0;
    for (const auto & kv : rejected) {
      if (++shown > 12) {
        rej += ", …";
        break;
      }
      rej += (rej.empty() ? "" : ", ") + kv.first + "×" + std::to_string(kv.second);
    }
    RCLCPP_INFO(get_logger(),
      "Not counted as trees (transparent to the radio): %s. Any TREE species in "
      "that list means this world needs it added to tree_name_substrings.",
      rej.empty() ? "(nothing)" : rej.c_str());
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
      }
    }
    link_states_pub_->publish(out);
  }

  void ComputePair(size_t i, size_t j, PairState & ps)
  {
    const double distance = std::max(Dist3(poses_[i], poses_[j]), 0.1);

    const double link_width = tree_radius_m_ + FresnelZoneRadius(distance, frequency_hz_);
    int trees_on_link = 0;
    for (const auto & tree : trees_) {
      if (PointToSegmentDistSq2D(poses_[i], poses_[j], tree) < link_width * link_width) {
        ++trees_on_link;
      }
    }

    // AR(1) shadow fade with stationary std == fade_sigma_db.
    std::normal_distribution<double> gauss(0.0, fade_sigma_db_);
    ps.fade_db = fade_alpha_ * ps.fade_db +
      std::sqrt(1.0 - fade_alpha_ * fade_alpha_) * gauss(fade_rng_);

    double path_loss = p0_db_ + 20.0 * std::log10(distance) +
      trees_on_link * tree_attenuation_db_ + ps.fade_db;
    // The radio horizon lands as path loss, not a connected=false override,
    // so every published column (rx power, SNR, BER, tier) tells the same
    // story and the 3-of-8 tier hysteresis still smooths the crossing.
    // 200 dB puts SNR near -100 dB: below every tier, beyond any fade.
    if (max_range_m_ > 0.0 && distance > max_range_m_) {
      path_loss += 200.0;
    }
    const double rx_dbm = tx_power_dbm_ - path_loss;
    const double snr_db = rx_dbm - noise_floor_dbm_;

    // Tier first, then BER at that tier. NextBandwidth reads only snr_history
    // and ps.bandwidth_mbps and writes only snr_history, so the order leaves
    // every delivery decision unchanged. (notes: comms-sim-tier-before-ber)
    ps.bandwidth_mbps = NextBandwidth(ps, snr_db);
    ps.connected = ps.bandwidth_mbps > 0.0;

    double ber;
    if (ps.bandwidth_mbps <= 0.0) {
      // No gear engaged: BER is undefined and nothing gets through, so publish
      // 1.0, as an invalid row does. Consumers should mask on connected; this
      // keeps an unmasked read wrong in the safe direction.
      // (notes: comms-sim-ber-when-no-gear)
      ber = 1.0;
    } else {
      const double spectral_efficiency = ps.bandwidth_mbps * 1e6 / kChannelBandwidthHz;
      ber = trees_on_link == 0 ?
        AwgnQam64Ber(DbmToPow(rx_dbm), DbmToPow(noise_floor_dbm_), spectral_efficiency) :
        RayleighQam64Ber(DbmToPow(rx_dbm), DbmToPow(noise_floor_dbm_), spectral_efficiency);
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
         << ",\"backlog_bytes\":" << queue_bytes << "}";
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

  // parameters
  std::vector<std::string> robot_names_;
  std::string pose_topic_pattern_;
  std::string world_sdf_;
  std::vector<std::string> tree_name_substrings_;
  double tree_radius_m_, frequency_hz_, tx_power_dbm_, noise_floor_dbm_;
  double p0_db_, tree_attenuation_db_, max_range_m_, fade_sigma_db_, fade_alpha_;
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
  std::vector<Vec3> trees_;
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

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr link_states_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_, robot_index_pub_;
  rclcpp::TimerBase::SharedPtr link_timer_, delivery_timer_, discovery_timer_, stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<HmrCommsSimNode> node;
  try {
    node = std::make_shared<HmrCommsSimNode>();
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
