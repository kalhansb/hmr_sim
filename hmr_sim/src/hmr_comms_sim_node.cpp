// HMR Comms Sim Node — message-level wireless-link emulator for multi-robot runs.
//
// Successor to the HMRNetSim gz plugin + hmr_comms_relay_node pair: both roles
// (link-physics "oracle" and message "cable") live in this one ROS2 node, so the
// same emulator runs against the live sim, a bag replay, or any pose source —
// Gazebo is not involved. Tree positions come from parsing the world SDF once at
// startup (trees are static; the plugin only ever read them once anyway).
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
//   Two per-topic policies:
//     reliable    — never dropped. FIFO queue per directional link, drained only
//                   while the link is up and airtime is available; a down link
//                   grows a backlog that is delivered late, never lost (bounded
//                   by reliable_queue_max_bytes, oldest dropped + warned).
//                   Airtime cost is bits / the CURRENT TIER: a bad link is slow.
//     best_effort — dropped when the link is down or when the shared channel has
//                   no airtime; otherwise delivered, less residual_pdr.
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

// 64-QAM BER over AWGN (line of sight).
//
// spectral_efficiency is bits/s/Hz AT THE TIER THE LINK IS ACTUALLY IN, passed
// in by the caller, not the top tier: Eb/N0 = (S/N) / (R/B), so a link that has
// downshifted spreads the same received power over fewer bits per second and
// each bit gets more energy. Hardcoding the 72 Mbps value here charged a
// downshifted link the error rate of a gear it was not in — see the
// "Deliberately removed" note below for what that cost.
//
// The constellation stays 64-QAM at every tier, which the 28.9 and 7.2 Mbps
// tiers are not (802.11n MCS3 is 16-QAM, MCS0 is BPSK). At equal Eb/N0 a
// lower-order constellation has a LOWER error rate, so this is a pessimistic
// bound on the BER of the real tier, not an estimate of it. That is deliberate:
// ber is a published diagnostic that gates nothing, and a bound written down as
// a bound beats a second pair of closed forms nobody has checked.
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

// Deliberately removed: MessageSuccessProbability(ber, bits).
//
// It applied the BER above to a WHOLE serialized message, and its result was
// used to drop best-effort messages and to inflate reliable airtime. Both were
// wrong. The BER functions hardcode spectral_efficiency = 72e6/20e6 — 64-QAM at
// the TOP rate — regardless of the tier NextBandwidth actually selected, so a
// link that had correctly downshifted to 7.2 Mbps was charged the error rate of
// a gear it was not in: slow AND lossy for one weak signal, which is the
// opposite of what rate adaptation is for. Measured at tx_power_dbm=-14, median
// BER on CONNECTED samples was 0.116, so a 200-byte beacon survived with
// probability ~1e-70: robots exchanged 3544 intent beacons and 7 arrived, peer
// records never formed, and the pursuit manoeuvre could never arm.
// HMRNetSim.cc, which this node was ported from, never did this — it gates on
// the SNR>=2 dB boundary (PDR 1.0 below, 1e-8 above) and publishes ber/per as
// diagnostics only.
//
// Generation 9 made BER a function of the selected tier, which is the second
// half of the condition this note used to set for reintroducing BER gating. The
// first half still stands and is the binding one: DO NOT gate delivery on BER.
// The bandwidth state machine is the only authority on whether a message gets
// through, and a second, independent loss mechanism on top of it is what
// produced the 7-of-3544 beacon result above.

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

    world_sdf_ = declare_parameter<std::string>("world_sdf", "");
    // A world may name its trees by species rather than by the word "tree"
    // (flatforestv2 has 80 "Oak tree" models plus 8 "pine_*" includes), so the
    // match is a set of substrings, ANY of which qualifies. A species missing
    // from this list is transparent to the radio, which inflates the link budget
    // rather than erroring — so LoadTreePositions() logs both the matched count
    // and the names it rejected, and adding a world means reading that line.
    // Defaults cover every tree species in the shipped worlds: "pine" alone does
    // NOT match `pinus_pinaster` (20 per forest world), hence "pinus".
    // To match nothing, write [""] — NOT []. An empty yaml/CLI list has no
    // inferable element type and arrives NOT_SET, which aborts the node during
    // base-class construction, before any code here runs. That is generic rclcpp
    // behaviour for every list parameter (`reliable_topics` included), not a
    // property of this one; main() turns the abort into a readable message.
    tree_name_substrings_ = declare_parameter<std::vector<std::string>>(
      "tree_name_substrings", std::vector<std::string>{
        "tree", "pine", "pinus", "oak", "euca", "ulex"});
    for (auto & s : tree_name_substrings_) {
      s = ToLower(s);
    }
    tree_name_substrings_.erase(
      std::remove(tree_name_substrings_.begin(), tree_name_substrings_.end(), std::string{}),
      tree_name_substrings_.end());
    // Superseded scalar. Replaces (not extends) the list, so a world config that
    // still sets it keeps its own species set rather than silently gaining the
    // new defaults. Not bit-for-bit the old behaviour even so: include URIs are
    // now tested alongside instance names, so legacy "tree" picks up flatforest's
    // pines anyway (their URI is model://cmu_pine_tree) — 88, where it used to
    // find 80. That widening is the point of this change, not a regression.
    // (Declared after the list so a NOT_SET list above cannot skip it.)
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
    // 70 dB/trunk makes ONE tree in the Fresnel zone fatal: the observed
    // one-tree link geometries needed 51-66 dB of extra loss to cross the
    // SNR floor, which the paper's fitted 11.98 (under which links survived
    // ~4 trunks) never supplied. Runs before 2026-09 used 11.98 and are a
    // different radio regime — never pool across the change.
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
    // Residual loss on a link the state machine reports as up, matching
    // HMRNetSim.cc's PDR of 1e-8 above the SNR>=2 dB boundary.
    residual_pdr_ = declare_parameter<double>("residual_pdr", 1e-8);
    // A pose older than this (in the node's own clock, so sim time when
    // use_sim_time is set) does not describe where the robot is now, and a link
    // model run on it reports a link that may not exist. Links touching a stale
    // endpoint go invalid: no delivery, and a zeroed diagnostic row. Set <= 0
    // to restore the pre-generation-9 behaviour of trusting a pose forever.
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 2.0);
    reliable_queue_max_bytes_ = static_cast<size_t>(
      declare_parameter<int64_t>("reliable_queue_max_bytes", 64LL * 1024 * 1024));
    rx_qos_depth_ = static_cast<size_t>(declare_parameter<int64_t>("rx_qos_depth", 100));
    const double stats_period_s = declare_parameter<double>("stats_period_s", 10.0);
    const int64_t seed = declare_parameter<int64_t>("seed", 42);

    if (robot_names_.size() < 2) {
      RCLCPP_FATAL(get_logger(), "robot_names needs at least 2 robots.");
      throw std::runtime_error("robot_names needs at least 2 robots");
    }
    for (const auto & t : reliable_topics) {
      if (std::find(best_effort_topics.begin(), best_effort_topics.end(), t) !=
          best_effort_topics.end())
      {
        RCLCPP_FATAL(get_logger(), "Topic '%s' listed as both reliable and best_effort.",
          t.c_str());
        throw std::runtime_error("topic in both policy lists");
      }
    }
    if (reliable_topics.empty() && best_effort_topics.empty()) {
      RCLCPP_WARN(get_logger(),
        "No reliable_topics/best_effort_topics configured — only link states will be published.");
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
      "hmr_comms_sim up: %zu robots, %zu trees, %zu relay topics pending, seed=%ld.",
      n, trees_.size(), pending_topics_.size(), static_cast<long>(seed));
  }

private:
  enum class Policy { kReliable, kBestEffort };

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
    // and ps.bandwidth_mbps and writes only snr_history, so this reordering
    // leaves the selected tier — and therefore every delivery decision —
    // bit-identical to generation 8. Only the ber column moves.
    ps.bandwidth_mbps = NextBandwidth(ps, snr_db);
    ps.connected = ps.bandwidth_mbps > 0.0;

    double ber;
    if (ps.bandwidth_mbps <= 0.0) {
      // No gear engaged: there is no rate to define a bit error rate at, and
      // nothing is getting through. 1.0, the same value an invalid row
      // publishes, rather than a number computed at a rate the link is not
      // using. Every consumer of this column should already be masking on
      // connected; this makes an unmasked read wrong in the safe direction.
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
      rclcpp::QoS qos{rclcpp::KeepLast(rx_qos_depth_)};
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
          policy == Policy::kReliable ? "reliable" : "best-effort",
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
    if (!pending_topics_.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "Waiting for %zu relay topics to appear...", pending_topics_.size());
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

      if (policy == Policy::kReliable) {
        auto & queue = reliable_queues_[dir];
        queue.push_back({msg, pub_it->second, bytes});
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
        DrainReliable(dir, ps, stats);
        continue;
      }

      // best-effort
      if (!ps.valid || !ps.connected) {
        ++stats.drop_disconnected;
        continue;
      }
      const size_t bits = bytes * 8;
      // Delivery is the bandwidth state machine's call, as in HMRNetSim.cc:
      // above the SNR>=2 dB boundary (i.e. ps.connected, checked above) the
      // reference plugin delivers with PDR=1e-8, and its ber/per exist only to
      // be published. Link quality reaches the experiment through the TIER —
      // a weak link is slow, which surfaces as airtime pressure and backlog in
      // cost_s below — not as vanished messages.
      if (uniform_(drop_rng_) < residual_pdr_) {
        ++stats.drop_ber;
        continue;
      }
      RefillAirtime();
      if (airtime_tokens_ <= 0.0) {
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
      // The tier is the whole quality model, so a byte costs bits/tier and
      // nothing more. The BER-derived retransmission multiplier that used to
      // scale this was never physical: it was capped at retx_cap while the
      // true expected number of transmissions at the BER this node computes is
      // astronomically larger, and that cap is the only reason 60 kB map
      // deltas flowed at all on a link where 200-byte beacons were dying.
      const double cost_s = bits / (ps.bandwidth_mbps * 1e6);
      airtime_tokens_ -= cost_s;
      ScheduleDelivery(front.pub, front.msg, cost_s);
      ++stats.relayed;
      stats.bytes_relayed += front.bytes;
      reliable_queue_bytes_[dir] -= front.bytes;
      queue.pop_front();
    }
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
    if (!reliable_queues_.empty()) {
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
      ss << (first ? "" : ",")
         << "{\"from\":\"" << robot_names_[dir / n] << "\",\"to\":\"" << robot_names_[dir % n]
         << "\",\"relayed\":" << s.relayed
         << ",\"bytes\":" << s.bytes_relayed
         << ",\"drop_ber\":" << s.drop_ber
         << ",\"drop_airtime\":" << s.drop_airtime
         << ",\"drop_disconnected\":" << s.drop_disconnected
         << ",\"drop_overflow\":" << s.drop_overflow
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
  double pose_timeout_s_;
  size_t reliable_queue_max_bytes_;
  size_t rx_qos_depth_;

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
    // catch inside the node body can see it. Overwhelmingly this is an empty
    // list written as `[]`: yaml and `--ros-args -p` cannot infer an element
    // type, the value arrives NOT_SET, and the default abort is an opaque
    // std::terminate that names neither the cause nor the fix.
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
