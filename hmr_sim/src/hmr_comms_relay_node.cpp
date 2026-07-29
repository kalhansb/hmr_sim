// HMR Comms Relay Node
// A ROS2 node that simulates a communications pipeline between robots.
// It subscribes to each robot's topics, reads network metrics from the
// HMRNetSim Gazebo plugin (bridged to ROS2), and relays or drops messages
// based on PER, PDR, and bandwidth.
//
// Topic convention:
//   Robot publishes on:  /<robot>/<comms_topic>        (existing topic)
//   Receiver gets on:    /<receiver>/rx/<sender>/<comms_topic>
//
// Parameters:
//   robot_names:   list of robot name strings, e.g. ["atlas", "bestla"]
//   comms_topics:  list of topic name strings, e.g. ["rgbd_camera/points", "velodyne_points"]
//
// The node auto-discovers message types from the actual topic publishers,
// so no message type configuration is needed.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>

class HmrCommsRelayNode : public rclcpp::Node
{
public:
  HmrCommsRelayNode()
  : Node("hmr_comms_relay")
  {
    // Declare and read parameters
    this->declare_parameter<std::vector<std::string>>("robot_names", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("comms_topics", std::vector<std::string>{});

    auto robot_names = this->get_parameter("robot_names").as_string_array();
    auto comms_topics = this->get_parameter("comms_topics").as_string_array();

    if (robot_names.empty() || comms_topics.empty())
    {
      RCLCPP_ERROR(this->get_logger(),
        "robot_names and comms_topics parameters must be non-empty.");
      return;
    }

    robot_names_ = robot_names;
    comms_topics_ = comms_topics;

    // Initialize link states and subscribe to network metrics from the Gazebo plugin
    for (const auto & r1 : robot_names_)
    {
      for (const auto & r2 : robot_names_)
      {
        if (r1 == r2) continue;

        // Initialize default link state
        link_states_[r1][r2] = LinkState{};

        std::string prefix = "/robot_hmr_net_sim/" + r1 + "_to_" + r2;

        // Subscribe to bandwidth
        auto bw_sub = this->create_subscription<std_msgs::msg::Float64>(
          prefix + "/bandwidth", 10,
          [this, r1, r2](const std_msgs::msg::Float64::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(link_mutex_);
            link_states_[r1][r2].bandwidth = msg->data;
          });
        metric_subs_.push_back(bw_sub);
      }
    }

    // Build the set of source topics we need to discover
    for (const auto & sender : robot_names_)
    {
      for (const auto & topic : comms_topics_)
      {
        std::string src_topic = "/" + sender + "/" + topic;
        pending_topics_[src_topic] = {sender, topic};
      }
    }

    // Timer to discover topic types and create generic subs/pubs
    discovery_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::seconds(1),
      std::bind(&HmrCommsRelayNode::discover_and_setup, this));

    RCLCPP_INFO(this->get_logger(),
      "HMR Comms Relay starting: %zu robots, %zu comms topics. Waiting for publishers...",
      robot_names_.size(), comms_topics_.size());
  }

private:
  struct LinkState
  {
    double bandwidth = 72.0;  // Mbps
  };

  struct TokenBucket
  {
    double tokens = 0.0;      // Available bytes
    rclcpp::Time last_refill;  // Last refill timestamp
    bool initialized = false;
  };

  struct TopicInfo
  {
    std::string sender;
    std::string comms_topic;
  };

  void discover_and_setup()
  {
    if (pending_topics_.empty())
    {
      // All topics discovered, stop the timer
      discovery_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "All comms topics discovered and connected.");
      return;
    }

    // Get all currently advertised topics and their types
    auto topic_names_and_types = this->get_topic_names_and_types();

    std::vector<std::string> discovered;

    for (const auto & [src_topic, info] : pending_topics_)
    {
      auto it = topic_names_and_types.find(src_topic);
      if (it == topic_names_and_types.end() || it->second.empty())
      {
        continue;  // Not yet advertised
      }

      const std::string & msg_type = it->second[0];

      RCLCPP_INFO(this->get_logger(),
        "Discovered %s [%s] for robot '%s'",
        src_topic.c_str(), msg_type.c_str(), info.sender.c_str());

      // Create generic publishers for each receiver
      for (const auto & receiver : robot_names_)
      {
        if (receiver == info.sender) continue;
        std::string rx_topic = "/" + receiver + "/rx/" + info.sender + "/" + info.comms_topic;

        auto pub = this->create_generic_publisher(rx_topic, msg_type, rclcpp::QoS(10));
        relay_publishers_[receiver][info.sender + "/" + info.comms_topic] = pub;

        RCLCPP_INFO(this->get_logger(), "  Relay: %s -> %s [%s]",
          src_topic.c_str(), rx_topic.c_str(), msg_type.c_str());
      }

      // Create generic subscription on the source topic
      std::string sender = info.sender;
      std::string comms_topic = info.comms_topic;
      auto sub = this->create_generic_subscription(
        src_topic, msg_type, rclcpp::QoS(10),
        [this, sender, comms_topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          on_tx_message(sender, comms_topic, msg);
        });
      tx_subs_.push_back(sub);

      discovered.push_back(src_topic);
    }

    for (const auto & t : discovered)
    {
      pending_topics_.erase(t);
    }

    if (!pending_topics_.empty())
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waiting for %zu topics to appear...", pending_topics_.size());
    }
  }

  void on_tx_message(
    const std::string & sender,
    const std::string & topic,
    std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    for (const auto & receiver : robot_names_)
    {
      if (receiver == sender) continue;

      LinkState ls;
      {
        std::lock_guard<std::mutex> lock(link_mutex_);
        auto it1 = link_states_.find(sender);
        if (it1 != link_states_.end())
        {
          auto it2 = it1->second.find(receiver);
          if (it2 != it1->second.end())
          {
            ls = it2->second;
          }
        }
      }

      // Bandwidth throttling via token bucket
      size_t msg_bytes = msg->size();
      {
        std::lock_guard<std::mutex> lock(bucket_mutex_);
        auto & bucket = token_buckets_[sender][receiver];
        rclcpp::Time now = this->get_clock()->now();

        if (!bucket.initialized)
        {
          bucket.last_refill = now;
          bucket.tokens = 0.0;
          bucket.initialized = true;
        }

        // Refill tokens based on elapsed time and current bandwidth
        double elapsed_sec = (now - bucket.last_refill).seconds();
        if (elapsed_sec > 0.0)
        {
          double bytes_per_sec = ls.bandwidth * 1e6 / 8.0;  // Mbps -> bytes/sec
          bucket.tokens += elapsed_sec * bytes_per_sec;
          // Cap at 1 second worth of burst
          double max_tokens = bytes_per_sec;
          if (bucket.tokens > max_tokens)
          {
            bucket.tokens = max_tokens;
          }
          bucket.last_refill = now;
        }

        if (bucket.tokens < static_cast<double>(msg_bytes))
        {
          RCLCPP_DEBUG(this->get_logger(),
            "DROPPED %s -> %s on %s (bandwidth: %.0f bytes needed, %.0f available)",
            sender.c_str(), receiver.c_str(), topic.c_str(),
            static_cast<double>(msg_bytes), bucket.tokens);
          continue;
        }

        bucket.tokens -= static_cast<double>(msg_bytes);
      }

      // Relay the message
      std::string key = sender + "/" + topic;
      auto pub_it = relay_publishers_.find(receiver);
      if (pub_it != relay_publishers_.end())
      {
        auto pub_it2 = pub_it->second.find(key);
        if (pub_it2 != pub_it->second.end())
        {
          pub_it2->second->publish(*msg);
          RCLCPP_DEBUG(this->get_logger(),
            "RELAYED %s -> %s on %s", sender.c_str(), receiver.c_str(), topic.c_str());
        }
      }
    }
  }

  std::vector<std::string> robot_names_;
  std::vector<std::string> comms_topics_;

  // Pending tx topics waiting for type discovery
  std::map<std::string, TopicInfo> pending_topics_;

  // Discovery timer
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  // Network metric subscribers (kept alive)
  std::vector<rclcpp::SubscriptionBase::SharedPtr> metric_subs_;

  // Generic tx subscriptions
  std::vector<rclcpp::GenericSubscription::SharedPtr> tx_subs_;

  // Relay publishers: [receiver][sender/topic] -> GenericPublisher
  std::map<std::string, std::map<std::string, rclcpp::GenericPublisher::SharedPtr>> relay_publishers_;

  // Cached link states from the Gazebo plugin metrics
  std::map<std::string, std::map<std::string, LinkState>> link_states_;
  std::mutex link_mutex_;

  // Token buckets for bandwidth throttling: [sender][receiver] -> TokenBucket
  std::map<std::string, std::map<std::string, TokenBucket>> token_buckets_;
  std::mutex bucket_mutex_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HmrCommsRelayNode>());
  rclcpp::shutdown();
  return 0;
}
