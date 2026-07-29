// ROS2 <-> Gazebo Generic Bridge Node
//
// A bidirectional bridge between ROS2 and Gazebo transport that serializes
// ROS2 messages to binary (CDR format) and transports them as
// gz::msgs::StringMsg payloads over Gazebo transport, and vice versa.
//
// This allows bridging ANY ROS2 message type without needing corresponding
// Gazebo protobuf message definitions or custom type conversion code.
//
// ROS2 → GZ:
//   Subscribes to ROS2 topics using generic subscriptions (CDR serialized),
//   wraps the binary payload in a gz::msgs::StringMsg, and publishes to
//   Gazebo transport topics.
//
// GZ → ROS2:
//   Subscribes to Gazebo transport topics for gz::msgs::StringMsg payloads,
//   unwraps the binary CDR data, and publishes to ROS2 topics using generic
//   publishers.
//
// Example: bridging octomap_msgs/msg/Octomap between ROS2 and Gazebo
//
// Parameters:
//   ros2_to_gz_ros_topics:  ROS2 source topic names
//   ros2_to_gz_gz_topics:   Gazebo destination topic names
//   ros2_to_gz_ros_types:   ROS2 message type strings
//
//   gz_to_ros2_gz_topics:   Gazebo source topic names
//   gz_to_ros2_ros_topics:  ROS2 destination topic names
//   gz_to_ros2_ros_types:   ROS2 message type strings

#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class Ros2GzGenericBridge : public rclcpp::Node
{
public:
  Ros2GzGenericBridge()
  : Node("ros2_gz_generic_bridge")
  {
    // ROS2 → GZ parameters
    this->declare_parameter<std::vector<std::string>>("ros2_to_gz_ros_topics", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("ros2_to_gz_gz_topics", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("ros2_to_gz_ros_types", std::vector<std::string>{});

    // GZ → ROS2 parameters
    this->declare_parameter<std::vector<std::string>>("gz_to_ros2_gz_topics", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("gz_to_ros2_ros_topics", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("gz_to_ros2_ros_types", std::vector<std::string>{});

    setup_ros2_to_gz();
    setup_gz_to_ros2();

    RCLCPP_INFO(this->get_logger(),
      "ROS2-GZ Generic Bridge started: %zu ROS2->GZ, %zu GZ->ROS2 topics",
      ros2_subs_.size(), ros2_pubs_.size());
  }

private:
  void setup_ros2_to_gz()
  {
    auto ros_topics = this->get_parameter("ros2_to_gz_ros_topics").as_string_array();
    auto gz_topics = this->get_parameter("ros2_to_gz_gz_topics").as_string_array();
    auto ros_types = this->get_parameter("ros2_to_gz_ros_types").as_string_array();

    if (ros_topics.empty()) return;

    if (ros_topics.size() != gz_topics.size() ||
        ros_topics.size() != ros_types.size())
    {
      RCLCPP_ERROR(this->get_logger(),
        "ros2_to_gz parameter arrays must all have the same length");
      return;
    }

    for (size_t i = 0; i < ros_topics.size(); ++i)
    {
      // Create Gazebo publisher
      gz_pubs_.push_back(
        gz_node_.Advertise<gz::msgs::StringMsg>(gz_topics[i]));

      size_t idx = gz_pubs_.size() - 1;

      // Create ROS2 generic subscription (receives CDR-serialized bytes)
      auto sub = this->create_generic_subscription(
        ros_topics[i], ros_types[i], rclcpp::QoS(10),
        [this, idx, ros_topic = ros_topics[i], gz_topic = gz_topics[i]]
        (std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
          auto & rcl_msg = msg->get_rcl_serialized_message();

          // Wrap CDR binary data in a StringMsg for Gazebo transport
          gz::msgs::StringMsg gz_msg;
          gz_msg.set_data(
            reinterpret_cast<const char *>(rcl_msg.buffer),
            rcl_msg.buffer_length);

          gz_pubs_[idx].Publish(gz_msg);

          RCLCPP_DEBUG(this->get_logger(),
            "ROS2->GZ: %s -> %s (%zu bytes)",
            ros_topic.c_str(), gz_topic.c_str(), rcl_msg.buffer_length);
        });

      ros2_subs_.push_back(sub);

      RCLCPP_INFO(this->get_logger(),
        "ROS2->GZ bridge: %s [%s] -> %s",
        ros_topics[i].c_str(), ros_types[i].c_str(), gz_topics[i].c_str());
    }
  }

  void setup_gz_to_ros2()
  {
    auto gz_topics = this->get_parameter("gz_to_ros2_gz_topics").as_string_array();
    auto ros_topics = this->get_parameter("gz_to_ros2_ros_topics").as_string_array();
    auto ros_types = this->get_parameter("gz_to_ros2_ros_types").as_string_array();

    if (gz_topics.empty()) return;

    if (gz_topics.size() != ros_topics.size() ||
        gz_topics.size() != ros_types.size())
    {
      RCLCPP_ERROR(this->get_logger(),
        "gz_to_ros2 parameter arrays must all have the same length");
      return;
    }

    // Reserve to prevent reallocation invalidating references
    gz_callbacks_.reserve(gz_topics.size());

    for (size_t i = 0; i < gz_topics.size(); ++i)
    {
      // Create ROS2 generic publisher
      auto pub = this->create_generic_publisher(
        ros_topics[i], ros_types[i], rclcpp::QoS(10));
      ros2_pubs_.push_back(pub);

      size_t idx = ros2_pubs_.size() - 1;

      // Create Gazebo subscription callback
      gz_callbacks_.emplace_back(
        [this, idx, gz_topic = gz_topics[i], ros_topic = ros_topics[i]]
        (const gz::msgs::StringMsg & gz_msg)
        {
          const auto & data = gz_msg.data();

          // Reconstruct ROS2 serialized message from the CDR binary payload
          rclcpp::SerializedMessage serialized_msg(data.size());
          auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
          std::memcpy(rcl_msg.buffer, data.data(), data.size());
          rcl_msg.buffer_length = data.size();

          ros2_pubs_[idx]->publish(serialized_msg);

          RCLCPP_DEBUG(this->get_logger(),
            "GZ->ROS2: %s -> %s (%zu bytes)",
            gz_topic.c_str(), ros_topic.c_str(), data.size());
        });

      gz_node_.Subscribe<gz::msgs::StringMsg>(
        gz_topics[i], gz_callbacks_[i]);

      RCLCPP_INFO(this->get_logger(),
        "GZ->ROS2 bridge: %s -> %s [%s]",
        gz_topics[i].c_str(), ros_topics[i].c_str(), ros_types[i].c_str());
    }
  }

  // Gazebo transport node
  gz::transport::Node gz_node_;

  // ROS2 -> GZ
  std::vector<rclcpp::GenericSubscription::SharedPtr> ros2_subs_;
  std::vector<gz::transport::Node::Publisher> gz_pubs_;

  // GZ -> ROS2
  std::vector<rclcpp::GenericPublisher::SharedPtr> ros2_pubs_;
  std::vector<std::function<void(const gz::msgs::StringMsg &)>> gz_callbacks_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2GzGenericBridge>());
  rclcpp::shutdown();
  return 0;
}
