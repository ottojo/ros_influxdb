#include <chrono>
#include <memory>
#include <nav2_util/robot_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <telegraf_ros_lib/telegraf_http_client.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TfTelegrafNode : public rclcpp::Node {
public:
  TfTelegrafNode()
      : rclcpp::Node("tf_monitoring_telegraf"),
        http_client{get_logger().get_child("http_client")},
        tf2_buffer{get_clock()}, tf2_listener{tf2_buffer, this, false},
        timer_handle{create_timer(std::chrono::milliseconds{1000},
                                  [this]() { timerCallback(); })} {
    declare_parameter("map_frame", "map");
    declare_parameter("target_frame", "base_link");
  };

private:
  telegraf_ros_lib::TelegrafHttpClient http_client;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  rclcpp::TimerBase::SharedPtr timer_handle;

  void timerCallback();
};

void TfTelegrafNode::timerCallback() {
  geometry_msgs::msg::PoseStamped global_pose;
  bool success = nav2_util::getCurrentPose(
      global_pose, tf2_buffer, get_parameter("map_frame").as_string(),
      get_parameter("target_frame").as_string(), 0);
  if (!success) {
    return;
  }

  http_client.postValues(std::string("tf") + "_" + get_parameter("map_frame").as_string() + "_" +get_parameter("target_frame").as_string(),
                          {{"x", global_pose.pose.position.x},
                           {"y", global_pose.pose.position.y},
                           {"z", global_pose.pose.position.z},
                          });
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfTelegrafNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
