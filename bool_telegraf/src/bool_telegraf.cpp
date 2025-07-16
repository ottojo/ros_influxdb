#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <telegraf_ros_lib/telegraf_http_client.hpp>

class BoolTelegrafNode : public rclcpp::Node {
public:
  BoolTelegrafNode()
      : rclcpp::Node("bool_telegraf"),
        http_client{get_logger().get_child("http_client")},
        last_sample_time{now()},
        bool_sub{create_subscription<std_msgs::msg::Bool>(
            "/bool_topic", 1,
            [this](const std_msgs::msg::Bool &msg) { topicCallback(msg); })} {

    declare_parameter("measurement_name", "bool");
    declare_parameter("field_name", "value");
  };

private:
  telegraf_ros_lib::TelegrafHttpClient http_client;
  rclcpp::Time last_sample_time;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub;

  void topicCallback(const std_msgs::msg::Bool &msg);
};

void BoolTelegrafNode::topicCallback(const std_msgs::msg::Bool &msg) {

  if ((now() - last_sample_time).seconds() < 5.0) {
    return;
  }
  last_sample_time = now();

  http_client.postValues(
      get_parameter("measurement_name").as_string(),
      {{get_parameter("field_name").as_string(), static_cast<int>(msg.data)}});
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BoolTelegrafNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
