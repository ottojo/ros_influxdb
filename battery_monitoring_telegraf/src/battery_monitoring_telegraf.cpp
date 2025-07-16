#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <string>
#include <telegraf_ros_lib/telegraf_http_client.hpp>

class BatteryMonitoringTelegrafNode : public rclcpp::Node {
public:
  BatteryMonitoringTelegrafNode()
      : rclcpp::Node("battery_monitoring_telegraf"),
        http_client{get_logger().get_child("http_client")},
        last_sample_time{now()},
        battery_state_sub{create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", 1,
            [this](const sensor_msgs::msg::BatteryState &msg) {
              batteryStateCallback(msg);
            })} {};

private:
  telegraf_ros_lib::TelegrafHttpClient http_client;
  rclcpp::Time last_sample_time;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
      battery_state_sub;

  void batteryStateCallback(const sensor_msgs::msg::BatteryState &msg);
};

void BatteryMonitoringTelegrafNode::batteryStateCallback(
    const sensor_msgs::msg::BatteryState &msg) {

  if ((now() - last_sample_time).seconds() < 5.0) {
    return;
  }
  last_sample_time = now();

  http_client.postValues("battery",
                         {{"percentage", msg.percentage},
                          {"current", msg.current},
                          {"power_supply_status", msg.power_supply_status},
                          {"voltage", msg.voltage}});
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryMonitoringTelegrafNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
