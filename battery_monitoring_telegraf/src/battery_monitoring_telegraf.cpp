#include <curl/curl.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <stdexcept>
#include <string>
#include <telegraf_ros_lib/telegraf_ros_lib.hpp>

void handle_curl_error(CURLcode code, const char *context,
                       rclcpp::Logger logger, bool fatal = false) {
  if (code != CURLE_OK) {
    std::string error_message = std::string("CURL error during ") + context +
                                ": " + curl_easy_strerror(code);
    if (fatal) {
      throw std::runtime_error{error_message};
    } else {
      RCLCPP_ERROR(logger, "%s", error_message.c_str());
    }
  }
}

class BatteryMonitoringTelegrafNode : public rclcpp::Node {
public:
  BatteryMonitoringTelegrafNode()
      : rclcpp::Node("battery_monitoring_telegraf"),
        curl_error_buffer(CURL_ERROR_SIZE, ' '), curl{curl_easy_init()},
        last_sample_time{now()},
        battery_state_sub{create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", 1,
            [this](const sensor_msgs::msg::BatteryState &msg) {
              batteryStateCallback(msg);
            })} {
    if (curl == nullptr) {
      throw std::runtime_error{"Initializing CURL failed!"};
    }
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_error_buffer.data());
    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:8080/telegraf");
    // curl_easy_setopt(curl, CURLOPT_URL, "http://httpbin.org/post");
  };
  ~BatteryMonitoringTelegrafNode() { curl_easy_cleanup(curl); }

private:
  std::vector<char> curl_error_buffer;
  CURL *curl;
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

  std::string post_data = telegraf_ros_lib::build_json_body(
      "battery",
      {{"percentage", std::to_string(msg.percentage)},
       {"current", std::to_string(msg.current)},
       {"power_supply_status", std::to_string(msg.power_supply_status)},
       {"voltage", std::to_string(msg.voltage)}});

  CURLcode res = CURLE_OK;

  struct curl_slist *headers = NULL;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  res = curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  handle_curl_error(res, "setting content type", get_logger());
  res = curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());
  handle_curl_error(res, "setting POST data", get_logger());
  res = curl_easy_perform(curl);
  handle_curl_error(res, "executing POST request", get_logger());
  curl_slist_free_all(headers);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  curl_global_init(CURL_GLOBAL_DEFAULT);
  auto node = std::make_shared<BatteryMonitoringTelegrafNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
