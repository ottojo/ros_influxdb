#include <curl/curl.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
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

class BoolTelegrafNode : public rclcpp::Node {
public:
  BoolTelegrafNode()
      : rclcpp::Node("bool_telegraf"), curl_error_buffer(CURL_ERROR_SIZE, ' '),
        curl{curl_easy_init()}, last_sample_time{now()},
        bool_sub{create_subscription<std_msgs::msg::Bool>(
            "/bool_topic", 1,
            [this](const std_msgs::msg::Bool &msg) { topicCallback(msg); })} {
    if (curl == nullptr) {
      throw std::runtime_error{"Initializing CURL failed!"};
    }
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_error_buffer.data());
    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:8080/telegraf");

    declare_parameter("measurement_name", "bool");
    declare_parameter("field_name", "value");
  };
  ~BoolTelegrafNode() { curl_easy_cleanup(curl); }

private:
  std::vector<char> curl_error_buffer;
  CURL *curl;
  rclcpp::Time last_sample_time;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub;

  void topicCallback(const std_msgs::msg::Bool &msg);
};

void BoolTelegrafNode::topicCallback(const std_msgs::msg::Bool &msg) {

  if ((now() - last_sample_time).seconds() < 5.0) {
    return;
  }
  last_sample_time = now();

  std::string post_data = telegraf_ros_lib::build_json_body(
      get_parameter("measurement_name").as_string(),
      {{get_parameter("field_name").as_string(),
        std::to_string(static_cast<int>(msg.data))}});

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
  auto node = std::make_shared<BoolTelegrafNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
