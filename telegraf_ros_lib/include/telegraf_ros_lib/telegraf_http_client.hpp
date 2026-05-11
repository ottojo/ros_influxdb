#pragma once
#include <curl/curl.h>
#include <map>
#include <rclcpp/logger.hpp>
#include <variant>

namespace telegraf_ros_lib {

using Value = std::variant<int, double, std::uint64_t>;

class TelegrafHttpClient {
public:
  explicit TelegrafHttpClient(rclcpp::Logger logger);
  ~TelegrafHttpClient();

  void postValues(const std::string &name, std::map<std::string, Value> data,
                  const std::map<std::string,std::string> &tags = {});

private:
  rclcpp::Logger logger;
  std::vector<char> curl_error_buffer;
  CURL *curl;
};
} // namespace telegraf_ros_lib
