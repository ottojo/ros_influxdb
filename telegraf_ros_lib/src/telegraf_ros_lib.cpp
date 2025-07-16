#include "telegraf_ros_lib/telegraf_ros_lib.hpp"
namespace telegraf_ros_lib {

std::string build_json_body(std::string name,
                            std::map<std::string, std::string> data) {
  std::string post_data = std::string("{\"name\": \"") + name + "\"";
  for (const auto &[key, value] : data) {
    post_data += ", \"" + key + "\": " + value;
  }
  post_data += "}";
  return post_data;
}

} // namespace telegraf_ros_lib