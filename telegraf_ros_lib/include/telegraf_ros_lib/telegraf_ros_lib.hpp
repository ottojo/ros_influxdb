#pragma once
#include <map>
#include <string>

namespace telegraf_ros_lib {

std::string build_json_body(std::string name,
                            std::map<std::string, std::string> data);

}