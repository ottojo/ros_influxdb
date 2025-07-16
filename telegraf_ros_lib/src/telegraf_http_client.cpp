#include "telegraf_ros_lib/telegraf_http_client.hpp"
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <variant>

namespace telegraf_ros_lib {

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

CURL *init_curl() {
  curl_global_init(CURL_GLOBAL_DEFAULT);
  return curl_easy_init();
}

TelegrafHttpClient::TelegrafHttpClient(rclcpp::Logger logger)
    : logger{logger}, curl_error_buffer(CURL_ERROR_SIZE, ' '),
      curl{init_curl()} {
  if (curl == nullptr) {
    throw std::runtime_error{"Initializing CURL failed!"};
  }
  CURLcode res = CURLE_OK;

  res = curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_error_buffer.data());
  handle_curl_error(res, "setting error buffer", logger, true);
  res = curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:8080/telegraf");
  handle_curl_error(res, "setting url", logger, true);
}

TelegrafHttpClient::~TelegrafHttpClient() { curl_easy_cleanup(curl); }

void TelegrafHttpClient::postValues(const std::string &name,
                                    std::map<std::string, Value> data) {

  std::string post_data = std::string("{\"name\": \"") + name + "\"";
  for (const auto &[key, value] : data) {
    post_data += ", \"" + key + "\": ";
    if (std::holds_alternative<int>(value)) {
      post_data += std::to_string(std::get<int>(value));
    } else if (std::holds_alternative<double>(value)) {
      post_data += std::to_string(std::get<double>(value));
    } else {
      throw std::runtime_error{"Value type not implemented."};
    }
  }
  post_data += "}";

  CURLcode res = CURLE_OK;
  struct curl_slist *headers = NULL;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  res = curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  handle_curl_error(res, "setting content type", logger);
  res = curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());
  handle_curl_error(res, "setting POST data", logger);
  res = curl_easy_perform(curl);
  handle_curl_error(res, "executing POST request", logger);
  curl_slist_free_all(headers);
}

} // namespace telegraf_ros_lib