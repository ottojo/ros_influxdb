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

/// Function for curl to write response body into std::string (see CURLOPT_WRITEFUNCTION)
size_t writeFunction(void *ptr, size_t size, size_t nmemb, std::string *data) {
  data->append((char *)ptr, size * nmemb);
  return size * nmemb;
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

template <typename... Ts> struct Overload : Ts... {
  using Ts::operator()...;
};
template <class... Ts> Overload(Ts...) -> Overload<Ts...>;

void TelegrafHttpClient::postValues(
    const std::string &name, std::map<std::string, Value> data,
    const std::map<std::string, std::string> &tags) {

  std::string post_data = std::string("{\"name\": \"") + name + "\"";
  for (const auto &[key, value] : data) {

    // Special case for string values: add "string_" prefix to key.
    // This requires json_string_fields = ["string_*"] in the
    // [[inputs.http_listener_v2]] telegraf config
    post_data += std::visit(
        Overload{[&key](const std::string &v) -> std::string {
                   return ", \"string_" + key + "\": ";
                 },
                 [&key](auto) -> std::string { return ", \"" + key + "\": "; }},
        value);

    // Special case for string: add quotes, also there is no
    // std::to_string(std::string)
    post_data += std::visit(
        Overload{
            [](const std::string &v) -> std::string { return "\"" + v + "\""; },
            [](auto v) -> std::string { return std::to_string(v); }},
        value);
  }

  for (const auto &[tag_name, tag_value] : tags) {
    post_data += ", \"tag_" + tag_name + "\": \"" + tag_value + "\"";
  }

  post_data += "}";

  CURLcode res = CURLE_OK;
  struct curl_slist *headers = NULL;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  res = curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  handle_curl_error(res, "setting content type", logger);
  res = curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());
  handle_curl_error(res, "setting POST data", logger);

  std::string response_string;
  res = curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeFunction);
  handle_curl_error(res, "setting body write function", logger);
  res = curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
  handle_curl_error(res, "setting body write data", logger);

  res = curl_easy_perform(curl);
  handle_curl_error(res, "executing POST request", logger);

  if (res == CURLE_OK) {
    long response_code = 0;
    res = curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
    handle_curl_error(res, "getting request information", logger);
    if (response_code != 204) {
      RCLCPP_ERROR(logger,
                   "Sending data to telegraf returned HTTP response code %ld. "
                   "Response: %s",
                   response_code, response_string.c_str());
    }
  }

  curl_slist_free_all(headers);
}

} // namespace telegraf_ros_lib