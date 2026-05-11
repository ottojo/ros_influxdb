#include <cstdio>
#include <cstring>
#include <ifaddrs.h>
#include <libsocketcan.h>
#include <linux/can/netlink.h>
#include <linux/if_arp.h>
#include <linux/if_link.h>
#include <netinet/in.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <telegraf_ros_lib/telegraf_http_client.hpp>

std::vector<std::string> getCanInterfaces() {
  std::vector<std::string> interface_names;

  struct ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1) {
    throw std::runtime_error{strerror(errno)};
  }

  // This socket is used for ioctl, i don't think the options matter
  int socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (socket_fd == -1) {
    throw std::runtime_error{strerror(errno)};
  }

  for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    ifreq ifr;
    strcpy(ifr.ifr_name, ifa->ifa_name);

    if (ioctl(socket_fd, SIOCGIFHWADDR, &ifr) == -1) {
      throw std::runtime_error{strerror(errno)};
    }

    if (ifr.ifr_hwaddr.sa_family != ARPHRD_CAN) {
      continue;
    }

    interface_names.push_back(std::string(ifa->ifa_name));
  }

  freeifaddrs(ifaddr);
  return interface_names;
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node("can_interface_telegraf");
  telegraf_ros_lib::TelegrafHttpClient http_client{
      node.get_logger().get_child("http_client")};

  auto logger = node.get_logger();

  rclcpp::Rate rate(1.0, node.get_clock());

  while (rclcpp::ok()) {
    auto interfaces = getCanInterfaces();
    if (interfaces.empty()) {
      RCLCPP_WARN(logger, "No CAN interfaces found.");
    }
    for (const auto &if_name : interfaces) {
      RCLCPP_DEBUG(logger, "Getting stats for interface %s", if_name.c_str());

      rtnl_link_stats64 link_stats;
      if (can_get_link_stats(if_name.c_str(), &link_stats) != 0) {
        RCLCPP_WARN(logger, "Error during can_get_link_stats for interface %s",
                    if_name.c_str());
        continue;
      }

      can_device_stats device_stats;
      if (can_get_device_stats(if_name.c_str(), &device_stats) != 0) {
        RCLCPP_WARN(logger,
                    "Error during can_get_device_stats for interface %s",
                    if_name.c_str());
        continue;
      }

      http_client.postValues(
          "can_interface_stats",
          {
              {"rx_packets", link_stats.rx_packets},
              {"tx_packets", link_stats.tx_packets},
              {"rx_errors", link_stats.rx_errors},
              {"tx_errors", link_stats.tx_errors},
              {"rx_dropped", link_stats.rx_dropped},
              {"tx_dropped", link_stats.tx_dropped},
              {"collisions", link_stats.collisions},
              {"bus_error", device_stats.bus_error},
              {"error_warning", device_stats.error_warning},
              {"error_passive", device_stats.error_passive},
              {"bus_off", device_stats.bus_off},
              {"arbitration_lost", device_stats.arbitration_lost},
              {"restarts", device_stats.restarts},
          },
          {
              {"interface_name", if_name},
          });
    }
    rate.sleep();
  }
  rclcpp::shutdown();
}
