cmake_minimum_required(VERSION 3.20)
project(can_interface_telegraf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(telegraf_ros_lib REQUIRED)
find_library(LIBSOCKETCAN socketcan REQUIRED)

add_executable(can_interface_telegraf src/can_interface_telegraf.cpp)
target_include_directories(can_interface_telegraf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(can_interface_telegraf PUBLIC c_std_99 cxx_std_20)
target_link_libraries(can_interface_telegraf telegraf_ros_lib::telegraf_ros_lib rclcpp::rclcpp ${LIBSOCKETCAN})

install(TARGETS can_interface_telegraf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
