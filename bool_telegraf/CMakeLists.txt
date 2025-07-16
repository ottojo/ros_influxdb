cmake_minimum_required(VERSION 3.8)
project(bool_telegraf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(telegraf_ros_lib REQUIRED)

add_executable(bool_telegraf src/bool_telegraf.cpp)
target_include_directories(bool_telegraf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(bool_telegraf PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(bool_telegraf rclcpp::rclcpp ${std_msgs_TARGETS} telegraf_ros_lib::telegraf_ros_lib)

install(TARGETS bool_telegraf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
