cmake_minimum_required(VERSION 3.8)
project(tf_telegraf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(telegraf_ros_lib REQUIRED)
find_package(nav2_util REQUIRED)

add_executable(tf_telegraf src/tf_telegraf.cpp)
target_include_directories(tf_telegraf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(tf_telegraf PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(tf_telegraf rclcpp::rclcpp tf2_ros::tf2_ros telegraf_ros_lib::telegraf_ros_lib nav2_util::nav2_util_core)

install(TARGETS tf_telegraf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
