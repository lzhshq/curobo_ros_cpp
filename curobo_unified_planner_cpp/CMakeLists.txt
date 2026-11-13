cmake_minimum_required(VERSION 3.22)
project(curobo_unified_planner_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(curobo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(unified_planner src/unified_planner_node.cpp)
target_include_directories(unified_planner PRIVATE include)
ament_target_dependencies(unified_planner
  rclcpp rclcpp_action curobo_msgs geometry_msgs sensor_msgs tf2_ros tf2 std_srvs)

install(TARGETS unified_planner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
