cmake_minimum_required(VERSION 3.22)
project(curobo_planner_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(curobo_planner_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(curobo_core_cpp REQUIRED)

add_executable(planner_node
  src/planner_node.cpp
  src/main.cpp)

target_include_directories(planner_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")

ament_target_dependencies(planner_node
  rclcpp
  curobo_planner_interfaces
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  trajectory_msgs)

target_link_libraries(planner_node
  curobo_core_cpp::curobo_core_cpp)

install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
