cmake_minimum_required(VERSION 3.22)
project(curobo_execution_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(trajectory_executor
  src/trajectory_executor.cpp
  src/jtc_adapter.cpp
  src/topic_adapter.cpp
  src/fpc_adapter.cpp
  src/main.cpp)

target_include_directories(trajectory_executor PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")

ament_target_dependencies(trajectory_executor
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_srvs
  std_msgs)

install(TARGETS trajectory_executor
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS tools/fake_jtc.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_jtc)

install(PROGRAMS tools/fake_fpc.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_fpc)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
