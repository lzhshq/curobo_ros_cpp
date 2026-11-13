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

add_library(curobo_execution_adapters SHARED
  src/jtc_adapter.cpp
  src/topic_adapter.cpp
  src/fpc_adapter.cpp)

target_include_directories(curobo_execution_adapters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(curobo_execution_adapters
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_srvs
  std_msgs)

add_executable(trajectory_executor
  src/trajectory_executor.cpp
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

target_link_libraries(trajectory_executor
  curobo_execution_adapters)

install(TARGETS curobo_execution_adapters trajectory_executor
  EXPORT curobo_execution_cpp_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(PROGRAMS tools/fake_jtc.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_jtc)

install(PROGRAMS tools/fake_fpc.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_fpc)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(curobo_execution_cpp_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_srvs
  std_msgs)
ament_package()
