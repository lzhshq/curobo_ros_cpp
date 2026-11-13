cmake_minimum_required(VERSION 3.22)
project(curobo_mpc_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(curobo_core_cpp REQUIRED)
find_package(curobo_execution_cpp REQUIRED)
find_package(curobo_planner_interfaces REQUIRED)
find_package(curobo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(CUDAToolkit 12.3 REQUIRED)

set(CUROBO_RUNTIME_ROOT "" CACHE PATH
  "Path to the cuRobo cpp_runtime checkout (contains build/robot and robot/include)")
if(NOT CUROBO_RUNTIME_ROOT)
  message(FATAL_ERROR "CUROBO_RUNTIME_ROOT is required")
endif()

find_library(CUROBO_ROBOT_RUNTIME
  NAMES curobo_robot_runtime
  PATHS "${CUROBO_RUNTIME_ROOT}/build/robot"
  NO_DEFAULT_PATH
  REQUIRED)

add_executable(mpc_node src/mpc_node.cpp)

add_executable(mpc_loop_spike examples/mpc_loop_spike.cpp)
target_include_directories(mpc_loop_spike PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/examples"
  "${CUROBO_RUNTIME_ROOT}/robot/include")
target_link_libraries(mpc_loop_spike PRIVATE
  curobo_core_cpp::curobo_core_cpp
  "${CUROBO_ROBOT_RUNTIME}"
  CUDA::cudart)
set_target_properties(mpc_loop_spike PROPERTIES
  BUILD_RPATH "${CUROBO_RUNTIME_ROOT}/build/robot")
target_include_directories(mpc_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CUROBO_RUNTIME_ROOT}/robot/include")
ament_target_dependencies(mpc_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  curobo_planner_interfaces curobo_msgs tf2 visualization_msgs nav_msgs)
target_link_libraries(mpc_node
  curobo_core_cpp::curobo_core_cpp
  curobo_execution_cpp::curobo_execution_adapters
  "${CUROBO_ROBOT_RUNTIME}"
  CUDA::cudart)
set_target_properties(mpc_node PROPERTIES
  BUILD_RPATH "${CUROBO_RUNTIME_ROOT}/build/robot")

install(TARGETS mpc_node DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS tools/bat_marker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bat_marker)

install(PROGRAMS tools/target_cycle.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_cycle)

install(PROGRAMS tools/mpc_regression.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mpc_regression)

install(PROGRAMS tools/unified_circle_target.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME unified_circle_target)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
