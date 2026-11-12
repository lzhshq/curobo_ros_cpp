cmake_minimum_required(VERSION 3.22)
project(curobo_core_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit 12.3 REQUIRED)

set(CUROBO_RUNTIME_ROOT "" CACHE PATH
  "Path to the cuRobo cpp_runtime checkout (contains build/robot and robot/include)")

if(NOT CUROBO_RUNTIME_ROOT)
  message(FATAL_ERROR
    "CUROBO_RUNTIME_ROOT is required, e.g. -DCUROBO_RUNTIME_ROOT=/path/to/curobo-fast/cpp_runtime")
endif()

find_library(CUROBO_ROBOT_RUNTIME
  NAMES curobo_robot_runtime
  PATHS "${CUROBO_RUNTIME_ROOT}/build/robot"
  NO_DEFAULT_PATH
  REQUIRED)

add_library(curobo_core_cpp SHARED
  src/planner_core.cpp)

target_include_directories(curobo_core_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    "${CUROBO_RUNTIME_ROOT}/robot/include")

target_link_libraries(curobo_core_cpp
  PUBLIC
    "${CUROBO_ROBOT_RUNTIME}"
  PRIVATE
    CUDA::cudart)

set_target_properties(curobo_core_cpp PROPERTIES
  BUILD_RPATH "${CUROBO_RUNTIME_ROOT}/build/robot"
  INSTALL_RPATH "${CUROBO_RUNTIME_ROOT}/build/robot")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  target_include_directories(test_planner_core PRIVATE
    "${CUROBO_RUNTIME_ROOT}/robot/include")
  target_link_libraries(test_planner_core
    curobo_core_cpp
    "${CUROBO_ROBOT_RUNTIME}"
    CUDA::cudart)
  set_target_properties(test_planner_core PROPERTIES
    BUILD_RPATH "${CUROBO_RUNTIME_ROOT}/build/robot")
endif()

option(BUILD_EXAMPLES "Build curobo_core_cpp examples" ON)
if(BUILD_EXAMPLES)
  add_executable(plan_pose_smoke examples/plan_pose_smoke.cpp)
  add_executable(scene_smoke examples/scene_smoke.cpp)
  target_include_directories(plan_pose_smoke PRIVATE
    "${CUROBO_RUNTIME_ROOT}/robot/include")
  target_link_libraries(plan_pose_smoke PRIVATE
    curobo_core_cpp
    "${CUROBO_ROBOT_RUNTIME}"
    CUDA::cudart)
  set_target_properties(plan_pose_smoke PROPERTIES
    BUILD_RPATH "${CUROBO_RUNTIME_ROOT}/build/robot")

  target_include_directories(scene_smoke PRIVATE
    "${CUROBO_RUNTIME_ROOT}/robot/include")
  target_link_libraries(scene_smoke PRIVATE
    curobo_core_cpp
    "${CUROBO_ROBOT_RUNTIME}"
    CUDA::cudart)
  set_target_properties(scene_smoke PROPERTIES
    BUILD_RPATH "${CUROBO_RUNTIME_ROOT}/build/robot")
endif()

install(TARGETS curobo_core_cpp
  EXPORT curobo_core_cpp_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

ament_export_targets(curobo_core_cpp_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)
ament_package()
