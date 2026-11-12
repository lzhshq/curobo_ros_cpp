// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include "curobo_core_cpp/planner_core.hpp"

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "curobo_robot_runtime/api.h"
#include "curobo_robot_runtime/kinematics.h"
#include "curobo_robot_runtime/motion_planner.h"
#include "curobo_robot_runtime/scene.h"

namespace curobo_ros_cpp {
namespace {

void check_cuda(cudaError_t status, const char* where) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(where) + ": " + cudaGetErrorString(status));
  }
}

template <typename T>
T* alloc_device(std::size_t count) {
  if (count == 0) {
    return nullptr;
  }
  T* pointer = nullptr;
  check_cuda(cudaMalloc(reinterpret_cast<void**>(&pointer), count * sizeof(T)),
             "cudaMalloc");
  return pointer;
}

template <typename T>
void copy_to_device(T* destination, const std::vector<T>& source, const char* where) {
  if (source.empty()) {
    return;
  }
  check_cuda(cudaMemcpy(destination, source.data(), source.size() * sizeof(T),
                        cudaMemcpyHostToDevice),
             where);
}

template <typename T>
void copy_from_device(std::vector<T>& destination, const T* source, std::size_t count,
                      const char* where) {
  destination.resize(count);
  if (count == 0) {
    return;
  }
  check_cuda(cudaMemcpy(destination.data(), source, count * sizeof(T),
                        cudaMemcpyDeviceToHost),
             where);
}

std::string join_names(const std::vector<std::string>& names) {
  std::string result;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i != 0) {
      result += ", ";
    }
    result += names[i];
  }
  return result;
}

}  // namespace

struct PlannerCore::Impl {
  explicit Impl(PlannerConfig input) : config(std::move(input)) {}
  ~Impl() {
    if (d_goal_pos != nullptr) cudaFree(d_goal_pos);
    if (d_goal_quat != nullptr) cudaFree(d_goal_quat);
    if (d_current != nullptr) cudaFree(d_current);
    if (d_current_velocity != nullptr) cudaFree(d_current_velocity);
    if (d_current_acceleration != nullptr) cudaFree(d_current_acceleration);
    if (d_knots != nullptr) cudaFree(d_knots);
    if (d_traj_pos != nullptr) cudaFree(d_traj_pos);
    if (d_traj_vel != nullptr) cudaFree(d_traj_vel);
    if (d_traj_acc != nullptr) cudaFree(d_traj_acc);
    if (d_traj_jerk != nullptr) cudaFree(d_traj_jerk);
    if (d_selected_goal != nullptr) cudaFree(d_selected_goal);
    if (d_success != nullptr) cudaFree(d_success);
    if (d_cost != nullptr) cudaFree(d_cost);
    if (d_collision_cost != nullptr) cudaFree(d_collision_cost);
    if (d_dt != nullptr) cudaFree(d_dt);
    if (d_position_error != nullptr) cudaFree(d_position_error);
    if (d_orientation_error != nullptr) cudaFree(d_orientation_error);
    if (d_used_rrt != nullptr) cudaFree(d_used_rrt);
    if (d_rrt_iterations != nullptr) cudaFree(d_rrt_iterations);
    if (d_selected_seed != nullptr) cudaFree(d_selected_seed);
    if (d_planning_attempts != nullptr) cudaFree(d_planning_attempts);
  }

  PlannerConfig config;
  std::unique_ptr<curobo::robot::Scene> scene;
  std::unique_ptr<curobo::robot::MotionPlanner> planner;
  std::unique_ptr<curobo::robot::Kinematics> kinematics;

  CuroboRobotInfo robot_info{};
  curobo::robot::MotionPlanningInfo planning_info{};
  std::vector<std::string> joint_names;
  std::vector<std::string> tool_frames;
  int tool_index = 0;
  bool initialized = false;

  int num_dof = 0;
  int num_knots = 0;
  int num_steps = 0;
  int num_tool_frames = 0;

  float* d_goal_pos = nullptr;
  float* d_goal_quat = nullptr;
  float* d_current = nullptr;
  float* d_current_velocity = nullptr;
  float* d_current_acceleration = nullptr;
  float* d_knots = nullptr;
  float* d_traj_pos = nullptr;
  float* d_traj_vel = nullptr;
  float* d_traj_acc = nullptr;
  float* d_traj_jerk = nullptr;
  float* d_selected_goal = nullptr;
  uint8_t* d_success = nullptr;
  float* d_cost = nullptr;
  float* d_collision_cost = nullptr;
  float* d_dt = nullptr;
  float* d_position_error = nullptr;
  float* d_orientation_error = nullptr;
  uint8_t* d_used_rrt = nullptr;
  int32_t* d_rrt_iterations = nullptr;
  int32_t* d_selected_seed = nullptr;
  int32_t* d_planning_attempts = nullptr;
};

PlannerCore::PlannerCore(PlannerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

PlannerCore::~PlannerCore() = default;

bool PlannerCore::initialize(std::string& error) {
  try {
    if (impl_->config.artifact_dir.empty()) {
      throw std::runtime_error("artifact_dir is empty");
    }

    impl_->kinematics =
        std::make_unique<curobo::robot::Kinematics>(impl_->config.artifact_dir);
    impl_->robot_info = impl_->kinematics->info();
    impl_->num_dof = impl_->robot_info.num_dof;
    impl_->num_tool_frames = impl_->robot_info.num_tool_frames;
    if (impl_->num_dof <= 0 || impl_->num_tool_frames <= 0) {
      throw std::runtime_error("artifact reports an invalid robot shape");
    }

    impl_->joint_names.reserve(impl_->num_dof);
    for (int i = 0; i < impl_->num_dof; ++i) {
      impl_->joint_names.push_back(impl_->kinematics->joint_name(i));
    }
    impl_->tool_frames.reserve(impl_->num_tool_frames);
    for (int i = 0; i < impl_->num_tool_frames; ++i) {
      impl_->tool_frames.push_back(impl_->kinematics->tool_frame_name(i));
    }

    impl_->tool_index = 0;
    if (!impl_->config.tool_frame.empty()) {
      bool found = false;
      for (int i = 0; i < impl_->num_tool_frames; ++i) {
        if (impl_->tool_frames[static_cast<std::size_t>(i)] == impl_->config.tool_frame) {
          impl_->tool_index = i;
          found = true;
          break;
        }
      }
      if (!found) {
        throw std::runtime_error("tool_frame '" + impl_->config.tool_frame +
                                 "' is not in the artifact (available: " +
                                 join_names(impl_->tool_frames) + ")");
      }
    }

    curobo::robot::SceneConfig scene_config;
    scene_config.max_cuboids = impl_->config.max_cuboids;
    scene_config.max_capsules = impl_->config.max_capsules;
    scene_config.max_cylinders = impl_->config.max_cylinders;
    scene_config.max_meshes = impl_->config.max_meshes;
    scene_config.max_voxel_grids = impl_->config.max_voxel_grids;
    scene_config.max_voxels_per_grid = impl_->config.max_voxels_per_grid;
    impl_->scene = std::make_unique<curobo::robot::Scene>(scene_config);

    curobo::robot::MotionPlanningConfig planner_config;
    planner_config.use_cuda_graph = true;
    impl_->planner = std::make_unique<curobo::robot::MotionPlanner>(
        impl_->config.artifact_dir, planner_config, impl_->scene.get());

    impl_->planning_info = impl_->planner->info();
    impl_->num_knots = impl_->planning_info.num_knots;
    impl_->num_steps = impl_->planning_info.trajectory_steps;
    if (impl_->num_knots <= 0 || impl_->num_steps <= 0) {
      throw std::runtime_error("artifact does not contain a planning layout");
    }

    impl_->d_goal_pos = alloc_device<float>(
        static_cast<std::size_t>(impl_->num_tool_frames) * 3);
    impl_->d_goal_quat = alloc_device<float>(
        static_cast<std::size_t>(impl_->num_tool_frames) * 4);
    impl_->d_current = alloc_device<float>(impl_->num_dof);
    impl_->d_current_velocity = alloc_device<float>(impl_->num_dof);
    impl_->d_current_acceleration = alloc_device<float>(impl_->num_dof);
    impl_->d_knots = alloc_device<float>(
        static_cast<std::size_t>(impl_->num_knots) * impl_->num_dof);
    impl_->d_traj_pos = alloc_device<float>(
        static_cast<std::size_t>(impl_->num_steps) * impl_->num_dof);
    impl_->d_traj_vel = alloc_device<float>(
        static_cast<std::size_t>(impl_->num_steps) * impl_->num_dof);
    impl_->d_traj_acc = alloc_device<float>(
        static_cast<std::size_t>(impl_->num_steps) * impl_->num_dof);
    impl_->d_traj_jerk = alloc_device<float>(
        static_cast<std::size_t>(impl_->num_steps) * impl_->num_dof);
    impl_->d_selected_goal = alloc_device<float>(impl_->num_dof);

    impl_->d_success = alloc_device<uint8_t>(1);
    impl_->d_cost = alloc_device<float>(1);
    impl_->d_collision_cost = alloc_device<float>(1);
    impl_->d_dt = alloc_device<float>(1);
    impl_->d_position_error = alloc_device<float>(1);
    impl_->d_orientation_error = alloc_device<float>(1);
    impl_->d_used_rrt = alloc_device<uint8_t>(1);
    impl_->d_rrt_iterations = alloc_device<int32_t>(1);
    impl_->d_selected_seed = alloc_device<int32_t>(1);
    impl_->d_planning_attempts = alloc_device<int32_t>(1);

    check_cuda(cudaDeviceSynchronize(), "cudaDeviceSynchronize after initialize");
    impl_->initialized = true;
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

PlanResult PlannerCore::plan_pose(const PlanRequest& request) {
  PlanResult result;
  result.num_dof = impl_->num_dof;
  result.num_steps = impl_->num_steps;

  if (!impl_->initialized) {
    result.message = "PlannerCore is not initialized";
    return result;
  }
  if (request.start_position.size() != static_cast<std::size_t>(impl_->num_dof)) {
    result.message = "start_position must contain " + std::to_string(impl_->num_dof) +
                     " values";
    return result;
  }
  if (request.start_velocity.size() != 0 &&
      request.start_velocity.size() != static_cast<std::size_t>(impl_->num_dof)) {
    result.message = "start_velocity must be empty or contain " +
                     std::to_string(impl_->num_dof) + " values";
    return result;
  }
  if (request.start_acceleration.size() != 0 &&
      request.start_acceleration.size() != static_cast<std::size_t>(impl_->num_dof)) {
    result.message = "start_acceleration must be empty or contain " +
                     std::to_string(impl_->num_dof) + " values";
    return result;
  }

  try {
    const auto started = std::chrono::steady_clock::now();

    std::vector<float> goal_positions(
        static_cast<std::size_t>(impl_->num_tool_frames) * 3, 0.0F);
    std::vector<float> goal_quaternions(
        static_cast<std::size_t>(impl_->num_tool_frames) * 4, 0.0F);
    for (int i = 0; i < impl_->num_tool_frames; ++i) {
      goal_positions[static_cast<std::size_t>(i) * 3 + 0] = request.position[0];
      goal_positions[static_cast<std::size_t>(i) * 3 + 1] = request.position[1];
      goal_positions[static_cast<std::size_t>(i) * 3 + 2] = request.position[2];
      goal_quaternions[static_cast<std::size_t>(i) * 4 + 0] = request.quaternion[0];
      goal_quaternions[static_cast<std::size_t>(i) * 4 + 1] = request.quaternion[1];
      goal_quaternions[static_cast<std::size_t>(i) * 4 + 2] = request.quaternion[2];
      goal_quaternions[static_cast<std::size_t>(i) * 4 + 3] = request.quaternion[3];
    }

    copy_to_device(impl_->d_goal_pos, goal_positions, "copy goal position");
    copy_to_device(impl_->d_goal_quat, goal_quaternions, "copy goal quaternion");
    copy_to_device(impl_->d_current, request.start_position, "copy start position");

    const std::vector<float> zero_velocity(static_cast<std::size_t>(impl_->num_dof), 0.0F);
    const std::vector<float>& velocity =
        request.start_velocity.empty() ? zero_velocity : request.start_velocity;
    const std::vector<float>& acceleration =
        request.start_acceleration.empty() ? zero_velocity : request.start_acceleration;
    copy_to_device(impl_->d_current_velocity, velocity, "copy start velocity");
    copy_to_device(impl_->d_current_acceleration, acceleration, "copy start acceleration");

    check_cuda(cudaMemset(impl_->d_success, 0, sizeof(uint8_t)), "memset success");
    check_cuda(cudaMemset(impl_->d_dt, 0, sizeof(float)), "memset dt");
    check_cuda(cudaMemset(impl_->d_position_error, 0, sizeof(float)), "memset pos error");
    check_cuda(cudaMemset(impl_->d_orientation_error, 0, sizeof(float)),
               "memset orientation error");
    check_cuda(cudaMemset(impl_->d_planning_attempts, 0, sizeof(int32_t)),
               "memset attempts");
    check_cuda(cudaMemset(impl_->d_used_rrt, 0, sizeof(uint8_t)), "memset used_rrt");

    curobo::robot::PoseMotionPlanningRequest pose_request;
    pose_request.goal_positions = impl_->d_goal_pos;
    pose_request.goal_quaternions = impl_->d_goal_quat;
    pose_request.current_position = impl_->d_current;
    pose_request.current_velocity = impl_->d_current_velocity;
    pose_request.current_acceleration = impl_->d_current_acceleration;
    pose_request.selected_goal_position = impl_->d_selected_goal;
    pose_request.optimized_knots = impl_->d_knots;
    pose_request.trajectory_position = impl_->d_traj_pos;
    pose_request.trajectory_velocity = impl_->d_traj_vel;
    pose_request.trajectory_acceleration = impl_->d_traj_acc;
    pose_request.trajectory_jerk = impl_->d_traj_jerk;
    pose_request.success = impl_->d_success;
    pose_request.cost = impl_->d_cost;
    pose_request.collision_cost = impl_->d_collision_cost;
    pose_request.trajectory_dt = impl_->d_dt;
    pose_request.position_error = impl_->d_position_error;
    pose_request.orientation_error = impl_->d_orientation_error;
    pose_request.used_rrt = impl_->d_used_rrt;
    pose_request.rrt_iterations = impl_->d_rrt_iterations;
    pose_request.selected_seed = impl_->d_selected_seed;
    pose_request.planning_attempts = impl_->d_planning_attempts;
    pose_request.max_attempts = impl_->config.max_attempts;
    pose_request.enable_graph_attempt = impl_->config.enable_graph_attempt;
    pose_request.outer_iterations = impl_->config.outer_iterations;
    pose_request.finetune_attempts = impl_->config.finetune_attempts;
    pose_request.finetune_dt_scale = impl_->config.finetune_dt_scale;

    impl_->planner->plan_pose(pose_request);
    check_cuda(cudaDeviceSynchronize(), "cudaDeviceSynchronize after plan_pose");

    uint8_t success = 0;
    check_cuda(cudaMemcpy(&success, impl_->d_success, sizeof(uint8_t),
                          cudaMemcpyDeviceToHost),
               "copy success");
    check_cuda(cudaMemcpy(&result.dt, impl_->d_dt, sizeof(float), cudaMemcpyDeviceToHost),
               "copy dt");
    check_cuda(cudaMemcpy(&result.position_error, impl_->d_position_error, sizeof(float),
                          cudaMemcpyDeviceToHost),
               "copy position error");
    check_cuda(cudaMemcpy(&result.orientation_error, impl_->d_orientation_error,
                          sizeof(float), cudaMemcpyDeviceToHost),
               "copy orientation error");
    check_cuda(cudaMemcpy(&result.planning_attempts, impl_->d_planning_attempts,
                          sizeof(int32_t), cudaMemcpyDeviceToHost),
               "copy planning attempts");
    uint8_t used_rrt = 0;
    check_cuda(cudaMemcpy(&used_rrt, impl_->d_used_rrt, sizeof(uint8_t),
                          cudaMemcpyDeviceToHost),
               "copy used_rrt");

    result.success = (success != 0);
    result.used_rrt = (used_rrt != 0);
    result.planning_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();

    if (!result.success) {
      result.message = "plan_pose failed (position_error=" +
                       std::to_string(result.position_error) +
                       ", orientation_error=" + std::to_string(result.orientation_error) +
                       ")";
      return result;
    }

    // timeopt_trajectory() exposes the executable horizon; the raw request
    // buffers are fixed-capacity and contain padding after sample_count.
    const auto timeopt = impl_->planner->timeopt_trajectory();
    int32_t sample_count = 0;
    if (timeopt.sample_count != nullptr) {
      check_cuda(cudaMemcpy(&sample_count, timeopt.sample_count, sizeof(int32_t),
                            cudaMemcpyDeviceToHost),
                 "copy timeopt sample_count");
    }
    if (sample_count <= 0 || sample_count > impl_->num_steps ||
        timeopt.position == nullptr || timeopt.velocity == nullptr) {
      sample_count = impl_->num_steps;
      const std::size_t samples =
          static_cast<std::size_t>(sample_count) * impl_->num_dof;
      copy_from_device(result.position, impl_->d_traj_pos, samples,
                       "copy trajectory position");
      copy_from_device(result.velocity, impl_->d_traj_vel, samples,
                       "copy trajectory velocity");
      copy_from_device(result.acceleration, impl_->d_traj_acc, samples,
                       "copy trajectory acceleration");
    } else {
      const std::size_t samples =
          static_cast<std::size_t>(sample_count) * impl_->num_dof;
      copy_from_device(result.position, timeopt.position, samples,
                       "copy timeopt position");
      copy_from_device(result.velocity, timeopt.velocity, samples,
                       "copy timeopt velocity");
      if (timeopt.acceleration != nullptr) {
        copy_from_device(result.acceleration, timeopt.acceleration, samples,
                         "copy timeopt acceleration");
      }
      if (timeopt.dt != nullptr) {
        check_cuda(cudaMemcpy(&result.dt, timeopt.dt, sizeof(float),
                              cudaMemcpyDeviceToHost),
                   "copy timeopt dt");
      }
    }
    result.num_steps = sample_count;
    result.message = "plan_pose succeeded (attempts=" +
                     std::to_string(result.planning_attempts) +
                     ", rrt=" + (result.used_rrt ? "true" : "false") + ")";
    return result;
  } catch (const std::exception& exception) {
    result.success = false;
    result.message = std::string("plan_pose exception: ") + exception.what();
    return result;
  }
}

const std::vector<std::string>& PlannerCore::joint_names() const {
  return impl_->joint_names;
}

const std::vector<std::string>& PlannerCore::tool_frames() const {
  return impl_->tool_frames;
}

int PlannerCore::num_dof() const { return impl_->num_dof; }

int PlannerCore::num_steps() const { return impl_->num_steps; }

}  // namespace curobo_ros_cpp
