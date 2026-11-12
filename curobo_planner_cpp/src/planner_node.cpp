// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include "curobo_planner_cpp/planner_node.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace curobo_ros_cpp {
namespace {

std::string join(const std::vector<std::string>& values) {
  std::string result;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      result += ", ";
    }
    result += values[i];
  }
  return result;
}

}  // namespace

PlannerNode::PlannerNode() : rclcpp::Node("curobo_planner") {
  this->declare_parameter<std::string>("artifact_dir", "");
  this->declare_parameter<std::string>("artifact_root", "");
  this->declare_parameter<std::string>("robot", "");
  this->declare_parameter<std::string>("tool_frame", "");
  this->declare_parameter<int>("max_attempts", 10);
  this->declare_parameter<int>("enable_graph_attempt", 1);
  this->declare_parameter<int>("outer_iterations", 2);
  this->declare_parameter<int>("scene_max_cuboids", 8);
  this->declare_parameter<int>("scene_max_capsules", 0);
  this->declare_parameter<int>("scene_max_cylinders", 0);
  this->declare_parameter<int>("scene_max_meshes", 0);
  this->declare_parameter<int>("scene_max_voxel_grids", 0);
  this->declare_parameter<int>("scene_max_voxels_per_grid", 0);
  this->declare_parameter<std::string>("joint_states_topic", "~/joint_states");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<bool>("publish_trajectory", true);
  this->declare_parameter<bool>("node_is_available", false);

  std::string artifact_dir = this->get_parameter("artifact_dir").as_string();
  if (artifact_dir.empty()) {
    const std::string artifact_root = this->get_parameter("artifact_root").as_string();
    const std::string robot = this->get_parameter("robot").as_string();
    if (!artifact_root.empty() && !robot.empty()) {
      artifact_dir = (std::filesystem::path(artifact_root) / robot).string();
    }
  }
  if (artifact_dir.empty()) {
    throw std::runtime_error(
        "artifact_dir is empty; pass artifact_dir:=/path/to/artifact or "
        "artifact_root:=/path/to/artifacts robot:=<name>");
  }

  PlannerConfig config;
  config.artifact_dir = artifact_dir;
  config.tool_frame = this->get_parameter("tool_frame").as_string();
  config.max_attempts = this->get_parameter("max_attempts").as_int();
  config.enable_graph_attempt = this->get_parameter("enable_graph_attempt").as_int();
  config.outer_iterations = this->get_parameter("outer_iterations").as_int();
  config.max_cuboids = this->get_parameter("scene_max_cuboids").as_int();
  config.max_capsules = this->get_parameter("scene_max_capsules").as_int();
  config.max_cylinders = this->get_parameter("scene_max_cylinders").as_int();
  config.max_meshes = this->get_parameter("scene_max_meshes").as_int();
  config.max_voxel_grids = this->get_parameter("scene_max_voxel_grids").as_int();
  config.max_voxels_per_grid = this->get_parameter("scene_max_voxels_per_grid").as_int();

  core_ = std::make_unique<PlannerCore>(config);
  std::string error;
  if (!core_->initialize(error)) {
    throw std::runtime_error("PlannerCore initialization failed: " + error);
  }

  base_frame_ = this->get_parameter("base_frame").as_string();
  publish_trajectory_enabled_ = this->get_parameter("publish_trajectory").as_bool();

  RCLCPP_INFO(this->get_logger(), "Loaded artifact: %s", artifact_dir.c_str());
  RCLCPP_INFO(this->get_logger(), "dof=%d joints=[%s] tools=[%s]", core_->num_dof(),
              join(core_->joint_names()).c_str(), join(core_->tool_frames()).c_str());

  const std::string state_topic = this->get_parameter("joint_states_topic").as_string();
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      state_topic, 10,
      std::bind(&PlannerNode::joint_state_callback, this, std::placeholders::_1));

  plan_srv_ = this->create_service<PlanPose>(
      "~/generate_trajectory",
      std::bind(&PlannerNode::plan_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/planned_trajectory", 10);

  this->set_parameter(rclcpp::Parameter("node_is_available", true));
  RCLCPP_INFO(this->get_logger(), "Ready: ~/generate_trajectory, ~/planned_trajectory");
}

void PlannerNode::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = *message;
  has_state_ = true;
}

bool PlannerNode::reorder_start(const sensor_msgs::msg::JointState& message,
                                const std::vector<std::string>& desired,
                                std::vector<float>& output, std::string& error) const {
  if (message.position.size() != desired.size()) {
    if (message.name.empty()) {
      error = "start_pose.position must contain " + std::to_string(desired.size()) +
              " values when name is empty";
      return false;
    }
  }
  if (message.name.empty()) {
    output.assign(message.position.begin(), message.position.end());
    return true;
  }
  if (message.name.size() != message.position.size()) {
    error = "start_pose.name and start_pose.position have different sizes";
    return false;
  }

  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < message.name.size(); ++i) {
    index.emplace(message.name[i], i);
  }

  output.resize(desired.size());
  for (std::size_t i = 0; i < desired.size(); ++i) {
    const auto it = index.find(desired[i]);
    if (it == index.end()) {
      error = "start_pose is missing joint '" + desired[i] + "'";
      return false;
    }
    output[i] = static_cast<float>(message.position[it->second]);
  }
  return true;
}

void PlannerNode::plan_callback(
    const std::shared_ptr<PlanPose::Request> request,
    std::shared_ptr<PlanPose::Response> response) {
  if (!request->target_poses.empty()) {
    response->success = false;
    response->message = "target_poses is not supported yet; use target_pose";
    return;
  }
  if (!request->target_joint_positions.empty()) {
    response->success = false;
    response->message =
        "target_joint_positions is not supported yet; use target_pose";
    return;
  }

  const double qw = request->target_pose.orientation.w;
  const double qx = request->target_pose.orientation.x;
  const double qy = request->target_pose.orientation.y;
  const double qz = request->target_pose.orientation.z;
  const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (norm < 1e-9) {
    response->success = false;
    response->message = "target_pose.orientation must be a non-zero quaternion (xyzw)";
    return;
  }

  std::vector<float> start;
  std::string error;
  if (!request->start_pose.position.empty()) {
    if (!reorder_start(request->start_pose, core_->joint_names(), start, error)) {
      response->success = false;
      response->message = error;
      return;
    }
  } else {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_state_) {
      response->success = false;
      response->message =
          "no start_pose and no joint state received on the configured topic";
      return;
    }
    if (!reorder_start(latest_state_, core_->joint_names(), start, error)) {
      response->success = false;
      response->message = error;
      return;
    }
  }

  PlanRequest plan_request;
  plan_request.start_position = std::move(start);
  plan_request.position = {static_cast<float>(request->target_pose.position.x),
                           static_cast<float>(request->target_pose.position.y),
                           static_cast<float>(request->target_pose.position.z)};
  plan_request.quaternion = {static_cast<float>(qw / norm), static_cast<float>(qx / norm),
                             static_cast<float>(qy / norm), static_cast<float>(qz / norm)};

  const PlanResult result = core_->plan_pose(plan_request);
  response->success = result.success;
  response->message = result.message;
  response->dt = static_cast<double>(result.dt);

  if (!result.success) {
    RCLCPP_WARN(this->get_logger(), "plan failed: %s", result.message.c_str());
    return;
  }

  const std::size_t dof = static_cast<std::size_t>(result.num_dof);
  const std::size_t steps = static_cast<std::size_t>(result.num_steps);
  if (result.position.size() < steps * dof) {
    response->success = false;
    response->message = "planner returned a short trajectory buffer";
    return;
  }

  response->trajectory.reserve(steps);
  for (std::size_t step = 0; step < steps; ++step) {
    sensor_msgs::msg::JointState waypoint;
    waypoint.header.stamp = this->now();
    waypoint.name = core_->joint_names();
    waypoint.position.resize(dof);
    waypoint.velocity.resize(dof);
    for (std::size_t joint = 0; joint < dof; ++joint) {
      const std::size_t offset = step * dof + joint;
      waypoint.position[joint] = static_cast<double>(result.position[offset]);
      if (result.velocity.size() >= steps * dof) {
        waypoint.velocity[joint] = static_cast<double>(result.velocity[offset]);
      }
    }
    response->trajectory.push_back(std::move(waypoint));
  }

  RCLCPP_INFO(this->get_logger(),
              "planned %zu steps, dt=%.4f, attempts=%d, rrt=%s, %.1f ms",
              steps, result.dt, result.planning_attempts,
              result.used_rrt ? "true" : "false", result.planning_ms);

  publish_trajectory(result);
}

void PlannerNode::publish_trajectory(const PlanResult& result) {
  if (!publish_trajectory_enabled_ || trajectory_pub_ == nullptr) {
    return;
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = this->now();
  trajectory.header.frame_id = base_frame_;
  trajectory.joint_names = core_->joint_names();

  const std::size_t dof = static_cast<std::size_t>(result.num_dof);
  const std::size_t steps = static_cast<std::size_t>(result.num_steps);
  trajectory.points.reserve(steps);
  for (std::size_t step = 0; step < steps; ++step) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(dof);
    point.velocities.resize(dof);
    for (std::size_t joint = 0; joint < dof; ++joint) {
      const std::size_t offset = step * dof + joint;
      point.positions[joint] = static_cast<double>(result.position[offset]);
      if (result.velocity.size() >= steps * dof) {
        point.velocities[joint] = static_cast<double>(result.velocity[offset]);
      }
    }
    point.time_from_start =
        rclcpp::Duration::from_seconds(static_cast<double>(step) * result.dt);
    trajectory.points.push_back(std::move(point));
  }

  trajectory_pub_->publish(trajectory);
}

}  // namespace curobo_ros_cpp
