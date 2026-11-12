// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include "curobo_planner_cpp/planner_node.hpp"

#include <array>
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

geometry_msgs::msg::Quaternion quaternion_from_z_to(float dx, float dy, float dz) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = 1.0;
  const float norm = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (norm < 1e-9F) {
    return quaternion;
  }
  dx /= norm;
  dy /= norm;
  dz /= norm;
  const float dot = dz;
  if (dot > 1.0F - 1e-6F) {
    return quaternion;
  }
  if (dot < -1.0F + 1e-6F) {
    quaternion.x = 1.0;
    quaternion.w = 0.0;
    return quaternion;
  }
  const float angle = std::acos(dot);
  const float axis_norm = std::sqrt(dx * dx + dy * dy);
  const float scale = std::sin(angle * 0.5F) / axis_norm;
  quaternion.x = -dy * scale;
  quaternion.y = dx * scale;
  quaternion.z = 0.0;
  quaternion.w = std::cos(angle * 0.5F);
  return quaternion;
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
  this->declare_parameter<int>("scene_max_capsules", 8);
  this->declare_parameter<int>("scene_max_cylinders", 8);
  this->declare_parameter<int>("scene_max_meshes", 0);
  this->declare_parameter<int>("scene_max_voxel_grids", 0);
  this->declare_parameter<int>("scene_max_voxels_per_grid", 0);
  this->declare_parameter<std::string>("joint_states_topic", "~/joint_states");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<bool>("publish_trajectory", true);
  this->declare_parameter<bool>("publish_path", true);
  this->declare_parameter<double>("path_marker_radius", 0.006);
  this->declare_parameter<bool>("publish_scene_markers", true);
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
  publish_path_enabled_ = this->get_parameter("publish_path").as_bool();
  path_marker_radius_ = this->get_parameter("path_marker_radius").as_double();
  publish_scene_markers_enabled_ = this->get_parameter("publish_scene_markers").as_bool();

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

  add_object_srv_ = this->create_service<AddObject>(
      "~/add_object",
      std::bind(&PlannerNode::add_object_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  remove_object_srv_ = this->create_service<RemoveObject>(
      "~/remove_object",
      std::bind(&PlannerNode::remove_object_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  clear_objects_srv_ = this->create_service<ClearObjects>(
      "~/clear_objects",
      std::bind(&PlannerNode::clear_objects_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/planned_trajectory", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/planned_path", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/planned_markers", 10);
  scene_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/scene_markers", rclcpp::QoS(1).transient_local());

  this->set_parameter(rclcpp::Parameter("node_is_available", true));
  RCLCPP_INFO(this->get_logger(),
              "Ready: ~/generate_trajectory, ~/planned_trajectory, "
              "~/add_object, ~/remove_object, ~/clear_objects");
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

  PlanResult result;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    result = core_->plan_pose(plan_request);
  }
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
  publish_path(result);
  publish_scene_markers();
}

void PlannerNode::add_object_callback(
    const std::shared_ptr<AddObject::Request> request,
    std::shared_ptr<AddObject::Response> response) {
  ObjectSpec object;
  object.name = request->name;
  object.position = {static_cast<float>(request->pose.position.x),
                     static_cast<float>(request->pose.position.y),
                     static_cast<float>(request->pose.position.z)};
  object.dimensions = {static_cast<float>(request->dimensions.x),
                       static_cast<float>(request->dimensions.y),
                       static_cast<float>(request->dimensions.z)};

  const double qw = request->pose.orientation.w;
  const double qx = request->pose.orientation.x;
  const double qy = request->pose.orientation.y;
  const double qz = request->pose.orientation.z;
  const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (norm < 1e-9) {
    response->success = false;
    response->message = "object pose orientation must be a non-zero quaternion (xyzw)";
    return;
  }
  object.quaternion = {static_cast<float>(qw / norm), static_cast<float>(qx / norm),
                       static_cast<float>(qy / norm), static_cast<float>(qz / norm)};

  switch (request->type) {
    case AddObject::Request::CUBOID:
      object.type = ObjectType::CUBOID;
      break;
    case AddObject::Request::SPHERE:
      object.type = ObjectType::SPHERE;
      break;
    case AddObject::Request::CAPSULE:
      object.type = ObjectType::CAPSULE;
      break;
    case AddObject::Request::CYLINDER:
      object.type = ObjectType::CYLINDER;
      break;
    case AddObject::Request::MESH:
      response->success = false;
      response->message = "MESH is not supported in v0.2";
      return;
    default:
      response->success = false;
      response->message = "unknown object type " + std::to_string(request->type);
      return;
  }

  std::string error;
  bool success = false;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    success = core_->add_object(object, error);
  }
  response->success = success;
  response->message = success ? ("object '" + object.name + "' added") : error;
  if (success) {
    publish_scene_markers();
    RCLCPP_INFO(this->get_logger(), "added object '%s'", object.name.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "add_object failed: %s", error.c_str());
  }
}

void PlannerNode::remove_object_callback(
    const std::shared_ptr<RemoveObject::Request> request,
    std::shared_ptr<RemoveObject::Response> response) {
  std::string error;
  bool success = false;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    success = core_->remove_object(request->name, error);
  }
  response->success = success;
  response->message = success ? ("object '" + request->name + "' removed") : error;
  if (success) {
    publish_scene_markers();
    RCLCPP_INFO(this->get_logger(), "removed object '%s'", request->name.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "remove_object failed: %s", error.c_str());
  }
}

void PlannerNode::clear_objects_callback(
    const std::shared_ptr<ClearObjects::Request> /*request*/,
    std::shared_ptr<ClearObjects::Response> response) {
  std::string error;
  bool success = false;
  int count = 0;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    count = core_->object_count();
    success = core_->clear_objects(error);
  }
  response->success = success;
  response->message = success ? ("cleared " + std::to_string(count) + " objects") : error;
  response->count = success ? count : core_->object_count();
  if (success) {
    publish_scene_markers();
    RCLCPP_INFO(this->get_logger(), "cleared %d objects", count);
  } else {
    RCLCPP_WARN(this->get_logger(), "clear_objects failed: %s", error.c_str());
  }
}

void PlannerNode::publish_path(const PlanResult& result) {
  if (!publish_path_enabled_ || path_pub_ == nullptr) {
    return;
  }
  if (base_frame_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "planned_path has an empty frame_id; set base_frame for RViz");
  }

  const auto tool_path = core_->compute_tool_path(result.position, result.num_steps);
  if (tool_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "FK failed; planned_path not published");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = base_frame_;
  path.poses.reserve(tool_path.size());
  for (const auto& pose : tool_path) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose.position.x = pose.position[0];
    pose_stamped.pose.position.y = pose.position[1];
    pose_stamped.pose.position.z = pose.position[2];
    pose_stamped.pose.orientation.x = pose.quaternion[1];
    pose_stamped.pose.orientation.y = pose.quaternion[2];
    pose_stamped.pose.orientation.z = pose.quaternion[3];
    pose_stamped.pose.orientation.w = pose.quaternion[0];
    path.poses.push_back(std::move(pose_stamped));
  }
  path_pub_->publish(path);

  visualization_msgs::msg::MarkerArray markers;
  auto make_sphere = [&](int id, const curobo_ros_cpp::ToolPose& pose, float r, float g,
                         float b) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "planned_path";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0F;
    marker.pose.position.x = pose.position[0];
    marker.pose.position.y = pose.position[1];
    marker.pose.position.z = pose.position[2];
    marker.pose.orientation.w = 1.0;
    markers.markers.push_back(std::move(marker));
  };
  for (std::size_t i = 1; i < tool_path.size(); ++i) {
    const auto& first = tool_path[i - 1];
    const auto& second = tool_path[i];
    const double dx = second.position[0] - first.position[0];
    const double dy = second.position[1] - first.position[1];
    const double dz = second.position[2] - first.position[2];
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (length < 1e-6) {
      continue;
    }

    visualization_msgs::msg::Marker tube;
    tube.header = path.header;
    tube.ns = "planned_tube";
    tube.id = static_cast<int>(100 + i);
    tube.type = visualization_msgs::msg::Marker::CYLINDER;
    tube.action = visualization_msgs::msg::Marker::ADD;
    tube.scale.x = path_marker_radius_;
    tube.scale.y = path_marker_radius_;
    tube.scale.z = length;
    tube.color.r = 0.0F;
    tube.color.g = 1.0F;
    tube.color.b = 1.0F;
    tube.color.a = 1.0F;
    tube.pose.position.x = static_cast<float>((first.position[0] + second.position[0]) * 0.5);
    tube.pose.position.y = static_cast<float>((first.position[1] + second.position[1]) * 0.5);
    tube.pose.position.z = static_cast<float>((first.position[2] + second.position[2]) * 0.5);
    tube.pose.orientation = quaternion_from_z_to(
        static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dz));
    markers.markers.push_back(std::move(tube));
  }

  make_sphere(0, tool_path.front(), 1.0F, 0.0F, 0.0F);
  make_sphere(1, tool_path.back(), 0.0F, 1.0F, 0.0F);
  marker_pub_->publish(markers);
}

void PlannerNode::publish_scene_markers() {
  if (!publish_scene_markers_enabled_ || scene_marker_pub_ == nullptr) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = this->now();
  clear.header.frame_id = base_frame_;
  clear.ns = "scene";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  int id = 1;
  for (const auto& object : core_->object_specs()) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = clear.header.stamp;
    marker.header.frame_id = base_frame_;
    marker.ns = "scene";
    marker.id = id++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = object.position[0];
    marker.pose.position.y = object.position[1];
    marker.pose.position.z = object.position[2];
    marker.pose.orientation.x = object.quaternion[1];
    marker.pose.orientation.y = object.quaternion[2];
    marker.pose.orientation.z = object.quaternion[3];
    marker.pose.orientation.w = object.quaternion[0];
    marker.color.r = 1.0F;
    marker.color.g = 0.55F;
    marker.color.b = 0.1F;
    marker.color.a = 0.6F;

    switch (object.type) {
      case ObjectType::CUBOID:
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = object.dimensions[0];
        marker.scale.y = object.dimensions[1];
        marker.scale.z = object.dimensions[2];
        break;
      case ObjectType::SPHERE:
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2.0F * object.dimensions[0];
        marker.scale.y = 2.0F * object.dimensions[0];
        marker.scale.z = 2.0F * object.dimensions[0];
        break;
      case ObjectType::CYLINDER:
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.scale.x = 2.0F * object.dimensions[0];
        marker.scale.y = 2.0F * object.dimensions[0];
        marker.scale.z = object.dimensions[1];
        break;
      case ObjectType::CAPSULE:
        // RViz has no capsule marker; approximate with a cylinder.
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.scale.x = 2.0F * object.dimensions[0];
        marker.scale.y = 2.0F * object.dimensions[0];
        marker.scale.z = object.dimensions[1];
        break;
      default:
        continue;
    }
    markers.markers.push_back(std::move(marker));
  }
  scene_marker_pub_->publish(markers);
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
