// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "curobo_core_cpp/planner_core.hpp"
#include "curobo_planner_interfaces/srv/add_object.hpp"
#include "curobo_planner_interfaces/srv/clear_objects.hpp"
#include "curobo_planner_interfaces/srv/plan_pose.hpp"
#include "curobo_planner_interfaces/srv/remove_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace curobo_ros_cpp {

/**
 * Thin ROS 2 adapter around PlannerCore.
 *
 * It translates ROS messages to the ROS-free core, calls plan_pose(), and
 * publishes the resulting trajectory. It does not execute the trajectory and
 * does not own a robot driver.
 */
class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode();
  ~PlannerNode() override = default;

 private:
  using PlanPose = curobo_planner_interfaces::srv::PlanPose;
  using AddObject = curobo_planner_interfaces::srv::AddObject;
  using RemoveObject = curobo_planner_interfaces::srv::RemoveObject;
  using ClearObjects = curobo_planner_interfaces::srv::ClearObjects;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void plan_callback(const std::shared_ptr<PlanPose::Request> request,
                     std::shared_ptr<PlanPose::Response> response);
  void add_object_callback(const std::shared_ptr<AddObject::Request> request,
                           std::shared_ptr<AddObject::Response> response);
  void remove_object_callback(const std::shared_ptr<RemoveObject::Request> request,
                              std::shared_ptr<RemoveObject::Response> response);
  void clear_objects_callback(const std::shared_ptr<ClearObjects::Request> request,
                              std::shared_ptr<ClearObjects::Response> response);

  bool reorder_start(const sensor_msgs::msg::JointState& message,
                     const std::vector<std::string>& desired,
                     std::vector<float>& output, std::string& error) const;

  void publish_trajectory(const curobo_ros_cpp::PlanResult& result);
  void publish_path(const curobo_ros_cpp::PlanResult& result);

  std::unique_ptr<PlannerCore> core_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<PlanPose>::SharedPtr plan_srv_;
  rclcpp::Service<AddObject>::SharedPtr add_object_srv_;
  rclcpp::Service<RemoveObject>::SharedPtr remove_object_srv_;
  rclcpp::Service<ClearObjects>::SharedPtr clear_objects_srv_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::mutex state_mutex_;
  std::mutex plan_mutex_;
  sensor_msgs::msg::JointState latest_state_;
  bool has_state_ = false;

  std::string base_frame_;
  bool publish_trajectory_enabled_ = true;
  bool publish_path_enabled_ = true;
  double path_marker_radius_ = 0.006;
};

}  // namespace curobo_ros_cpp
