// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "curobo_core_cpp/planner_core.hpp"
#include "curobo_planner_interfaces/srv/plan_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void plan_callback(const std::shared_ptr<PlanPose::Request> request,
                     std::shared_ptr<PlanPose::Response> response);

  bool reorder_start(const sensor_msgs::msg::JointState& message,
                     const std::vector<std::string>& desired,
                     std::vector<float>& output, std::string& error) const;

  void publish_trajectory(const curobo_ros_cpp::PlanResult& result);

  std::unique_ptr<PlannerCore> core_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<PlanPose>::SharedPtr plan_srv_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  std::mutex state_mutex_;
  sensor_msgs::msg::JointState latest_state_;
  bool has_state_ = false;

  std::string base_frame_;
  bool publish_trajectory_enabled_ = true;
};

}  // namespace curobo_ros_cpp
