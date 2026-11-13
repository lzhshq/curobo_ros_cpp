// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "curobo_execution_interfaces/action/execute_segment.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "curobo_execution_cpp/execution_adapter.hpp"

namespace curobo_execution_cpp {

class TrajectoryExecutor : public rclcpp::Node {
 public:
  using ExecuteSegment = curobo_execution_interfaces::action::ExecuteSegment;
  using SegmentGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSegment>;
  TrajectoryExecutor();

 private:
  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory);
  void execute_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void cancel_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void status_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  rclcpp_action::GoalResponse handle_segment_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ExecuteSegment::Goal> goal);
  rclcpp_action::CancelResponse handle_segment_cancel(
      const std::shared_ptr<SegmentGoalHandle> goal_handle);
  void handle_segment_accepted(const std::shared_ptr<SegmentGoalHandle> goal_handle);

  bool prepare_trajectory(trajectory_msgs::msg::JointTrajectory& trajectory,
                          std::string& error) const;

  std::unique_ptr<ExecutionAdapter> adapter_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;
  rclcpp_action::Server<ExecuteSegment>::SharedPtr segment_action_server_;

  mutable std::mutex mutex_;
  trajectory_msgs::msg::JointTrajectory latest_;
  bool has_latest_ = false;
  double time_scale_ = 1.0;
};

}  // namespace curobo_execution_cpp
