// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "curobo_execution_cpp/execution_adapter.hpp"

namespace curobo_execution_cpp {

class JtcAdapter : public ExecutionAdapter {
 public:
  using Goal = control_msgs::action::FollowJointTrajectory;

  JtcAdapter(rclcpp::Node* node, std::string action_name,
             std::vector<std::string> joint_names, double goal_tolerance,
             double goal_time_tolerance);

  bool send(const trajectory_msgs::msg::JointTrajectory& trajectory,
            std::string& error) override;
  void cancel() override;
  bool active() const override;
  std::string status() const override;

 private:
  void goal_response_callback(
      const rclcpp_action::ClientGoalHandle<Goal>::SharedPtr& goal_handle);
  void feedback_callback(
      rclcpp_action::ClientGoalHandle<Goal>::SharedPtr goal_handle,
      const std::shared_ptr<const Goal::Feedback> feedback);
  void result_callback(const rclcpp_action::ClientGoalHandle<Goal>::WrappedResult& result);

  rclcpp::Node* node_;
  std::string action_name_;
  std::vector<std::string> joint_names_;
  double goal_tolerance_;
  double goal_time_tolerance_;
  rclcpp_action::Client<Goal>::SharedPtr client_;

  mutable std::mutex mutex_;
  bool server_available_ = false;
  bool pending_ = false;
  bool active_ = false;
  std::string status_ = "idle";
  rclcpp_action::ClientGoalHandle<Goal>::SharedPtr goal_handle_;
};

}  // namespace curobo_execution_cpp
