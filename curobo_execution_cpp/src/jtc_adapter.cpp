// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include "curobo_execution_cpp/jtc_adapter.hpp"

#include <chrono>
#include <cmath>
#include <utility>

#include "control_msgs/msg/joint_tolerance.hpp"

namespace curobo_execution_cpp {
namespace {

void set_duration(builtin_interfaces::msg::Duration& duration, double seconds) {
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9);
}

}  // namespace

JtcAdapter::JtcAdapter(rclcpp::Node* node, std::string action_name,
                       std::vector<std::string> joint_names, double goal_tolerance,
                       double goal_time_tolerance)
    : node_(node),
      action_name_(std::move(action_name)),
      joint_names_(std::move(joint_names)),
      goal_tolerance_(goal_tolerance),
      goal_time_tolerance_(goal_time_tolerance) {
  client_ = rclcpp_action::create_client<Goal>(node_, action_name_);
  server_available_ = client_->wait_for_action_server(std::chrono::seconds(2));
  if (!server_available_) {
    status_ = "action server unavailable: " + action_name_;
    RCLCPP_WARN(node_->get_logger(), "%s", status_.c_str());
  } else {
    status_ = "idle";
  }
}

bool JtcAdapter::send(const trajectory_msgs::msg::JointTrajectory& trajectory,
                      std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!server_available_) {
    server_available_ = client_->wait_for_action_server(std::chrono::milliseconds(0));
    if (!server_available_) {
      status_ = "action server unavailable: " + action_name_;
      error = status_;
      return false;
    }
  }
  if (pending_ || active_) {
    // MPC sends a new full horizon every command interval; preempt the old
    // goal instead of rejecting the update.
    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
    pending_ = false;
    active_ = false;
    status_ = "replacing";
  }
  if (trajectory.points.empty()) {
    error = "trajectory has no points";
    return false;
  }
  if (trajectory.joint_names.empty()) {
    error = "trajectory has no joint_names";
    return false;
  }
  if (!joint_names_.empty() && trajectory.joint_names.size() != joint_names_.size()) {
    error = "trajectory has " + std::to_string(trajectory.joint_names.size()) +
            " joints but the adapter expects " + std::to_string(joint_names_.size());
    return false;
  }

  Goal::Goal goal;
  goal.trajectory = trajectory;
  if (!joint_names_.empty()) {
    // Artifact names and controller names can differ (e.g. j1..j7 vs joint1..7).
    // Positions are passed in trajectory order; the adapter assumes the configured
    // controller order is the same order.
    goal.trajectory.joint_names = joint_names_;
  }
  set_duration(goal.goal_time_tolerance, goal_time_tolerance_);

  goal.goal_tolerance.clear();
  const auto& names = goal.trajectory.joint_names;
  for (const auto& name : names) {
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = name;
    tolerance.position = goal_tolerance_;
    tolerance.velocity = 0.0;
    tolerance.acceleration = 0.0;
    goal.goal_tolerance.push_back(tolerance);
  }

  rclcpp_action::Client<Goal>::SendGoalOptions options;
  options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<Goal>::SharedPtr& handle) {
        goal_response_callback(handle);
      };
  options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<Goal>::SharedPtr handle,
             const std::shared_ptr<const Goal::Feedback> feedback) {
        feedback_callback(std::move(handle), feedback);
      };
  options.result_callback = [this](const rclcpp_action::ClientGoalHandle<Goal>::WrappedResult&
                                       result) { result_callback(result); };

  pending_ = true;
  status_ = "sending";
  client_->async_send_goal(goal, options);
  return true;
}

void JtcAdapter::goal_response_callback(
    const rclcpp_action::ClientGoalHandle<Goal>::SharedPtr& goal_handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = false;
  if (!goal_handle) {
    active_ = false;
    status_ = "rejected";
    RCLCPP_WARN(node_->get_logger(), "JTC rejected trajectory on %s", action_name_.c_str());
    return;
  }
  goal_handle_ = goal_handle;
  active_ = true;
  status_ = "executing";
  RCLCPP_INFO(node_->get_logger(), "JTC accepted trajectory on %s", action_name_.c_str());
}

void JtcAdapter::feedback_callback(
    rclcpp_action::ClientGoalHandle<Goal>::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const Goal::Feedback> /*feedback*/) {
  // Keep the status string cheap; detailed feedback can be added later.
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    status_ = "executing";
  }
}

void JtcAdapter::result_callback(
    const rclcpp_action::ClientGoalHandle<Goal>::WrappedResult& result) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  pending_ = false;
  goal_handle_.reset();
  if (result.result == nullptr) {
    status_ = "result missing";
    return;
  }
  const int code = result.result->error_code;
  status_ = "result code=" + std::to_string(code) + " " + result.result->error_string;
  if (code == Goal::Result::SUCCESSFUL) {
    RCLCPP_INFO(node_->get_logger(), "JTC result: %s", status_.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "JTC result: %s", status_.c_str());
  }
}

void JtcAdapter::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    status_ = "canceling";
  } else {
    status_ = "idle";
  }
}

bool JtcAdapter::active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_ || active_;
}

std::string JtcAdapter::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

}  // namespace curobo_execution_cpp
