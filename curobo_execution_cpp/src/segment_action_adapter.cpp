// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include "curobo_execution_cpp/segment_action_adapter.hpp"

#include <chrono>
#include <utility>

namespace curobo_execution_cpp {

SegmentActionAdapter::SegmentActionAdapter(rclcpp::Node* node, std::string action_name)
    : node_(node), action_name_(std::move(action_name)) {
  client_ = rclcpp_action::create_client<ExecuteSegment>(node_, action_name_);
  server_available_ = client_->wait_for_action_server(std::chrono::seconds(2));
  status_ = server_available_ ? "idle" : "execution server unavailable";
  if (!server_available_) {
    RCLCPP_WARN(node_->get_logger(), "%s: %s", action_name_.c_str(), status_.c_str());
  }
}

bool SegmentActionAdapter::send(
    const trajectory_msgs::msg::JointTrajectory& trajectory, std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!server_available_) {
    server_available_ = client_->wait_for_action_server(std::chrono::milliseconds(0));
    if (!server_available_) {
      error = "execution server unavailable: " + action_name_;
      return false;
    }
  }
  if (pending_ || active_) {
    if (goal_handle_) client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
    pending_ = false;
    active_ = false;
    status_ = "replacing";
  }
  if (trajectory.points.empty() || trajectory.joint_names.empty()) {
    error = "empty segment";
    return false;
  }
  ExecuteSegment::Goal goal;
  goal.trajectory = trajectory;
  rclcpp_action::Client<ExecuteSegment>::SendGoalOptions options;
  options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<ExecuteSegment>::SharedPtr handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_ = false;
        if (!handle) {
          active_ = false;
          status_ = "rejected";
          return;
        }
        goal_handle_ = handle;
        active_ = true;
        status_ = "executing";
      };
  options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<ExecuteSegment>::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = false;
        pending_ = false;
        goal_handle_.reset();
        status_ = result.result && result.result->success ? "completed" : "failed";
      };
  pending_ = true;
  active_ = false;
  status_ = "sending";
  client_->async_send_goal(goal, options);
  return true;
}

void SegmentActionAdapter::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_) client_->async_cancel_goal(goal_handle_);
  goal_handle_.reset();
  pending_ = false;
  active_ = false;
  status_ = "canceled";
}

bool SegmentActionAdapter::active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_ || pending_;
}

std::string SegmentActionAdapter::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

}  // namespace curobo_execution_cpp
