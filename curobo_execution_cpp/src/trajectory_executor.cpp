// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include "curobo_execution_cpp/trajectory_executor.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "curobo_execution_cpp/jtc_adapter.hpp"
#include "curobo_execution_cpp/topic_adapter.hpp"

namespace curobo_execution_cpp {
namespace {

double to_seconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

void set_seconds(builtin_interfaces::msg::Duration& duration, double seconds) {
  const double whole = std::floor(seconds);
  duration.sec = static_cast<int32_t>(whole);
  duration.nanosec = static_cast<uint32_t>((seconds - whole) * 1e9);
  if (duration.nanosec >= 1000000000U) {
    ++duration.sec;
    duration.nanosec = 0;
  }
}

}  // namespace

TrajectoryExecutor::TrajectoryExecutor() : rclcpp::Node("curobo_executor") {
  const std::string adapter = this->declare_parameter<std::string>("adapter", "jtc");
  const std::string action = this->declare_parameter<std::string>(
      "action", "/position_joint_trajectory_controller/follow_joint_trajectory");
  const std::string command_topic =
      this->declare_parameter<std::string>("command_topic", "/execute_trajectory");
  const double publish_rate = this->declare_parameter<double>("publish_rate", 100.0);
  const std::string trajectory_topic = this->declare_parameter<std::string>(
      "trajectory_topic", "/curobo_planner/planned_trajectory");
  time_scale_ = this->declare_parameter<double>("time_scale", 1.0);
  const double goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.02);
  const double goal_time_tolerance =
      this->declare_parameter<double>("goal_time_tolerance", 2.0);
  const auto joint_names =
      this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});

  if (adapter == "jtc") {
    adapter_ = std::make_unique<JtcAdapter>(this, action, joint_names, goal_tolerance,
                                            goal_time_tolerance);
  } else if (adapter == "topic") {
    adapter_ = std::make_unique<TopicAdapter>(this, command_topic);
  } else {
    throw std::runtime_error("unknown adapter '" + adapter + "' (use jtc or topic)");
  }

  trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic, 10,
      std::bind(&TrajectoryExecutor::trajectory_callback, this, std::placeholders::_1));

  execute_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/execute_latest",
      std::bind(&TrajectoryExecutor::execute_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  cancel_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/cancel",
      std::bind(&TrajectoryExecutor::cancel_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  status_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/status",
      std::bind(&TrajectoryExecutor::status_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(),
              "executor ready: adapter=%s, trajectory=%s, action=%s", adapter.c_str(),
              trajectory_topic.c_str(), action.c_str());
}

void TrajectoryExecutor::trajectory_callback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = *trajectory;
  has_latest_ = true;
  RCLCPP_INFO(this->get_logger(), "cached trajectory: %zu points, %zu joints",
              latest_.points.size(), latest_.joint_names.size());
}

bool TrajectoryExecutor::prepare_trajectory(
    trajectory_msgs::msg::JointTrajectory& trajectory, std::string& error) const {
  if (trajectory.points.empty()) {
    error = "trajectory has no points";
    return false;
  }
  if (trajectory.joint_names.empty()) {
    error = "trajectory has no joint_names";
    return false;
  }
  if (!std::isfinite(time_scale_) || time_scale_ <= 0.0) {
    error = "time_scale must be positive";
    return false;
  }

  double previous = -1.0;
  for (auto& point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      error = "trajectory point positions do not match joint_names";
      return false;
    }
    const double seconds = to_seconds(point.time_from_start);
    if (!std::isfinite(seconds) || seconds < 0.0) {
      error = "trajectory point has a negative time_from_start";
      return false;
    }
    if (seconds < previous) {
      error = "time_from_start is not monotonic";
      return false;
    }
    previous = seconds;
    set_seconds(point.time_from_start, seconds * time_scale_);
  }

  // JTC rejects trajectories with a stale header stamp.
  trajectory.header.stamp = this->now();
  return true;
}

void TrajectoryExecutor::execute_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_latest_) {
    response->success = false;
    response->message = "no planned trajectory received yet";
    return;
  }
  if (adapter_->active()) {
    response->success = false;
    response->message = "execution already active: " + adapter_->status();
    return;
  }

  auto trajectory = latest_;
  std::string error;
  if (!prepare_trajectory(trajectory, error)) {
    response->success = false;
    response->message = error;
    return;
  }

  if (!adapter_->send(trajectory, error)) {
    response->success = false;
    response->message = error;
    return;
  }

  response->success = true;
  response->message = "trajectory sent; status=" + adapter_->status();
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void TrajectoryExecutor::cancel_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  adapter_->cancel();
  response->success = true;
  response->message = "cancel requested; status=" + adapter_->status();
}

void TrajectoryExecutor::status_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  response->success = true;
  response->message = std::string(adapter_->active() ? "active: " : "idle: ") +
                      adapter_->status();
}

}  // namespace curobo_execution_cpp
