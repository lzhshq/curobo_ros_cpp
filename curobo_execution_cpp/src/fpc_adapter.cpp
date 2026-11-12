// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include "curobo_execution_cpp/fpc_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace curobo_execution_cpp {
namespace {

double to_seconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

}  // namespace

FpcAdapter::FpcAdapter(rclcpp::Node* node, std::string command_topic,
                       std::vector<std::string> joint_names, double publish_rate)
    : node_(node),
      joint_names_(std::move(joint_names)),
      publish_rate_(publish_rate > 0.0 ? publish_rate : 100.0) {
  publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      std::move(command_topic), 10);
  timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&FpcAdapter::timer_callback, this));
}

bool FpcAdapter::send(const trajectory_msgs::msg::JointTrajectory& trajectory,
                      std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    error = "an FPC stream is already active";
    return false;
  }
  if (trajectory.points.empty()) {
    error = "trajectory has no points";
    return false;
  }
  if (trajectory.joint_names.empty()) {
    error = "trajectory has no joint_names";
    return false;
  }
  if (!joint_names_.empty() && joint_names_.size() != trajectory.joint_names.size()) {
    error = "trajectory has " + std::to_string(trajectory.joint_names.size()) +
            " joints but the adapter expects " + std::to_string(joint_names_.size());
    return false;
  }

  std::vector<std::size_t> source_indices;
  if (joint_names_.empty()) {
    source_indices.resize(trajectory.joint_names.size());
    for (std::size_t i = 0; i < source_indices.size(); ++i) {
      source_indices[i] = i;
    }
  } else {
    for (const auto& name : joint_names_) {
      const auto it = std::find(trajectory.joint_names.begin(),
                                trajectory.joint_names.end(), name);
      if (it == trajectory.joint_names.end()) {
        error = "trajectory is missing controller joint '" + name + "'";
        return false;
      }
      source_indices.push_back(
          static_cast<std::size_t>(std::distance(trajectory.joint_names.begin(), it)));
    }
  }

  positions_.clear();
  times_.clear();
  positions_.reserve(trajectory.points.size());
  times_.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      error = "trajectory point positions do not match joint_names";
      return false;
    }
    positions_.push_back(point.positions);
    times_.push_back(to_seconds(point.time_from_start));
  }
  if (!std::is_sorted(times_.begin(), times_.end())) {
    error = "time_from_start is not monotonic";
    return false;
  }

  source_indices_ = std::move(source_indices);
  output_size_ = joint_names_.empty() ? trajectory.joint_names.size() : joint_names_.size();
  duration_ = times_.back();
  start_ = std::chrono::steady_clock::now();
  active_ = true;
  status_ = "executing";
  publish_sample(0, 0.0);
  return true;
}

void FpcAdapter::timer_callback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return;
  }
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
  if (elapsed >= duration_) {
    publish_sample(times_.size() - 1, 0.0);
    active_ = false;
    status_ = "completed";
    return;
  }

  std::size_t index = 0;
  while (index + 1 < times_.size() && times_[index + 1] <= elapsed) {
    ++index;
  }
  double ratio = 0.0;
  if (index + 1 < times_.size()) {
    const double span = times_[index + 1] - times_[index];
    if (span > 0.0) {
      ratio = (elapsed - times_[index]) / span;
    }
  }
  publish_sample(index, std::clamp(ratio, 0.0, 1.0));
}

void FpcAdapter::publish_sample(std::size_t index, double ratio) {
  if (positions_.empty() || index >= positions_.size()) {
    return;
  }
  std_msgs::msg::Float64MultiArray message;
  message.data.resize(output_size_);
  for (std::size_t j = 0; j < output_size_; ++j) {
    const std::size_t source = source_indices_[j];
    double value = positions_[index][source];
    if (ratio > 0.0 && index + 1 < positions_.size()) {
      value += ratio * (positions_[index + 1][source] - value);
    }
    message.data[j] = value;
  }
  publisher_->publish(message);
}

void FpcAdapter::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  status_ = "canceled";
}

bool FpcAdapter::active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

std::string FpcAdapter::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

}  // namespace curobo_execution_cpp
