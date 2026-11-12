// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include "curobo_execution_cpp/topic_adapter.hpp"

#include <utility>

namespace curobo_execution_cpp {

TopicAdapter::TopicAdapter(rclcpp::Node* node, std::string topic) : node_(node) {
  publisher_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(std::move(topic), 10);
}

bool TopicAdapter::send(const trajectory_msgs::msg::JointTrajectory& trajectory,
                        std::string& error) {
  if (trajectory.points.empty()) {
    error = "trajectory has no points";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  publisher_->publish(trajectory);
  status_ = "published (fire-and-forget)";
  return true;
}

void TopicAdapter::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  publisher_->publish(trajectory_msgs::msg::JointTrajectory());
  status_ = "cancel published";
}

bool TopicAdapter::active() const { return false; }

std::string TopicAdapter::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

}  // namespace curobo_execution_cpp
