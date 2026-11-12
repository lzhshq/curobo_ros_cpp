// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "curobo_execution_cpp/execution_adapter.hpp"

namespace curobo_execution_cpp {

/** Fire-and-forget JointTrajectory topic bridge (curobo_ros-style). */
class TopicAdapter : public ExecutionAdapter {
 public:
  TopicAdapter(rclcpp::Node* node, std::string topic);

  bool send(const trajectory_msgs::msg::JointTrajectory& trajectory,
            std::string& error) override;
  void cancel() override;
  bool active() const override;
  std::string status() const override;

 private:
  rclcpp::Node* node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  mutable std::mutex mutex_;
  std::string status_ = "idle";
};

}  // namespace curobo_execution_cpp
