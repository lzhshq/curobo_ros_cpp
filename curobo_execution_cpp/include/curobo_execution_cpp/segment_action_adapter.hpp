// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "curobo_execution_cpp/execution_adapter.hpp"
#include "curobo_execution_interfaces/action/execute_segment.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace curobo_execution_cpp {

class SegmentActionAdapter : public ExecutionAdapter {
 public:
  using ExecuteSegment = curobo_execution_interfaces::action::ExecuteSegment;

  explicit SegmentActionAdapter(rclcpp::Node* node, std::string action_name);

  bool send(const trajectory_msgs::msg::JointTrajectory& trajectory,
            std::string& error) override;
  void cancel() override;
  bool active() const override;
  std::string status() const override;

 private:
  rclcpp::Node* node_;
  std::string action_name_;
  rclcpp_action::Client<ExecuteSegment>::SharedPtr client_;
  mutable std::mutex mutex_;
  rclcpp_action::ClientGoalHandle<ExecuteSegment>::SharedPtr goal_handle_;
  bool pending_ = false;
  bool active_ = false;
  bool server_available_ = false;
  std::string status_ = "idle";
};

}  // namespace curobo_execution_cpp
