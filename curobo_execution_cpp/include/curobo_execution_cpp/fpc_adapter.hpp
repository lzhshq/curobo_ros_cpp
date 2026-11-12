// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "curobo_execution_cpp/execution_adapter.hpp"

namespace curobo_execution_cpp {

/**
 * Streams joint positions to a ros2_control ForwardCommandController.
 *
 * FPC has no action result: send() starts a timer that interpolates the planned
 * trajectory at publish_rate and publishes Float64MultiArray commands. "completed"
 * means the stream finished, not that the hardware tracked it.
 */
class FpcAdapter : public ExecutionAdapter {
 public:
  FpcAdapter(rclcpp::Node* node, std::string command_topic,
             std::vector<std::string> joint_names, double publish_rate);

  bool send(const trajectory_msgs::msg::JointTrajectory& trajectory,
            std::string& error) override;
  void cancel() override;
  bool active() const override;
  std::string status() const override;

 private:
  void timer_callback();
  void publish_sample(std::size_t index, double ratio);

  rclcpp::Node* node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  std::size_t output_size_ = 0;
  double publish_rate_;

  mutable std::mutex mutex_;
  bool active_ = false;
  std::string status_ = "idle";
  std::vector<std::size_t> source_indices_;
  std::vector<std::vector<double>> positions_;
  std::vector<double> times_;
  double duration_ = 0.0;
  std::chrono::steady_clock::time_point start_{};
};

}  // namespace curobo_execution_cpp
