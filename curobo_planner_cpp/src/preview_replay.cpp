// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace curobo_ros_cpp {
namespace {

double to_seconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

}  // namespace

/** Replays a planned JointTrajectory as /preview/joint_states for a ghost robot. */
class PreviewReplay : public rclcpp::Node {
 public:
  PreviewReplay() : rclcpp::Node("preview_replay") {
    const std::string trajectory_topic = this->declare_parameter<std::string>(
        "trajectory_topic", "/curobo_planner/planned_trajectory");
    const std::string joint_states_topic =
        this->declare_parameter<std::string>("joint_states_topic", "/preview/joint_states");
    time_scale_ = this->declare_parameter<double>("time_scale", 1.0);
    loop_ = this->declare_parameter<bool>("loop", false);
    const double rate = this->declare_parameter<double>("publish_rate", 100.0);

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 20);
    subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        trajectory_topic, 10,
        std::bind(&PreviewReplay::trajectory_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(rate, 1.0)),
        std::bind(&PreviewReplay::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "preview replay: %s -> %s", trajectory_topic.c_str(),
                joint_states_topic.c_str());
  }

 private:
  void trajectory_callback(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory) {
    if (trajectory->points.empty() || trajectory->joint_names.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory_ = *trajectory;
    has_trajectory_ = true;
    start_ = std::chrono::steady_clock::now();
    duration_ = to_seconds(trajectory_.points.back().time_from_start);
    RCLCPP_INFO(this->get_logger(), "replaying %zu points / %.3f s",
                trajectory_.points.size(), duration_);
  }

  void timer_callback() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_trajectory_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_).count() * time_scale_;
    if (elapsed >= duration_) {
      if (loop_) {
        start_ = now;
        elapsed = 0.0;
      } else {
        elapsed = duration_;
      }
    }

    const auto& points = trajectory_.points;
    std::size_t index = 0;
    while (index + 1 < points.size() &&
           to_seconds(points[index + 1].time_from_start) <= elapsed) {
      ++index;
    }
    double ratio = 0.0;
    if (index + 1 < points.size()) {
      const double first = to_seconds(points[index].time_from_start);
      const double second = to_seconds(points[index + 1].time_from_start);
      if (second > first) {
        ratio = (elapsed - first) / (second - first);
      }
    }

    sensor_msgs::msg::JointState message;
    message.header.stamp = this->now();
    message.name = trajectory_.joint_names;
    message.position.resize(message.name.size());
    for (std::size_t joint = 0; joint < message.name.size(); ++joint) {
      const double first = points[index].positions[joint];
      double value = first;
      if (index + 1 < points.size() && ratio > 0.0) {
        const double second = points[index + 1].positions[joint];
        value = first + ratio * (second - first);
      }
      message.position[joint] = value;
    }
    publisher_->publish(message);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  trajectory_msgs::msg::JointTrajectory trajectory_;
  bool has_trajectory_ = false;
  bool loop_ = false;
  double time_scale_ = 1.0;
  double duration_ = 0.0;
  std::chrono::steady_clock::time_point start_{};
};

}  // namespace curobo_ros_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<curobo_ros_cpp::PreviewReplay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
