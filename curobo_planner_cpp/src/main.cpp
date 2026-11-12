// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <exception>

#include "curobo_planner_cpp/planner_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<curobo_ros_cpp::PlannerNode>();
    rclcpp::spin(node);
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("curobo_planner"), "startup failed: %s",
                 exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
