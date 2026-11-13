// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace curobo_execution_cpp {

/**
 * Transport adapter for a planned JointTrajectory.
 *
 * The executor owns timing, validation, and status; the adapter owns the
 * robot/controller-specific transport (JTC action, topic bridge, ...).
 */
class ExecutionAdapter {
 public:
  virtual ~ExecutionAdapter() = default;

  virtual bool send(const trajectory_msgs::msg::JointTrajectory& trajectory,
                    std::string& error) = 0;
  virtual void cancel() = 0;
  virtual bool active() const = 0;
  virtual std::string status() const = 0;
};

}  // namespace curobo_execution_cpp
