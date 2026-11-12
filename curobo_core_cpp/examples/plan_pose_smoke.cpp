// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <string>

#include "curobo_core_cpp/planner_core.hpp"
#include "curobo_robot_runtime/kinematics.h"
#include "smoke_common.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: plan_pose_smoke ARTIFACT_DIRECTORY\n";
    return 2;
  }

  try {
    const std::string artifact_dir = argv[1];
    curobo::robot::Kinematics kinematics(artifact_dir);
    const auto info = kinematics.info();

    std::cout << "dof=" << info.num_dof << " tools=" << info.num_tool_frames << "\n";
    for (int i = 0; i < info.num_dof; ++i) {
      std::cout << "  joint[" << i << "]=" << kinematics.joint_name(i) << "\n";
    }
    for (int i = 0; i < info.num_tool_frames; ++i) {
      std::cout << "  tool[" << i << "]=" << kinematics.tool_frame_name(i) << "\n";
    }

    const auto start = curobo_ros_cpp::smoke::read_default_joint_position(
        artifact_dir, info.num_dof);
    auto goal = start;
    goal[0] += 0.2F;

    const auto goal_pose =
        curobo_ros_cpp::smoke::forward_kinematics(kinematics, info, goal);
    std::cout << "goal_position=" << goal_pose.position[0] << " "
              << goal_pose.position[1] << " " << goal_pose.position[2] << "\n";
    std::cout << "goal_quaternion_wxyz=" << goal_pose.quaternion[0] << " "
              << goal_pose.quaternion[1] << " " << goal_pose.quaternion[2] << " "
              << goal_pose.quaternion[3] << "\n";

    curobo_ros_cpp::PlannerConfig config;
    config.artifact_dir = artifact_dir;
    config.max_attempts = 5;
    config.max_cuboids = 8;

    curobo_ros_cpp::PlannerCore core(config);
    std::string error;
    if (!core.initialize(error)) {
      std::cerr << "initialize failed: " << error << "\n";
      return 1;
    }

    curobo_ros_cpp::PlanRequest request;
    request.start_position = start;
    request.position = {goal_pose.position[0], goal_pose.position[1],
                        goal_pose.position[2]};
    request.quaternion = {goal_pose.quaternion[0], goal_pose.quaternion[1],
                          goal_pose.quaternion[2], goal_pose.quaternion[3]};

    const auto result = core.plan_pose(request);
    std::cout << "success=" << (result.success ? "true" : "false")
              << " dt=" << result.dt
              << " steps=" << result.num_steps
              << " attempts=" << result.planning_attempts
              << " rrt=" << (result.used_rrt ? "true" : "false")
              << " planning_ms=" << result.planning_ms
              << "\nmessage: " << result.message << "\n";
    if (!result.success) {
      return 1;
    }

    std::cout << "first=";
    for (int j = 0; j < result.num_dof; ++j) {
      std::cout << result.position[static_cast<std::size_t>(j)] << " ";
    }
    std::cout << "\nlast=";
    const std::size_t last =
        static_cast<std::size_t>(result.num_steps - 1) * result.num_dof;
    for (int j = 0; j < result.num_dof; ++j) {
      std::cout << result.position[last + static_cast<std::size_t>(j)] << " ";
    }
    std::cout << "\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << "exception: " << exception.what() << "\n";
    return 1;
  }
}
