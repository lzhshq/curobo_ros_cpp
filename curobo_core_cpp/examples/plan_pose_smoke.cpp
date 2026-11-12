// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <cuda_runtime_api.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "curobo_core_cpp/planner_core.hpp"
#include "curobo_robot_runtime/kinematics.h"

namespace {

void check_cuda(cudaError_t status, const char* where) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(where) + ": " + cudaGetErrorString(status));
  }
}

std::vector<float> read_default_joint_position(const std::string& artifact_dir, int dof) {
  std::ifstream file(artifact_dir + "/default_joint_position.f32", std::ios::binary);
  std::vector<float> values(static_cast<std::size_t>(dof), 0.0F);
  if (!file) {
    return values;
  }
  file.read(reinterpret_cast<char*>(values.data()),
            static_cast<std::streamsize>(values.size() * sizeof(float)));
  if (!file) {
    std::fill(values.begin(), values.end(), 0.0F);
  }
  return values;
}

struct ToolPose {
  std::vector<float> position;
  std::vector<float> quaternion;
};

ToolPose forward_kinematics(const curobo::robot::Kinematics& kinematics,
                            const CuroboRobotInfo& info,
                            const std::vector<float>& q) {
  float* d_q = nullptr;
  float* d_pos = nullptr;
  float* d_quat = nullptr;
  float* d_jac = nullptr;
  try {
    const std::size_t dof = static_cast<std::size_t>(info.num_dof);
    const std::size_t tools = static_cast<std::size_t>(info.num_tool_frames);
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_q), dof * sizeof(float)), "malloc q");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_pos), tools * 3 * sizeof(float)),
               "malloc pos");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_quat), tools * 4 * sizeof(float)),
               "malloc quat");
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&d_jac),
                          tools * 6 * dof * sizeof(float)),
               "malloc jac");
    check_cuda(cudaMemcpy(d_q, q.data(), dof * sizeof(float), cudaMemcpyHostToDevice),
               "copy q");
    kinematics.forward(d_q, d_pos, d_quat, d_jac, 1, 1);
    check_cuda(cudaDeviceSynchronize(), "sync fk");

    ToolPose pose;
    pose.position.resize(tools * 3);
    pose.quaternion.resize(tools * 4);
    check_cuda(cudaMemcpy(pose.position.data(), d_pos, tools * 3 * sizeof(float),
                          cudaMemcpyDeviceToHost),
               "copy pos");
    check_cuda(cudaMemcpy(pose.quaternion.data(), d_quat, tools * 4 * sizeof(float),
                          cudaMemcpyDeviceToHost),
               "copy quat");
    cudaFree(d_q);
    cudaFree(d_pos);
    cudaFree(d_quat);
    cudaFree(d_jac);
    return pose;
  } catch (...) {
    if (d_q != nullptr) cudaFree(d_q);
    if (d_pos != nullptr) cudaFree(d_pos);
    if (d_quat != nullptr) cudaFree(d_quat);
    if (d_jac != nullptr) cudaFree(d_jac);
    throw;
  }
}

}  // namespace

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

    const auto start = read_default_joint_position(artifact_dir, info.num_dof);
    auto goal = start;
    goal[0] += 0.2F;

    const auto start_pose = forward_kinematics(kinematics, info, start);
    const auto goal_pose = forward_kinematics(kinematics, info, goal);
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
    const std::size_t last = static_cast<std::size_t>(result.num_steps - 1) * result.num_dof;
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
