// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cuda_runtime_api.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "curobo_robot_runtime/kinematics.h"

namespace curobo_ros_cpp::smoke {

inline void check_cuda(cudaError_t status, const char* where) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(where) + ": " + cudaGetErrorString(status));
  }
}

inline std::vector<float> read_default_joint_position(const std::string& artifact_dir,
                                                      int dof) {
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

inline ToolPose forward_kinematics(const curobo::robot::Kinematics& kinematics,
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

}  // namespace curobo_ros_cpp::smoke
