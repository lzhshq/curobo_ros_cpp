// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cuda_runtime_api.h>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "curobo_robot_runtime/kinematics.h"
#include "curobo_robot_runtime/motion_planner.h"
#include "curobo_robot_runtime/mpc.h"
#include "curobo_robot_runtime/scene.h"
#include "smoke_common.hpp"

namespace {

using curobo::robot::MpcConfig;
using curobo::robot::MpcController;
using curobo::robot::MotionPlanner;
using curobo::robot::MotionPlanningConfig;
using curobo::robot::Scene;
using curobo::robot::SceneConfig;

void check(cudaError_t status, const char* where) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(where) + ": " + cudaGetErrorString(status));
  }
}

float percentile(std::vector<float> values, double fraction) {
  if (values.empty()) return 0.0F;
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
      std::min<double>(values.size() - 1, fraction * (values.size() - 1)));
  return values[index];
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: mpc_spike ARTIFACT_DIRECTORY [steps] [trajectory_dt]\n";
    return 2;
  }

  try {
    const std::string artifact = argv[1];
    const int steps = argc >= 3 ? std::max(1, std::stoi(argv[2])) : 25;
    const float trajectory_dt = argc >= 4 ? std::stof(argv[3]) : 0.02F;

    SceneConfig scene_config;
    scene_config.max_cuboids = 8;
    scene_config.max_capsules = 8;
    scene_config.max_cylinders = 8;
    Scene scene(scene_config);

    MotionPlanningConfig planner_config;
    planner_config.use_cuda_graph = true;
    MotionPlanner planner(artifact, planner_config, &scene);
    curobo::robot::Kinematics kinematics(artifact);
    const auto robot_info = kinematics.info();

    const auto start =
        curobo_ros_cpp::smoke::read_default_joint_position(artifact, robot_info.num_dof);
    auto goal = start;
    goal[0] += 0.2F;

    MpcConfig mpc_config;
    mpc_config.trajectory_dt = trajectory_dt;
    MpcController mpc(planner, mpc_config);
    const auto info = mpc.info();
    const auto candidates = mpc.candidate_dts();
    std::cout << "dof=" << robot_info.num_dof
              << " knots=" << info.num_knots
              << " traj_steps=" << info.trajectory_steps
              << " requested_dt=" << trajectory_dt << "\n";
    std::cout << "candidate dt (s):";
    for (float dt : candidates) std::cout << " " << dt;
    std::cout << "\n";

    const int dof = robot_info.num_dof;
    std::vector<float> seed(static_cast<std::size_t>(info.num_knots) * dof);
    for (int knot = 0; knot < info.num_knots; ++knot) {
      const float alpha = info.num_knots > 1
                              ? static_cast<float>(knot) / (info.num_knots - 1)
                              : 0.0F;
      for (int joint = 0; joint < dof; ++joint) {
        seed[static_cast<std::size_t>(knot) * dof + joint] =
            start[joint] + alpha * (goal[joint] - start[joint]);
      }
    }

    float* d_goal = nullptr;
    float* d_seed = nullptr;
    float* d_q = nullptr;
    float* d_v = nullptr;
    float* d_a = nullptr;
    float* d_j = nullptr;
    check(cudaMalloc(reinterpret_cast<void**>(&d_goal), dof * sizeof(float)), "malloc goal");
    check(cudaMalloc(reinterpret_cast<void**>(&d_seed), seed.size() * sizeof(float)), "malloc seed");
    check(cudaMalloc(reinterpret_cast<void**>(&d_q), dof * sizeof(float)), "malloc q");
    check(cudaMalloc(reinterpret_cast<void**>(&d_v), dof * sizeof(float)), "malloc v");
    check(cudaMalloc(reinterpret_cast<void**>(&d_a), dof * sizeof(float)), "malloc a");
    check(cudaMalloc(reinterpret_cast<void**>(&d_j), dof * sizeof(float)), "malloc jerk");
    std::vector<float> zero(dof, 0.0F);
    check(cudaMemcpy(d_goal, goal.data(), dof * sizeof(float), cudaMemcpyHostToDevice), "goal");
    check(cudaMemcpy(d_seed, seed.data(), seed.size() * sizeof(float), cudaMemcpyHostToDevice),
          "seed");
    check(cudaMemcpy(d_q, start.data(), dof * sizeof(float), cudaMemcpyHostToDevice), "q");
    check(cudaMemcpy(d_v, zero.data(), dof * sizeof(float), cudaMemcpyHostToDevice), "v");
    check(cudaMemcpy(d_a, zero.data(), dof * sizeof(float), cudaMemcpyHostToDevice), "a");
    check(cudaMemcpy(d_j, zero.data(), dof * sizeof(float), cudaMemcpyHostToDevice), "jerk");

    mpc.reset(d_goal, d_seed);
    // Warm the graph; it is not part of the timing samples.
    for (int i = 0; i < 3; ++i) {
      const auto result = mpc.step(d_q, d_v, d_a, d_j, 0);
      if (result.feasible) mpc.accept();
    }
    mpc.reset(d_goal, d_seed);

    std::vector<float> solve_ms;
    solve_ms.reserve(static_cast<std::size_t>(steps));
    int feasible = 0;
    for (int i = 0; i < steps; ++i) {
      const auto begin = std::chrono::steady_clock::now();
      const auto result = mpc.step(d_q, d_v, d_a, d_j, 0);
      const float elapsed = std::chrono::duration<float, std::milli>(
                                std::chrono::steady_clock::now() - begin)
                                .count();
      solve_ms.push_back(elapsed);
      if (result.feasible) {
        ++feasible;
        mpc.accept();
      }
    }

    const double mean = std::accumulate(solve_ms.begin(), solve_ms.end(), 0.0) /
                        static_cast<double>(solve_ms.size());
    std::cout << "steps=" << steps << " feasible=" << feasible
              << " mean_ms=" << mean
              << " median_ms=" << percentile(solve_ms, 0.5)
              << " p95_ms=" << percentile(solve_ms, 0.95)
              << " max_ms=" << percentile(solve_ms, 1.0)
              << " graph_captures=" << mpc.graph_capture_count() << "\n";
    std::cout << "implied_rate_hz=" << (mean > 0.0 ? 1000.0 / mean : 0.0) << "\n";

    cudaFree(d_goal);
    cudaFree(d_seed);
    cudaFree(d_q);
    cudaFree(d_v);
    cudaFree(d_a);
    cudaFree(d_j);
    return feasible > 0 ? 0 : 1;
  } catch (const std::exception& exception) {
    std::cerr << "mpc spike exception: " << exception.what() << "\n";
    return 1;
  }
}
