// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cuda_runtime_api.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "curobo_robot_runtime/kinematics.h"
#include "curobo_robot_runtime/motion_planner.h"
#include "curobo_robot_runtime/mpc.h"
#include "curobo_robot_runtime/scene.h"
#include "fake_quintic_controller.hpp"

namespace {
using curobo::robot::Kinematics;
using curobo::robot::MotionPlanner;
using curobo::robot::MotionPlanningConfig;
using curobo::robot::MpcConfig;
using curobo::robot::MpcController;
using curobo::robot::PoseMotionPlanningRequest;
using curobo::robot::Scene;
using curobo::robot::SceneConfig;

void check(cudaError_t status, const char* where) {
  if (status != cudaSuccess) throw std::runtime_error(std::string(where) + ": " +
                                                      cudaGetErrorString(status));
}

class Buffer {
 public:
  explicit Buffer(std::size_t count) : count_(count) {
    check(cudaMalloc(reinterpret_cast<void**>(&ptr_), count * sizeof(float)), "cudaMalloc");
  }
  ~Buffer() { cudaFree(ptr_); }
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;
  float* data() const { return ptr_; }
  void upload(const std::vector<float>& values) {
    if (values.size() != count_) throw std::runtime_error("buffer size mismatch");
    check(cudaMemcpy(ptr_, values.data(), count_ * sizeof(float), cudaMemcpyHostToDevice),
          "upload");
  }
  std::vector<float> download() const {
    std::vector<float> values(count_);
    check(cudaMemcpy(values.data(), ptr_, count_ * sizeof(float), cudaMemcpyDeviceToHost),
          "download");
    return values;
  }

 private:
  float* ptr_ = nullptr;
  std::size_t count_ = 0;
};

std::vector<float> fk(const Kinematics& kinematics, const std::array<float, 7>& q,
                      std::array<float, 3>& position, std::array<float, 4>& quaternion) {
  float* d_q = nullptr;
  float* d_pos = nullptr;
  float* d_quat = nullptr;
  float* d_jac = nullptr;
  const auto info = kinematics.info();
  check(cudaMalloc(reinterpret_cast<void**>(&d_q), 7 * sizeof(float)), "q");
  check(cudaMalloc(reinterpret_cast<void**>(&d_pos), info.num_tool_frames * 3 * sizeof(float)),
        "pos");
  check(cudaMalloc(reinterpret_cast<void**>(&d_quat), info.num_tool_frames * 4 * sizeof(float)),
        "quat");
  check(cudaMalloc(reinterpret_cast<void**>(&d_jac),
                   info.num_tool_frames * 6 * 7 * sizeof(float)), "jac");
  check(cudaMemcpy(d_q, q.data(), 7 * sizeof(float), cudaMemcpyHostToDevice), "q copy");
  kinematics.forward(d_q, d_pos, d_quat, d_jac, 1, 1);
  check(cudaDeviceSynchronize(), "fk sync");
  check(cudaMemcpy(position.data(), d_pos, 3 * sizeof(float), cudaMemcpyDeviceToHost), "pos");
  check(cudaMemcpy(quaternion.data(), d_quat, 4 * sizeof(float), cudaMemcpyDeviceToHost),
        "quat");
  cudaFree(d_q); cudaFree(d_pos); cudaFree(d_quat); cudaFree(d_jac);
  return {};
}

struct PlanBuffers {
  PlanBuffers(int dof, int knots, int steps, int tools)
      : current(dof), goal_pos(tools * 3), goal_quat(tools * 4), knots(knots * dof),
        q(steps * dof), v(steps * dof), a(steps * dof), j(steps * dof),
        success(1), used_rrt(1), rrt_iters(1), cost(1), collision(1), dt(1),
        pos_error(1), ori_error(1) {}
  Buffer current, goal_pos, goal_quat, knots, q, v, a, j;
  Buffer success, used_rrt, rrt_iters, cost, collision, dt, pos_error, ori_error;
};

float max_joint_error(const std::array<float, 7>& a, const std::array<float, 7>& b) {
  float error = 0;
  for (int i = 0; i < 7; ++i) error = std::max(error, std::abs(a[i] - b[i]));
  return error;
}

void run_target(const std::string& artifact, const std::array<float, 7>& start,
                const std::array<float, 7>& goal) {
  SceneConfig scene_config;
  scene_config.max_cuboids = 0;
  scene_config.max_capsules = 0;
  scene_config.max_cylinders = 0;
  scene_config.max_meshes = 0;
  scene_config.max_voxel_grids = 0;
  scene_config.max_voxels_per_grid = 0;
  Scene scene(scene_config);
  MotionPlanningConfig planner_config;
  planner_config.trajectory_optimization.num_seeds = 4;
  planner_config.rrt.max_plans = 4;
  planner_config.trajectory_optimization.finetune_attempts = 0;
  MotionPlanner planner(artifact, planner_config, &scene);
  Kinematics kinematics(artifact);
  const auto info = planner.info();
  std::array<float, 3> position{};
  std::array<float, 4> quaternion{};
  fk(kinematics, goal, position, quaternion);
  PlanBuffers buffers(info.num_dof, info.num_knots, info.trajectory_steps,
                      planner.robot_info().num_tool_frames);
  buffers.current.upload(std::vector<float>(start.begin(), start.end()));
  buffers.goal_pos.upload(std::vector<float>(position.begin(), position.end()));
  buffers.goal_quat.upload(std::vector<float>(quaternion.begin(), quaternion.end()));
  PoseMotionPlanningRequest request;
  request.current_position = buffers.current.data();
  request.goal_positions = buffers.goal_pos.data();
  request.goal_quaternions = buffers.goal_quat.data();
  request.optimized_knots = buffers.knots.data();
  request.trajectory_position = buffers.q.data();
  request.trajectory_velocity = buffers.v.data();
  request.trajectory_acceleration = buffers.a.data();
  request.trajectory_jerk = buffers.j.data();
  request.success = reinterpret_cast<uint8_t*>(buffers.success.data());
  request.used_rrt = reinterpret_cast<uint8_t*>(buffers.used_rrt.data());
  request.rrt_iterations = reinterpret_cast<int32_t*>(buffers.rrt_iters.data());
  request.cost = buffers.cost.data();
  request.cost = buffers.cost.data();
  request.collision_cost = buffers.collision.data();
  request.trajectory_dt = buffers.dt.data();
  request.position_error = buffers.pos_error.data();
  request.orientation_error = buffers.ori_error.data();
  planner.plan_pose(request);
  uint8_t planned = 0;
  check(cudaMemcpy(&planned, request.success, 1, cudaMemcpyDeviceToHost), "plan success");
  if (!planned) {
    std::cout << "plan failed for target\n";
    return;
  }
  const auto dt_host = buffers.dt.download();
  const float dt = dt_host[0];
  MpcConfig mpc_config;
  mpc_config.trajectory_dt = dt;
  MpcController mpc(planner, mpc_config);
  Buffer goal_q(info.num_dof);
  goal_q.upload(std::vector<float>(goal.begin(), goal.end()));
  mpc.reset(goal_q.data(), buffers.knots.data());

  mpc_test::FakeQuinticController controller(start);
  std::vector<float> initial(5 * 7, 0.0F);
  for (int i = 0; i < 5; ++i) std::copy(start.begin(), start.end(), initial.begin() + i * 7);
  std::vector<float> zero(5 * 7, 0.0F);
  controller.accept(initial, zero, zero, dt, 4);

  Buffer q(7), v(7), a(7), j(7);
  const float knot_dt = dt * (info.trajectory_steps - 1) / (info.num_knots + 4);
  const float period = 16.0F * dt;
  const float handoff = 4.0F * dt;
  bool previous_accepted = false;
  int accepted = 0;
  for (int tick = 0; tick < 300; ++tick) {
    const auto predicted = controller.predict(handoff);
    q.upload(std::vector<float>(predicted.q.begin(), predicted.q.end()));
    v.upload(std::vector<float>(predicted.v.begin(), predicted.v.end()));
    a.upload(std::vector<float>(predicted.a.begin(), predicted.a.end()));
    j.upload(std::vector<float>(predicted.jerk.begin(), predicted.jerk.end()));
    const auto candidate = mpc.step(q.data(), v.data(), a.data(), j.data(),
                                    tick == 0 ? 1 : (previous_accepted ? 4 : 0));
    std::vector<float> cq(info.trajectory_steps * 7), cv(cq.size()), ca(cq.size());
    check(cudaMemcpy(cq.data(), candidate.position, cq.size() * sizeof(float),
                     cudaMemcpyDeviceToHost), "cq");
    check(cudaMemcpy(cv.data(), candidate.velocity, cv.size() * sizeof(float),
                     cudaMemcpyDeviceToHost), "cv");
    check(cudaMemcpy(ca.data(), candidate.acceleration, ca.size() * sizeof(float),
                     cudaMemcpyDeviceToHost), "ca");
    bool executable = false;
    if (candidate.feasible) {
      try {
        controller.accept(cq, cv, ca, candidate.trajectory_dt, 4);
        executable = true;
      } catch (const std::exception&) {
        executable = false;
      }
    }
    if (executable) {
      mpc.accept();
      controller.advance(period);
      previous_accepted = true;
      ++accepted;
    } else {
      controller.decelerate(0.2F);
      controller.advance(period);
      previous_accepted = false;
    }
    const auto state = controller.predict();
    const float error = max_joint_error(state.q, goal);
    if (error < 0.005F && std::abs(*std::max_element(state.v.begin(), state.v.end())) < 0.01F) {
      std::cout << "reached error=" << error << " accepted=" << accepted << " tick=" << tick << "\n";
      return;
    }
  }
  const auto state = controller.predict();
  std::cout << "not reached error=" << max_joint_error(state.q, goal)
            << " accepted=" << accepted << "\n";
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  const std::array<float, 7> start{0.0F, -1.3F, 0.0F, -2.5F, 0.0F, 1.5F, 0.8F};
  const std::array<float, 7> goal_a{0.5F, -1.0F, 0.2F, -2.2F, 0.2F, 1.2F, 0.8F};
  const std::array<float, 7> goal_b{-0.4F, -1.5F, -0.2F, -2.7F, -0.2F, 1.7F, 0.6F};
  try {
    std::cout << "A: "; run_target(argv[1], start, goal_a);
    std::cout << "B: "; run_target(argv[1], start, goal_b);
  } catch (const std::exception& e) {
    std::cerr << "exception: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
