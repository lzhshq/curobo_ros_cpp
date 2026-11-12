// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace curobo_ros_cpp {

struct PlannerConfig {
  std::string artifact_dir;
  std::string tool_frame;
  int max_cuboids = 8;
  int max_capsules = 8;
  int max_cylinders = 8;
  int max_meshes = 0;
  int max_voxel_grids = 0;
  int max_voxels_per_grid = 0;
  int max_attempts = 10;
  int enable_graph_attempt = 1;
  int outer_iterations = 2;
  int finetune_attempts = 1;
  float finetune_dt_scale = 0.6F;
};

struct PlanRequest {
  std::array<float, 3> position{{0.0F, 0.0F, 0.0F}};
  // wxyz, normalized by the caller.
  std::array<float, 4> quaternion{{1.0F, 0.0F, 0.0F, 0.0F}};
  std::vector<float> start_position;
  std::vector<float> start_velocity;
  std::vector<float> start_acceleration;
};

enum class ObjectType : int8_t {
  CUBOID = 0,
  SPHERE = 1,
  CAPSULE = 2,
  CYLINDER = 3,
  MESH = 4,
};

struct ObjectSpec {
  ObjectType type = ObjectType::CUBOID;
  std::string name;
  std::array<float, 3> position{{0.0F, 0.0F, 0.0F}};
  // wxyz, normalized by add_object().
  std::array<float, 4> quaternion{{1.0F, 0.0F, 0.0F, 0.0F}};
  // CUBOID: full extents [dx,dy,dz]; SPHERE: [radius,_,_];
  // CAPSULE/CYLINDER: [radius,height,_].
  std::array<float, 3> dimensions{{0.0F, 0.0F, 0.0F}};
};

struct ToolPose {
  std::array<float, 3> position{{0.0F, 0.0F, 0.0F}};
  // wxyz
  std::array<float, 4> quaternion{{1.0F, 0.0F, 0.0F, 0.0F}};
};

struct PlanResult {
  bool success = false;
  std::string message;
  int num_dof = 0;
  int num_steps = 0;
  float dt = 0.0F;
  // Row-major [num_steps, num_dof].
  std::vector<float> position;
  std::vector<float> velocity;
  std::vector<float> acceleration;
  float position_error = 0.0F;
  float orientation_error = 0.0F;
  int planning_attempts = 0;
  bool used_rrt = false;
  double planning_ms = 0.0;
};

/**
 * Robot-agnostic planning core.
 *
 * The class owns the exported cuRobo artifact, the scene storage, and one
 * MotionPlanner. It has no ROS dependency so it can be unit-tested and reused
 * by other front ends.
 */
class PlannerCore {
 public:
  explicit PlannerCore(PlannerConfig config);
  ~PlannerCore();

  PlannerCore(const PlannerCore&) = delete;
  PlannerCore& operator=(const PlannerCore&) = delete;

  bool initialize(std::string& error);

  PlanResult plan_pose(const PlanRequest& request);

  // Scene mutation. The caller must serialize these calls with plan_pose().
  bool add_object(const ObjectSpec& object, std::string& error);
  bool remove_object(const std::string& name, std::string& error);
  bool clear_objects(std::string& error);
  std::vector<std::string> object_names() const;
  int object_count() const;

  // Forward kinematics for the selected tool frame. positions is row-major
  // [num_steps, num_dof]. Returns an empty vector on invalid input.
  std::vector<ToolPose> compute_tool_path(const std::vector<float>& positions,
                                          int num_steps) const;

  const std::vector<std::string>& joint_names() const;
  const std::vector<std::string>& tool_frames() const;
  int num_dof() const;
  int num_steps() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace curobo_ros_cpp
