// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <string>
#include <vector>

#include "curobo_core_cpp/planner_core.hpp"
#include "curobo_robot_runtime/kinematics.h"
#include "smoke_common.hpp"

namespace {

using curobo_ros_cpp::smoke::forward_kinematics;
using curobo_ros_cpp::smoke::read_default_joint_position;

int failures = 0;

void expect(bool condition, const std::string& label) {
  std::cout << (condition ? "[PASS] " : "[FAIL] ") << label << "\n";
  if (!condition) {
    ++failures;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: scene_smoke ARTIFACT_DIRECTORY\n";
    return 2;
  }

  try {
    const std::string artifact_dir = argv[1];
    curobo::robot::Kinematics kinematics(artifact_dir);
    const auto info = kinematics.info();
    const auto start = read_default_joint_position(artifact_dir, info.num_dof);
    auto goal = start;
    goal[0] += 0.2F;
    const auto goal_pose = forward_kinematics(kinematics, info, goal);

    curobo_ros_cpp::PlannerConfig config;
    config.artifact_dir = artifact_dir;
    config.max_attempts = 5;
    config.max_cuboids = 8;
    config.max_capsules = 8;
    config.max_cylinders = 8;

    curobo_ros_cpp::PlannerCore core(config);
    std::string error;
    if (!core.initialize(error)) {
      std::cerr << "initialize failed: " << error << "\n";
      return 1;
    }

    curobo_ros_cpp::PlanRequest plan_request;
    plan_request.start_position = start;
    plan_request.position = {goal_pose.position[0], goal_pose.position[1],
                             goal_pose.position[2]};
    plan_request.quaternion = {goal_pose.quaternion[0], goal_pose.quaternion[1],
                               goal_pose.quaternion[2], goal_pose.quaternion[3]};

    const auto baseline = core.plan_pose(plan_request);
    expect(baseline.success, "baseline plan in empty scene succeeds");

    curobo_ros_cpp::ObjectSpec cycle_object;
    cycle_object.type = curobo_ros_cpp::ObjectType::CUBOID;
    cycle_object.name = "cycle_blocker";
    cycle_object.position = plan_request.position;
    cycle_object.quaternion = plan_request.quaternion;
    cycle_object.dimensions = {0.2F, 0.2F, 0.2F};
    bool cycles_ok = true;
    for (int i = 0; i < 20; ++i) {
      if (!core.add_object(cycle_object, error) ||
          core.plan_pose(plan_request).success ||
          !core.remove_object("cycle_blocker", error) ||
          !core.plan_pose(plan_request).success) {
        cycles_ok = false;
        break;
      }
    }
    expect(cycles_ok, "20 add/remove/plan cycles stay consistent");

    curobo_ros_cpp::ObjectSpec cuboid;
    cuboid.type = curobo_ros_cpp::ObjectType::CUBOID;
    cuboid.name = "goal_blocker";
    cuboid.position = plan_request.position;
    cuboid.quaternion = plan_request.quaternion;
    cuboid.dimensions = {0.2F, 0.2F, 0.2F};
    expect(core.add_object(cuboid, error), "add cuboid at goal");

    const auto blocked = core.plan_pose(plan_request);
    expect(!blocked.success, "goal inside cuboid is rejected");

    expect(core.remove_object("goal_blocker", error), "remove cuboid");
    const auto unblocked = core.plan_pose(plan_request);
    expect(unblocked.success, "same goal succeeds after removal");

    expect(core.add_object(cuboid, error), "re-add cuboid");
    expect(!core.add_object(cuboid, error), "duplicate name is rejected");
    expect(core.remove_object("goal_blocker", error), "remove cuboid again");

    auto bad = cuboid;
    bad.name = "bad_dims";
    bad.dimensions = {0.0F, 0.2F, 0.2F};
    expect(!core.add_object(bad, error), "non-positive dimensions are rejected");

    curobo_ros_cpp::ObjectSpec sphere;
    sphere.type = curobo_ros_cpp::ObjectType::SPHERE;
    sphere.name = "sphere";
    sphere.position = {0.0F, 0.0F, -0.5F};
    sphere.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
    sphere.dimensions = {0.05F, 0.0F, 0.0F};
    expect(core.add_object(sphere, error), "add sphere");
    expect(core.remove_object("sphere", error), "remove sphere");

    curobo_ros_cpp::ObjectSpec cylinder;
    cylinder.type = curobo_ros_cpp::ObjectType::CYLINDER;
    cylinder.name = "cylinder";
    cylinder.position = {0.0F, 0.0F, -0.5F};
    cylinder.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
    cylinder.dimensions = {0.05F, 0.2F, 0.0F};
    expect(core.add_object(cylinder, error), "add cylinder");
    expect(core.remove_object("cylinder", error), "remove cylinder");

    curobo_ros_cpp::ObjectSpec capsule;
    capsule.type = curobo_ros_cpp::ObjectType::CAPSULE;
    capsule.name = "capsule";
    capsule.position = {0.0F, 0.0F, -0.5F};
    capsule.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
    capsule.dimensions = {0.05F, 0.2F, 0.0F};
    expect(core.add_object(capsule, error), "add capsule");
    expect(core.remove_object("capsule", error), "remove capsule");

    bool capacity_ok = true;
    for (int i = 0; i < 8; ++i) {
      curobo_ros_cpp::ObjectSpec box;
      box.type = curobo_ros_cpp::ObjectType::CUBOID;
      box.name = "cap_" + std::to_string(i);
      box.position = {0.0F, 0.0F, -0.5F - 0.02F * static_cast<float>(i)};
      box.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
      box.dimensions = {0.01F, 0.01F, 0.01F};
      if (!core.add_object(box, error)) {
        capacity_ok = false;
        break;
      }
    }
    expect(capacity_ok, "fill cuboid capacity");
    curobo_ros_cpp::ObjectSpec overflow;
    overflow.type = curobo_ros_cpp::ObjectType::CUBOID;
    overflow.name = "cap_overflow";
    overflow.position = {0.0F, 0.0F, -0.8F};
    overflow.dimensions = {0.01F, 0.01F, 0.01F};
    expect(!core.add_object(overflow, error), "capacity overflow is rejected");
    expect(core.remove_object("cap_0", error), "remove one full-capacity cuboid");
    expect(core.add_object(overflow, error), "reuse capacity after removal");
    expect(core.clear_objects(error), "clear full scene");
    expect(core.object_count() == 0, "capacity test leaves empty scene");

    curobo_ros_cpp::ObjectSpec mesh;
    mesh.type = curobo_ros_cpp::ObjectType::MESH;
    mesh.name = "mesh";
    mesh.dimensions = {1.0F, 1.0F, 1.0F};
    expect(!core.add_object(mesh, error), "mesh is rejected");

    expect(core.object_count() == 0, "scene is empty after cleanup");

    std::cout << "failures=" << failures << "\n";
    return failures == 0 ? 0 : 1;
  } catch (const std::exception& exception) {
    std::cerr << "exception: " << exception.what() << "\n";
    return 1;
  }
}
