// SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "curobo_core_cpp/planner_core.hpp"
#include "curobo_robot_runtime/kinematics.h"

namespace {

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

class PlannerCoreTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* artifact = std::getenv("CUROBO_TEST_ARTIFACT");
    if (artifact == nullptr || std::string(artifact).empty()) {
      GTEST_SKIP() << "set CUROBO_TEST_ARTIFACT to a cuRobo artifact directory";
    }
    artifact_ = artifact;
    curobo::robot::Kinematics kinematics(artifact_);
    dof_ = kinematics.info().num_dof;
    start_ = read_default_joint_position(artifact_, dof_);

    curobo_ros_cpp::PlannerConfig config;
    config.artifact_dir = artifact_;
    config.max_attempts = 5;
    config.max_cuboids = 8;
    config.max_capsules = 8;
    config.max_cylinders = 8;
    core_ = std::make_unique<curobo_ros_cpp::PlannerCore>(config);
    std::string error;
    ASSERT_TRUE(core_->initialize(error)) << error;

    goal_joints_ = start_;
    goal_joints_[0] += 0.2F;
    const auto goal_poses = core_->compute_tool_path(goal_joints_, 1);
    ASSERT_FALSE(goal_poses.empty());
    goal_pose_ = goal_poses.front();
  }

  curobo_ros_cpp::PlanRequest request() const {
    curobo_ros_cpp::PlanRequest request;
    request.start_position = start_;
    request.position = goal_pose_.position;
    request.quaternion = goal_pose_.quaternion;
    return request;
  }

  std::string artifact_;
  int dof_ = 0;
  std::vector<float> start_;
  std::vector<float> goal_joints_;
  curobo_ros_cpp::ToolPose goal_pose_;
  std::unique_ptr<curobo_ros_cpp::PlannerCore> core_;
};

TEST_F(PlannerCoreTest, PlansFromDefaultJointState) {
  const auto result = core_->plan_pose(request());
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_GT(result.num_steps, 0);
  EXPECT_GT(result.dt, 0.0F);
}

TEST_F(PlannerCoreTest, SceneObjectBlocksGoalThenRemovalRestoresPlan) {
  const auto baseline = core_->plan_pose(request());
  ASSERT_TRUE(baseline.success) << baseline.message;

  curobo_ros_cpp::ObjectSpec blocker;
  blocker.type = curobo_ros_cpp::ObjectType::CUBOID;
  blocker.name = "goal_blocker";
  blocker.position = goal_pose_.position;
  blocker.quaternion = goal_pose_.quaternion;
  blocker.dimensions = {0.2F, 0.2F, 0.2F};

  std::string error;
  ASSERT_TRUE(core_->add_object(blocker, error)) << error;
  const auto blocked = core_->plan_pose(request());
  EXPECT_FALSE(blocked.success);

  ASSERT_TRUE(core_->remove_object("goal_blocker", error)) << error;
  const auto unblocked = core_->plan_pose(request());
  EXPECT_TRUE(unblocked.success) << unblocked.message;
  EXPECT_EQ(core_->object_count(), 0);
}

TEST_F(PlannerCoreTest, RejectsInvalidSceneObjects) {
  std::string error;
  curobo_ros_cpp::ObjectSpec object;
  object.type = curobo_ros_cpp::ObjectType::CUBOID;
  object.name = "bad_dims";
  object.dimensions = {0.0F, 0.2F, 0.2F};
  EXPECT_FALSE(core_->add_object(object, error));

  curobo_ros_cpp::ObjectSpec mesh;
  mesh.type = curobo_ros_cpp::ObjectType::MESH;
  mesh.name = "mesh";
  mesh.dimensions = {1.0F, 1.0F, 1.0F};
  EXPECT_FALSE(core_->add_object(mesh, error));
  EXPECT_EQ(core_->object_count(), 0);
}

}  // namespace
