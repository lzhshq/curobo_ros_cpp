# Changelog

## v0.5.1

- `planner.launch.py` reads `CUROBO_ARTIFACT_ROOT` as the default artifact root.
- README documents the four-terminal fake Franka chain.

## v0.5.0

- Added `tools/fake_jtc.py`: a kinematic FollowJointTrajectory server that
  publishes `/joint_states`.
- Added `fake_franka.launch.py` with `panda_joint1..7`.
- Verified Franka plan -> fake JTC -> `/joint_states` end-to-end without MuJoCo.

## v0.4.0

- Added `~/planned_path` (`nav_msgs/Path`) from FK tool poses.
- Added `~/planned_markers` with start/goal spheres.
- Added `rviz/planner.rviz` and `rviz.launch.py`.
- Verified a 22-pose path with frame `right_arm_base`; final pose matches the
  requested target.

## v0.3.0

- Added `curobo_execution_cpp`.
- Added `ExecutionAdapter`, `JtcAdapter`, and `TopicAdapter`.
- Added `TrajectoryExecutor` with explicit `execute_latest`, `cancel`, and
  `status` services.
- Added per-robot execution configs for tickbot and Franka.
- Verified plan-to-JTC execution and cancel against a fake JTC.

## v0.2.0

- Added `AddObject`, `RemoveObject`, and `ClearObjects` services.
- Added primitive scene support: `CUBOID`, `SPHERE`, `CAPSULE`, `CYLINDER`.
- Added fixed-capacity checks and name uniqueness checks.
- Added `scene_smoke` regression: goal blocking, removal, 20 add/remove/plan
  cycles, capacity reuse, invalid dimensions, and mesh rejection.
- Serialized scene mutation and `plan_pose` with one node mutex.
- `MESH` is rejected in v0.2; Warp/mesh support is deferred.

## v0.1.0

- Initial planning-only C++ ROS 2 node.
- `PlanPose` service, `planned_trajectory` publisher.
- Artifact-driven robot selection; no robot-specific C++ code.
- Verified with tickbot and Franka artifacts.
