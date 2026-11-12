# Changelog

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
