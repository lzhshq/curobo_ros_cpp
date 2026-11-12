# curobo_ros_cpp

A **planning-only** C++ ROS 2 front end for cuRobo Fast (cuRobo 0.9) exported artifacts.

The node is intentionally thin:

```text
ROS 2 service / topics
        │
        ▼
curobo_planner_cpp::PlannerNode   (ROS adapter: parameters, messages, lifecycle)
        │
        ▼
curobo_core_cpp::PlannerCore      (ROS-free: artifact + Scene + MotionPlanner)
        │
        ▼
cuRobo C++ runtime (libcurobo_robot_runtime.so)
```

It does **not** execute trajectories, drive hardware, run MPC, or contain a
robot-specific model. A robot is selected by an exported artifact directory.

## Packages

| Package | Role |
|---|---|
| `curobo_core_cpp` | ROS-free planning library (`PlannerCore`) |
| `curobo_planner_cpp` | Thin `rclcpp` node + `PlanPose`/scene services + `planned_trajectory` publisher |
| `curobo_planner_interfaces` | Minimal `PlanPose`, `AddObject`, `RemoveObject`, `ClearObjects` services (no nav2 dependency) |

`curobo_msgs` is intentionally not required: its `GetVoxelGrid.srv` depends on
`nav2_msgs`, which a planning-only node does not need.

## Prerequisites

- ROS 2 Humble
- A built cuRobo C++ runtime (`cpp_runtime/build/robot/libcurobo_robot_runtime.so`)
- One exported robot artifact (`cpp_runtime/robot/artifacts/<robot>`)
- CUDA 12.3+ and a matching GPU architecture

Export an artifact from Python first (example):

```bash
python cpp_runtime/robot/tools/export_robot.py \
  --robot-config curobo/content/configs/robot/franka.yml \
  --tool-frame panda_hand \
  --out cpp_runtime/robot/artifacts/franka
```

## Build

```bash
mkdir -p ~/curobo_ros_ws/src
cd ~/curobo_ros_ws/src
git clone git@github.com:lzhshq/curobo_ros_cpp.git

cd ~/curobo_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to curobo_planner_cpp \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCUROBO_RUNTIME_ROOT=/home/vitai/curobo-fast/cpp_runtime
```

`CUROBO_RUNTIME_ROOT` must point to the cuRobo `cpp_runtime` checkout; it is used
for `robot/include` and `build/robot/libcurobo_robot_runtime.so`.

## Run

Edit `curobo_planner_cpp/config/planner.yaml` once (set `artifact_dir`, or
`artifact_root` + `robot`), then launch:

```bash
source /opt/ros/humble/setup.bash
source ~/curobo_ros_ws/install/setup.bash
ros2 launch curobo_planner_cpp planner.launch.py
```

`planner.launch.py` loads the installed `config/planner.yaml` by default.

To keep one config per robot, create e.g. `config/tickbot.yaml` and
`config/franka.yaml`, then:

```bash
ros2 launch curobo_planner_cpp planner.launch.py \
  config:=$HOME/curobo_ros_ws/src/curobo_ros_cpp/curobo_planner_cpp/config/tickbot.yaml
```

Command-line launch arguments override the YAML when non-empty:

```bash
ros2 launch curobo_planner_cpp planner.launch.py \
  artifact_dir:=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka \
  base_frame:=panda_link0
```

For `ros2 run` without launch:

```bash
ros2 run curobo_planner_cpp planner_node --ros-args \
  --params-file $HOME/curobo_ros_ws/src/curobo_ros_cpp/curobo_planner_cpp/config/planner.yaml
```

The YAML key must match the node name (`curobo_planner`):

```yaml
curobo_planner:
  ros__parameters:
    artifact_dir: /home/vitai/curobo-fast/cpp_runtime/robot/artifacts/tickbot
    base_frame: right_arm_base
```

`artifact_dir` is a construction-time parameter. Changing it with
`ros2 param set` does **not** reload the artifact; restart the node (or switch
`config:=...`) to change robots.

Startup logs the artifact, DOF, joint names, and tool frames. `node_is_available`
becomes `true` after initialization.

## Call the service

```bash
ros2 service call /curobo_planner/generate_trajectory \
  curobo_planner_interfaces/srv/PlanPose \
  "{start_pose: {name: [j1, j2, j3, j4, j5, j6, j7],
                  position: [0.0, 0.0, 0.0, 0.74175, 0.0, 0.0, 0.0]},
    target_pose: {position: {x: 0.246493, y: 0.0480554, z: 0.787101},
                  orientation: {x: -0.0344956, y: 0.360789, z: 0.0935708, w: 0.9273}}}"
```

- `target_pose` is in the artifact base frame.
- `orientation` is `geometry_msgs/Pose` xyzw; the core converts it to wxyz.
- If `start_pose.position` is empty, the latest `~/joint_states` message is used.
- Names are reordered to the artifact joint order; unknown names fail the call.
- The response contains `trajectory` (`sensor_msgs/JointState[]`) and `dt`.

The node also publishes the same trajectory as
`trajectory_msgs/JointTrajectory` on `~/planned_trajectory` for a separate
execution/controller node.

A small Python client is included:

```bash
python3 scripts/call_plan.py \
  --target 0.246493 0.0480554 0.787101 -0.0344956 0.360789 0.0935708 0.9273 \
  --start 0 0 0 0.74175 0 0 0
```

## Scene services (v0.2)

The planner owns one `Scene` with fixed capacities. Obstacles are expressed in
the artifact base frame, using `geometry_msgs/Pose` (xyzw) and
`geometry_msgs/Vector3`.

| Type | `dimensions` |
|---|---|
| `CUBOID` (0) | full extents `[dx, dy, dz]` |
| `SPHERE` (1) | `[radius, _, _]` |
| `CAPSULE` (2) | `[radius, height, _]` |
| `CYLINDER` (3) | `[radius, height, _]` |
| `MESH` (4) | not supported in v0.2 |

Add a table:

```bash
ros2 service call /curobo_planner/add_object   curobo_planner_interfaces/srv/AddObject   "{type: 0, name: 'table',
    pose: {position: {x: 0.5, y: 0.0, z: 0.1}, orientation: {w: 1.0}},
    dimensions: {x: 0.8, y: 1.0, z: 0.05}}"
```

Remove it:

```bash
ros2 service call /curobo_planner/remove_object   curobo_planner_interfaces/srv/RemoveObject "{name: 'table'}"
```

Clear everything:

```bash
ros2 service call /curobo_planner/clear_objects   curobo_planner_interfaces/srv/ClearObjects "{}"
```

- Adding/removing updates the same `Scene`; the planner is not rebuilt.
- Scene mutation and `PlanPose` are serialized by one mutex in the node.
- Adding an object at the goal makes `PlanPose` fail; removing it restores success.
- Duplicate names, non-positive dimensions, unknown types, `MESH`, and capacity
  overflow return `success=false` with a message.

Run the scene regression without ROS:

```bash
./build/curobo_core_cpp/scene_smoke /path/to/artifact
```

V0.2 verification (same binary, two artifacts):

```text
tickbot: 24/24 checks passed, 0 failures
franka:  24/24 checks passed, 0 failures
ROS:     add_object -> PlanPose failed -> remove_object -> PlanPose succeeded
```

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `artifact_dir` | `""` | Absolute artifact path; wins over root/robot |
| `artifact_root` | `""` | Directory containing `<robot>/` artifacts |
| `robot` | `""` | Artifact subdirectory name |
| `tool_frame` | `""` | Empty = first tool frame in the artifact |
| `base_frame` | `""` | `frame_id` on the published trajectory |
| `max_attempts` | `10` | `PoseMotionPlanningRequest::max_attempts` |
| `enable_graph_attempt` | `1` | RRT fallback attempt index |
| `outer_iterations` | `2` | TrajOpt initial solve/check passes |
| `scene_max_cuboids` | `8` | Cuboid capacity |
| `scene_max_capsules` | `8` | Capsule + sphere capacity |
| `scene_max_cylinders` | `8` | Cylinder capacity |
| `joint_states_topic` | `~/joint_states` | Start-state source |
| `publish_trajectory` | `true` | Publish `~/planned_trajectory` |

Path resolution order: `artifact_dir` → `artifact_root/robot` → error.

## Scope (v0.2)

Included:

- One robot artifact, one `MotionPlanner`, one `plan_pose` path
- Joint-name-aware start state
- Executable trajectory from `timeopt_trajectory()` (padding trimmed)
- `PlanPose` service and `JointTrajectory` output
- Primitive scene: `CUBOID`, `SPHERE`, `CAPSULE`, `CYLINDER`
- `AddObject` / `RemoveObject` / `ClearObjects` services

Not included (next steps):

- `MESH` / Warp backend
- `target_poses` / `target_joint_positions`
- TF transforms for object poses
- IK/FK services
- RViz path markers / ghost robot
- Execution / FPC / JTC
- MPC (`curobo_mpc_cpp` will be a separate package)

## Notes

- Artifacts are GPU-architecture specific (`sm_89` here). Re-export for another GPU.
- Locked joints are fixed at export time.
- The C++ runtime uses internal CUDA graphs; the first request is slower.
- `PlannerCore` can be unit-tested without ROS; see `curobo_core_cpp/examples/plan_pose_smoke.cpp` and `scene_smoke.cpp`.

## License

Apache-2.0.
