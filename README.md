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
| `curobo_planner_cpp` | Thin `rclcpp` node + `PlanPose` service + `planned_trajectory` publisher |
| `curobo_planner_interfaces` | Minimal `PlanPose.srv` (no nav2 dependency) |

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

```bash
source /opt/ros/humble/setup.bash
source ~/curobo_ros_ws/install/setup.bash

ros2 run curobo_planner_cpp planner_node --ros-args \
  -p artifact_dir:=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/tickbot \
  -p base_frame:=right_arm_base
```

Or via launch:

```bash
ros2 launch curobo_planner_cpp planner.launch.py \
  artifact_root:=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts \
  robot:=tickbot \
  base_frame:=right_arm_base
```

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
| `scene_max_cuboids` | `8` | Scene capacity; no scene editor in v0.1 |
| `joint_states_topic` | `~/joint_states` | Start-state source |
| `publish_trajectory` | `true` | Publish `~/planned_trajectory` |

Path resolution order: `artifact_dir` → `artifact_root/robot` → error.

## Scope (v0.1)

Included:

- One robot artifact, one `MotionPlanner`, one `plan_pose` path
- Joint-name-aware start state
- Executable trajectory from `timeopt_trajectory()` (padding trimmed)
- Generic `PlanPose` service and `JointTrajectory` output

Not included (next steps):

- `target_poses` / `target_joint_positions`
- Scene obstacles / `add_object`
- IK/FK services
- RViz path markers
- Execution / FPC / JTC
- MPC (`curobo_mpc_cpp` will be a separate package)

## Notes

- Artifacts are GPU-architecture specific (`sm_89` here). Re-export for another GPU.
- Locked joints are fixed at export time.
- The C++ runtime uses internal CUDA graphs; the first request is slower.
- `PlannerCore` can be unit-tested without ROS; see `curobo_core_cpp/examples/plan_pose_smoke.cpp`.

## License

Apache-2.0.
