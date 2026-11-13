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

The **planning node** does not execute trajectories or drive hardware. Execution
lives in the separate `curobo_execution_cpp` package. MPC is experimental in
`curobo_mpc_cpp`. Robot-specific models are not part of this repository; a robot
is selected by an exported artifact directory.

Local layout: the source repository is at `~/curobo_ros_cpp`; the colcon
workspace `~/curobo_ros_ws/src/curobo_ros_cpp` is a symlink to it.

## Documentation

- [Architecture](docs/architecture.md): packages, data flow, threading, artifacts
- [Bringup](docs/bringup.md): robot descriptor and one-command stack
- [MPC](docs/mpc.md): official timing, C++ spike, frequency budget
- [Bat demo](docs/bat_demo.md): RViz Y/Z interactive bat obstacle
- [Target cycle](docs/target_cycle.md): alternating A/B MPC goals
- [RViz](docs/rviz.md): tube path, robot model, ghost, launch arguments
- [Fake hardware](docs/fake_hardware.md): fake Franka/AR5 JTC and terminals
- [Troubleshooting](docs/troubleshooting.md): common errors and checks
- [Testing](docs/testing.md): C++ gtest, smoke tests, ROS integration

## Packages

| Package | Role |
|---|---|
| `curobo_core_cpp` | ROS-free planning library (`PlannerCore`) |
| `curobo_planner_cpp` | Thin `rclcpp` node + `PlanPose`/scene services + `planned_trajectory` publisher |
| `curobo_execution_cpp` | Trajectory executor + JTC/topic adapters |
| `curobo_planner_interfaces` | Minimal `PlanPose`, `AddObject`, `RemoveObject`, `ClearObjects` services (no nav2 dependency) |
| `curobo_bringup` | Robot descriptors + combined planner/executor/RViz launch |
| `curobo_mpc_cpp` | Experimental producer/consumer C++ MPC node |

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
  --packages-up-to curobo_bringup curobo_planner_cpp curobo_execution_cpp curobo_mpc_cpp \
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
The default YAML keeps `artifact_dir` empty; a machine-local artifact root can
be supplied through `CUROBO_ARTIFACT_ROOT`:

```bash
export CUROBO_ARTIFACT_ROOT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts
ros2 launch curobo_planner_cpp planner.launch.py \
  robot:=franka base_frame:=panda_link0
```

To switch robots without editing the default YAML:

```bash
export CUROBO_ARTIFACT_ROOT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts
ros2 launch curobo_planner_cpp planner.launch.py \
  robot:=ar5 base_frame:=right_arm_base
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
    artifact_dir: /home/vitai/curobo-fast/cpp_runtime/robot/artifacts/ar5
    base_frame: right_arm_base
```

`artifact_dir` is a construction-time parameter. Changing it with
`ros2 param set` does **not** reload the artifact; restart the node (or switch
`planner_config:=...`) to change robots.

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
AR5:   24/24 checks passed, 0 failures
franka:  24/24 checks passed, 0 failures
ROS:     add_object -> PlanPose failed -> remove_object -> PlanPose succeeded
```

## Execution (v0.3)

Execution is a separate package. The planner only publishes a
`trajectory_msgs/JointTrajectory`; `curobo_execution_cpp` consumes it and sends
it through an adapter.

```text
/curobo_planner/planned_trajectory
        │
        ▼
curobo_executor
  ├─ JtcAdapter   → control_msgs/action/FollowJointTrajectory
  └─ TopicAdapter → trajectory_msgs/JointTrajectory topic
```

Start it with a robot config:

```bash
ros2 launch curobo_execution_cpp execution.launch.py \
  executor_config:=$HOME/curobo_ros_ws/src/curobo_ros_cpp/curobo_execution_cpp/config/robots/ar5.yaml
```

Then, after a successful `PlanPose`:

```bash
ros2 service call /curobo_executor/execute_latest std_srvs/srv/Trigger
ros2 service call /curobo_executor/status std_srvs/srv/Trigger
ros2 service call /curobo_executor/cancel std_srvs/srv/Trigger
```

- Execution is **explicit** (`execute_latest`); planning does not move the robot.
- `JtcAdapter` sends a `FollowJointTrajectory` goal and reports the action result.
- `TopicAdapter` publishes the trajectory for a driver bridge (curobo_ros style).
- `time_scale < 1` slows down the trajectory; `> 1` speeds it up.
- `joint_names` maps artifact names to controller names. AR5 uses
  `j1..j7` in the artifact and `joint1..7` in JTC; positions are assumed to be
  in the same order.
- `open_loop_control: true` in the AR5 JTC means tracking-error supervision
  must be done outside JTC; this adapter currently relies on the action result.

The tested chain:

```text
PlanPose (22 points) → execute_latest → JTC accepted → result code=0
cancel while running → result code=-1 canceled
```

## RViz (v0.4)

After a successful `PlanPose`, the planner also publishes:

| Topic | Type | Content |
|---|---|---|
| `~/planned_path` | `nav_msgs/Path` | end-effector path from FK |
| `~/planned_markers` | `visualization_msgs/MarkerArray` | start (red) and goal (green) |

Start RViz with the matching fixed frame:

```bash
ros2 launch curobo_planner_cpp rviz.launch.py base_frame:=right_arm_base
```

For Franka:

```bash
ros2 launch curobo_planner_cpp rviz.launch.py base_frame:=panda_link0
```

The default RViz config shows:

- `Path` on `/curobo_planner/planned_path` (disabled by default)
- `MarkerArray` on `/curobo_planner/planned_markers`
- `RobotModel` disabled by default

The trajectory is rendered as **thin 3D cylinders** (radius
`path_marker_radius`, default 0.006 m) between consecutive tool poses, plus
red/green start/goal spheres. The flat `Path` display is kept for diagnostics
but disabled by default.

To show a Franka model, start `robot_state_publisher` through the same launch
with a URDF:

```bash
ros2 launch curobo_planner_cpp rviz.launch.py \
  base_frame:=panda_link0 \
  robot_description:=/opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf \
  show_robot:=true
```

`robot_state_publisher` consumes `/joint_states` from the fake JTC and updates
the TF tree; RViz then shows the Franka model moving with the fake hardware.
This is the same approach as curobo_ros: it does not embed a model inside the
planner; it relies on URDF + `robot_state_publisher` + `/joint_states`.

Add a translucent whole-arm ghost:

```bash
ros2 launch curobo_planner_cpp rviz.launch.py \
  base_frame:=panda_link0 \
  robot_description:=/opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf \
  show_robot:=true show_ghost:=true ghost_time_scale:=0.2
```

`preview_replay` replays `/curobo_planner/planned_trajectory` as
`/preview/joint_states`; a second `robot_state_publisher` uses the same URDF
with `frame_prefix=preview/`, and RViz shows a semi-transparent `GhostRobot`.
`ghost_time_scale < 1` slows the replay; `ghost_loop:=true` repeats it from the
red start to the green goal.

The planner subscribes to `/joint_states` by default. If `PlanPose.start_pose`
is empty, it plans from the current state, so the red start marker and ghost
start match the robot's current/initial pose. The fake Franka starts at
`[0, -1.3, 0, -2.5, 0, 1.5, 0.8]` (folded home), not the singular zero pose.

Scene objects added through `add_object` are published as
`/curobo_planner/scene_markers` (`MarkerArray`) so the table/box/sphere is
visible in RViz.

## Franka fake JTC (no MuJoCo)

`curobo_execution_cpp` ships a kinematic fake JTC: it accepts
`FollowJointTrajectory`, interpolates positions, publishes `/joint_states`, and
returns a standard result. It is **not** a physics simulator; MuJoCo is not
required for the ROS 2 JTC/RViz validation.

The fake Franka chain uses four terminals:

| Terminal | Command | Role |
|---|---|---|
| 1 | `fake_franka.launch.py` | fake plant: JTC action + `/joint_states` |
| 2 | `planner.launch.py robot:=franka` | cuRobo planning; `PlanPose`, `/planned_path` |
| 3 | `execution.launch.py executor_config:=franka.yaml` | sends the cached plan to JTC |
| 4 | `call_plan.py` + `execute_latest` | user trigger; optional RViz |

```bash
source /opt/ros/humble/setup.bash
source ~/curobo_ros_ws/install/setup.bash
export ROS_DOMAIN_ID=79
export CUROBO_ARTIFACT_ROOT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts

# terminal 1
ros2 launch curobo_execution_cpp fake_franka.launch.py

# terminal 2
ros2 launch curobo_planner_cpp planner.launch.py \
  robot:=franka base_frame:=panda_link0

# terminal 3
ros2 launch curobo_execution_cpp execution.launch.py \
  executor_config:=$HOME/curobo_ros_ws/src/curobo_ros_cpp/curobo_execution_cpp/config/robots/franka.yaml
```

Plan and execute:

```bash
python3 scripts/call_plan.py   --target 0.158068 0.032042 0.636847 -0.984526 -0.0915269 -0.148579 0.0160041   --start 0 -1.3 0 -2.5 0 1.5 0.8
ros2 service call /curobo_executor/execute_latest std_srvs/srv/Trigger
ros2 service call /curobo_executor/status std_srvs/srv/Trigger
```

Verified: 10 trajectory points, result `code=0 trajectory completed`, and
`/joint_states` ends at the planned `panda_joint1..7` positions.

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
| `publish_path` | `true` | Publish `~/planned_path` and markers |
| `path_marker_radius` | `0.006` | Radius of the 3D tube marker (m) |
| `publish_scene_markers` | `true` | Publish `~/scene_markers` for scene objects |
| `joint_states_topic` | `/joint_states` | Current state source for an empty `start_pose` |
| `joint_name_map` | `""` | Controller→artifact names for the start state |

Path resolution order: `artifact_dir` → `artifact_root/robot` → error.

## Scope (v0.7)

Included:

- One robot artifact, one `MotionPlanner`, one `plan_pose` path
- Joint-name-aware start state
- Executable trajectory from `timeopt_trajectory()` (padding trimmed)
- `PlanPose` service and `JointTrajectory` output
- Primitive scene: `CUBOID`, `SPHERE`, `CAPSULE`, `CYLINDER`
- `AddObject` / `RemoveObject` / `ClearObjects` services
- `curobo_execution_cpp`: JTC and topic adapters
- RViz thin-cylinder path + start/goal markers
- Optional Franka `RobotModel` via `robot_state_publisher`
- Translucent ghost trajectory replay (`preview_replay`)
- Scene object markers for `add_object`
- Kinematic fake JTC for Franka/AR5 (`tools/fake_jtc.py`)

Not included (next steps):

- `MESH` / Warp backend
- `target_poses` / `target_joint_positions`
- TF transforms for object poses
- IK/FK services

## Notes

- Artifacts are GPU-architecture specific (`sm_89` here). Re-export for another GPU.
- Locked joints are fixed at export time.
- The C++ runtime uses internal CUDA graphs; the first request is slower.
- `PlannerCore` can be unit-tested without ROS; see `curobo_core_cpp/examples/plan_pose_smoke.cpp` and `scene_smoke.cpp`.

## License

Apache-2.0.

## Franka bat demo

```bash
ros2 launch curobo_mpc_cpp franka_bat_demo.launch.py
```

See [docs/franka_bat_demo.md](docs/franka_bat_demo.md).
