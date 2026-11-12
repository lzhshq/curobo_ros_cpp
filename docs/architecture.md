# Architecture

## Layers

```text
ROS 2 clients / RViz
        │
        ▼
curobo_planner_cpp::PlannerNode        ROS adapter, parameters, services
        │
        ▼
curobo_core_cpp::PlannerCore           ROS-free planning library
        │
        ▼
libcurobo_robot_runtime.so             cuRobo Fast C++ runtime
        │
        ▼
artifact directory                     robot model + kernels + tensors
```

Execution is separate:

```text
/curobo_planner/planned_trajectory
        │
        ▼
curobo_execution_cpp::TrajectoryExecutor
        ├─ JtcAdapter   → FollowJointTrajectory action
        ├─ TopicAdapter → JointTrajectory topic
        └─ FpcAdapter   → Float64MultiArray position stream
```

Visualization is separate too:

```text
PlannerNode ──► /planned_path, /planned_markers, /scene_markers ──► RViz
PlannerNode ──► /planned_trajectory ──► preview_replay ──► /preview/joint_states
                                                     └──► preview robot_state_publisher
```

## Packages

| Package | Responsibility | Does not do |
|---|---|---|
| `curobo_core_cpp` | artifact loading, `Scene`, `MotionPlanner`, CUDA buffers, FK tool path | ROS, execution, robot-specific code |
| `curobo_planner_cpp` | `PlanPose`, scene services, path/tube markers, ghost replay | execution, physics |
| `curobo_execution_cpp` | consumes planned trajectories, JTC/topic adapters | planning |
| `curobo_planner_interfaces` | `PlanPose`, `AddObject`, `RemoveObject`, `ClearObjects` | anything else |
| `curobo_bringup` | robot descriptor, mapping validation, combined launch | planning/execution logic |

## Planning data flow

```text
PlanPose::Request
  start_pose (optional) + target_pose
        │
        ▼
PlannerNode
  1. start = request.start_pose or latest /joint_states
  2. reorder joints to artifact order
  3. xyzw → wxyz, normalize
        │
        ▼
PlannerCore::plan_pose()
  4. copy to CUDA buffers
  5. fill PoseMotionPlanningRequest
  6. MotionPlanner::plan_pose()
       IK → TrajOpt / RRT → timeopt
  7. copy timeopt_trajectory (trim padding) to PlanResult
        │
        ▼
PlannerNode
  8. PlanPose::Response (JointState[] + dt)
  9. /planned_trajectory (JointTrajectory)
 10. /planned_path + /planned_markers (FK tube)
```

## State and threading

- `PlannerCore` owns `Kinematics`, `Scene`, `MotionPlanner`, and CUDA buffers.
  Keep it alive for the node lifetime.
- The node uses a single-threaded executor (`rclcpp::spin`), so service
  callbacks are serialized.
- `plan_mutex_` also guards `plan_pose`, scene add/remove/clear, and FK path
  publishing, so a scene mutation cannot interleave with a plan.
- Execution adapters are asynchronous; `JtcAdapter` reports the action result
  through callbacks.

## Artifacts

An artifact is per robot **and per GPU architecture**:

```text
artifacts/franka/
  robot_kinematics_manifest.json
  *.cubin
  *.f32
  *.bin
```

- `artifact_dir` is resolved at construction time; changing it requires a node
  restart.
- `CUROBO_RUNTIME_ROOT` is a **build-time** path to the cuRobo `cpp_runtime`
  checkout.
- `CUROBO_ARTIFACT_ROOT` is a **launch-time** convenience for
  `artifact_root/robot`.

## Why planning and execution are separate

- Planning is request/response; execution is a control/transport loop.
- A different robot only needs a different artifact and execution config.
- Fake JTC, real JTC, and topic bridges can be swapped without touching the
  planner.
- MPC will be a separate `curobo_mpc_cpp` package, not part of `PlannerCore`.
