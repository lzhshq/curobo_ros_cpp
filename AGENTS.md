# AGENTS.md — C++ cuRobo ROS engineering contract

## Mission

This repository is a **C++ ROS 2 wrapper around cuRobo Fast**. It is not a
independent planner architecture. Every non-trivial behavior must be traceable
to one of these references:

```text
CUROBO_RUNTIME_ROOT/                  # cuRobo Fast C++ runtime
  cpp_runtime/robot/include/
  cpp_runtime/robot/src/
  cpp_runtime/robot/tests/

CUROBO_ROS_SOURCE/                    # official curobo_ros (Python reference)
  curobo_ros/planners/
  curobo_ros/core/
  docs/concepts/
```

`CUROBO_RUNTIME_ROOT` is a build-time variable. The official `curobo_ros`
checkout is usually the local `curobo_ros` source tree; use the configured path,
not a hard-coded absolute path in new code.

## Non-negotiable architecture rules

1. **Use cuRobo solvers directly.**
   `MotionPlanner`, `MpcController`, `Kinematics`, `Scene`, and `PlannerCore`
   are the source of truth. Do not reimplement planning/MPC math in ROS code.

2. **One ROS-facing solver architecture.**
   The planning side follows the official `curobo_ros` shape:

   ```text
   World / Scene
   → PlannerCore / MotionPlanner
   → MpcController
   → planning ROS interface
   ```

   Execution is a separate layer:

   ```text
   planning ROS interface
   → execution node / executor
   → JTC or hardware controller
   ```

   Do not add a parallel facade, a second world owner, or a second MPC control
   path as a workaround.

3. **Planning and execution are separate layers.**
   This repository intentionally keeps:

   ```text
   curobo_planner_cpp    planning only
   curobo_mpc_cpp        planning/control only
   curobo_execution_cpp  execution only (JTC/topic adapter, hardware)
   ```

   Rules:

   - Planning/MPC code must not create or own a `JtcAdapter`.
   - Planning/MPC code must not call `FollowJointTrajectory` directly.
   - Execution code must not own `PlannerCore`, `MpcController`, or `Scene`.
   - The boundary is a ROS interface: `JointTrajectory`, command topic, or
     action. It must be explicit and testable.
   - A unified planner facade may expose `execute_trajectory`, but it must
     delegate execution to the execution layer, not bypass it.

   Current known violation: `curobo_mpc_cpp/src/mpc_node.cpp` directly owns a
   `JtcAdapter`. This is a migration target, not a pattern to copy.

4. **Official interfaces are the public contract.**
   Prefer, in this order:

   ```text
   curobo_msgs/srv/SetPlanner
   curobo_msgs/srv/GetPlanners
   curobo_msgs/srv/TrajectoryGeneration
   curobo_msgs/action/SendTrajectory
   geometry_msgs/Pose /unified_planner/mpc_goal
   ```

   Custom `/curobo_mpc/*` topics are internal/test-only unless explicitly
   documented as a temporary migration aid.

5. **MPC semantics come from C++ `mpc.cpp`, not intuition.**
   - `executed_knots` is a **knot count**, not a sample count.
   - `step()` shifts the seed by whole knots and consumes the accepted result.
   - A segment shorter than the B-spline support can be flat and produce no
     motion. Align segment length with `samples_per_knot`.
   - Preserve last-action velocity/acceleration when feeding the next state,
     as official `curobo_ros` `ReactiveController` does.

6. **No blocking planning inside ROS callbacks.**
   ROS callbacks may enqueue state/goal updates. Expensive
   `plan_pose`/`reset_cartesian`/`update_cartesian_target` work belongs to the
   control-loop/producer thread (or a dedicated worker). A single-threaded
   executor must keep processing `/joint_states`.

7. **Do not mix open-loop planner behavior into MPC mode.**
   Official `curobo_ros` has separate planner families:

   ```text
   SinglePlanner      open-loop
   ReactiveController closed-loop MPC
   ```

   Far-target planner replay is a custom hybrid. It must not be hidden inside
   the MPC action contract. If retained, it must be a separately named mode or
   an explicit outer waypoint/replan layer.

8. **World ownership.**
   Obstacle mutation goes through the shared `PlannerCore` object registry and
   `Scene`. Dynamic obstacles use `update_object_pose` without rebuilding the
   planner. Do not create a separate MPC-only obstacle system.

## Required behavior for changes

- Read the referenced C++/Python implementation before changing interface or
  timing semantics.
- Keep changes small and traceable. State which reference justifies a change.
- Do not introduce a new architecture, thread model, message, or control loop
  without first documenting the reference behavior and the required deviation.
- Preserve user changes in the working tree; do not revert unrelated files.
- Update regression tests or the demo verification when behavior changes.

## Known traps found in this repository

- Hard-coded `executed_knots = 4` while changing `command_steps`.
- Zeroing measured velocity/acceleration each control cycle.
- Running `plan_pose` synchronously inside `mpc_goal`/action callbacks.
- Calling blocking TF `lookupTransform` from a single-threaded feedback timer.
- Computing predicted paths in the same executor as the MPC control loop.
- Publishing a new action target without preserving action/cancel state.
- Treating a demo bat capsule as the world model instead of using the generic
  obstacle update path.
- Coupling a planning/MPC node directly to `JtcAdapter`, violating the
  planning/execution boundary.

## Verification baseline

At minimum, preserve:

```text
core tests
bringup tests
headless MPC A/B regression
single-target near MPC convergence
unified service/action smoke test
```

Do not claim a behavior is complete based only on RViz appearance.
