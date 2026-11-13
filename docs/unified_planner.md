# Unified planner (single process)

The official `curobo_msgs` MPC interface is embedded directly in the
`curobo_mpc` node. There is no separate facade process.

## Interfaces

```text
/unified_planner/set_planner          curobo_msgs/srv/SetPlanner
/unified_planner/get_planners         curobo_msgs/srv/GetPlanners
/unified_planner/generate_trajectory  curobo_msgs/srv/TrajectoryGeneration
/unified_planner/execute_trajectory   curobo_msgs/action/SendTrajectory
/unified_planner/mpc_goal             geometry_msgs/Pose
```

The current implementation supports MPC only (`SetPlanner.MPC = 1`).

## Unified circle entry

```bash
ros2 launch curobo_mpc_cpp unified_franka_circle.launch.py
```

The node itself owns `PlannerCore`, the World, `MpcController`, and the ROS
interfaces.

## Status

The services, action server, and live `mpc_goal` path are now handled inside the
MPC node. The unified circle demo starts the action and streams the circle
through `/unified_planner/mpc_goal`.

Action feedback currently reports tracking state only. TF-based position-error
feedback was removed because a blocking `lookupTransform` in the single-threaded
executor stalled joint-state callbacks; a non-blocking/dedicated-thread feedback
implementation is still pending.
