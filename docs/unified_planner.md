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

Live targets are now enqueued by the ROS callback and applied by the MPC
producer thread, so `plan_pose/reset_cartesian/update_cartesian_target` no longer
run inside the executor. `/mpc_goal_marker` is published; `/mpc_costs` reports
available FK/collision terms. Action feedback currently carries state only;
TF-based position feedback is deferred to a dedicated thread.

## Planning / execution split

The MPC node no longer owns JTC. It sends short segments through:

```text
/curobo_execution/execute_segment   curobo_execution_interfaces/action/ExecuteSegment
```

The execution node (`curobo_executor`) owns the JTC adapter; MPC only owns
`PlannerCore`, World, and `MpcController`.
