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

The services and action server are present in the MPC node. The live
`mpc_goal` action path is still being debugged; do not treat this as a completed
unified solver yet.
