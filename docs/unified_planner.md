# Unified planner facade

`curobo_unified_planner_cpp` exposes the official `curobo_ros` MPC interface
surface while the existing C++ MPC implementation stays behind it.

## Interfaces

```text
/unified_planner/set_planner          curobo_msgs/srv/SetPlanner
/unified_planner/get_planners         curobo_msgs/srv/GetPlanners
/unified_planner/generate_trajectory  curobo_msgs/srv/TrajectoryGeneration
/unified_planner/execute_trajectory   curobo_msgs/action/SendTrajectory
```

The facade currently supports MPC only:

```text
SetPlanner.MPC = 1
```

`generate_trajectory` in MPC mode sets the target and returns success with an
empty trajectory. `execute_trajectory` starts the live MPC loop and publishes
`state`, `position_error`, `on_target`, and `hold_count` feedback.

## Run

```bash
ros2 launch curobo_unified_planner_cpp unified_franka_mpc.launch.py
```

The current implementation translates to the existing MPC node:

```text
generate_trajectory → /curobo_mpc/goal
execute_trajectory  → /curobo_mpc/live_goal
```

This is a migration facade, not yet a single-process unified solver.
