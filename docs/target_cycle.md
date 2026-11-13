# A/B MPC target cycle

`target_cycle.py` publishes two Cartesian goals alternately:

```text
goal A -> wait until TCP is near A -> dwell -> goal B
goal B -> wait until TCP is near B -> dwell -> goal A
```

If a goal is not reached within `max_hold`, it switches anyway so the demo does
not freeze.

## Run

```bash
# 1
ros2 launch curobo_execution_cpp fake_fpc.launch.py

# 2
ros2 launch curobo_mpc_cpp mpc.launch.py \
  artifact_dir:=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka

# 3: TF for panda_link0 -> panda_hand
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat /opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf)"

# 4
ros2 launch curobo_mpc_cpp target_cycle.launch.py
```

Or use RViz instead of step 3; the `RobotModel` publishes the same TF.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `target_a` | `[0.30, 0.30, 0.30, ...]` | A pose `x y z qx qy qz qw` |
| `target_b` | `[0.30, -0.30, 0.30, ...]` | B pose |
| `position_tolerance` | `0.01` | TCP distance; planner reaches exactly, MPC near |
| `orientation_tolerance` | `0.20` | quaternion angle (rad) |
| `dwell` | `0.6` | hold time at a reached target |
| `max_hold` | `5.0` | switch anyway after this many seconds |
| `base_frame` | `panda_link0` | TF base |
| `tool_frame` | `panda_hand` | TCP frame |

## Notes

- A and B are at the same height, symmetric about the base X axis, in +X.
- The MPC node uses planner execution for far targets and MPC for near targets.
- Planner execution reaches the exact A/B points; target cycle then switches.
- This is the target driver only; no bat obstacle.
