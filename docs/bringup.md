# Bringup (robot descriptor)

## One command

```bash
export CUROBO_ARTIFACT_ROOT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts

# AR5: fake hardware is an external stack, so fake:=false
ros2 launch curobo_bringup robot_stack.launch.py robot:=ar5 fake:=false

# Franka: the repo ships a fake JTC
ros2 launch curobo_bringup robot_stack.launch.py robot:=franka fake:=auto
```

`robot_stack.launch.py` starts:

```text
[curobo_planner_cpp/planner_node]
[curobo_execution_cpp/trajectory_executor]
[robot_state_publisher]                  real model
[preview/robot_state_publisher]          ghost model
[preview_replay]                         ghost joint states
[preview_base_tf]                        right_arm_base -> preview/right_arm_base
[rviz2]
```

For AR5, start the external fake hardware first:

```bash
ros2 launch tickbot_rokae_hardware robot_hw.launch.py \
  use_fake_hardware:=true update_rate:=500
```

Then `robot_stack` with `fake:=false`.

## Descriptor

`curobo_bringup/config/robots/<name>.yaml`:

```yaml
name: ar5
artifact: ar5
base_frame: right_arm_base
tool_frame: flange

joints:
  artifact: [j1, j2, j3, j4, j5, j6, j7]
  controller: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
  urdf: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]

execution:
  adapter: jtc
  action: /position_joint_trajectory_controller/follow_joint_trajectory
  time_scale: 1.0
  goal_tolerance: 0.02
  goal_time_tolerance: 2.0

visualization:
  urdf: package://curobo_bringup/config/urdf/ar5.urdf

fake_launch: ""
```

The loader validates:

- artifact directory and `robot_kinematics_manifest.json`;
- descriptor `joints.artifact` matches manifest `joint_names` **in order**;
- `base_frame` matches artifact `base_link`;
- `tool_frame` is in artifact `tool_frames`;
- controller/URDF lists have the same length, no duplicates;
- the URDF contains every `joints.urdf` joint;
- execution action/adapter are present;
- `fake_launch` exists when set.

It then generates the three mappings:

```text
controller -> artifact   planner start-state lookup
artifact   -> controller executor goal names
artifact   -> urdf       ghost replay
```

## New robot

```text
1. export the cuRobo artifact to $CUROBO_ARTIFACT_ROOT/<name>
2. provide a URDF whose active joints match the controller
3. copy curobo_bringup/config/robots/ar5.yaml -> <name>.yaml
4. edit artifact/base_frame/tool_frame/joints/execution/URDF
5. ros2 launch curobo_bringup robot_stack.launch.py robot:=<name>
```

No C++ changes and no manual `joint_name_map` / `ghost_joint_name_map` strings.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `robot` | `ar5` | descriptor name or YAML path |
| `artifact_root` | `$CUROBO_ARTIFACT_ROOT` | artifact root |
| `fake` | `auto` | start `fake_launch` when present |
| `rviz` | `true` | start RViz + ghost |
| `show_robot` | `true` | real RobotModel |
| `show_ghost` | `true` | ghost replay |
| `ghost_time_scale` | `0.2` | ghost speed |
| `ghost_loop` | `true` | loop red-to-green |

## Low-level launches

`robot_stack` is a composition. These remain available:

```bash
ros2 launch curobo_planner_cpp planner.launch.py ...
ros2 launch curobo_execution_cpp execution.launch.py ...
ros2 launch curobo_planner_cpp rviz.launch.py ...
ros2 launch curobo_execution_cpp fake_franka.launch.py
```

Use them when debugging one component or replacing it. `robot_stack` does not
duplicate their node logic; it includes them.
