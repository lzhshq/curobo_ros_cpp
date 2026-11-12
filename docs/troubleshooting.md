# Troubleshooting

## `Package 'curobo_planner_cpp' not found`

The workspace is not sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/curobo_ros_ws/install/setup.bash
ros2 pkg prefix curobo_planner_cpp
```

If the prefix is missing, rebuild:

```bash
cd ~/curobo_ros_ws
colcon build --symlink-install \
  --packages-up-to curobo_planner_cpp curobo_execution_cpp \
  --cmake-args \
    -DCUROBO_RUNTIME_ROOT=/home/vitai/curobo-fast/cpp_runtime \
    -DCMAKE_BUILD_TYPE=Release
```

## `artifact_dir is empty`

The default `planner.yaml` intentionally keeps it empty. Use:

```bash
export CUROBO_ARTIFACT_ROOT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts
ros2 launch curobo_planner_cpp planner.launch.py \
  robot:=franka base_frame:=panda_link0
```

or:

```bash
ros2 launch curobo_planner_cpp planner.launch.py \
  artifact_dir:=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka \
  base_frame:=panda_link0
```

## Robot model is visible but does not move

- `PlanPose` only updates the path; it does not move the robot.
- Run `execute_latest` to send a JTC goal.
- The fake JTC is fast and then holds the final pose; use a larger target or a
  slower execution config to watch it.
- Check `/joint_states` changes and TF `panda_link0 -> panda_link7` exists.

## Robot model is missing

RViz needs `/robot_description` and TF:

```bash
ros2 launch curobo_planner_cpp rviz.launch.py \
  base_frame:=panda_link0 \
  robot_description:=/opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf \
  show_robot:=true
```

The URDF must use the same joint names as `/joint_states`.

## Ghost is missing

- Start RViz with `show_ghost:=true` and a URDF.
- The fixed frame must match `base_frame`.
- `preview_replay` must receive a plan; call `PlanPose` first.
- `ghost_loop:=true` repeats the replay; `ghost_time_scale` slows it down.
- Check `/preview/joint_states` is publishing and `preview/panda_link0` TF
  exists.

## JTC is unavailable

- Check the action:

  ```bash
  ros2 action list | grep follow_joint_trajectory
  ```

- Start the fake JTC or the real controller.
- The default action is
  `/position_joint_trajectory_controller/follow_joint_trajectory`.
- If the action name differs, edit the robot executor YAML.

## Joint name mismatch

Artifact joint names and controller names can differ:

- AR5 artifact: `j1..j7`
- AR5 JTC: `joint1..7`
- Franka artifact: `panda_joint1..7`

Set `joint_names` in the executor YAML. The adapter maps by order; the order
must match.

## `PlanPose` fails from zero pose

The Franka zero pose is nearly singular/self-colliding. Use the folded home:

```text
[0.0, -1.3, 0.0, -2.5, 0.0, 1.5, 0.8]
```

The fake Franka launch already starts there.

## Scene object not visible

- Add it after RViz is running, or replan; the planner republishes
  `/curobo_planner/scene_markers` after each plan.
- `MESH` is not supported; use `CUBOID`/`SPHERE`/`CAPSULE`/`CYLINDER`.
- Check the marker frame matches `base_frame`.
