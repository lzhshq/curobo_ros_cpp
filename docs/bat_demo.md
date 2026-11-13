# Interactive bat obstacle demo

A Franka MPC demo with a bat-like capsule obstacle that can be dragged in the
RViz Y/Z plane.

## Control

The bat is a capsule with its long axis along world **X**. RViz exposes only
**Y/Z translation**; X is fixed. It is not rotated by the marker.

## Run

```bash
# 1. fake FPC plant
ros2 launch curobo_execution_cpp fake_fpc.launch.py

# 2. MPC with the bat capsule
ros2 launch curobo_mpc_cpp mpc.launch.py \
  artifact_dir:=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka

# 3. interactive marker
ros2 launch curobo_mpc_cpp bat_marker.launch.py

# 4. RViz
ros2 launch curobo_planner_cpp rviz.launch.py \
  base_frame:=panda_link0 \
  robot_description:=/opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf \
  show_robot:=true show_ghost:=true
```

In RViz, enable the `BatControl` InteractiveMarkers display (it is included in
`planner.rviz`). Hover the orange bat and drag the Y/Z axis handles. The marker
publishes `/curobo_demo/bat_pose`; the MPC node updates the scene capsule before
each `step()`.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `bat_length` | 0.6 | capsule tip-to-tip length |
| `bat_radius` | 0.03 | capsule radius |
| `bat_initial_x` | 0.45 | fixed X |
| `bat_initial_y` | 0.0 | initial Y |
| `bat_initial_z` | 0.55 | initial Z |
| `bat_topic` | `/curobo_demo/bat_pose` | marker pose input |

## Important

- Move the bat slowly; a fast teleport can tunnel through the robot.
- Dragging into the robot/target can make MPC infeasible; that is expected.
- The capsule is added once and moved with `Scene::update_obstacle_pose()`; the
  scene is not rebuilt.
- Scene updates and `step()` are serialized by the MPC mutex.
- Simulation only; no real hardware.
