# Fake hardware / fake JTC

No real robot and no MuJoCo are required for this workflow.

## Components

```text
fake_jtc              kinematic plant: FollowJointTrajectory + /joint_states
curobo_planner        cuRobo planning: PlanPose + /planned_trajectory
curobo_executor       adapter: execute_latest -> JTC goal
robot_state_publisher current model + TF
preview_replay        ghost replay to /preview/joint_states
rviz2                 visualization
```

`fake_jtc` is **not** a physics simulator. It interpolates joint positions and
holds the final pose; there is no gravity, contact, or torque.

## Franka

The fake Franka starts at the folded home:

```text
[0.0, -1.3, 0.0, -2.5, 0.0, 1.5, 0.8]
```

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/curobo_ros_ws/install/setup.bash
export ROS_DOMAIN_ID=79
ros2 launch curobo_execution_cpp fake_franka.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/curobo_ros_ws/install/setup.bash
export ROS_DOMAIN_ID=79
export CUROBO_ARTIFACT_ROOT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts
ros2 launch curobo_planner_cpp planner.launch.py \
  robot:=franka base_frame:=panda_link0
```

Terminal 3:

```bash
ros2 launch curobo_execution_cpp execution.launch.py \
  config:=$HOME/curobo_ros_cpp/curobo_execution_cpp/config/robots/franka.yaml
```

Terminal 4 (plan from the current home, then execute):

```bash
python3 ~/curobo_ros_cpp/scripts/call_plan.py \
  --target 0.2872 0.0 0.5223 0.995 -0.0073 -0.0998 0.0007

ros2 service call /curobo_executor/execute_latest std_srvs/srv/Trigger
ros2 service call /curobo_executor/status std_srvs/srv/Trigger
```

Terminal 5 (optional RViz with model + ghost):

```bash
ros2 launch curobo_planner_cpp rviz.launch.py \
  base_frame:=panda_link0 \
  robot_description:=/opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf \
  show_robot:=true show_ghost:=true \
  ghost_time_scale:=0.2 ghost_loop:=true
```

## Tickbot

Use the AR5 fake hardware stack (historical ROS package name: `tickbot_rokae_hardware`):

```bash
ros2 launch tickbot_rokae_hardware robot_hw.launch.py \
  use_fake_hardware:=true update_rate:=500
```

Then the planner with `robot:=ar5` and
`base_frame:=right_arm_base`, and the executor with
`config:=.../config/robots/ar5.yaml`.

Important: the AR5 artifact uses `j1..j7`, while the JTC uses
`joint1..7`. `curobo_execution_cpp/config/robots/ar5.yaml` maps names and
assumes the same joint order. The ROS package names remain `tickbot_*` for
historical reasons; that is the package, not the arm.

## Terminal roles

| Terminal | Process | Role |
|---|---|---|
| 1 | `fake_franka.launch.py` | fake plant, `/joint_states`, JTC action |
| 2 | `planner.launch.py` | planning only |
| 3 | `execution.launch.py` | sends cached plan to JTC |
| 4 | `call_plan.py` + `execute_latest` | triggers plan/execute |
| 5 | `rviz.launch.py` | visualization |

Planning never moves the robot. Only `execute_latest` sends a JTC goal.

## Current limitations

- Kinematic only: no dynamics/contact.
- The fake JTC holds its last pose; it does not return to home automatically.
- Execution has no external tracking-error supervision; it relies on the JTC
  action result.
- The AR5 JTC uses `open_loop_control: true`, so tracking is not corrected
  by the controller.
