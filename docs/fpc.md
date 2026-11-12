# FPC (ForwardCommandController)

`FpcAdapter` streams joint positions to a ros2_control
`ForwardCommandController`:

```text
planned JointTrajectory
        │
        ▼
FpcAdapter timer @ publish_rate
        │
        ▼
std_msgs/Float64MultiArray
/position_forward_controller/commands
        │
        ▼
forward position controller
```

## Difference from JTC

| | JTC | FPC |
|---|---|---|
| Transport | `FollowJointTrajectory` action | `Float64MultiArray` topic |
| Controller interpolates | yes | no; adapter streams at `publish_rate` |
| Result | action result (`code=0`) | no action result |
| `completed` meaning | controller reached goal | stream finished |
| Tracking error | controller/action | not observed by the adapter |
| Use case | classic planned motion | servo/MPC-like streaming |

FPC **cannot confirm hardware tracking**. The adapter reports `completed` when
it has published the last sample; it does not prove the robot reached it.

## Descriptor

`curobo_bringup/config/robots/franka_fpc.yaml`:

```yaml
name: franka_fpc
artifact: franka
base_frame: panda_link0
tool_frame: panda_hand

joints:
  artifact: [panda_joint1, ..., panda_joint7]
  controller: [panda_joint1, ..., panda_joint7]
  urdf: [panda_joint1, ..., panda_joint7]

execution:
  adapter: fpc
  command_topic: /position_forward_controller/commands
  publish_rate: 100.0
  time_scale: 1.0

visualization:
  urdf: /opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf

fake_launch: package://curobo_execution_cpp/launch/fake_fpc.launch.py
```

## Run

```bash
export CUROBO_ARTIFACT_ROOT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts
ros2 launch curobo_bringup robot_stack.launch.py \
  robot:=franka_fpc fake:=auto
```

`fake:=auto` starts `fake_fpc.py`, a kinematic plant that subscribes to the
command topic and republishes `/joint_states`. It is only for tests/demos.

For a real FPC controller:

- set `fake_launch: ""` in the descriptor;
- start your real controller/hardware;
- point `execution.command_topic` at its command topic;
- keep `publish_rate` at or below the controller's update rate.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `adapter` | `jtc` | `jtc`, `topic`, or `fpc` |
| `command_topic` | `/execute_trajectory` | FPC command topic (descriptor overrides) |
| `publish_rate` | `100.0` | FPC stream rate (Hz) |
| `time_scale` | `1.0` | trajectory time scaling before streaming |
| `joint_names` | `[]` | controller order; empty = use trajectory order |

## Limitations

- No action result / tracking error.
- No feedback-based correction.
- If the controller stalls, the adapter still completes the stream.
- For final motion validation use JTC; FPC is for streaming/servo paths.
