# Testing

## C++ core tests

`curobo_core_cpp` has a gtest target:

```bash
cd ~/curobo_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select curobo_core_cpp \
  --cmake-args \
    -DCUROBO_RUNTIME_ROOT=/home/vitai/curobo-fast/cpp_runtime \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

CUROBO_TEST_ARTIFACT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka \
  ./build/curobo_core_cpp/test_planner_core

CUROBO_TEST_ARTIFACT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/ar5 \
  ./build/curobo_core_cpp/test_planner_core
```

The tests skip if `CUROBO_TEST_ARTIFACT` is not set. They cover:

- plan from the artifact default joint state;
- a cuboid at the goal blocks planning, removal restores it;
- invalid dimensions and `MESH` are rejected.

## Smoke tests

```bash
cd ~/curobo_ros_cpp
./scripts/run_smoke_tests.sh
```

It runs `plan_pose_smoke` and `scene_smoke` for both `franka` and `ar5`
artifacts under `CUROBO_ARTIFACT_ROOT` (default:
`/home/vitai/curobo-fast/cpp_runtime/robot/artifacts`).

`scene_smoke` includes:

- baseline plan;
- 20 add/remove/plan cycles;
- goal blocked by a cuboid;
- duplicate/invalid/capacity checks;
- sphere/capsule/cylinder add/remove;
- `MESH` rejection.

## Bringup tests

```bash
cd ~/curobo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export CUROBO_ARTIFACT_ROOT=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts
colcon test --packages-select curobo_bringup --event-handlers console_direct+
```

It runs:

- descriptor validation for AR5/Franka;
- an end-to-end Franka JTC stack test:
  - starts `robot_stack.launch.py robot:=franka fake:=auto rviz:=false`
  - calls `PlanPose`
  - calls `execute_latest`
  - waits for JTC `result code=0`
  - shuts the stack down

The end-to-end tests use `ROS_DOMAIN_ID=91`/`92`, so they do not collide with a
running demo on domain 79. They skip when `CUROBO_ARTIFACT_ROOT` is unset.

## ROS integration

The fake Franka chain is a manual integration test:

```bash
# terminal 1
ros2 launch curobo_execution_cpp fake_franka.launch.py
# terminal 2
ros2 launch curobo_planner_cpp planner.launch.py robot:=franka base_frame:=panda_link0
# terminal 3
ros2 launch curobo_execution_cpp execution.launch.py \
  executor_config:=$HOME/curobo_ros_cpp/curobo_execution_cpp/config/robots/franka.yaml
# terminal 4
python3 ~/curobo_ros_cpp/scripts/call_plan.py --target 0.2872 0.0 0.5223 0.995 -0.0073 -0.0998 0.0007
ros2 service call /curobo_executor/execute_latest std_srvs/srv/Trigger
ros2 service call /curobo_executor/status std_srvs/srv/Trigger
```

Expected: `PlanPose.success=true`, JTC result `code=0`, and `/joint_states`
ends at the trajectory end.

## Notes

- Tests require a CUDA GPU and a matching artifact; they are not CPU-only.
- CI can run build/lint without a GPU; artifact tests stay local.
- The tests do not cover RViz rendering; RViz is validated manually.

## Headless MPC demo regression

The demo can be started without the A/B driver, then checked with TF:

```bash
# terminal 1
ros2 launch curobo_mpc_cpp franka_bat_demo.launch.py autostart_cycle:=false

# terminal 2
ros2 run curobo_mpc_cpp mpc_regression
```

It checks home, a near point, A and B. Expected on the reference machine:

```text
home:    error=0.00 cm  PASS
small_y: error≈2 cm    PASS
a:       error≈1.3 cm  PASS
b:       error≈1.8 cm  PASS
```

The default pass threshold is 2 cm; use `--tolerance` to tighten it.

The script assumes the Franka artifact in the running demo is already exported
for the local GPU. On a new machine set:

```bash
export CUROBO_ARTIFACT_DIR=/path/to/artifacts/franka
```
