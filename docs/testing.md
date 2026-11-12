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

## ROS integration

The fake Franka chain is a manual integration test:

```bash
# terminal 1
ros2 launch curobo_execution_cpp fake_franka.launch.py
# terminal 2
ros2 launch curobo_planner_cpp planner.launch.py robot:=franka base_frame:=panda_link0
# terminal 3
ros2 launch curobo_execution_cpp execution.launch.py \
  config:=$HOME/curobo_ros_cpp/curobo_execution_cpp/config/robots/franka.yaml
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
