# MPC timing and C++ spike

## Official / reference frequencies

MPC has two different rates; do not conflate them.

| Layer | Setting | Rate |
|---|---|---|
| cuRobo Python default | `optimization_dt=0.02` | **50 Hz** optimization |
| cuRobo C++ default | `MpcConfig.trajectory_dt=0.025` | **40 Hz** sample |
| curobo_ros MPC | `mpc_step_dt=0.03` | **33.3 Hz** optimization |
| curobo_ros command window | `mpc_command_interval=0.24` | **4.17 Hz** execution windows |
| curobo_ros RViz arrow stream | `mpc_goal` publisher | **10 Hz** target updates |

The C++ `MpcController` candidate sample dt range is `0.01–0.1 s`
(`MpcConfig::MINIMUM_KNOT_DT` / `DEFAULT_MAXIMUM_KNOT_DT`).

The optimization rate is the `step()` rate. The command/execution rate is how
often a new accepted command is handed to the robot; curobo_ros deliberately
paces that at 0.24 s on real hardware.

## C++ spike

`curobo_core_cpp/examples/mpc_spike.cpp`:

```bash
cd ~/curobo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/curobo_core_cpp/mpc_spike \
  /home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka 20 0.025
```

It:

- loads the artifact into `MotionPlanner`;
- creates `MpcController` (`trajectory_dt=0.025`);
- builds a linear seed to a joint goal;
- warms the CUDA graph;
- runs 20 timed `step()` calls;
- reports feasibility and latency.

### Results on RTX 4060 Laptop (this machine)

| Robot | dt | mean | median | p95 | implied rate |
|---|---|---|---|---|---|
| Franka | 0.020 | 22.95 ms | 23.12 ms | 25.98 ms | 43.6 Hz |
| Franka | 0.025 | 23.22 ms | 22.31 ms | 26.60 ms | 43.1 Hz |
| Franka | 0.030 | 23.29 ms | 22.52 ms | 26.91 ms | 42.9 Hz |
| AR5 | 0.025 | 27.27 ms | 26.77 ms | 29.52 ms | 36.7 Hz |

All runs were feasible and used **one graph capture**.

## Interpretation

- **50 Hz (20 ms) is not met** on this GPU: p95 is ~26–30 ms.
- **33 Hz (30 ms)** fits with margin for both Franka and AR5.
- Use `optimization_dt`/`trajectory_dt` around **0.03 s (33 Hz)** as the safe
  default on this machine.
- The FPC stream can still publish at **100 Hz**; that only interpolates the
  accepted MPC trajectory, it does not run the optimizer at 100 Hz.

## Next MPC step

Build `curobo_mpc_cpp` around this spike:

1. `MpcController` per artifact/planner.
2. 30–33 Hz `step()` loop from `/joint_states`.
3. `accept()` only after the execution handoff is admitted.
4. `validate_trajectory()` before sending.
5. FPC stream at 100 Hz.
6. RViz goal and fake FPC first; real hardware later.

Open question for the node: command-window pacing. curobo_ros uses 0.24 s
(4.17 Hz) on real hardware; that is a safety/stability choice, not the
optimizer rate.
