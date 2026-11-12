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

## MPC node

`curobo_mpc_cpp` contains an **experimental closed-loop node**:

```bash
# terminal 1
ros2 launch curobo_execution_cpp fake_fpc.launch.py

# terminal 2
ros2 launch curobo_mpc_cpp mpc.launch.py \
  artifact_dir:=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka

# terminal 3: send a small Cartesian goal in panda_link0
ros2 topic pub --once /curobo_mpc/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: panda_link0}, \
    pose: {position: {x: 0.161283, y: 0.0, z: 0.641847}, \
           orientation: {x: -0.988745, y: 0.007219, z: -0.149434, w: 0.001091}}}"
```

It:

1. waits for `/joint_states`;
2. calls `MpcController::reset_cartesian()` on a goal;
3. runs `step()` at `mpc_rate` until feasible;
4. calls `validate_trajectory()` on the result;
5. streams **four knot intervals** at `command_rate` to the FPC topic;
6. calls `accept()` at the handoff boundary and replans from the measured
   joint state;
7. repeats until `max_handoffs` or `/curobo_mpc/cancel`.

The first handoff is `4 * (trajectory_steps - 1) / (num_knots + 4)` samples
(16 samples for Franka). `executed_knots` is 4 after each handoff, matching the
MPC fixture's advance.

Current limitations:

- no hardware admission/ACK: `accept()` is called after the command window
  finishes, not after a controller acknowledgement;
- no terminal-pose convergence check; it stops at `max_handoffs` (default 200)
  or cancel;
- the seed is the current joint state repeated; difficult goals may need a
  better seed or `constrain_path=false`;
- goal is assumed to already be in the artifact base frame (no TF transform);
- command window is 4 knots (~0.48 s for dt=0.03), not yet tuned to
  curobo_ros's 0.24 s;
- no RViz goal interaction yet.

Next tuning:

1. convergence stop (TCP error) instead of `max_handoffs`;
2. command-window sweep (1, 2, 4, 8 knots) against fake FPC;
3. real hardware admission/ACK and braking validation;
4. only then real hardware.
