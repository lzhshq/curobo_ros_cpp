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
- JTC executes the accepted segment; it does not run the optimizer at 100 Hz.

## MPC node

`curobo_mpc_cpp` now uses a producer/consumer structure, matching the official
curobo_ros pattern:

```text
producer thread (control_rate=30 Hz):
  read /joint_states
  → step()
  → validate_trajectory()
  → store latest proposal

accept poll (20 ms):
  wait until the previous JTC segment is complete
  → accept the latest valid proposal
  → send the next segment to the JTC adapter
  → advance executed_knots
```

The producer never blocks on publishing. Event-driven acceptance avoids the
fixed command-interval gap between JTC segments.

Current configuration:

```yaml
control_rate: 30.0       # desired producer cadence
adapter: jtc             # jtc/topic
trajectory_dt: 0.03      # MPC sample interval
```

`knots_per_interval` is derived from `command_interval / knot_dt`; for Franka
0.24 s / 0.12 s = 2 knots. `executed_knots` advances by that amount on each
accept.

Run:

```bash
# terminal 1
ros2 launch curobo_execution_cpp fake_franka.launch.py

# terminal 2
ros2 launch curobo_mpc_cpp mpc.launch.py \
  artifact_dir:=/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka
```

Admission checks before `accept()`:

- state freshness (`state_timeout`, default 0.2 s);
- fault input (`/curobo_mpc/fault`, `std_msgs/Bool`);
- brake feasibility from the current q/v (`brake_check`,
  `brake_deceleration`); a zero-velocity state passes without a brake solve.

Measured-vs-predicted tracking error is **not** an admission gate. The official
C++ MPC test validates the candidate trajectory instead; gating on measured
tracking error blocked far targets and was removed.

Current limitations:

- no real controller ACK yet; fault is an external Bool input;
- no terminal-pose convergence check; cancel manually;
- the producer uses a 30 Hz target timer, while official curobo_ros runs the
  producer loop as fast as possible and only paces sends; the wall-clock solve
  rate still needs measurement;
- goal is assumed to be in the artifact base frame (no TF transform);
- no RViz goal interaction yet;
- tested against fake JTC only.

## Official-style loop spike

`curobo_mpc_cpp/examples/mpc_loop_spike.cpp` follows the official
`cpp_runtime/robot/tests/mpc_test.cpp` state progression:

```text
plan_pose → seed knots
→ mpc.reset(goal_q, seed_knots)
→ FakeQuinticController
→ step(executed_knots = 1 / 4 / 0)
→ period = 16 * dt, handoff = 4 * dt
→ accept
```

Run:

```bash
./build/curobo_mpc_cpp/mpc_loop_spike \
  /home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka
```

Result on Franka:

```text
A: reached error=0.00035 rad, accepted=43, tick=85
B: reached error=0.00146 rad, accepted=25, tick=47
```

ROS/JTC regression after removing measured-tracking admission and using the
predicted state for planning:

```text
home:    0.00 cm
small_y: 0.08 cm
A:       1.12 cm
B:       1.51 cm
known:   0.96 cm
```

The A/B target cycle uses a 4 cm switch tolerance because the current MPC
converges to ~1–2 cm, not sub-cm.

## Short-segment closed loop

`franka_mpc_closed_loop.launch.py` exercises the MPC closed loop with short
commands. It sets:

```text
command_steps = 12   # samples, 3 knots for Franka
use_measured_state = true
planner_distance_threshold = 100
```

The C++ `MpcController::step()` takes `executed_knots`, not sample count. The
node now derives:

```text
samples_per_knot = (trajectory_steps - 1) / (num_knots + spline_degree)
command_knots = command_steps / samples_per_knot
executed_knots_ = command_knots
```

Using 4 samples (one knot) falls inside the B-spline's initial flat segment and
produces no motion. 12 samples (three knots) is the current tuned value;
16 samples is the safer fallback.

Run one target without the A/B driver:

```bash
ros2 launch curobo_mpc_cpp franka_mpc_closed_loop.launch.py \
  autostart_cycle:=false
```

Reference result at the near goal `(0.161283, 0.04, 0.645)`:

```text
TCP ≈ (0.161, 0.038, 0.645)
```

Still to do: tune `command_steps` down only after checking knot alignment and
B-spline support; do not cut the horizon arbitrarily.

## Live target / circle tracking

`/curobo_mpc/live_goal` updates a Cartesian target **without resetting MPC**.
The first live target calls `reset_cartesian()`; subsequent targets call
`update_cartesian_target()` only. This matches the C++ `MpcController` API.

Demo:

```bash
ros2 launch curobo_mpc_cpp franka_mpc_circle.launch.py
```

It tracks a YZ circle:

```text
center = (0.161283, 0.0, 0.637)
radius = 0.04 m
period = 10 s
target rate = 10 Hz
```

The normal `/curobo_mpc/goal` still performs a full plan/IK/reset for one-shot
goals. Use `/curobo_mpc/live_goal` for continuous target streams.
