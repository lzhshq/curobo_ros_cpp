# Franka bat MPC demo

One command starts the complete demo: fake JTC, MPC, RViz,
the Z-draggable bat capsule, and the alternating A/B target cycle.

```bash
ros2 launch curobo_mpc_cpp franka_bat_demo.launch.py
```

No arguments are required on the development machine. On a new machine set the
artifact once, then the same command works:

```bash
export CUROBO_ARTIFACT_DIR=/path/to/artifacts/franka
ros2 launch curobo_mpc_cpp franka_bat_demo.launch.py
```

The defaults are:

```text
A: (0.30, +0.30, 0.30)
B: (0.30, -0.30, 0.30)
bat: (0.30, 0.00, 0.30), length 0.40, radius 0.03, axis X
```

## What starts

- `fake_jtc` — fake Franka joint trajectory controller;
- `curobo_mpc` — planner for far targets, MPC for local targets;
- `robot_state_publisher` + RViz;
- `bat_marker` — interactive Z-only capsule;
- `target_cycle` — alternating A/B goals after a short startup delay.

## Interact

- Drag the orange bat only along **Z**; X and Y are fixed.
- A/B targets are shown as green/red spheres.
- RViz shows the robot and the actual collision capsule used by cuRobo.

## Optional overrides

Only needed for a different artifact/URDF:

```bash
ros2 launch curobo_mpc_cpp franka_bat_demo.launch.py \
  artifact_dir:=/path/to/artifacts/franka \
  robot_description:=/path/to/panda.urdf
```

## Regression

See [testing.md](testing.md#headless-mpc-demo-regression).
