# rim_teleop

RIM-based bilateral haptic teleoperation for the Franka Research 3.

Implements the DelayRIM algorithm from [*High-Fidelity Haptic Teleoperation in Robotics Simulation via Multi-Rate Reduced Interface Models*]() on real hardware. A 1 kHz inertial proxy (RIM) sits between the Haply Inverse3 haptic device and a 50 Hz OSC robot controller, resolving unilateral contacts locally at haptic rate without waiting for the control loop.

## Installation

The package lives in `python/rim_teleop/` and is installed automatically as part of the pixi environment:

```bash
pixi install
```

It depends on [`pyrim`](../pyrim/) (also in this repo) and `arm_client` (ROS2, `src/arm_client/`).

## Running

All commands require the `humble` pixi environment (ROS2 + arm_client):

```bash
# Dry run — no robot required, tests loops and logging
pixi run -e humble rim_teleop --dry-run

# Real hardware with default config
pixi run -e humble rim_teleop

# Override specific parameters on the command line
pixi run -e humble rim_teleop --axis z --rim-rate 1000 --control-rate 50

# Load a custom config file
pixi run -e humble rim_teleop --config path/to/my_config.yaml
```

### Before the first hardware run

1. **Set `contact_surface`** in `configs/rim_teleop_default.yaml` to the z-coordinate of the physical table surface in the FR3 base frame. Measure this with the robot (e.g. jog the EE to the surface and read `robot.end_effector_pose.position[2]`).
2. Ensure `osc_pd_controller` is running on franka-server.
3. Connect the Haply Inverse3 and confirm the `uri` in the config matches.
4. Do a `--dry-run` first to confirm logging and I3 are working before connecting the arm.
5. **Verify feedforward sign**: if the robot moves *away* from the surface on contact instead of pressing into it, negate `interface_force_1d` in `orchestrator.py:_send_osc_command`.

## Key files

```
python/rim_teleop/
├── src/rim_teleop/
│   ├── orchestrator.py              # Top-level coordinator — owns all threads
│   ├── config.py                    # All configuration dataclasses + YAML loader
│   ├── main.py                      # CLI entrypoint (argparse → orchestrator)
│   ├── monitoring.py                # Per-loop rate monitor
│   ├── filters.py                   # Low-pass filter for joint state smoothing
│   ├── configs/
│   │   └── rim_teleop_default.yaml  # Default parameters (copy and edit for each setup)
│   └── adapters/
│       ├── model_estimator_adapter.py    # Pinocchio dynamics at 50 Hz → DynModel
│       ├── teleop_interface_adapter.py   # Maps Inverse3 3D position → 1D interface axis
│       └── experiment_logger_adapter.py  # MCAP file sink + live Foxglove sink
└── tests/
    ├── test_rim_compute.py    # RIMCalculator shape checks (via pyrim)
    └── test_config_loader.py  # Verifies YAML loading against bundled defaults
```

## Three-loop architecture

| Loop | Rate | Thread | Responsibility |
|------|------|--------|----------------|
| Haptic | 1 kHz | `_haptic_loop` | Read Inverse3 position/velocity → `delay_rim.add_leader_state()`; send interface force to device |
| RIM | 1 kHz | `_rim_loop` | `delay_rim.step()` — semi-implicit Euler integration with LCP contact projection |
| Control | 50 Hz | `_control_loop` | Pinocchio dynamics → `rim_calc.compute()` → `delay_rim.update_rim()`; publish `target_pose` + `target_wrench` to `osc_pd_controller` |

The RIM math (`RIMCalculator`, `DelayRIM`) lives in the standalone [`pyrim`](../pyrim/) package.

## Configuration

Copy `configs/rim_teleop_default.yaml` and edit for your setup. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `interface.axis` | `z` | Interface axis in robot base frame (`x`, `y`, or `z`) |
| `interface.contact_surface` | `0.0` | **Must be set.** Z-coordinate of the physical surface (m) |
| `interface.stiffness` | `1000.0` | Virtual coupling stiffness Ki (N/m) |
| `interface.damping` | `90.0` | Virtual coupling damping Di (Ns/m) |
| `interface.force_cap` | `12.0` | Maximum force rendered to Inverse3 (N) |
| `inverse3.uri` | `ws://localhost:10001` | Haply websocket URI |
| `inverse3.enable_force_feedback` | `false` | Enable haptic force rendering |
| `inverse3.i3_origin` | `[0, -0.17, 0.16]` | Inverse3 base position in robot frame (m) |
| `robot.controller_name` | `osc_pd_controller` | Must match the active controller on franka-server |
| `logging.file_sink.output_dir` | `/home/athena/csirois/data/franka/rim` | Where MCAP runs are saved |

## Data logging

Each run writes an MCAP file to `output_dir/<run_name>/samples.mcap`. Open it in [Foxglove Studio](https://foxglove.dev) for visualization. A live Foxglove sink also streams to `ws://<host>:8765` during the run.

Logged streams:

| Stream | Rate | Contents |
|--------|------|----------|
| `haptic` | 1 kHz | leader position/velocity, force command, deadman state |
| `rim` | 1 kHz | RIM proxy position and velocity |
| `control` | 50 Hz | RIM state, interface force, model timestamp |
| `robot/joint_states` | 50 Hz | q, dq, tau |
| `robot/end_effector/pose` | 50 Hz | EE position and orientation |
| `robot/end_effector/velocity` | 50 Hz | linear and angular twist |
| `robot/end_effector/force` | 50 Hz | external wrench |
| `metrics` | 0.2 Hz | Measured Hz, mean/p95 loop period per thread |
