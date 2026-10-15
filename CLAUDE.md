# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test

Uses [Pixi](https://pixi.sh) for environment and task management. All commands must run inside the `humble` environment.

```bash
# First-time setup
pixi install
pixi run setup-colcon

# Build
pixi run -e humble build

# Test (depends on build)
pixi run -e humble test

# Smoke test installation
pixi run -e humble test-install

# Generate VS Code env file (for debugger)
pixi run gen-vscode-env
```

Build artifacts go to `build_humble/` and `install_humble/`. Tests use pytest via `colcon test --python-testing pytest`; test files live in `src/*/test/`.

The pixi environment auto-sources `install_humble/setup.sh` on activation ([scripts/activate_overlay.sh](scripts/activate_overlay.sh), POSIX so it works from bash and zsh), so no manual `source` is needed. A package newly added by a build is only visible in a fresh `pixi run` / `pixi shell`.

To run a single test file directly:
```bash
pixi run -e humble pytest src/<package>/test/test_foo.py
```

Tests for the python-only packages in `python/`: `pixi run -e humble test-py`.

## Architecture

**Two-repository system:**
- **franka-server** (separate repo, mini-PC): real-time ROS2 C++ controllers, libfranka interface, hardware drivers. Never edit server-side controller logic here.
- **adl-ros2** (this repo): high-level Python client API for research workflows — control, planning, teleoperation, experiment pipelines.

**Keep all real-time loops and torque-level logic in franka-server. Keep all orchestration, planning, and research workflow code in arm_client.**

## Key Source Files

| File | Role |
|------|------|
| [src/arm_client/arm_client/robot.py](src/arm_client/arm_client/robot.py) | Primary `Robot` class — state I/O, controller switching, trajectory planning/execution, teleop helpers |
| [src/arm_client/arm_client/robot_config.py](src/arm_client/arm_client/robot_config.py) | FR3 defaults: joint names, home config, topics, IK/planner weights/limits |
| [src/arm_client/arm_client/control/controller_switcher.py](src/arm_client/arm_client/control/controller_switcher.py) | controller_manager service client (load/configure/list/switch) |
| [src/arm_client/arm_client/control/joint_trajectory_controller_client.py](src/arm_client/arm_client/control/joint_trajectory_controller_client.py) | `FollowJointTrajectory` action and joint trajectory topic publishing |
| [src/arm_client/arm_client/planning/ik_pyroki.py](src/arm_client/arm_client/planning/ik_pyroki.py) | Sequential dense IK and online planning via PyRoki + JAX/jaxls |
| [configs/controllers/](configs/controllers/) | Runtime parameter YAMLs for `osc_pd`, `joint_space`, `fr3_pose` controllers |

## ROS2 Packages in `src/`

- **arm_client** — main user-facing Python API
- **arm_interfaces** — ROS2 message/service/action definitions (ament_cmake)
- **acoustic_sensing** — TCP bridge for BeagleBone acoustic sensing
- **adg_ros2_utils** — general ROS2 utilities
- **robot_tasks** — task automation and experiment pipelines
- **robot_arm/franka_rim** — Reduced Interface Model computation for FR3
- **teleop/teleop** — main teleoperation package
- **teleop/inverse3_ros2** — Inverse3 haptic device integration
- **teleop/network_sim** — network simulation for teleoperation

## Python-only Packages in `python/`

Plain Python packages (no `package.xml`), installed into the pixi env as editable `[pypi-dependencies]` — not built by colcon.

- **pyrim** — ROS-free RIM core: interface frame, `RIMCalculator`, thread-safe `RIMIntegrator`, `RIM` facade
- **rim_teleop** — RIM bilateral teleoperation for FR3; imports `arm_client`/`rclpy`, so it needs the colcon overlay sourced

Put new code in `src/` as a colcon package if it needs ROS resources (msgs, launch, `ament_index` lookups); otherwise in `python/` with a `pyproject.toml` added to `pixi.toml`. Test with `pixi run -e humble test-py`.

## Controller Strategy

**Default precision strategy for Cartesian tracking:**
1. Generate Cartesian waypoints in Python
2. Solve sequential dense IK → joint trajectory (via PyRoki + JAX)
3. Execute through `joint_trajectory_controller`

Controller switching is explicit via controller_manager services. `Robot.home()` and trajectory execution automatically switch to `joint_trajectory_controller` when needed.

**Server-side controllers to be aware of:** `joint_trajectory_controller`, `osc_pd_controller`, `joint_space_controller`, `fr3_pose_controller`, `cartesian_impedance_controller`, `joint_impedance_controller`.

## IK and Planning

- Use PyRoki + JAX (`jax`, `jaxls`, `jaxlie`, `jax_dataclasses`) as the primary IK/planning stack.
- Preferred pattern: sequential dense IK with warm-start between waypoints and a similarity/rest cost to prevent joint flips.
- Always maintain strong pose tracking costs (position + orientation) and joint limit constraints.
- Seed each trajectory segment with the previous segment's endpoint to maintain joint continuity.
- Keep endpoint-accuracy safeguards (final-point exact IK solve).

## Robot API Patterns

- Call `Robot.wait_until_ready()` before commanding motion.
- `Robot.move_to()` dispatches based on active controller: joint-space active → IK + joint trajectory; otherwise → Cartesian interpolation + target pose publishing.
- `Robot.execute_sequence()` for ordered multi-step experiment pipelines (supports trajectories and callables).
- `Robot.online_planning()` for online IK in teleoperation loops.
- Avoid simultaneous command streams (continuous pose publishing vs. trajectory mode conflict).

## ROS Topics/Services (arm_client)

**Published:** `target_pose` (PoseStamped), `target_joint` (JointState), `target_trajectory` (CartesianTrajectory), `target_wrench` (WrenchStamped), `target_twist` (TwistStamped)

**Subscribed:** `current_pose` (PoseStamped), `joint_states` (JointState), `/fr3/franka_robot_state_broadcaster/external_wrench_in_base_frame` (WrenchStamped)

**Services:** `controller_manager/{load,configure,list,switch}_controller`

**Action:** `joint_trajectory_controller/follow_joint_trajectory`

## Data Logging

Do not assume synchronized sample counts between pose and joint streams. Always use explicit per-stream timestamps when computing derivatives, delays, or tracking error.

## RIM Haptic Teleoperation (`rim_teleop` + `pyrim`)

### What it does
Implements real-time bilateral haptic teleoperation on the real FR3 via a Reduced Interface Model (RIM). The RIM acts as a physically consistent 1 kHz inertial proxy between the Haply Inverse3 haptic device and the 50 Hz robot controller, resolving unilateral contacts (e.g., table) locally at haptic rate without waiting for the control loop.

### Three-loop architecture
| Loop | Rate | Thread | Responsibility |
|------|------|--------|---------------|
| Haptic | 1 kHz | `_haptic_loop` | Read Inverse3 → `rim.add_leader_state()` → `rim.step()` (semi-implicit Euler + contact projection); send interface force (or measured robot force) to device |
| RIM | 1 kHz | `_rim_loop` | Logs RIM proxy state and interface force (integration happens in the haptic loop) |
| Control | 50 Hz | `_control_loop` | Pinocchio dynamics → `rim.update(model)` (`RIMCalculator` + integrator refresh); publish `target_pose` + `target_wrench` to `osc_pd_controller` |

`rim` is a `pyrim.RIM` facade owning an `InterfaceFrame` (geometry: `project`/`lift`/`compose`), a `RIMCalculator`, and a thread-safe `RIMIntegrator`.

### Key files
| File | Role |
|------|------|
| [python/rim_teleop/src/rim_teleop/orchestrator.py](python/rim_teleop/src/rim_teleop/orchestrator.py) | Top-level multi-rate coordinator; owns all threads |
| [python/pyrim/src/pyrim/rim.py](python/pyrim/src/pyrim/rim.py) | `RIM` facade tying frame, calculator and integrator together |
| [python/pyrim/src/pyrim/integrator.py](python/pyrim/src/pyrim/integrator.py) | `RIMIntegrator` — thread-safe RIM integrator with unilateral contact projection |
| [python/pyrim/src/pyrim/calculator.py](python/pyrim/src/pyrim/calculator.py) | `RIMCalculator` — computes `M_eff`, `z_i`, `f_eff` from `DynModel` |
| [python/rim_teleop/src/rim_teleop/adapters/model_estimator_adapter.py](python/rim_teleop/src/rim_teleop/adapters/model_estimator_adapter.py) | Pinocchio-based dynamics; produces `DynModel` from live robot state |
| [python/rim_teleop/src/rim_teleop/adapters/teleop_interface_adapter.py](python/rim_teleop/src/rim_teleop/adapters/teleop_interface_adapter.py) | Extracts the interface axis from the 3D Inverse3 device; applies force cap and scale |
| [python/rim_teleop/configs/rim_teleop_default.yaml](python/rim_teleop/configs/rim_teleop_default.yaml) | All tunable parameters: rates, interface stiffness/damping, contact surface, Inverse3 origin |

### Robot command pattern (OSC)
The control loop sends two continuous commands via `Robot`:
- `robot.set_target(position=...)` — RIM proxy position along interface axis; home position on the other two axes
- `robot.set_target_wrench(force=...)` — feedforward coupling force `−λi` along interface axis (paper eq. 20)

`osc_pd_controller` on franka-server subscribes to `target_pose` and `target_wrench` and applies `τ = J^T(Fcmd − λi)`.

### Sign convention for feedforward
`rim.get_interface_force()` returns `K*(x_rim − x_leader) + D*(v_rim − v_leader)`. The control loop currently sends `feedforward = rim.lift(-interface_force)` (see `_control_loop` in the orchestrator). If the robot moves away from the surface on contact instead of pressing in, the sign is wrong.

### Key tuning parameters
- `contact_surface` in the YAML: z-coordinate of the physical surface in the robot base frame — must be set correctly for each experimental setup
- `stiffness` / `damping` (Ki=1000, Di=90): virtual coupling between haptic device and RIM proxy
- `force_cap`: maximum force rendered to the Inverse3 device (default 12 N)

## Development Conventions

- Extend `Robot` and the planning/control helper modules rather than duplicating ROS plumbing in new packages.
- Reuse existing controller parameter clients and config YAMLs — do not hardcode gains in scripts.
- When adding new controller support, update both the arm_client API surface/config references and the server-side controller configuration/launch assumptions.
- Reference examples in `examples/` (especially `00_home.py`, `02_move_to.py`, `07_follow_trajectory.py`, `08_execute_sequence.py`) to stay aligned with current API behavior.
