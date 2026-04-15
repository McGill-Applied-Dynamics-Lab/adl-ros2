---
applyTo: '**'
---
# Franka Controller --- Copilot Instructions

## System Architecture
- **Two-repo architecture**:
  - **franka-server (mini PC, real-time)**: ROS 2 control stack and low-level C++ controllers interfacing with libfranka.
  - **adl-ros2 / arm_client (this repo, high-level Python)**: user-facing API for control, planning, teleoperation, controller switching, and experiment logic.
- **Strict boundary**: keep real-time loops and torque-level controller logic in franka-server; keep orchestration/planning/research workflows in arm_client.
- **Project purpose**: provide a practical and reproducible FR3 control client for robotics research, RL, and experiment automation.

## Source Of Truth Files
- `src/arm_client/arm_client/robot.py`: primary client API (`Robot`) for state I/O, controller switching, trajectory planning/execution, teleop helpers.
- `src/arm_client/arm_client/robot_config.py`: FR3 defaults (joint names, home config, topics, IK/planner weights/limits).
- `src/arm_client/arm_client/control/controller_switcher.py`: controller_manager service client (`load/configure/list/switch`).
- `src/arm_client/arm_client/control/joint_trajectory_controller_client.py`: `FollowJointTrajectory` action and direct joint trajectory topic publishing.
- `src/arm_client/arm_client/planning/ik_pyroki.py`: sequential dense IK planning and online planning with PyRoki + JAX/jaxls.
- `configs/controllers/`: runtime parameter sets for `osc_pd`, `joint_space`, and `fr3_pose` controllers.

## Controller Strategy
- **Default precision strategy**: for accurate Cartesian tracking, prefer:
  1. Cartesian waypoint generation in Python.
  2. Sequential IK to joint trajectory.
  3. Execute through `joint_trajectory_controller`.
- **Controller switching is explicit** through controller_manager services; `Robot.home()` and trajectory execution paths switch to `joint_trajectory_controller` automatically when needed.
- **Server-side controllers to be aware of** (franka-server):
  - `joint_trajectory_controller`
  - `osc_pd_controller`
  - `joint_space_controller`
  - `fr3_pose_controller`
  - `cartesian_impedance_controller`
  - `joint_impedance_controller`

## IK And Planning Conventions
- Use PyRoki + JAX (`jax`, `jaxls`, `jaxlie`, `jax_dataclasses`) as the primary IK/planning stack.
- **Preferred planning pattern**: sequential dense IK with warm-start between waypoints and a similarity/rest cost to prevent joint flips.
- Maintain strong pose tracking costs (position and orientation) and include joint limit constraints.
- Endpoint behavior matters: keep endpoint-accuracy safeguards (final-point exact IK solve) and stable boundary conditions.
- Keep joint continuity across multi-segment trajectories by seeding each segment with the previous segment endpoint.

## ROS Interfaces (Client Expectations)
- **Published by arm_client**:
  - `target_pose` (`PoseStamped`)
  - `target_joint` (`JointState`)
  - `target_trajectory` (`CartesianTrajectory`)
  - `target_wrench` (`WrenchStamped`)
  - `target_twist` (`TwistStamped`)
- **Subscribed by arm_client**:
  - `current_pose` (`PoseStamped`)
  - `joint_states` (`JointState`)
  - `/fr3/franka_robot_state_broadcaster/external_wrench_in_base_frame` (`WrenchStamped`)
- **Services used** (controller_manager):
  - `controller_manager/load_controller`
  - `controller_manager/configure_controller`
  - `controller_manager/list_controllers`
  - `controller_manager/switch_controller`
- **Action used**:
  - `joint_trajectory_controller/follow_joint_trajectory` (`FollowJointTrajectory`)

## Runtime Patterns In Robot API
- `Robot.wait_until_ready()` before commanding motion.
- `Robot.move_to()` dispatches based on active controller:
  - joint-space controller active -> plan IK + send joint trajectory.
  - otherwise -> Cartesian interpolation + target pose publishing.
- `Robot.execute_sequence()` supports ordered trajectories and callables; use it for experiment pipelines.
- `Robot.online_planning()` is the online IK entrypoint for teleoperation loops.
- Avoid simultaneous command streams that conflict (continuous target pose publishing vs trajectory mode).

## Build, Test, And Execution
- Prefer pixi task wrappers in this repo:
  - Build: `pixi run -e humble build`
  - Test: `pixi run -e humble test`
- Common VS Code task for package tests:
  - `ROS2 Test - robot_arm_testing`
- Useful examples to keep aligned with API behavior:
  - `examples/00_home.py`
  - `examples/01_switch_controller.py`
  - `examples/02_move_to.py`
  - `examples/07_follow_trajectory.py`
  - `examples/08_execute_sequence.py`

## Data Logging And Analysis Notes
- Do not assume synchronized sample counts between pose and joint streams.
- Always use explicit timestamps per stream when computing derivatives, delays, or tracking error.

## Agent Guidance For Future Changes
- Preserve the client/server separation above.
- Favor extending `Robot` and planning/control helper modules over duplicating ROS plumbing in new packages.
- Reuse existing controller parameter clients and config YAMLs instead of hardcoding gains in scripts.
- When adding new controller support, update both:
  - arm_client API surface and config references.
  - server controller configuration/launch assumptions.
