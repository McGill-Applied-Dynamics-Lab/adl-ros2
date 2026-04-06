# Issues Summary

This file summarizes only the fixes outside:

- `src/acoustic_sensing/`
- `examples/`

Changes in those paths are intentionally omitted.

## 1. Direct Pixi Python runs could not load `arm_interfaces` typesupport

File:

- [src/arm_client/arm_client/__init__.py](/home/sni22/Documents/sim2real_adlros/src/arm_client/arm_client/__init__.py)

Problem:

- Running scripts directly with the Pixi Python interpreter failed with:
  `libarm_interfaces__rosidl_typesupport_fastrtps_c.so: cannot open shared object file`
- The preload helper computed the workspace root incorrectly, so it never found
  `install_humble/arm_interfaces/lib`.

Fix:

- Replaced the hardcoded parent traversal with an upward search for
  `install_humble/arm_interfaces/lib`.
- Prepend that directory to `LD_LIBRARY_PATH` if needed.
- Preload all `libarm_interfaces__rosidl_typesupport_*.so` libraries with `ctypes`.

Result:

- Direct Python execution can now resolve the generated ROS 2 interface shared libraries.

## 2. `Robot.wait_until_ready()` subscribed to the wrong default state topics

File:

- [src/arm_client/arm_client/robot_config.py](/home/sni22/Documents/sim2real_adlros/src/arm_client/arm_client/robot_config.py)

Problem:

- The client expected pose on `current_pose`, but the Franka stack in this repo publishes
  the namespaced broadcaster topic `franka_robot_state_broadcaster/current_pose`.
- The wrench topic was also hardcoded as an absolute namespaced path instead of a relative topic.
- Controller parameter client target names were stored with hardcoded `/fr3/...` prefixes.

Fix:

- Changed `current_pose_topic` to `franka_robot_state_broadcaster/current_pose`.
- Changed `current_wrench_topic` to `franka_robot_state_broadcaster/external_wrench_in_base_frame`.
- Changed controller parameter client defaults from absolute `/fr3/...` names to relative controller names:
  `cartesian_impedance_controller` and `joint_impedance_controller`.

Result:

- `Robot(namespace="fr3")` now binds to the actual server topics/services expected by this workspace.

## 3. `Robot` continuously published a seeded pose target even when no pose command was requested

File:

- [src/arm_client/arm_client/robot.py](/home/sni22/Documents/sim2real_adlros/src/arm_client/arm_client/robot.py)

Problem:

- `_callback_current_pose()` initialized `_target_pose` from the first measured pose.
- The 100 Hz target-pose timer treated that seeded state as an active command.
- This caused unintended `target_pose` publishing and interference with controllers during trajectory execution.

Fix:

- Added `_target_pose_command_active` to distinguish:
  measured pose used for readiness/state initialization
  from
  explicit pose commands requested by the client.
- `set_target()` and streamed `move_to()` now mark pose commands as active.
- Auto-seeded `_target_pose` from incoming state does not activate publishing.
- `reset_targets()`, `home()`, and Cartesian trajectory execution clear active pose commands.
- `execute_trajectory()` explicitly clears any latched single-pose target before trajectory mode begins.

Result:

- The client no longer publishes `target_pose` unless a pose command was explicitly requested.

## 4. `wait_until_ready()` timeout message was misleading

File:

- [src/arm_client/arm_client/robot.py](/home/sni22/Documents/sim2real_adlros/src/arm_client/arm_client/robot.py)

Problem:

- It always raised `Timeout waiting for end-effector pose.`
- In practice, readiness depends on current pose, target pose initialization, current joints, and target joint initialization.

Fix:

- Replaced the generic timeout with a message listing which required topics/messages are still missing.

Result:

- Startup failures are now diagnosable from the exception itself.

## 5. Background ROS spin/shutdown handling was brittle

File:

- [src/arm_client/arm_client/robot.py](/home/sni22/Documents/sim2real_adlros/src/arm_client/arm_client/robot.py)

Problem:

- The spin thread was anonymous and not tracked.
- Shutdown races could surface noisy thread exceptions.
- Exiting after errors could leave the node/spin thread in an unclear state.

Fix:

- Store the background thread in `_spin_thread`.
- Guard `_spin_node()` against shutdown-time executor/runtime exceptions.
- Remove the node from the executor in a `finally` block.
- In `shutdown()`, try to destroy the node, call `rclpy.shutdown()`, and join the spin thread briefly.

Result:

- Client shutdown is cleaner and less likely to leave thread-related runtime warnings.

## 6. IK planning could not find `franka_rim` unless the ROS install environment was sourced

File:

- [src/arm_client/arm_client/planning/ik_pyroki.py](/home/sni22/Documents/sim2real_adlros/src/arm_client/arm_client/planning/ik_pyroki.py)

Problem:

- `load_fr3_urdf()` relied solely on `get_package_share_directory("franka_rim")`.
- In this environment, that could fail either because:
  `franka_rim` was not present in the active ament index
  or
  `AMENT_PREFIX_PATH` was unset.

Fix:

- Added `_find_franka_rim_share()`:
  first tries `get_package_share_directory("franka_rim")`
  then falls back to local workspace locations:
  `install_humble/franka_rim/share/franka_rim`
  `src/robot_arm/franka_rim`
- Catch both `PackageNotFoundError` and the `OSError` raised when `AMENT_PREFIX_PATH` is empty.

Result:

- IK planning can now load the FR3 URDF/meshes from the workspace even without a fully sourced ROS environment.
