---
applyTo: '**'
---
# adl-ros2 Copilot Instructions

## Architecture & Design Principles
- **Client-Server Separation**: 
  - **Server**: Runs the real-time ROS 2 hardware interface and low-level C++ controllers (e.g., `joint_trajectory_controller`, `cartesian_impedance_controller`).
  - **Client (This Project)**: `adl-ros2` acts as the overarching high-level Python client. It does not contain the real-time control loops. Instead, it computes Inverse Kinematics, handles teleoperation/policy logic, orchestrates controller switching via ROS services, and publishes commands over ROS topics.
- **Project Purpose**: High-level ROS 2 (Humble) python interface for precise cartesian tracking and policy deployment on the Franka Research 3 (FR3) robot.
- **Control Strategy**: 
  - The native `cartesian_impedance_controller` is too compliant for exact trajectory tracking (suffers from droop/steady-state error). 
  - **Preferred**: We use the `joint_trajectory_controller` for stiffness and precision. This requires converting desired Cartesian paths into Joint Space using Inverse Kinematics (IK).
- **Trajectory Planning Methodology**:
  - We strictly avoid **Global Trajectory Optimization** for precise Cartesian paths (like straight lines or spheres). When Cartesian tracking competes as a "soft cost" against joint smoothness and acceleration, the optimizer "cuts corners", resulting in arclike trajectories.
  - **Preferred Pattern (Sequential Dense IK)**: 
    1. Discretize the Cartesian path into fine, dense waypoints (linear interpolation for translation, SLERP for orientation).
    2. Solve IK for each sequentially using **PyRoki / JAX (jaxls)**.
    3. Warm-start waypoint $N$ using the solved joints from waypoint $N-1$, adding a similarity cost to prevent elbow-flipping.
    4. Use `numpy.gradient` to differentiate positions into velocities and accelerations, clamping them to conservative robot limits (e.g. `2.0 rad/s`).

## Key Files & Boundaries
- `src/arm_client/arm_client/robot.py`: The main high-level Python wrapper orchestrating controller switching, ROS 2 topics (`/fr3/joint_trajectory_controller/joint_trajectory`), and robot states.
- `scripts/validate_pose_tracking.py`: Testing ground for trajectory interpolation, sequential PyRoki IK, and visualization using `viser`.
- `src/robot_arm/` & `src/robot_tasks/`: Lower-level FR3 commands and task execution environments.

## Workflows & Commands (pixi + colcon)
- **Build**: `colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Debug'` (runs from workspace root context).
- **Run Tests**: Select the `ROS2 Test - robot_arm_testing` VS Code task to run Python tests using `colcon test --event-handlers console_direct+ --packages-select robot_arm_testing && xunit-viewer -r build/robot_arm_testing/test_results`.
- **Data Recording**: Pose streams exist on `/fr3/current_pose` (`PoseStamped`) and `/fr3/franka/joint_states` (`JointState`). When plotting, note they arrive at different rates; track time derivatives explicitly (`joint_timestamps` vs `pose_timestamps`) instead of assuming matching arrays.

## Conventions
- **Math/IK**: Use JAX (`jnp`, `jaxls`, `pk` (pyroki)) extensively over raw numerical methods. Ensure JAX variables are correctly batched/unbatched (`jax.tree.map(lambda x: x[None], robot)` vs `robot`) based on whether you're solving a single state or optimizing a trajectory over time.
- **Node Cleanup**: Disable default rclpy signal handlers if binding custom GUIs or async loops (e.g. `rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)`) to prevent `publisher context invalid` errors on `Ctrl+C`.
