#!/usr/bin/env python3
"""Example showing how to use `Robot.online_planning()` with Inverse3 haptic device for continuous teleoperation."""

import time
import numpy as np

from arm_client.robot import Robot, Pose
from arm_client.teleop.inverse3_teleop import Inverse3Device, Inverse3Config
from arm_client.planning.types import PlannedJointTrajectory

from jaxlie import SE3
import jax.numpy as jnp


def main():
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    print("Switching to joint trajectory controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    # 1) Home
    print("Homing...")
    robot.home()

    # Get initial robot position for clutching
    start_pose: Pose = robot.end_effector_pose
    initial_robot_position = start_pose.position
    # initial_robot_position = np.zeros(3)
    initial_robot_orientation = start_pose.orientation

    # 2) Setup Inverse3 Teleop interface
    print("Initializing Inverse3 teleop device...")

    i3_config = Inverse3Config(
        translation_scale=5.0,  # Maps 1mm I3 movement to 5mm Franka movement
        workspace_center=[0.0, -0.17, 0.16],
        enable_force_feedback=True,
        stiffness=400.0,  # Kp for virtual spring tracking
        force_cap=2.0,  # N
        orientation_default=[0.0, 1.0, 0.0, 0.0],  # w, x, y, z downward
    )

    i3_teleop = Inverse3Device(initial_robot_position=initial_robot_position, config=i3_config)
    i3_teleop.start()

    trajectory_length = 5
    dt = 0.1

    print("Starting teleoperation loop... Press Ctrl+C to stop.")
    try:
        while True:
            t0 = time.time()
            target_position = i3_teleop.target_position
            target_orientation = initial_robot_orientation

            # Plan trajectory continuously via PyRoki IK mapping
            robot.online_planning(
                target_position=target_position,
                target_orientation=target_orientation,
                trajectory_length=trajectory_length,
                dt=dt,
            )

            # Extract generated configurations
            arm_joint_count = len(robot.config.joint_names)
            horizon_positions = robot._online_planning_sols[:, :arm_joint_count]

            # Compute velocities to prevent halting at each coordinate
            horizon_velocities = np.gradient(horizon_positions, dt, axis=0)
            horizon_accelerations = np.gradient(horizon_velocities, dt, axis=0)

            # Send only the immediate immediate next point over ROS to avoid splining overlap collisions
            horizon_times = [dt]
            traj_msg = PlannedJointTrajectory(
                joint_names=robot.config.joint_names,
                time_from_start=horizon_times,
                joint_positions=horizon_positions[:1],
                joint_velocities=horizon_velocities[:1],
                joint_accelerations=horizon_accelerations[:1],
            )
            robot.send_joint_trajectory(traj_msg)

            # Get actual robot position and stream it back as force feedback
            actual_pos = robot.end_effector_pose.position
            i3_teleop.provide_feedback(actual_pos)

            # Fast wait loop keeping the loop frequency bounded (~50Hz)
            elapsed = time.time() - t0
            sleep_time = max(0.001, 0.02 - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping teleoperation loop.")

    print("Sequence execution complete.")
    i3_teleop.stop()
    robot.shutdown()


if __name__ == "__main__":
    main()
