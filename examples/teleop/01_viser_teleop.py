#!/usr/bin/env python3
"""Example showing how to use `Robot.online_planning()` with IK-planned continuous trajectories."""

import time
import numpy as np

from arm_client.robot import Robot
from arm_client.teleop.viser_teleop import ViserTeleop
from arm_client.planning.types import PlannedJointTrajectory

from jaxlie import SE3
import jax.numpy as jnp

import jax
import os

LOOP_HZ = 0.1

# Add near the top of your script, before any JAX calls
jax.config.update("jax_compilation_cache_dir", "/tmp/jax_cache")
os.makedirs("/tmp/jax_cache", exist_ok=True)


def main():
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    print("Switching to joint trajectory controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    # 1) Home
    print("Homing...")
    robot.home()

    # Start teleop interface
    viser_teleop = ViserTeleop(robot)
    viser_teleop.start()

    trajectory_length = 5
    dt = 0.1
    loop_dt = 1.0 / LOOP_HZ
    loop_count = 0

    BUFFER_S = 0.05  # how far ahead the first point is scheduled

    # Single epoch for the whole session — never reset this
    t_epoch = time.monotonic()

    print("Starting teleoperation loop...")
    try:
        while True:
            t_loop_start = time.monotonic()

            target_position = viser_teleop.target_position
            target_orientation = viser_teleop.target_orientation

            # Plan trajectory continuously
            robot.online_planning(
                target_position=target_position,
                target_orientation=target_orientation,
                trajectory_length=trajectory_length,
                dt=dt,
            )

            # Send immediate command to controller via a stream

            # Get the full planned horizon specifically formatted for the arm
            arm_joint_count = len(robot.config.joint_names)
            horizon_positions = robot._online_planning_sols[:, :arm_joint_count]
            horizon_velocities = np.gradient(horizon_positions, dt, axis=0)
            horizon_accelerations = np.gradient(horizon_velocities, dt, axis=0)

            # Compute times relative to t_epoch, not relative to now
            # This means each message extends the same timeline — no resets
            t_now_rel = time.monotonic() - t_epoch
            horizon_times = [t_now_rel + BUFFER_S + (i + 1) * dt for i in range(trajectory_length)]

            traj_msg = PlannedJointTrajectory(
                joint_names=robot.config.joint_names,
                time_from_start=horizon_times,
                joint_positions=horizon_positions,
                joint_velocities=horizon_velocities,
                joint_accelerations=horizon_accelerations,
            )

            # Publish trajectory sequentially, allowing built-in ROS splining
            robot.send_joint_trajectory(traj_msg)

            # Visualize the result
            # Here we grab the last planned sequence from the state in robot obj.
            sequence_solutions = robot._online_planning_sols

            # To show actual projected coordinates we can do FK
            link_idx = robot.pyroki_robot.links.names.index(robot.config.ik_target_link_name)
            target_poses_fk = robot.pyroki_robot.forward_kinematics(sequence_solutions)[..., link_idx, :]

            poses = SE3(target_poses_fk)
            target_positions = np.array(poses.translation())
            target_wxyzs = np.array(poses.rotation().wxyz)

            viser_teleop.update_visualization(
                joint_configuration=sequence_solutions[0], target_positions=target_positions, target_wxyzs=target_wxyzs
            )

            elapsed = time.monotonic() - t_loop_start
            sleep_time = loop_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            if loop_count % (0.5 * LOOP_HZ) == 0:
                # print(f"Loop compute time: {elapsed}")
                ...

            loop_count += 1

    except KeyboardInterrupt:
        print("\nStopping teleoperation loop.")

    print("Sequence execution complete.")
    viser_teleop.stop()
    robot.shutdown()


if __name__ == "__main__":
    main()
