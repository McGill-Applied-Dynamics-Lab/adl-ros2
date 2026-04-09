#!/usr/bin/env python3
"""
Test pure end-effector rotation about Z-axis using trajectory planning.

This script demonstrates how to rotate the end-effector without moving
its position in space, using Cartesian waypoint interpolation.
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from arm_client.robot import Robot
from arm_client.planning import generate_linear_waypoints


def rotate_end_effector_z(
    robot: Robot,
    rotation_degrees: float,
    num_waypoints: int = 20,
    execution_time: float = 2.0,
    visualize: bool = True,
) -> bool:
    """
    Rotate the end-effector about the Z-axis by a specified angle.

    Args:
        robot: Robot client instance
        rotation_degrees: Rotation angle in degrees (positive = counterclockwise)
        num_waypoints: Number of interpolation waypoints
        execution_time: Time in seconds to complete the rotation
        visualize: Whether to visualize trajectory before execution

    Returns:
        True if successful, False otherwise
    """
    try:
        print(f"\n{'='*60}")
        print(f"Rotating end-effector {rotation_degrees}° about Z-axis")
        print(f"{'='*60}")

        # Get current pose
        current_pose = robot.end_effector_pose
        print(f"Current position: {current_pose.position}")
        print(f"Current orientation (quat): {current_pose.orientation.as_quat()}")

        # Create rotation: positive angle = counterclockwise about Z
        rotation_about_z = R.from_euler("z", rotation_degrees, degrees=True)

        # Target orientation = current orientation + rotation
        target_orientation = rotation_about_z * current_pose.orientation
        print(f"Target orientation (quat): {target_orientation.as_quat()}")

        # Generate waypoints: same position, interpolated orientation
        print(f"Generating {num_waypoints} waypoints...")
        waypoints = generate_linear_waypoints(
            start_position=current_pose.position,
            start_orientation=current_pose.orientation,
            end_position=current_pose.position,  # Keep position fixed
            end_orientation=target_orientation,
            num_waypoints=num_waypoints,
        )
        print(f"✓ Generated {len(waypoints)} waypoints")

        # Plan trajectory
        print(f"Planning trajectory ({execution_time}s duration)...")
        trajectory = robot.plan_joint_trajectory_from_cartesian(
            waypoints,
            execution_time=execution_time,
        )
        print(
            f"✓ Planned trajectory with {len(trajectory.joint_trajectory.points)} points"
        )

        # Visualize if requested
        if visualize:
            print("Visualizing trajectory...")
            robot.visualize_cartesian_trajectory(
                waypoints,
                trajectory.joint_trajectory,
                title=f"Pure Z-axis Rotation ({rotation_degrees}°)",
            )

        # Execute trajectory
        print(f"Executing rotation...")
        start_time = time.perf_counter()
        robot.execute_trajectory(trajectory)
        elapsed = time.perf_counter() - start_time
        print(f"✓ Rotation completed in {elapsed:.2f}s")

        # Verify final pose
        final_pose = robot.end_effector_pose
        print(f"\nFinal position: {final_pose.position}")
        print(f"Final orientation (quat): {final_pose.orientation.as_quat()}")

        # Check that position didn't change significantly
        pos_error = np.linalg.norm(final_pose.position - current_pose.position)
        print(f"Position error: {pos_error*1000:.2f} mm")

        if pos_error > 0.01:  # 1cm threshold
            print(f"⚠ Warning: Position drifted {pos_error*1000:.2f}mm during rotation")

        return True

    except Exception as e:
        print(f"✗ Error during rotation: {e}")
        import traceback

        traceback.print_exc()
        return False


def main():
    """Main test routine."""
    print("=" * 60)
    print("Pure End-Effector Z-axis Rotation Test")
    print("=" * 60)

    # Initialize robot
    print("\nInitializing robot...")
    robot = Robot(namespace="fr3")
    robot.wait_until_ready(timeout=5.0)
    print("✓ Robot ready")

    # Switch to Cartesian controller
    print("\nSwitching to Cartesian controller...")
    robot.controller_switcher_client.switch_controller("osc_pd_controller")
    print("✓ Switched to osc_pd_controller")

    # Get current pose
    current_pose = robot.end_effector_pose
    print(f"\nCurrent end-effector pose:")
    print(f"  Position: {current_pose.position}")
    print(f"  Orientation: {current_pose.orientation.as_rotvec()} (rotvec)")

    # Test sequence
    rotations = [0, 90, -45]  # degrees

    for rotation in rotations:
        success = rotate_end_effector_z(
            robot,
            rotation_degrees=rotation,
            num_waypoints=20,
            execution_time=2.0,
            visualize=False,  # Set to True to visualize each rotation
        )

        if not success:
            print(f"✗ Failed to execute rotation")
            break

        time.sleep(1.0)  # Pause between rotations

    print("\n" + "=" * 60)
    print("Test complete")
    print("=" * 60)

    robot.shutdown()


if __name__ == "__main__":
    main()
