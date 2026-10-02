import pickle
import time
from pathlib import Path

import numpy as np
from arm_client.robot import Robot

# Configuration
SETTLE_SEC = 1.00  # wait time after moves (s)
DELAY_SEC = 1.00  # delay between forward and reverse motion (s)

# File paths
PROJECT_ROOT = Path(__file__).resolve().parent
PARAMETERS_FILE = PROJECT_ROOT / "results" / "grids" / "rotation_params.pkl"
RESULTS_FILE = PROJECT_ROOT / "results" / "TEST.pkl"


def execute_wrist_rotation_pair(robot: Robot, target_angle_rad: float, speed_rad_s: float):
    """
    Execute a forward rotation of the wrist (joint 7) followed by reverse rotation.

    Each rotation is done via joint-space control at constant speed.

    Args:
        robot: Robot instance
        target_angle_rad: Target rotation angle in radians (forward direction)
        speed_rad_s: Rotation speed in radians/second

    Returns:
        tuple: (ts_fwd, angles_fwd, torques_fwd, ts_rev, angles_rev, torques_rev)
               where angles are absolute joint 7 angles in radians
               and torques are 3-element torque vectors
    """
    # Initialize joint 7 to 45 degrees before every rotation
    q_init = robot.q.copy()
    q_init[6] = np.radians(55.0)
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_init.tolist(),
        time_to_goal=2.0,
        blocking=True,
    )
    time.sleep(SETTLE_SEC)  # wait for arm to settle

    # Capture starting joint configuration
    start_q = robot.q.copy()
    start_joint7 = start_q[6]  # joint 7 is at index 6 (0-indexed)

    # Calculate trajectory duration
    speed_rad_s = abs(speed_rad_s)
    duration_per_direction = abs(target_angle_rad / speed_rad_s)

    # Forward rotation
    target_joint7_forward = start_joint7 + target_angle_rad
    q_target_forward = start_q.copy()
    q_target_forward[6] = target_joint7_forward

    ts_fwd = []
    angles_fwd = []
    torques_fwd = []
    pressures_fwd = []  # (n-samples x 2 sensors)

    t0_fwd = time.perf_counter()
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_target_forward.tolist(),
        time_to_goal=duration_per_direction,
        blocking=False,
    )

    # Trigger Teensy sampling
    # ...

    while (time.perf_counter() - t0_fwd) < duration_per_direction + DELAY_SEC:  # record data during delay
        elapsed = time.perf_counter() - t0_fwd
        current_q = robot.q.copy()
        current_j7 = current_q[6]
        ee_wrench = robot.end_effector_external_wrench

        ts_fwd.append(elapsed)
        angles_fwd.append(current_j7)  # absolute angle
        torques_fwd.append(ee_wrench["torque"].copy())

        time.sleep(0.01)

    # Trigger serial dump
    # ...

    # Reverse direction
    q_target_reverse = start_q.copy()
    q_target_reverse[6] = start_joint7

    ts_rev = []
    angles_rev = []
    torques_rev = []
    pressures_rev = []  # (n-samples x 2 sensors)

    t0_rev = time.perf_counter()
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_target_reverse.tolist(),
        time_to_goal=duration_per_direction,
        blocking=False,
    )

    # Trigger Teensy sampling
    # ...

    while (time.perf_counter() - t0_rev) < duration_per_direction:
        elapsed = time.perf_counter() - t0_rev
        current_q = robot.q.copy()
        current_j7 = current_q[6]
        ee_wrench = robot.end_effector_external_wrench

        ts_rev.append(elapsed)
        angles_rev.append(current_j7)  # absolute angle
        torques_rev.append(ee_wrench["torque"].copy())

        time.sleep(0.01)

    # Trigger serial dump
    # ...

    time.sleep(SETTLE_SEC)

    return (
        ts_fwd,
        angles_fwd,
        torques_fwd,
        pressures_fwd,
        ts_rev,
        angles_rev,
        torques_rev,
        pressures_rev,
    )


def main():
    """
    Main function to execute wrist rotations using joint-space control.
    Each rotation consists of forward + reverse motion, all recorded.

    Requires pre-generated parameters file. Generate with:
        python generate_rotation_parameters.py
    """
    # Setup
    robot = Robot(namespace="fr3")
    robot.wait_until_ready(timeout=2.0)

    # Switch to joint trajectory controller for smooth joint-space motion
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    # Initialize joint 7 to 45 degrees
    print("Initializing joint 7 to 45 degrees...")
    q_init = robot.q.copy()
    q_init[6] = np.radians(45.0)  # Set joint 7 to 45 degrees
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_init.tolist(),
        time_to_goal=3.0,
        blocking=True,
    )
    print(f"Joint 7 initialized to 45°")

    # Load parameters
    if not PARAMETERS_FILE.exists():
        print(f"ERROR: Parameters file not found: {PARAMETERS_FILE}")
        print("Generate parameters first with:")
        print("  python generate_rotation_parameters.py")
        robot.shutdown()
        return

    with open(PARAMETERS_FILE, "rb") as f:
        params = pickle.load(f)
        planned_angles = params["angles"]
        planned_speeds = params["speeds"]
        num_points = params["num_points"]

    print(f"Loaded {num_points} rotation parameters from {PARAMETERS_FILE}")

    # Initialize results file
    RESULTS_FILE.parent.mkdir(parents=True, exist_ok=True)

    start_idx = 0
    if RESULTS_FILE.exists():
        # Load existing results to resume
        with open(RESULTS_FILE, "rb") as f:
            exp_dict = pickle.load(f)
            # Calculate how many rotations have been completed
            start_idx = len(exp_dict["target_angles"])
            print(f"Loaded existing results file: {RESULTS_FILE}")
            print(f"Already completed {start_idx} rotation pairs")
    else:
        # Create new results file
        exp_dict = {
            "ts_forward": [],
            "joint7_angles_forward": [],
            "torques_forward": [],
            "ts_reverse": [],
            "joint7_angles_reverse": [],
            "torques_reverse": [],
            "target_angles": [],
            "target_speeds": [],
        }
        with open(RESULTS_FILE, "wb") as f:
            pickle.dump(exp_dict, f)
        print(f"Created new results file: {RESULTS_FILE}")

    if start_idx >= num_points:
        print(f"All {num_points} rotation pairs have already been collected.")
        robot.shutdown()
        return

    print(f"Resuming from rotation pair {start_idx + 1} / {num_points}")

    # Iterate over the pre-planned angles and speeds
    for i in range(start_idx, num_points):
        angle = planned_angles[i]
        speed = planned_speeds[i]

        angle_deg = np.degrees(angle)  # for print-out only
        speed_deg_s = np.degrees(speed)  # for print-out only

        print(
            f"\nRotation Pair {i + 1} / {num_points} | Target Angle: {angle_deg:.3g} deg., Speed: {speed_deg_s:.3g} deg./s"
        )

        try:
            # Perform Forward and Reverse Rotation
            print("\tExecuting forward + reverse rotation...")
            (ts_fwd, angles_fwd, torques_fwd, ts_rev, angles_rev, torques_rev) = execute_wrist_rotation_pair(
                robot=robot, target_angle_rad=angle, speed_rad_s=speed
            )

            # Store results
            exp_dict["ts_forward"].append(ts_fwd)
            exp_dict["joint7_angles_forward"].append(angles_fwd)
            exp_dict["torques_forward"].append(torques_fwd)

            exp_dict["ts_reverse"].append(ts_rev)
            exp_dict["joint7_angles_reverse"].append(angles_rev)
            exp_dict["torques_reverse"].append(torques_rev)

            exp_dict["target_angles"].append(angle)
            exp_dict["target_speeds"].append(speed)

            # Save data iteratively
            with open(RESULTS_FILE, "wb") as f:
                pickle.dump(exp_dict, f)
                print(f"\tResults saved to: {RESULTS_FILE}")

        except Exception as e:
            print(f"\tERROR during rotation pair {i + 1}: {e}")
            print(f"\tSkipping to next rotation pair...")
            continue

    robot.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
