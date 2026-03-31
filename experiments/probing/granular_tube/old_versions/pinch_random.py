import time
import numpy as np
import pickle
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import rclpy
from arm_client.robot import Robot, Pose
from arm_client.gripper.franka_hand import Gripper
from arm_client import CONFIG_DIR

# --- Configuration ---
SETTLE_SEC = 1.0  # Wait time after moves (s)
SAFE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)
SAFE_POS = np.array([0.40, 0, 0.50]) # safe starting location

TUBE_CENTER_XY = np.array([0.50, 0]) # center of tube mapped into XY
Z_START = 0.50 # top Z height
TUBE_LENGTH = 0.30 # probe distance downwards
Z_END = Z_START - TUBE_LENGTH

OUTER_RADIUS = 0.20
INNER_RADIUS = 0.10

PROJECT_ROOT = Path(__file__).resolve().parent
PARAMETERS_FILE = PROJECT_ROOT / "results" / "PINCH_PARAMS.pkl"
RESULTS_FILE = PROJECT_ROOT / "results" / "RANDOM_PINCH_DATA.pkl"

def execute_pinch_sequence(robot: Robot, gripper: Gripper, target_height: float, azimuth: float):
    """
    Executes a single pinch sequence and records gripper width during pinch.
    """
    poll_rate = 20.0
    poll_period = 1.0 / poll_rate

    # Calculate outer approach position at the specified target height
    z_target = target_height
    tube_center_pos = np.array([TUBE_CENTER_XY[0], TUBE_CENTER_XY[1], z_target])
    
    outer_pos = tube_center_pos + np.array([np.cos(azimuth), np.sin(azimuth), 0]) * OUTER_RADIUS
    inner_pos = tube_center_pos + np.array([np.cos(azimuth), np.sin(azimuth), 0]) * INNER_RADIUS

    # Calculate Normal Rotation
    target_rot = R.from_euler('z', azimuth, degrees=False) * SAFE_ORI

    # 1. Approach Outer target
    target_pose = Pose(outer_pos, target_rot)
    robot.move_to(pose=target_pose, speed=0.08)
    time.sleep(SETTLE_SEC)

    # 2. Open Gripper
    gripper.open()
    time.sleep(SETTLE_SEC)

    # 3. Move radially inward
    inner_target_pose = Pose(inner_pos, target_rot)
    robot.move_to(pose=inner_target_pose, speed=0.05)
    time.sleep(SETTLE_SEC)

    # 4. Close Gripper AND record opening extent
    ts_pinch = []
    gripper_widths = []

    t0 = time.perf_counter()
    gripper.close(block=False)
    
    # Record data while it closes
    while gripper.is_action_done(gripper._current_action_future) is False:
        elapsed = time.perf_counter() - t0
        ts_pinch.append(elapsed)
        gripper_widths.append(gripper.value)
        time.sleep(poll_period)
        
        # Stop recording if timeout
        if elapsed > 10.0:
            break

    # Add final closed state observation
    ts_pinch.append(time.perf_counter() - t0)
    gripper_widths.append(gripper.value)
    time.sleep(SETTLE_SEC)

    # 5. Open Gripper
    gripper.open()
    time.sleep(SETTLE_SEC)

    # 6. Retract safely back out
    robot.move_to(pose=target_pose, speed=0.08)
    time.sleep(SETTLE_SEC)

    return ts_pinch, gripper_widths


def main():
    # Setup
    rclpy.init()
    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3")
    
    robot.wait_until_ready()
    try:
        gripper.wait_until_ready(timeout=5.0)
    except Exception as e:
        print(f"Warning: Gripper could not be initialized: {e}")

    # Configure Robot
    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"
    )

    # Move to safe place
    robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI), speed=0.05)
    time.sleep(SETTLE_SEC)

    # Load parameters
    if not PARAMETERS_FILE.exists():
        print(f"ERROR: Parameters file not found: {PARAMETERS_FILE}")
        print("Generate parameters first with:")
        print("  python generate_pinch_parameters.py")
        robot.shutdown()
        rclpy.shutdown()
        return

    with open(PARAMETERS_FILE, "rb") as f:
        params = pickle.load(f)
        planned_azimuths = params["azimuths"]
        planned_norm_heights = params["normalized_heights"]
        num_points = params["n_samples"]

    print(f"Loaded {num_points} pinch parameters from {PARAMETERS_FILE}")

    # Scale normalized heights to [Z_START, Z_END]
    scaled_heights = [Z_START - h * TUBE_LENGTH for h in planned_norm_heights]

    RESULTS_FILE.parent.mkdir(parents=True, exist_ok=True)

    start_idx = 0
    if RESULTS_FILE.exists():
        with open(RESULTS_FILE, "rb") as f:
            exp_dict = pickle.load(f)
            start_idx = len(exp_dict["target_azimuths"])
            print(f"Loaded existing results file: {RESULTS_FILE}")
            print(f"Already completed {start_idx} pinch sequences")
    else:
        exp_dict = {
            "ts_pinch": [],
            "gripper_widths": [],
            "target_azimuths": [],
            "target_heights": [],
        }
        with open(RESULTS_FILE, "wb") as f:
            pickle.dump(exp_dict, f)
        print(f"Created new results file: {RESULTS_FILE}")

    if start_idx >= num_points:
        print(f"All {num_points} sequences have already been collected.")
        robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI), speed=0.1)
        robot.shutdown()
        rclpy.shutdown()
        return

    print(f"Resuming from pinch sequence {start_idx + 1} / {num_points}")

    for i in range(start_idx, num_points):
        azimuth = planned_azimuths[i]
        target_z = scaled_heights[i]

        print(f"\nSequence {i + 1} / {num_points} | Target Z: {target_z:.3f}m, Azimuth: {np.degrees(azimuth):.2f} deg")

        try:
            ts_pinch, gripper_widths = execute_pinch_sequence(robot, gripper, target_z, azimuth)

            # Store results
            exp_dict["ts_pinch"].append(ts_pinch)
            exp_dict["gripper_widths"].append(gripper_widths)
            exp_dict["target_azimuths"].append(azimuth)
            exp_dict["target_heights"].append(target_z)

            # Iterative explicit save 
            with open(RESULTS_FILE, "wb") as f:
                pickle.dump(exp_dict, f)
                print(f"\tResults saved.")

        except Exception as e:
            print(f"\tERROR during sequence {i + 1}: {e}")
            print(f"\tSkipping to next sequence...")
            continue

    # Clean up
    print("\nReturning to safe pos...")
    robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI), speed=0.1)
    
    robot.shutdown()
    rclpy.shutdown()
    print("\nDone.")

if __name__ == "__main__":
    main()
