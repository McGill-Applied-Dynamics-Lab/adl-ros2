from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import numpy as np
import time

SETTLE_SEC = 1.0  # Wait time after moves (s)
SAFE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)
SAFE_POS = np.array([0.40, 0, 0.50]) # safe starting location
PROJECT_ROOT = Path(__file__).resolve().parent

# Target probe location
TUBE_CENTER_POS = np.array([0.45, 0.10, 0.20]) # center of tube
OUTER_RADIUS = 0.20
INNER_RADIUS = 0.05
AZIMUTH = np.radians(45) # angle in x-y plane (rad.)

def main():
    import rclpy
    rclpy.init()

    from arm_client.gripper.franka_hand import Gripper
    
    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3")
    
    robot.wait_until_ready()
    try:
        gripper.wait_until_ready(timeout=5.0)
    except Exception as e:
        print(f"Warning: Gripper could not be initialized: {e}")

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"
    )

    # Move to safe start location first
    robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI), speed=0.05)
    time.sleep(SETTLE_SEC)

    # Move to outer position
    outer_pos = TUBE_CENTER_POS + np.array([np.cos(AZIMUTH), np.sin(AZIMUTH), 0]) * OUTER_RADIUS
    target_rot = R.from_euler('z', AZIMUTH, degrees=False) * SAFE_ORI
    robot.set_target(pose=Pose(outer_pos, SAFE_ORI))
    time.sleep(SETTLE_SEC)
    robot.set_target(pose=Pose(outer_pos, target_rot))
    time.sleep(SETTLE_SEC)

    gripper.open()
    time.sleep(SETTLE_SEC*2)

    # Move radially inward
    print("\nMoving radially inward...")
    inner_pos = TUBE_CENTER_POS + np.array([np.cos(AZIMUTH), np.sin(AZIMUTH), 0]) * INNER_RADIUS
    robot.set_target(pose=Pose(inner_pos, target_rot))
    time.sleep(SETTLE_SEC)

    # Close the gripper
    gripper.close()
    time.sleep(SETTLE_SEC)

    gripper.open()
    time.sleep(SETTLE_SEC)

    # Move back to safe position
    print("\nReturning to safe pos...")
    robot.set_target(pose=Pose(SAFE_POS, SAFE_ORI))
    time.sleep(SETTLE_SEC)

    print("\nShutting down...")
    robot.shutdown()
    
if __name__ == "__main__":
    main()
