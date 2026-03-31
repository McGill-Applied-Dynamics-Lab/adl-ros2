from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import numpy as np
import time
from arm_client.gripper.franka_hand import Gripper


SETTLE_SEC = 5.0  # Wait time after moves (s)
SAFE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)
SAFE_POS = np.array([0.40, 0, 0.30]) # safe starting location
PROJECT_ROOT = Path(__file__).resolve().parent
TRAJ_FREQ = 10 # (Hz)
MOVE_SPEED = 0.01 # (m/s)

Z_MIN = 0.20 # minimum sensor height from table (m)
TUBE_CENTER_POS = np.array([0.40, 0]) # center of tube (x, y), (m)
OUTER_RADIUS = 0.15
INNER_RADIUS = 0.05
PINCH_TIME = 1.0
TUBE_LENGTH = 0.10

def move_waypoints(start_rad, end_rad, start_z, end_z):
    """
    Generate trajectory between consecutive pinch locations.
    """
    # Calculate shortest angular path (handles wrapping around 0/360)
    delta = (end_rad - start_rad + np.pi) % (2 * np.pi) - np.pi

    waypoints = []
    arc_length = OUTER_RADIUS * np.abs(delta)
    z_diff = end_z - start_z
    ds = np.sqrt(arc_length**2 + z_diff**2)
    dt = 1 / TRAJ_FREQ
    N = int(np.ceil(ds / (MOVE_SPEED * dt))) if ds > 0 else 1
    dz = z_diff / N # increment in z (m)

    for i in range(N + 1):
        current_theta = start_rad + (delta * i / N)

        # Offset by the tube's center position
        x = TUBE_CENTER_POS[0] + (OUTER_RADIUS * np.cos(current_theta))
        y = TUBE_CENTER_POS[1] + (OUTER_RADIUS * np.sin(current_theta))

        # Add the starting z height
        z = start_z + (dz * i)

        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        # Calculate rotation relative to SAFE_ORI directly
        rotation = R.from_euler('z', current_theta, degrees=False) * SAFE_ORI
        waypoints.append((Pose(np.array([x, y, z]), rotation), twist))

    return np.linspace(0, N * dt, N + 1), waypoints

def main():
    # Initialization
    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3")
    robot.wait_until_ready()
    gripper.wait_until_ready(timeout=5.0)

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"
    )

    robot.set_target(pose=Pose(SAFE_POS, SAFE_ORI))
    time.sleep(SETTLE_SEC)

    angles = np.radians(np.random.uniform(low=0, high=180, size=(50,)))
    heights = np.random.uniform(low=0, high=1.0, size=(50,))*TUBE_LENGTH + Z_MIN

    for i in range(len(angles)):
        target_rot = R.from_euler('z', angles[i], degrees=False) * SAFE_ORI

        if i > 0:
            times, waypoints = move_waypoints(angles[i-1], angles[i], heights[i-1], heights[i])
            robot.execute_trajectory(waypoints, times)
            robot.wait_for_trajectory_completion(times[-1], timeout_margin=0.5)

        center_x, center_y = TUBE_CENTER_POS[0], TUBE_CENTER_POS[1]

        outer_pos = np.array([
            center_x + (np.cos(angles[i]) * OUTER_RADIUS),
            center_y + (np.sin(angles[i]) * OUTER_RADIUS),
            heights[i]
        ])

        inner_pos = np.array([
            center_x + (np.cos(angles[i]) * INNER_RADIUS),
            center_y + (np.sin(angles[i]) * INNER_RADIUS),
            heights[i]
        ])

        robot.set_target(pose=Pose(outer_pos, target_rot))
        time.sleep(SETTLE_SEC)

        # Open gripper
        gripper.open()
        time.sleep(SETTLE_SEC)

        # Move radially inward
        robot.set_target(pose=Pose(inner_pos, target_rot))
        time.sleep(SETTLE_SEC)

        # Cycle the gripper
        gripper.close()
        time.sleep(PINCH_TIME)
        gripper.open()
        time.sleep(SETTLE_SEC)

        # Move radially outward
        robot.set_target(pose=Pose(outer_pos, target_rot))
        time.sleep(SETTLE_SEC)

    # Move back to safe position
    print("\nReturning to safe pos...")
    robot.set_target(pose=Pose(SAFE_POS, SAFE_ORI))
    time.sleep(SETTLE_SEC)

    print("\nShutting down...")
    robot.shutdown()

if __name__ == "__main__":
    main()
