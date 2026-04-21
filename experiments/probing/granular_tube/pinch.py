from typing import Any, List, Sequence

from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import numpy as np
import time
from arm_client.gripper.franka_hand import Gripper

from arm_client.planning.types import CartesianWaypoint, PlannedJointTrajectory

# TODO:
# - pinch to depth (random)
# - serial read
# - record/saving (height, angle, caretsian pose, gripper position)
# - recover (crash) (see other scripts)
# - threading for IK


SETTLE_SEC = 5.0  # Wait time after moves (s)
Z_MIN = 0.20  # minimum sensor height from table (m)
SAFE_ORI = R.from_euler(
    "xyz", [-180, 90, -90], degrees=True
)  # [-180,90,90]: Facing along -y, [-180,90, -90]: Facing along +y
SAFE_POS = np.array([0.40, 0.2, 0.30])  # safe starting location
START_POS = np.array([0.40, -0.10, Z_MIN])  # start location
PROJECT_ROOT = Path(__file__).resolve().parent
TRAJ_FREQ = 10  # (Hz)
MOVE_SPEED = 0.01  # (m/s)

TUBE_CENTER_POS = np.array([0.40, 0])  # center of tube (x, y), (m)
OUTER_RADIUS = 0.15
INNER_RADIUS = 0.05
PINCH_TIME = 1.0
TUBE_LENGTH = 0.10


def move_waypoints(
    start_rad, end_rad, start_height, end_height
) -> tuple[float, Sequence[CartesianWaypoint]]:
    """
    Generate trajectory between consecutive pinch locations.

    Returns:
        float: Trajectory duration
        Sequence[CartesianWaypoint]: List with the `n_points` CartesianWaypoints
    """
    # Calculate shortest angular path (handles wrapping around 0/360)
    delta = (end_rad - start_rad + np.pi) % (2 * np.pi) - np.pi

    waypoints = []
    arc_length = OUTER_RADIUS * np.abs(delta)
    z_diff = end_height - start_height
    # n_points = int(np.ceil(ds / (MOVE_SPEED * dt))) if ds > 0 else 1
    n_points = 20

    ds = np.sqrt(arc_length**2 + z_diff**2)
    dt = 1 / TRAJ_FREQ
    dz = z_diff / n_points  # increment in z (m)
    traj_duration = ds / (MOVE_SPEED)

    for i in range(n_points + 1):
        current_theta = start_rad + (delta * i / n_points)
        current_height = start_height + (dz * i)

        waypoint = coord_to_pose(current_theta, current_height)

        # twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        # Calculate rotation relative to SAFE_ORI directly
        # rotation = R.from_euler("z", current_theta, degrees=False) * SAFE_ORI

        cartesian_wp = CartesianWaypoint(
            position=waypoint.position, orientation=waypoint.orientation, s=i / n_points
        )

        waypoints.append(cartesian_wp)

    return traj_duration, waypoints


def coord_to_pose(angle, height) -> Pose:
    """To convert the (angle, height) variables to workspace cartesian Pose.

    Args:
        angle (float): Angle, measured around the +z_world axis. 0 when facing along +y_world
        height (float): Along the +z_world axis from the sensor's base.

    Returns:
        Pose: The corresponding cartesian pose.
    """

    x = TUBE_CENTER_POS[0] + (OUTER_RADIUS * np.sin(angle))
    y = TUBE_CENTER_POS[1] - (OUTER_RADIUS * np.cos(angle))
    z = height + Z_MIN

    return Pose(np.array([x, y, z]), R.from_euler("z", angle, degrees=False) * SAFE_ORI)


def main():
    # Initialization
    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3")
    robot.wait_until_ready()
    gripper.wait_until_ready(timeout=5.0)

    print("Switching to joint trajectory controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    print(f"Moving to safe position...")
    # robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI), time_to_move=5.0)
    # time.sleep(SETTLE_SEC)

    # Move to (angle, height) = (0, 0)
    last_angle = 0.0
    last_height = 0.0
    start_pose = coord_to_pose(last_angle, last_height)
    print(
        f"Moving to start position (angle: {np.degrees(last_angle):.2f}°, height: {last_height:.2f}m)"
    )
    robot.move_to(pose=start_pose)
    last_joint_cfg = robot.q.copy()

    # Compute the random probing points
    angles = np.radians(np.random.uniform(low=-45, high=45, size=(50,)))
    heights = np.random.uniform(low=0, high=1.0, size=(50,)) * TUBE_LENGTH

    for i, (target_angle, target_height) in enumerate(zip(angles, heights)):
        print(f"Processing probe {i + 1}/{len(angles)}")
        print(f"\tTarget angle: {np.degrees(target_angle):.2f}°")
        print(f"\tTarget height: {target_height:.2f}m")

        # target_rot = R.from_euler("z", target_angle, degrees=False) * SAFE_ORI

        traj_duration, waypoints = move_waypoints(
            last_angle, target_angle, last_height, target_height
        )

        joint_traj = robot.plan_joint_trajectory(
            waypoints=waypoints,
            duration=traj_duration,
            visualize=False,
            n_points=len(waypoints),
            show_progress=True,
            initial_joint_config=last_joint_cfg,
        )

        robot.follow_joint_trajectory(joint_traj, blocking=True)

        last_height = target_height
        last_angle = target_angle
        last_joint_cfg = robot.q.copy()

        # center_x, center_y = TUBE_CENTER_POS[0], TUBE_CENTER_POS[1]

        # outer_pos = np.array(
        #     [
        #         center_x + (np.cos(target_angle) * OUTER_RADIUS),
        #         center_y + (np.sin(target_angle) * OUTER_RADIUS),
        #         target_height,
        #     ]
        # )

        # inner_pos = np.array(
        #     [
        #         center_x + (np.cos(target_angle) * INNER_RADIUS),
        #         center_y + (np.sin(target_angle) * INNER_RADIUS),
        #         target_height,
        #     ]
        # )

        # robot.set_target(pose=Pose(outer_pos, target_rot))
        # time.sleep(SETTLE_SEC)

        # # Open gripper
        # gripper.open()
        # time.sleep(SETTLE_SEC)

        # # Move radially inward
        # robot.set_target(pose=Pose(inner_pos, target_rot))
        # time.sleep(SETTLE_SEC)

        # # Cycle the gripper
        # gripper.close()
        # time.sleep(PINCH_TIME)
        # gripper.open()
        # time.sleep(SETTLE_SEC)

        # # Move radially outward
        # robot.set_target(pose=Pose(outer_pos, target_rot))
        # time.sleep(SETTLE_SEC)

    # Move back to safe position
    print("\nReturning to safe pos...")
    robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI))
    time.sleep(SETTLE_SEC)

    print("\nShutting down...")
    robot.shutdown()


if __name__ == "__main__":
    main()
