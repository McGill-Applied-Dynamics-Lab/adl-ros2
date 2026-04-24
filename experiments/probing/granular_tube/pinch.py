from typing import Any, List, Sequence

from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import numpy as np
import time
import threading
import queue
from arm_client.gripper.franka_hand import Gripper, GripperConfig

from arm_client.planning.types import CartesianWaypoint, PlannedJointTrajectory

# TODO:
# - [ ] pinch to depth (random)
# - [ ] serial read
# - [ ] record/saving (height, angle, caretsian pose, gripper position)
# - [ ] recover (crash) (see other scripts)
# - [x] threading for IK
# - [ ] Fix max speed

# Experiment Setup
TUBE_CENTER_POS = np.array([0.40, 0])  # center of tube (x, y), (m)
OUTER_RADIUS = 0.0  # Distance from fingers center to tube center
TUBE_LENGTH = 0.10
TUBE_DIAMETER = 0.05
MOVE_SPEED = 0.01  # (m/s)
MOVE_SPEED_ROT = 0.2  # (rad/s)
Z_MIN = 0.20  # minimum sensor height from table (m)

SAFE_ORI = R.from_euler(
    "xyz", [-180, 90, -90], degrees=True
)  # [-180,90,90]: Facing along -y, [-180,90, -90]: Facing along +y
SAFE_POS = np.array([0.40, 0.2, 0.30])  # safe starting location
START_POS = np.array([0.40, -0.10, Z_MIN])  # start location
PROJECT_ROOT = Path(__file__).resolve().parent

# Experiment Configurations
NUM_PROBES = 50  # Number of probing points

ANGLE_RANGE = [-45, 45]  # minimum and maximum angles (deg), 0 is gripper facing along +y
PINCH_DEPTH_RANGE = [0.005, 0.02]  # .5cm to 2cm (distance pushed by one finger)
PINCH_SPEED_RANGE = [0.01, 0.08]  # 1 to 8 cm/s

PINCH_TIME = 1.0  # Time to wait after closing gripper
SETTLE_SEC = 0.1  # Wait time after moves (s)

# Other configurations
QUEUE_SIZE = 3  # How far ahead to compute the joint trajectories
TRAJ_N_POINTS = 10  # Number of points in the trajectory

gripper_cfg = GripperConfig(
    max_width=0.08,
    min_width=0.0,
    default_speed=0.01,
)


def move_waypoints(start_rad, end_rad, start_height, end_height) -> tuple[float, Sequence[CartesianWaypoint]]:
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
    n_points = TRAJ_N_POINTS

    ds = np.sqrt(arc_length**2 + z_diff**2)
    dz = z_diff / n_points  # increment in z (m)

    time_trans = ds / MOVE_SPEED
    time_rot = np.abs(delta) / MOVE_SPEED_ROT
    traj_duration = max(time_trans, time_rot, 0.1)  # Ensure non-zero duration

    for i in range(n_points + 1):
        current_theta = start_rad + (delta * i / n_points)
        current_height = start_height + (dz * i)

        waypoint = coord_to_pose(current_theta, current_height)

        # twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        # Calculate rotation relative to SAFE_ORI directly
        # rotation = R.from_euler("z", current_theta, degrees=False) * SAFE_ORI

        cartesian_wp = CartesianWaypoint(position=waypoint.position, orientation=waypoint.orientation, s=i / n_points)

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


def planner_worker(robot, angles, heights, start_angle, start_height, start_joint_cfg, plan_queue, abort_event):
    """Background thread to pre-compute joint trajectories."""
    last_angle = start_angle
    last_height = start_height
    last_joint_cfg = start_joint_cfg

    for i, (target_angle, target_height) in enumerate(zip(angles, heights)):
        if abort_event.is_set():
            break

        # print(f"[Planner] Planning for coordinate {i + 1}/{len(angles)}")
        traj_duration, waypoints = move_waypoints(last_angle, target_angle, last_height, target_height)

        try:
            joint_traj = robot.plan_joint_trajectory(
                waypoints=waypoints,
                duration=traj_duration,
                visualize=False,
                n_points=len(waypoints),
                show_progress=False,  # disabled progress bar in background thread to avoid console spam
                initial_joint_config=last_joint_cfg,
            )

        except Exception as e:
            print(f"\n[Planner] Error planning trajectory: {e}")
            # Send exception to main thread to trigger abort
            try:
                plan_queue.put(e, timeout=1.0)
            except queue.Full:
                pass
            break

        # Wait for space in queue, checking abort_event periodically
        while not abort_event.is_set():
            try:
                plan_queue.put(joint_traj, timeout=0.1)
                break
            except queue.Full:
                continue

        if abort_event.is_set():
            break

        last_height = target_height
        last_angle = target_angle
        # Use the final joint configuration of the planned trajectory for the next IK seed
        last_joint_cfg = joint_traj.joint_positions[-1]


def main():
    # Initialization
    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3", gripper_config=gripper_cfg)

    robot.wait_until_ready()
    gripper.wait_until_ready(timeout=5.0)

    print("Switching to joint trajectory controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    success = gripper.open(speed=0.05)  # Custom speed
    success = gripper.set_target(0.00, speed=0.08)  # Custom speed

    gripper.value

    while True:
        # --- Go to start position (angle, height) = (0, 0)
        last_angle = 0.0
        last_height = 0.0
        start_pose = coord_to_pose(last_angle, last_height)

        print("=================================")
        print("FR3 Pinch Experiment")
        print("=================================")

        print(f"\nMoving to start position (angle: {np.degrees(last_angle):.2f}°, height: {last_height:.2f}m)")
        try:
            robot.move_to(pose=start_pose)
            gripper_success = gripper.open(speed=0.05)  # Custom speed

        except Exception as e:
            print(f"Failed to move to start position: {e}. Retrying in 2s...")
            time.sleep(2.0)
            continue

        last_joint_cfg = robot.q.copy()

        print("Robot initialization complete.")

        # --- Compute the random probing points
        angles = np.radians(np.random.uniform(low=ANGLE_RANGE[0], high=ANGLE_RANGE[1], size=(NUM_PROBES,)))
        heights = np.random.uniform(low=0, high=1.0, size=(NUM_PROBES,)) * TUBE_LENGTH
        pinch_depths = np.random.uniform(low=PINCH_DEPTH_RANGE[0], high=PINCH_DEPTH_RANGE[1], size=(NUM_PROBES,))
        pinch_vels = np.random.uniform(low=PINCH_SPEED_RANGE[0], high=PINCH_SPEED_RANGE[1], size=(NUM_PROBES,))

        plan_queue = queue.Queue(maxsize=QUEUE_SIZE)
        abort_event = threading.Event()

        planner_thread = threading.Thread(
            target=planner_worker,
            args=(robot, angles, heights, last_angle, last_height, last_joint_cfg, plan_queue, abort_event),
            daemon=True,
        )
        planner_thread.start()

        success = True
        for i in range(len(angles)):
            try:
                print(f"Point {i + 1}/{len(angles)}")
                print(f"\tAngle: {np.degrees(angles[i]):.2f}°")
                print(f"\tHeight: {heights[i]:.2f}m")
                print(f"\tPinch Depth: {pinch_depths[i]:.2f}m")
                print(f"\tPinch Speed: {pinch_vels[i]:.2f}m/s")

                item = plan_queue.get()

                if isinstance(item, Exception):
                    raise RuntimeError(f"Planner failed: {item}")

                joint_traj = item

                print(f"\tExecuting move...")
                robot.follow_joint_trajectory(joint_traj, blocking=True)

                print(f"\tPinching...")
                depth_target = TUBE_DIAMETER - 2 * pinch_depths[i]
                gripper_success = gripper.set_target(depth_target, speed=pinch_vels[i])

                if not gripper_success:
                    print("Gripper failed to close.")
                    raise RuntimeError("Failed to close gripper.")

                gripper_val = gripper.value
                print(f"\tGripper value: {gripper_val}")
                time.sleep(PINCH_TIME)

                gripper.open(speed=0.1)
                time.sleep(SETTLE_SEC)

                plan_queue.task_done()

            except Exception as e:
                print(f"\n[Error] Execution aborted at probe {i + 1}: {e}")
                success = False
                abort_event.set()

                # Flush the queue to unblock the planner thread
                while not plan_queue.empty():
                    try:
                        plan_queue.get_nowait()
                        plan_queue.task_done()
                    except queue.Empty:
                        break

                print("Moving back to safe position and restarting...")
                robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI))
                time.sleep(SETTLE_SEC)
                planner_thread.join(timeout=2.0)
                break

        if success:
            print("All probing points completed successfully!")
            abort_event.set()
            planner_thread.join(timeout=1.0)
            break

    # Move back to safe position
    print("Returning to safe pos...")
    robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI))
    time.sleep(SETTLE_SEC)

    print("Shutting down...")
    robot.shutdown()


if __name__ == "__main__":
    main()
