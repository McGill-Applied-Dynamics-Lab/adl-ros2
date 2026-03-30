import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR

# --- Configuration ---
SETTLE_SEC = 1.00  # wait time after moves (s)
TRAJ_FREQ = 10.0   # Hz

# Base orientation (maintained throughout the trajectory)
BASE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)

def execute_circular_arc(
    robot: Robot,
    radius: float,
    polar_angle: float,     # theta (radians) - tilt from Z axis
    azimuthal_angle: float, # phi (radians) - rotation around Z axis
    duration: float
):
    """
    Executes circular trajectory while keeping the end-effector normal to the arc.
    """
    current_pose = robot.end_effector_pose
    start_pos = current_pose.position.copy()

    start_pose = Pose(start_pos, BASE_ORI)
    robot.set_target(pose=start_pose)
    time.sleep(SETTLE_SEC)

    # Compute trajectory waypoints
    print("Computing trajectory...")
    N = max(1, int(duration * TRAJ_FREQ))
    dt = 1.0 / TRAJ_FREQ

    waypoints = []
    time_from_start = []

    for k in range(N + 1):
        s = k / N  # parameter from 0 to 1
        t = k * dt # time-step

        # Polar angle: ramp from 0 to polar_angle following the trajectory
        current_angle = s * polar_angle

        # --- Calculate Dynamic Orientation ---
        # The rotation axis is perpendicular to the radial direction and the Z-axis
        rot_axis = np.array([-np.sin(azimuthal_angle), np.cos(azimuthal_angle), 0.0])

        # Create a rotation vector and apply it to the base orientation
        # (Pre-multiplication is used because the rotation axis is defined in the world frame)
        incremental_rot = R.from_rotvec(rot_axis * current_angle)
        current_ori = incremental_rot * BASE_ORI

        # --- Calculate Position ---
        # Spherical coordinates: radius remains constant
        r_prime = radius * np.sin(current_angle)
        z_offset = radius * (np.cos(current_angle) - 1)

        target_position = np.array([
            start_pos[0] + r_prime * np.cos(azimuthal_angle),
            start_pos[1] + r_prime * np.sin(azimuthal_angle),
            start_pos[2] + z_offset
        ])

        target_pose = Pose(target_position, current_ori)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        waypoints.append((target_pose, twist))
        time_from_start.append(t)

    print(f"Executing circular arc (radius={radius} m, duration={duration} s)...")
    robot.execute_trajectory(waypoints, time_from_start)

    while robot.wait_for_trajectory_completion(duration, timeout_margin=0.5):
        time.sleep(0.01)

    time.sleep(SETTLE_SEC)

    # Optional: Return to the start position/orientation after completion
    print("Returning to start pose...")
    robot.set_target(pose=start_pose)
    time.sleep(SETTLE_SEC)

    return None

def main():
    # Setup robot
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    # Use Cartesian impedance controller for smooth trajectory execution
    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "osc_pd" / "bending.yaml"
    )

    # --- Trajectory Parameters ---
    RADIUS = 0.30  # 10 cm radius
    POLAR_ANGLE = np.radians(45)      # 90 degrees tilt from vertical
    AZIMUTHAL_ANGLE = np.radians(0)  # 90 degrees yaw (y-z plane)
    DURATION = 12.0  # Complete the arc in 4 seconds

    try:
        execute_circular_arc(
            robot=robot,
            radius=RADIUS,
            polar_angle=POLAR_ANGLE,
            azimuthal_angle=AZIMUTHAL_ANGLE,
            duration=DURATION
        )
    finally:
        # Ensure safe shutdown even if interrupted
        robot.shutdown()
        print("Robot shutdown sequence complete.")

if __name__ == "__main__":
    main()