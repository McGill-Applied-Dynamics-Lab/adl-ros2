import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose
from arm_client import CONFIG_DIR

# --- Configuration ---
SETTLE_SEC = 1.00  # eait time after moves (s)
TRAJ_FREQ = 10.0   # (Hz)
BASE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True) # base orientation (deg.)

def execute_circular_arc(
    robot: Robot,
    radius: float,
    polar_angle: float, # theta (radians) - tilt from Z axis
    azimuthal_angle: float, # phi (radians) - rotation around Z axis
    duration: float
):
    """
    Executes circular trajectory using high-frequency streaming to the osc_pd_controller.
    """
    print("Reading current position...")
    current_pose = robot.end_effector_pose
    start_pos = current_pose.position.copy()

    # Define the starting pose with the safe downward orientation
    start_pose = Pose(start_pos, BASE_ORI)
    robot.move_to(pose=start_pose, speed=0.05)
    time.sleep(SETTLE_SEC)

    print(f"Executing circular arc (radius={radius} m, duration={duration} s) at {TRAJ_FREQ} Hz...")

    dt = 1.0 / TRAJ_FREQ
    start_time = time.perf_counter()
    slow_loops = 0
    t = 0.0

    # High-frequency control loop
    while t <= duration:
        loop_start = time.perf_counter()
        t = loop_start - start_time

        # Fraction of completion [0.0, 1.0]
        s = min(t / duration, 1.0)

        # Current polar angle based on fraction
        current_angle = s * polar_angle

        # --- Calculate Dynamic Orientation ---
        # The rotation axis is perpendicular to the radial direction and the Z-axis
        rot_axis = np.array([-np.sin(azimuthal_angle), np.cos(azimuthal_angle), 0.0])

        # Create a rotation vector and apply it to the base orientation
        incremental_rot = R.from_rotvec(rot_axis * current_angle)
        current_ori = incremental_rot * BASE_ORI

        # --- Calculate Position ---
        r_prime = radius * np.sin(current_angle)
        z_offset = radius * (np.cos(current_angle) - 1)

        target_position = np.array([
            start_pos[0] + r_prime * np.cos(azimuthal_angle),
            start_pos[1] + r_prime * np.sin(azimuthal_angle),
            start_pos[2] + z_offset
        ])

        target_pose = Pose(target_position, current_ori)

        # Stream the target pose to the controller
        robot.set_target(pose=target_pose)

        # Precise timing to maintain loop frequency
        elapsed = time.perf_counter() - loop_start
        sleep_time = dt - elapsed

        if elapsed > dt:
            slow_loops += 1
            if slow_loops <= 5:
                print(f"⚠️ Slow loop {slow_loops}: took {elapsed*1000:.1f}ms (target {dt*1000:.1f}ms)")

        if sleep_time > 0:
            time.sleep(sleep_time)

    print("Forward arc complete.")
    time.sleep(SETTLE_SEC)

    # Return to the start position smoothly
    print("Returning to start pose...")
    robot.move_to(pose=start_pose, speed=0.15)
    time.sleep(SETTLE_SEC)

    return None

def main():
    # Setup robot
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    # Switch to Operational Space Controller (Proportional-Derivative)
    # This allows direct high-frequency streaming via set_target
    print("Switching to osc_pd_controller...")
    robot.controller_switcher_client.switch_controller("osc_pd_controller")

    # Load default parameters for osc_pd_controller
    config_path = CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
    robot.osc_pd_controller_parameters_client.load_param_config(file_path=config_path)
    print("Controller configured and ready.")

    # --- Trajectory Parameters ---
    RADIUS = 0.25                    # 25 cm radius
    POLAR_ANGLE = np.radians(45)     # 45 degrees tilt from vertical
    AZIMUTHAL_ANGLE = np.radians(-60)  # 0 degrees yaw (x-z plane)
    DURATION = 5.0                  # Complete the arc in 12 seconds

    print("-" * 50)
    print("BEND TEST V2 (OSC_PD Regulation Streaming)")
    print(f"Radius: {RADIUS}m | Sweep: {np.degrees(POLAR_ANGLE)}deg | Duration: {DURATION}s")
    print("-" * 50)

    try:
        execute_circular_arc(
            robot=robot,
            radius=RADIUS,
            polar_angle=POLAR_ANGLE,
            azimuthal_angle=AZIMUTHAL_ANGLE,
            duration=DURATION
        )
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        # Ensure safe shutdown even if interrupted
        robot.shutdown()
        print("Robot shutdown sequence complete.")

if __name__ == "__main__":
    main()
