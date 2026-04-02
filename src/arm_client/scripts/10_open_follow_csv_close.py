"""Open gripper, follow CSV trajectory anchored at ORIGIN_MM_FR, with waypoint gripper actions."""

import csv
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR
from arm_client.gripper.franka_hand import Gripper
from arm_client.robot import Pose, Robot
from spots import ORIGIN_MM_FR

CSV_PATH = Path(__file__).with_name("gripper_steps_4_to_11_relative.csv")
CLOSE_WIDTH_M = 0.01
CLOSE_DETECTION_TOLERANCE_M = 0.05
STEP_TIME_S = 3.0
DELTA_SCALE = 0.60
INVERT_RELATIVE_MOVES = False
POSITION_TOLERANCE_M = 0.002
MIN_EXPECTED_SPEED_MPS = 0.01
WAYPOINT_TIMEOUT_S = 12.0
REACH_CHECK_DT_S = 0.05
CLOSE_AT_WAYPOINT = 1
OPEN_AT_WAYPOINT = 8


def load_waypoints_m(csv_path: Path, origin_mm: tuple[float, float, float]) -> list[np.ndarray]:
    """Load x,y,z from CSV and anchor trajectory so first point equals origin_mm."""
    points_m = []
    with csv_path.open(newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            # CSV axes are (x, y, z) but map to robot axes (x, z, y).
            csv_x = float(row["x"])
            csv_y = float(row["y"])
            csv_z = float(row["z"])
            points_m.append(np.array([csv_x, csv_z, csv_y], dtype=float))

    if not points_m:
        raise RuntimeError(f"No trajectory points found in {csv_path}.")

    first_point_m = points_m[0]
    origin_mm_arr = np.array(origin_mm, dtype=float)

    waypoints_m = []
    for point_m in points_m:
        delta_mm = (point_m - first_point_m) * 1000.0 * DELTA_SCALE
        if INVERT_RELATIVE_MOVES:
            delta_mm = -delta_mm
        waypoint_mm = origin_mm_arr + delta_mm
        waypoints_m.append(waypoint_mm / 1000.0)

    return waypoints_m


def wait_until_position_reached(
    robot: Robot,
    target_position_m: np.ndarray,
    tolerance_m: float = POSITION_TOLERANCE_M,
    timeout_s: float = WAYPOINT_TIMEOUT_S,
) -> bool:
    """Wait until end-effector is within tolerance of target_position_m."""
    start = time.time()
    while time.time() - start < timeout_s:
        current_position_m = robot.end_effector_pose.position
        error_m = np.linalg.norm(current_position_m - target_position_m)
        if error_m <= tolerance_m:
            return True
        time.sleep(REACH_CHECK_DT_S)
    return False


def compute_waypoint_timeout_s(
    start_position_m: np.ndarray,
    target_position_m: np.ndarray,
    min_expected_speed_mps: float = MIN_EXPECTED_SPEED_MPS,
    base_timeout_s: float = WAYPOINT_TIMEOUT_S,
) -> float:
    """Compute a conservative timeout based on distance and expected speed."""
    distance_m = np.linalg.norm(target_position_m - start_position_m)
    travel_time_s = distance_m / max(min_expected_speed_mps, 1e-6)
    return max(base_timeout_s, travel_time_s + 2.0)


def main():
    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3")

    try:
        robot.wait_until_ready(timeout=5.0)
        gripper.wait_until_ready(timeout=5.0)

        print("Switching to fr3_pose_controller...")
        robot.controller_switcher_client.switch_controller("fr3_pose_controller")
        robot.fr3_pose_controller_parameters_client.load_param_config(
            file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
        )
        time.sleep(0.5)

        waypoints_m = load_waypoints_m(CSV_PATH, ORIGIN_MM_FR)
        print(f"Loaded {len(waypoints_m)} waypoints from: {CSV_PATH.name}")
        print(f"First waypoint (m): {waypoints_m[0].tolist()} (anchored to ORIGIN_MM_FR)")

        print("Opening gripper...")
        opened = gripper.open(block=True)
        if not opened:
            print(f"WARNING: Failed to open gripper. Current width: {gripper.value:.4f} m")
        time.sleep(0.3)

        target_orientation = Rotation.from_euler("xyz", [180.0, 0.0, 0.0], degrees=True)

        for i, waypoint_m in enumerate(waypoints_m, start=1):
            start_position_m = robot.end_effector_pose.position
            timeout_s = compute_waypoint_timeout_s(start_position_m, waypoint_m)
            print(f"Moving to waypoint {i}/{len(waypoints_m)} (m): {waypoint_m.tolist()}")
            waypoint_pose = Pose(position=waypoint_m, orientation=target_orientation)
            robot.move_to(pose=waypoint_pose, time_to_move=STEP_TIME_S)
            reached = wait_until_position_reached(robot, waypoint_m, timeout_s=timeout_s)
            current_position_m = robot.end_effector_pose.position
            error_m = np.linalg.norm(current_position_m - waypoint_m)
            if reached:
                print(f"Reached waypoint {i} (error: {error_m * 1000:.1f} mm)")
            else:
                raise TimeoutError(
                    f"Timeout waiting for waypoint {i} after {timeout_s:.1f}s; "
                    f"position error is {error_m * 1000:.1f} mm."
                )

            if i == CLOSE_AT_WAYPOINT:
                print(f"Waypoint {i} reached: setting gripper width to {CLOSE_WIDTH_M:.4f} m...")
                closed = gripper.set_target(CLOSE_WIDTH_M, speed=0.05, block=True)
                if not closed:
                    print(
                        f"WARNING: Failed to set gripper width to {CLOSE_WIDTH_M:.4f} m. "
                        f"Current width: {gripper.value:.4f} m"
                    )
                else:
                    actual_close_width_m = gripper.value
                    print(
                        f"Gripper close width target: {CLOSE_WIDTH_M:.4f} m, actual: {actual_close_width_m:.4f} m"
                    )
                    is_closed_within_tolerance = actual_close_width_m <= CLOSE_DETECTION_TOLERANCE_M
                    print(
                        f"Close detection (<= {CLOSE_DETECTION_TOLERANCE_M:.4f} m): "
                        f"{'PASS' if is_closed_within_tolerance else 'FAIL'}"
                    )
                time.sleep(0.3)

            if i == OPEN_AT_WAYPOINT:
                print(f"Waypoint {i} reached: opening gripper...")
                opened = gripper.open(block=True)
                if not opened:
                    print(f"WARNING: Failed to open gripper. Current width: {gripper.value:.4f} m")
                else:
                    print(f"Gripper opened. Current width: {gripper.value:.4f} m")
                time.sleep(0.3)

        print("Sequence complete.")
    finally:
        robot.shutdown()
        gripper.shutdown()


if __name__ == "__main__":
    main()
