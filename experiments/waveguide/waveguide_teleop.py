"""Waveguide Teleoperation

Read the Waveguide outputs and process them to robot commands.

The commands are:
- slide in x
- slide in y
- up/down in z
- close/open gripper

For now, commands are read from the terminal as `<axis> <distance> <speed>`, e.g. `x 0.05 0.02`
moves 5 cm along +x at 2 cm/s. Later, the waveguide sensor will produce the same commands.

Terminal commands:
    <axis> <distance> <speed>   axis in {x, y, z}, distance in m (signed), speed in m/s
    home                        go back to the home position
    q                           quit
"""

import time

import numpy as np
from arm_client.robot import Pose, Robot

from arm_client import CONFIG_DIR

# --- Parameters ---
AXES = {"x": 0, "y": 1, "z": 2}
MAX_SPEED = 0.15  # m/s, average speed of a move (the peak is 1.875x, minimum-jerk profile)
MAX_DISTANCE = 0.2  # m, per command
CONTROLLER_MIN_MOVE_TIME = 0.5  # s, fr3_pose_controller stretches shorter moves to this

# Workspace box, relative to the home position [m]
WORKSPACE_LOWER = np.array([-0.15, -0.30, -0.25])
WORKSPACE_UPPER = np.array([0.30, 0.30, 0.20])

# Initialization
robot = Robot(namespace="fr3")
robot.wait_until_ready()

print(robot.end_effector_pose)
print(robot.q)

print("Going to home position...")
robot.home()
homing_pose = robot.end_effector_pose.copy()


def switch_to_pose_controller():
    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml")
    # Publish each goal once instead of republishing at 100 Hz: once a move ends, fr3_pose_controller
    # takes every repeat as a new (zero-length, >= 0.5 s) move and ignores new goals meanwhile
    robot.set_target_streaming(True)


switch_to_pose_controller()

# Commanded position, kept separately from the measured pose so tracking errors don't accumulate
target_position = homing_pose.position.copy()


# --- Control Mappings ---
def move_along_axis(axis: str, distance: float, speed: float):
    """Move the end-effector by `distance` [m] along a base-frame `axis` at `speed` [m/s].

    The orientation is kept at the home orientation. The target is clipped to the workspace box.
    """
    global target_position

    if axis not in AXES:
        raise ValueError(f"Unknown axis '{axis}', expected one of {list(AXES)}")
    if not 0.0 < speed <= MAX_SPEED:
        raise ValueError(f"Speed must be in (0, {MAX_SPEED}] m/s, got {speed}")
    if abs(distance) > MAX_DISTANCE:
        raise ValueError(f"|distance| must be <= {MAX_DISTANCE} m, got {distance}")

    new_position = target_position.copy()
    new_position[AXES[axis]] += distance
    clipped = np.clip(new_position, homing_pose.position + WORKSPACE_LOWER, homing_pose.position + WORKSPACE_UPPER)
    if not np.allclose(clipped, new_position):
        print(f"Target clipped to workspace: {new_position} -> {clipped}")

    if np.allclose(clipped, target_position):
        print("Already at the workspace limit, not moving.")
        return

    # fr3_pose_controller moves to a single target pose with a minimum-jerk profile (rest to rest) whose
    # duration is distance / motion.max_linear_velocity, so set that parameter to the requested speed.
    # Streaming intermediate poses (`move_to`) instead makes it stop at every one of them, and a
    # `target_trajectory` is limited to 5 cm/s by the ramp the controller prepends to it.
    robot.fr3_pose_controller_parameters_client.set_parameters([("motion.max_linear_velocity", float(speed))])
    time.sleep(0.01)  # The controller refreshes its parameters on its next update

    duration = max(float(np.linalg.norm(clipped - target_position)) / speed, CONTROLLER_MIN_MOVE_TIME)
    robot.publish_target(pose=Pose(clipped, homing_pose.orientation))
    target_position = clipped
    time.sleep(duration + 0.1)


def go_home():
    global target_position

    robot.home()
    switch_to_pose_controller()
    target_position = homing_pose.position.copy()


def parse_command(line: str) -> tuple[str, float, float]:
    """Parse `<axis> <distance> <speed>`."""
    parts = line.split()
    if len(parts) != 3:
        raise ValueError("Expected '<axis> <distance> <speed>'")
    axis, distance, speed = parts[0].lower(), float(parts[1]), float(parts[2])
    return axis, distance, speed


# --- Main Loop ---
print(__doc__)
try:
    while True:
        line = input("> ").strip()
        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            break
        if line.lower() == "home":
            go_home()
            continue

        try:
            axis, distance, speed = parse_command(line)
            move_along_axis(axis, distance, speed)
        except ValueError as e:
            print(f"Invalid command: {e}")
            continue

        print(f"Target: {target_position}, measured: {robot.end_effector_pose.position}")
except (KeyboardInterrupt, EOFError):
    print()


# --- Cleanup
robot.shutdown()
