"""Waveguide Teleoperation

Read the Waveguide outputs and process them to robot commands.

The commands are:
- slide in x
- slide in y
- up/down in z
- close/open gripper

Each command `<axis> <distance> <speed>` moves the end-effector by `distance` [m] along `axis` at
`speed` [m/s]. Commands come from the terminal or from the waveguide sensor (see `commands.py` and
`sensor.py`). Commands created while the robot is moving are ignored.

    pixi run -e wg python experiments/waveguide/waveguide_teleop.py                    # terminal
    pixi run -e wg python experiments/waveguide/waveguide_teleop.py --input waveguide --port /dev/ttyACM0
"""

import argparse
import time

import numpy as np
from arm_client.robot import Pose, Robot
from commands import CommandSource, Control, MoveCommand, TerminalSource, WaveguideSource
from sensor import DEFAULT_BAUDRATE

from arm_client import CONFIG_DIR

# --- Parameters ---
AXES = {"x": 0, "y": 1, "z": 2}
MAX_SPEED = 0.15  # m/s, average speed of a move (the peak is 1.875x, minimum-jerk profile)
MAX_DISTANCE = 0.2  # m, per command
CONTROLLER_MIN_MOVE_TIME = 0.5  # s, fr3_pose_controller stretches shorter moves to this

# Workspace box, relative to the home position [m]
WORKSPACE_LOWER = np.array([-0.15, -0.30, -0.25])
WORKSPACE_UPPER = np.array([0.30, 0.30, 0.20])

POLL_TIMEOUT = 0.1  # s, wait for a command


class AxisTeleop:
    """Moves the end-effector along the base axes with fr3_pose_controller, keeping the home orientation."""

    def __init__(self, robot: Robot):
        self.robot = robot

        print("Going to home position...")
        self.robot.home()
        self.homing_pose = self.robot.end_effector_pose.copy()
        self._switch_to_pose_controller()

        # Commanded position, kept separately from the measured pose so tracking errors don't accumulate
        self.target_position = self.homing_pose.position.copy()

    def _switch_to_pose_controller(self):
        self.robot.controller_switcher_client.switch_controller("fr3_pose_controller")
        self.robot.fr3_pose_controller_parameters_client.load_param_config(file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml")
        # Publish each goal once instead of republishing at 100 Hz: once a move ends, fr3_pose_controller
        # takes every repeat as a new (zero-length, >= 0.5 s) move and ignores new goals meanwhile
        self.robot.set_target_streaming(True)

    def move_along_axis(self, axis: str, distance: float, speed: float):
        """Move the end-effector by `distance` [m] along a base-frame `axis` at `speed` [m/s] (blocking).

        The target is clipped to the workspace box.
        """
        if axis not in AXES:
            raise ValueError(f"Unknown axis '{axis}', expected one of {list(AXES)}")
        if not 0.0 < speed <= MAX_SPEED:
            raise ValueError(f"Speed must be in (0, {MAX_SPEED}] m/s, got {speed}")
        if abs(distance) > MAX_DISTANCE:
            raise ValueError(f"|distance| must be <= {MAX_DISTANCE} m, got {distance}")

        new_position = self.target_position.copy()
        new_position[AXES[axis]] += distance
        clipped = np.clip(new_position, self.homing_pose.position + WORKSPACE_LOWER, self.homing_pose.position + WORKSPACE_UPPER)
        if not np.allclose(clipped, new_position):
            print(f"Target clipped to workspace: {new_position} -> {clipped}")

        if np.allclose(clipped, self.target_position):
            print("Already at the workspace limit, not moving.")
            return

        # fr3_pose_controller moves to a single target pose with a minimum-jerk profile (rest to rest) whose
        # duration is distance / motion.max_linear_velocity, so set that parameter to the requested speed.
        # Streaming intermediate poses (`move_to`) instead makes it stop at every one of them, and a
        # `target_trajectory` is limited to 5 cm/s by the ramp the controller prepends to it.
        self.robot.fr3_pose_controller_parameters_client.set_parameters([("motion.max_linear_velocity", float(speed))])
        time.sleep(0.01)  # The controller refreshes its parameters on its next update

        duration = max(float(np.linalg.norm(clipped - self.target_position)) / speed, CONTROLLER_MIN_MOVE_TIME)
        self.robot.publish_target(pose=Pose(clipped, self.homing_pose.orientation))
        self.target_position = clipped
        time.sleep(duration + 0.1)

    def go_home(self):
        self.robot.home()
        self._switch_to_pose_controller()
        self.target_position = self.homing_pose.position.copy()


def run(teleop: AxisTeleop, source: CommandSource):
    """Execute the commands of `source` until it asks to quit."""
    ready_since = time.monotonic()  # Commands created before this arrived while the robot was moving

    while True:
        command = source.get_command(POLL_TIMEOUT)
        if command is None:
            continue
        if command is Control.QUIT:
            return
        if command is Control.HOME:
            teleop.go_home()
        elif isinstance(command, MoveCommand):
            if command.t_created < ready_since:
                print(f"Ignored (robot was moving): {command}")
                continue
            try:
                teleop.move_along_axis(command.axis, command.distance, command.speed)
            except ValueError as e:
                print(f"Invalid command: {e}")
                continue
            print(f"Target: {teleop.target_position}, measured: {teleop.robot.end_effector_pose.position}")

        ready_since = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", choices=["terminal", "waveguide"], default="terminal")
    parser.add_argument("--port", help="Serial port of the waveguide Teensy, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    args = parser.parse_args()

    if args.input == "waveguide":
        if args.port is None:
            parser.error("--port is required with --input waveguide")
        source = WaveguideSource(args.port, args.baudrate)
    else:
        source = TerminalSource()

    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    try:
        teleop = AxisTeleop(robot)
        source.start()
        run(teleop, source)
    except KeyboardInterrupt:
        print()
    finally:
        # The robot holds the last target
        source.close()
        robot.shutdown()


if __name__ == "__main__":
    main()
