"""Waveguide Demo 1 - Slide Teleoperation

Read the Waveguide outputs and process them to robot commands.

The commands are:
- slide in x
- slide in y
- up/down in z
- close/open gripper

Each command `<axis> <distance> <speed>` moves the end-effector by `distance` [m] along `axis` at
`speed` [m/s]. Commands come from the terminal or from the waveguide sensor (see `commands.py` and
`sensor.py`). Commands created while the robot is moving are ignored.

    pixi run -e wg python experiments/waveguide/wg_teleop.py                     # waveguide, the Teensy sending frames (--port to pick one)
    pixi run -e wg python experiments/waveguide/wg_teleop.py --live-plot         # same, with the sensor live plot
    pixi run -e wg python experiments/waveguide/wg_teleop.py --input terminal    # commands typed in the terminal
    pixi run -e wg python experiments/waveguide/wg_teleop.py --record --notes "demo 1, take 3"   # records the run

`--record` saves the sensor frames, detections and swipes, the commands and what the robot did with them, and the
end-effector state, to ~/data/waveguide/teleop/<date-time> (see `recording.py`).
"""

import argparse
import time

import numpy as np
import recording
from arm_client.robot import Pose, Robot
from commands import CommandSource, Control, MoveCommand, TerminalSource, WaveguideSource
from scipy.spatial.transform import Rotation
from sensor import DEFAULT_BAUDRATE, DEFAULT_PORT

from arm_client import CONFIG_DIR

# --- Parameters ---
AXES = {"x": 0, "y": 1, "z": 2}
MAX_SPEED = 0.15  # m/s, average speed of a move (the peak is 1.875x, minimum-jerk profile)
MAX_DISTANCE = 0.2  # m, per command
CONTROLLER_MIN_MOVE_TIME = 0.5  # s, fr3_pose_controller stretches shorter moves to this
CONTROLLER_MAX_ANGULAR_SPEED = 1.0  # rad/s, motion.max_angular_velocity in configs/controllers/fr3_pose/default.yaml

# Workspace box, in the robot base frame [m] (home is at ~[0.31, 0.0, 0.49])
WORKSPACE_LOWER = np.array([0.0, -0.25, 0.09])
WORKSPACE_UPPER = np.array([0.8, 0.25, 0.8])

# The run starts at a random position drawn uniformly in this box (lower, upper), in the robot base frame [m]
START_POSITION_BOUNDS = np.array([[0.25, -0.10, 0.35], [0.40, 0.10, 0.55]])
START_ORIENTATION = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])  # Gripper pointing down, the home orientation
START_SPEED = 0.1  # m/s, move to the start position

POLL_TIMEOUT = 0.1  # s, wait for a command


class AxisTeleop:
    """Moves the end-effector along the base axes with fr3_pose_controller, keeping START_ORIENTATION."""

    def __init__(self, robot: Robot, rng: np.random.Generator | None = None):
        self.robot = robot
        current_position = self.robot.end_effector_pose.position.copy()
        target_position = current_position
        target_position[2] += 0.1
        rng = np.random.default_rng() if rng is None else rng
        start_position = rng.uniform(*START_POSITION_BOUNDS)
        self._switch_to_pose_controller()

        print(f"Going to the start position {np.round(start_position, 3)}...")
        # Not `robot.move_to`: with fr3_pose_controller it streams interpolated poses through the periodic
        # republishing, which streaming mode (`_switch_to_pose_controller`) turns off: nothing would be sent
        self._move_to(target_position, START_SPEED)

        self._move_to(start_position, START_SPEED)

    def _switch_to_pose_controller(self):
        self.robot.controller_switcher_client.switch_controller("fr3_pose_controller")
        self.robot.fr3_pose_controller_parameters_client.load_param_config(
            file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
        )
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
        clipped = np.clip(new_position, WORKSPACE_LOWER, WORKSPACE_UPPER)
        if not np.allclose(clipped, new_position):
            print(f"Target clipped to workspace: {new_position} -> {clipped}")

        if np.allclose(clipped, self.target_position):
            print("Already at the workspace limit, not moving.")
            return

        self._move_to(clipped, speed)

    def _move_to(self, position: np.ndarray, speed: float):
        """Move the end-effector to `position` [m] with START_ORIENTATION, at `speed` [m/s] (blocking).

        fr3_pose_controller moves to a single target pose with a minimum-jerk profile (rest to rest) whose
        duration is distance / motion.max_linear_velocity (or the rotation / motion.max_angular_velocity if
        longer), so set that parameter to the requested speed and publish the goal once. Streaming intermediate
        poses (`move_to`) instead makes it stop at every one of them, and a `target_trajectory` is limited to
        5 cm/s by the ramp the controller prepends to it.
        """
        self.robot.fr3_pose_controller_parameters_client.set_parameters([("motion.max_linear_velocity", float(speed))])
        time.sleep(0.01)  # The controller refreshes its parameters on its next update

        current = self.robot.end_effector_pose
        rotation = (START_ORIENTATION * current.orientation.inv()).magnitude()  # rad
        duration = max(
            float(np.linalg.norm(position - current.position)) / speed,
            rotation / CONTROLLER_MAX_ANGULAR_SPEED,
            CONTROLLER_MIN_MOVE_TIME,
        )
        self.robot.publish_target(pose=Pose(position, START_ORIENTATION))
        # Commanded position, kept separately from the measured pose so tracking errors don't accumulate
        self.target_position = np.asarray(position, dtype=float).copy()
        time.sleep(duration + 0.1)

    def go_home(self):
        self.robot.home()
        self.target_position = self.robot.end_effector_pose.position.copy()
        self._switch_to_pose_controller()


def run(teleop: AxisTeleop, source: CommandSource, recorder: recording.RobotRecorder | None = None):
    """Execute the commands of `source` until it asks to quit; `recorder` records each one and its outcome."""
    ready_since = time.monotonic()  # Commands created before this arrived while the robot was moving

    def record(command, status, detail=""):
        if recorder is not None:
            recorder.command(command, status, detail, target=teleop.target_position)

    while True:
        command = source.get_command(POLL_TIMEOUT)
        if command is None:
            continue
        if command is Control.QUIT:
            return
        if command is Control.HOME:
            teleop.go_home()
            record(None, "home")
        elif isinstance(command, MoveCommand):
            if command.t_created < ready_since:
                print(f"Ignored (robot was moving): {command}")
                record(command, "ignored_moving")
                continue
            requested = teleop.target_position.copy()
            if command.axis in AXES:
                requested[AXES[command.axis]] += command.distance
            try:
                teleop.move_along_axis(command.axis, command.distance, command.speed)
            except ValueError as e:
                print(f"Invalid command: {e}")
                record(command, "invalid", str(e))
                continue
            clipped = not np.allclose(teleop.target_position, requested)
            record(command, "executed", "clipped to the workspace" if clipped else "")
            print(f"Target: {teleop.target_position}, measured: {teleop.robot.end_effector_pose.position}")

        ready_since = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", choices=["terminal", "waveguide"], default="waveguide")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port of the Teensy (default: {DEFAULT_PORT})")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument(
        "--live-plot", action="store_true", help="Show the sensor live plot (closing it stops the teleop)"
    )
    parser.add_argument(
        "--record", action="store_true", help=f"Record the run to {recording.DATA_DIR / 'teleop'}/<date-time>"
    )
    parser.add_argument("--notes", default="", help="--record: purpose of the run, saved in its metadata")
    args = parser.parse_args()

    logger = None
    if args.record:
        from sensor import run_metadata

        metadata = {
            "args": vars(args),
            "teleop": {"max_speed": MAX_SPEED, "max_distance": MAX_DISTANCE},
            "workspace": {"lower": WORKSPACE_LOWER.tolist(), "upper": WORKSPACE_UPPER.tolist()},
            "start_position_bounds": START_POSITION_BOUNDS.tolist(),
            **run_metadata(),
        }
        logger = recording.start_logger("teleop", args.notes, metadata)

    if args.input == "waveguide":
        try:
            source = WaveguideSource(args.port, args.baudrate, plot=args.live_plot, logger=logger)
        except FileNotFoundError as e:
            if logger is not None:
                logger.stop()
            parser.exit(1, f"Error: {e}\n")
    else:
        if args.live_plot:
            parser.error("--live-plot needs --input waveguide")
        source = TerminalSource()

    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    robot_recorder = None
    try:
        teleop = AxisTeleop(robot)
        if logger is not None:

            def state():
                pose = robot.end_effector_pose
                return pose.position, pose.orientation.as_quat(), teleop.target_position

            robot_recorder = recording.RobotRecorder(logger, state)
            robot_recorder.start()
        source.start()
        run(teleop, source, robot_recorder)
    except KeyboardInterrupt:
        print()
    except RuntimeError as e:  # The sensor process ended
        print(f"Stopping: {e}")
    finally:
        # The robot holds the last target
        source.close()  # Also ends the sensor recording
        if robot_recorder is not None:
            robot_recorder.stop()
        if logger is not None:
            logger.stop()
            print(f"Recorded to {logger.run_dir}")
        robot.shutdown()


if __name__ == "__main__":
    main()
