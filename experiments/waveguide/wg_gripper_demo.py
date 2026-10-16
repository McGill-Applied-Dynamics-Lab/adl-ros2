"""Waveguide Demo 2 - Gripper: grasp an egg

The waveguide is mounted on the fingers of the Franka Hand. The demo:
1. Moves above the egg, then down to it (hard-coded pose, `EGG_POSITION`)
2. Closes fast to `GripperDemoConfig.fast_close_width` (the egg is not touched yet)
3. Closes slowly until contact: a hole is covered
4. Squeezes, by one of (`--mode`):
    - `squeeze`: closing `--squeeze-mm` more after the contact
    - `amplitude`: closing until the amplitude rose by `--amplitude-rise` from the contact, at most
      `GripperDemoConfig.max_squeeze_mm`
5. Lifts the egg, holds it, puts it down, releases it and goes back home

Closing never blocks: the gripper moves towards the target width with a non-blocking action, which is
cancelled (the gripper stops where it is) as soon as the waveguide shows the stop condition. The
franka_gripper server checks for cancel requests at its `feedback_publish_rate` (30 Hz), so the gripper
stops within ~35 ms + the stop time: about 0.2 mm at the default contact speed.

    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py                       # squeeze 1 mm after the contact
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --squeeze-mm 2
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --mode amplitude --amplitude-rise 50
"""

import argparse
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from arm_client.gripper.franka_hand import Gripper
from arm_client.robot import Pose, Robot
from touch import DEFAULT_BAUDRATE, DEFAULT_PORT, TeensyTouchSource, TouchFrame, TouchSource

# --- Update as desired
EGG_POSITION = np.array([0.45, 0.0, 0.05])  # m, base frame: end-effector (TCP) position to grasp the egg. TODO: measure


@dataclass(frozen=True)
class GripperDemoConfig:
    approach_height: float = 0.10  # m, above the egg before going down to it
    lift_height: float = 0.05  # m
    hold_s: float = 2.0  # Time the egg is held up
    move_speed: float = 0.05  # m/s, arm motions

    open_width: float = 0.08  # m
    fast_close_width: float = 0.055  # m, closed to at `fast_speed` before looking for the contact (> egg width)
    fast_speed: float = 0.05  # m/s
    contact_speed: float = 0.005  # m/s, closing until the contact
    squeeze_speed: float = 0.002  # m/s, closing after the contact
    min_width: float = 0.01  # m, never closes further: the contact search fails there (no egg?)
    max_squeeze_mm: float = 5.0  # Closing after the contact is limited to this, in both modes

    poll_s: float = 0.002  # Period of the waveguide checks while closing
    close_timeout_s: float = 20.0
    # The gripper width is published at 15 Hz (franka_gripper state_publish_rate): wait for a fresh one
    width_settle_s: float = 0.15


CFG = GripperDemoConfig()


class EggGrasp:
    def __init__(self, robot: Robot, gripper: Gripper, touch: TouchSource, cfg: GripperDemoConfig = CFG):
        self.robot, self.gripper, self.touch, self.cfg = robot, gripper, touch, cfg

    def _frame(self) -> TouchFrame:
        frame = self.touch.latest()
        return frame if frame is not None else TouchFrame()

    def close_until(
        self, target_width: float, speed: float, stop: Callable[[TouchFrame], bool]
    ) -> tuple[bool, TouchFrame]:
        """Close towards `target_width` [m] at `speed` [m/s] until `stop(frame)` is true.

        Returns whether `stop` happened (False: the target width was reached first) and the frame then.
        """
        future = self.gripper.set_target(target_width, speed=speed, block=False)
        t_end = time.monotonic() + self.cfg.close_timeout_s
        try:
            while not future.done():
                frame = self._frame()
                if stop(frame):
                    future.cancel()
                    self._wait(future)
                    return True, frame
                if time.monotonic() > t_end:
                    raise TimeoutError("Gripper did not reach its target width in time")
                time.sleep(self.cfg.poll_s)
        except BaseException:  # Also on Ctrl+C: never leave the gripper closing
            future.cancel()
            raise
        return False, self._frame()

    def _wait(self, future) -> None:
        t_end = time.monotonic() + self.cfg.close_timeout_s
        while not future.done():
            if time.monotonic() > t_end:
                raise TimeoutError("Gripper did not stop in time")
            time.sleep(0.01)

    def width(self) -> float:
        time.sleep(self.cfg.width_settle_s)
        return self.gripper.value

    def grasp(self, mode: str, squeeze_mm: float, amplitude_rise: float) -> None:
        cfg = self.cfg
        # 2. Fast close, the egg must not be touched yet
        self.gripper.set_target(cfg.fast_close_width, speed=cfg.fast_speed, block=True)
        if self._frame().covered:
            raise RuntimeError(f"Holes {sorted(self._frame().covered)} covered before the contact search")

        # 3. Close until contact
        contact, frame = self.close_until(cfg.min_width, cfg.contact_speed, lambda f: bool(f.covered))
        if not contact:
            raise RuntimeError(f"No contact down to {cfg.min_width * 1e3:.0f} mm: is the egg there?")
        contact_width, contact_amplitude = self.width(), frame.amplitude
        print(
            f"Contact at {contact_width * 1e3:.1f} mm: holes {sorted(frame.covered)}, amplitude {contact_amplitude:.1f}"
        )

        # 4. Squeeze
        limit_mm = min(squeeze_mm, cfg.max_squeeze_mm) if mode == "squeeze" else cfg.max_squeeze_mm
        target = max(contact_width - limit_mm * 1e-3, cfg.min_width)
        if mode == "squeeze":
            self.gripper.set_target(target, speed=cfg.squeeze_speed, block=True)
            print(f"Squeezed {(contact_width - self.width()) * 1e3:.1f} mm")
        else:
            rose, frame = self.close_until(
                target, cfg.squeeze_speed, lambda f: f.amplitude - contact_amplitude >= amplitude_rise
            )
            squeezed_mm = (contact_width - self.width()) * 1e3
            if rose:
                print(f"Amplitude {frame.amplitude:.1f} (+{frame.amplitude - contact_amplitude:.1f}): squeezed {squeezed_mm:.1f} mm")
            else:
                print(f"Amplitude rise not reached after {squeezed_mm:.1f} mm (max_squeeze_mm), holding there")

    def run(self, mode: str, squeeze_mm: float, amplitude_rise: float) -> None:
        cfg = self.cfg
        print("Going to home position...")
        self.robot.home()  # joint_trajectory_controller: move_to plans with IK
        orientation = self.robot.end_effector_pose.orientation  # Top-down grasp, as at home
        self.gripper.set_target(cfg.open_width, speed=cfg.fast_speed, block=True)

        # 1. Move to the egg
        above = Pose(EGG_POSITION + [0.0, 0.0, cfg.approach_height], orientation)
        self.robot.move_to(pose=above, speed=cfg.move_speed)
        self.robot.move_to(pose=Pose(EGG_POSITION, orientation), speed=cfg.move_speed)

        self.grasp(mode, squeeze_mm, amplitude_rise)

        # 5. Lift, hold, put down, release
        self.robot.move_to(pose=Pose(EGG_POSITION + [0.0, 0.0, cfg.lift_height], orientation), speed=cfg.move_speed)
        print(f"Holding for {cfg.hold_s} s, amplitude {self._frame().amplitude:.1f}")
        time.sleep(cfg.hold_s)
        self.robot.move_to(pose=Pose(EGG_POSITION, orientation), speed=cfg.move_speed)
        self.gripper.set_target(cfg.open_width, speed=cfg.fast_speed, block=True)
        self.robot.move_to(pose=above, speed=cfg.move_speed)
        self.robot.home()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["squeeze", "amplitude"], default="squeeze")
    parser.add_argument("--squeeze-mm", type=float, default=1.0, help="squeeze mode: closing after the contact [mm]")
    parser.add_argument("--amplitude-rise", type=float, default=None, help="amplitude mode: rise to stop at")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port of the Teensy (default: {DEFAULT_PORT})")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    args = parser.parse_args()
    if args.mode == "amplitude" and args.amplitude_rise is None:
        parser.error("--mode amplitude needs --amplitude-rise")
    if args.squeeze_mm > CFG.max_squeeze_mm:
        parser.error(f"--squeeze-mm is limited to {CFG.max_squeeze_mm} mm (GripperDemoConfig.max_squeeze_mm)")

    try:
        touch = TeensyTouchSource(args.port, args.baudrate)
    except FileNotFoundError as e:
        parser.exit(1, f"Error: {e}\n")

    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3")
    robot.wait_until_ready()
    gripper.wait_until_ready()

    try:
        touch.start()
        EggGrasp(robot, gripper, touch).run(args.mode, args.squeeze_mm, args.amplitude_rise)
    except KeyboardInterrupt:
        print()
    except (RuntimeError, TimeoutError) as e:
        print(f"Stopping: {e}")
    finally:
        # The robot holds the last target, the gripper its width
        touch.close()
        robot.shutdown()


if __name__ == "__main__":
    main()
