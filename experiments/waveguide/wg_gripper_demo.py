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
cancelled (the gripper stops where it is) as soon as the waveguide shows the stop condition. The Teensy
detects the holes on board and sends a result every cycle (~20/s, + the BLE latency), and the
franka_gripper server checks for cancel requests at its `feedback_publish_rate` (30 Hz): the gripper stops
within ~100 ms of the contact, about 0.5 mm at the default contact speed.

The Teensy (TEENSY_BLUETOOTH firmware) is reached over BLE by default (`--link serial` for a serial port), see
`touch.TeensyConfig`. Its calibration is uploaded on the first run after a power cycle (about a minute over BLE).

    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py                       # squeeze 1 mm after the contact
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --squeeze-mm 2
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --mode amplitude --amplitude-rise 50
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --sensor-calibration calibration/gripper.npz --upload
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --record --notes "egg 3"   # records the run

`--record` saves the Teensy results, the raw frames if the Teensy's USB cable is plugged in, the end-effector and
gripper width (50 Hz) and the demo phases (contact width and amplitude, squeeze, ...), to
~/data/waveguide/gripper/<date-time> (see `recording.py`; plot it with `plot_run.py`).
"""

import argparse
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from arm_client.gripper.franka_hand import Gripper
from arm_client.robot import Pose, Robot
from touch import (
    TeensyTouchSource,
    TouchFrame,
    TouchSource,
    add_record_args,
    add_teensy_args,
    start_demo_recording,
    stop_demo_recording,
    teensy_config,
)

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
    """`recorder` (`recording.DemoRecorder`, optional): records each phase (`_event`)."""

    def __init__(
        self, robot: Robot, gripper: Gripper, touch: TouchSource, cfg: GripperDemoConfig = CFG, recorder=None
    ):
        self.robot, self.gripper, self.touch, self.cfg, self.recorder = robot, gripper, touch, cfg, recorder

    def _event(self, kind: str, message: str, **data) -> None:
        """Print `message`; with a recorder, also record it (`demo/event`) with the covered holes and `data`."""
        print(message)
        if self.recorder is not None:
            frame = self._frame()
            self.recorder.event(kind, message.strip(), frame.covered, amplitude=frame.amplitude, **data)

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
        self._event(
            "fast_close", f"\tFast closing to {cfg.fast_close_width * 1e3:.1f} mm at {cfg.fast_speed * 1e3:.1f} mm/s"
        )
        self.gripper.set_target(cfg.fast_close_width, speed=cfg.fast_speed, block=True)
        if self._frame().covered:
            raise RuntimeError(f"Holes {sorted(self._frame().covered)} covered before the contact search")

        # 3. Close until contact
        self._event("contact_search", f"\tClosing until contact (holes covered) at {cfg.contact_speed * 1e3:.1f} mm/s")
        contact, frame = self.close_until(cfg.min_width, cfg.contact_speed, lambda f: bool(f.covered))
        if not contact:
            raise RuntimeError(f"No contact down to {cfg.min_width * 1e3:.0f} mm: is the egg there?")
        contact_width, contact_amplitude = self.width(), frame.amplitude
        self._event(
            "contact",
            f"Contact at {contact_width * 1e3:.1f} mm: holes {sorted(frame.covered)}, amplitude {contact_amplitude:.1f}",
            width=contact_width,
            contact_amplitude=contact_amplitude,
            contact_holes=sorted(frame.covered),
        )

        # 4. Squeeze
        limit_mm = min(squeeze_mm, cfg.max_squeeze_mm) if mode == "squeeze" else cfg.max_squeeze_mm
        target = max(contact_width - limit_mm * 1e-3, cfg.min_width)
        self._event(
            "squeeze",
            f"\tSqueezing in {mode} mode, amplitude rise {amplitude_rise}, max {cfg.max_squeeze_mm:.1f} mm",
            mode=mode,
            squeeze_mm=squeeze_mm,
            amplitude_rise=amplitude_rise,
            target_width=target,
        )
        if mode == "squeeze":
            self.gripper.set_target(target, speed=cfg.squeeze_speed, block=True)
            width = self.width()
            self._event(
                "squeezed",
                f"Squeezed {(contact_width - width) * 1e3:.1f} mm",
                width=width,
                squeezed_mm=(contact_width - width) * 1e3,
            )
        else:
            rose, frame = self.close_until(
                target, cfg.squeeze_speed, lambda f: f.amplitude - contact_amplitude >= amplitude_rise
            )
            width = self.width()
            squeezed_mm = (contact_width - width) * 1e3
            if rose:
                message = (
                    f"Amplitude {frame.amplitude:.1f} (+{frame.amplitude - contact_amplitude:.1f}): squeezed {squeezed_mm:.1f} mm"
                )
            else:
                message = f"Amplitude rise not reached after {squeezed_mm:.1f} mm (max_squeeze_mm), holding there"
            self._event("squeezed", message, width=width, squeezed_mm=squeezed_mm, rise_reached=rose)

    def run(self, mode: str, squeeze_mm: float, amplitude_rise: float) -> None:
        cfg = self.cfg
        print("Going to home position...")
        # self.robot.home()  # joint_trajectory_controller: move_to plans with IK
        orientation = self.robot.end_effector_pose.orientation  # Top-down grasp, as at home
        self.gripper.set_target(cfg.open_width, speed=cfg.fast_speed, block=True)

        # Change to fr3_pose_controller
        # controller_name = "osc_controller"  # osc_controller, gravity_compensation, ....
        # self.robot.controller_switcher_client.switch_controller(controller_name)
        # self.robot.osc_controller_parameters_client.load_param_config(
        #     file_path=CONFIG_DIR / "controllers" / "osc" / "default.yaml"
        # )

        # 1. Move to the egg
        self._event("approach", f"Moving to the egg at {EGG_POSITION} m, approach height {cfg.approach_height} m")
        above = Pose(EGG_POSITION + [0.0, 0.0, cfg.approach_height], orientation)
        self.robot.move_to(pose=above, speed=cfg.move_speed)
        time.sleep(0.5)  # Wait for the robot to settle before going down

        self._event("down", "Going down to the egg")
        self.robot.move_to(pose=Pose(EGG_POSITION, orientation), speed=cfg.move_speed)

        print("Grasping...")
        self.grasp(mode, squeeze_mm, amplitude_rise)

        # 5. Lift, hold, put down, release
        self._event("lift", f"Lifting by {cfg.lift_height} m")
        self.robot.move_to(pose=Pose(EGG_POSITION + [0.0, 0.0, cfg.lift_height], orientation), speed=cfg.move_speed)
        self._event("hold", f"Holding for {cfg.hold_s} s, amplitude {self._frame().amplitude:.1f}")
        time.sleep(cfg.hold_s)
        self._event("put_down", "Putting the egg down")
        self.robot.move_to(pose=Pose(EGG_POSITION, orientation), speed=cfg.move_speed)
        self._event("release", "Releasing")
        self.gripper.set_target(cfg.open_width, speed=cfg.fast_speed, block=True)
        self.robot.move_to(pose=above, speed=cfg.move_speed)
        self._event("home", "Going home")
        self.robot.home()
        self._event("done", "Done")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["squeeze", "amplitude"], default="squeeze")
    parser.add_argument("--squeeze-mm", type=float, default=1.0, help="squeeze mode: closing after the contact [mm]")
    parser.add_argument("--amplitude-rise", type=float, default=None, help="amplitude mode: rise to stop at")
    add_teensy_args(parser)
    add_record_args(parser, "gripper")
    args = parser.parse_args()
    if args.mode == "amplitude" and args.amplitude_rise is None:
        parser.error("--mode amplitude needs --amplitude-rise")
    if args.squeeze_mm > CFG.max_squeeze_mm:
        parser.error(f"--squeeze-mm is limited to {CFG.max_squeeze_mm} mm (GripperDemoConfig.max_squeeze_mm)")

    # Started before the sensor, so the raw frames (USB cable plugged in) include its background
    logger, recorder, frames = start_demo_recording(args, "gripper", {"gripper_demo": CFG})
    if recorder is not None:
        recorder.event("config", "EGG_POSITION", egg_position=EGG_POSITION)
    try:
        touch = TeensyTouchSource(teensy_config(args), recorder)
    except (FileNotFoundError, ValueError) as e:
        stop_demo_recording(logger, frames)
        parser.exit(1, f"Error: {e}\n")

    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3")
    robot.wait_until_ready()
    gripper.wait_until_ready()

    state_recorder = None
    try:
        if logger is not None:
            import recording

            def state():
                pose = robot.end_effector_pose
                return pose.position, pose.orientation.as_quat(), None, {"gripper_width": gripper.value}

            state_recorder = recording.RobotRecorder(logger, state)
            state_recorder.start()
        touch.start()
        EggGrasp(robot, gripper, touch, recorder=recorder).run(args.mode, args.squeeze_mm, args.amplitude_rise)
    except KeyboardInterrupt:
        print()
        if recorder is not None:
            recorder.event("interrupted", "Ctrl+C")
    except (RuntimeError, TimeoutError) as e:
        print(f"Stopping: {e}")
        if recorder is not None:
            recorder.event("error", str(e))
    finally:
        # The robot holds the last target, the gripper its width
        touch.close()
        if state_recorder is not None:
            state_recorder.stop()
        stop_demo_recording(logger, frames)
        robot.shutdown()


if __name__ == "__main__":
    main()
