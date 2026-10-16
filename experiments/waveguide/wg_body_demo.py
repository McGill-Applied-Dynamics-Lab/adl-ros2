"""Waveguide Demo 3 - Robot body: move the arm by touching it

The waveguide is wrapped around link 5. Each group of holes is mapped to a base-frame direction (+x, -x,
..., `BodyDemoConfig.direction_holes`); the end-effector orientation is kept constant, so the mapping
stays valid as long as link 5 does not rotate much (its drift is printed, see `link_drift_deg`).

- Holding holes of a direction accelerates the arm along it; on release, the arm keeps its velocity
- Touching holes opposite to the velocity stops the arm immediately ("collision detection"). It stays
  stopped until every hole is released
- The arm moves within an absolute bounding box (base frame): reaching it stops the arm

The arm streams pose + velocity targets to osc_pd_controller (`OscJogger`); fr3_pose_controller's jog
mode can replace it once available (same `Jogger` interface).

    pixi run -e wg python experiments/waveguide/wg_body_demo.py                    # waveguide, Teensy on /dev/ttyACM0 (--port)
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --input terminal   # covered holes typed in the terminal
"""

import argparse
import time
from dataclasses import dataclass, field

import numpy as np
from touch import DEFAULT_BAUDRATE, DEFAULT_PORT, TeensyTouchSource, TerminalTouchSource, TouchSource

DIRECTIONS = {
    "+x": np.array([1.0, 0.0, 0.0]),
    "-x": np.array([-1.0, 0.0, 0.0]),
    "+y": np.array([0.0, 1.0, 0.0]),
    "-y": np.array([0.0, -1.0, 0.0]),
    "+z": np.array([0.0, 0.0, 1.0]),
    "-z": np.array([0.0, 0.0, -1.0]),
}


# --- Update as desired
@dataclass(frozen=True)
class BodyDemoConfig:
    # Holes of each base-frame direction, with the arm at home. TODO: measure on the mounted waveguide
    direction_holes: dict[str, tuple[int, ...]] = field(
        default_factory=lambda: {
            "+x": (1, 2, 3, 4, 5),
            "-x": (6, 7, 8, 9, 10),
            "+y": (11, 12, 13, 14, 15),
            "-y": (16, 17, 18, 19, 20),
            "+z": (21, 22, 23, 24, 25),
            "-z": (26, 27, 28, 29, 30),
        }
    )
    acceleration: float = 0.05  # m/s^2, along each held direction
    max_speed: float = 0.10  # m/s
    # Bounding box of the end-effector position, absolute (base frame) [m]. TODO: set for the setup
    box_lower: tuple[float, float, float] = (0.30, -0.25, 0.20)
    box_upper: tuple[float, float, float] = (0.60, 0.25, 0.60)

    rate_hz: float = 100.0  # Control loop
    max_dt_s: float = 0.05  # A stalled loop step integrates at most this, so the target does not jump
    touch_timeout_s: float = 0.5  # No frame from the sensor for this long: stop
    link_name: str = "fr3_link5"  # Link the waveguide is on
    link_drift_warn_deg: float = 10.0  # Warn once its orientation drifted this much from the start
    drift_print_s: float = 1.0


CFG = BodyDemoConfig()


class JogLogic:
    """Touches -> end-effector velocity. Pure logic, no ROS: see the module docstring for the behavior."""

    def __init__(self, cfg: BodyDemoConfig = CFG):
        self.cfg = cfg
        self._hole_direction: dict[int, str] = {}
        for name, holes in cfg.direction_holes.items():
            if name not in DIRECTIONS:
                raise ValueError(f"Unknown direction '{name}', expected one of {list(DIRECTIONS)}")
            for hole in holes:
                if hole in self._hole_direction:
                    raise ValueError(f"Hole {hole} is mapped to both {self._hole_direction[hole]} and {name}")
                self._hole_direction[hole] = name
        self.velocity = np.zeros(3)
        self.stopped = False  # After a stop, until every hole is released

    def directions(self, covered: frozenset[int]) -> set[str]:
        """Directions of the covered holes (unmapped holes are ignored)."""
        return {self._hole_direction[h] for h in covered if h in self._hole_direction}

    def stop(self) -> None:
        self.velocity = np.zeros(3)
        self.stopped = True

    def update(self, dt: float, covered: frozenset[int]) -> bool:
        """Update `velocity` from the holes covered during the last `dt` [s]. Returns True on a stop."""
        directions = self.directions(covered)
        if self.stopped:
            self.stopped = bool(directions)
            return False
        if any(np.dot(DIRECTIONS[d], self.velocity) < 0.0 for d in directions):
            self.stop()
            return True
        for d in directions:
            self.velocity = self.velocity + self.cfg.acceleration * dt * DIRECTIONS[d]
        speed = float(np.linalg.norm(self.velocity))
        if speed > self.cfg.max_speed:
            self.velocity *= self.cfg.max_speed / speed
        return False


class Jogger:
    """Interface of the end-effector motion: velocity steps at constant orientation."""

    def start(self) -> np.ndarray:
        """Take control of the arm; returns the start position."""
        raise NotImplementedError

    def step(self, position: np.ndarray, velocity: np.ndarray) -> None:
        """Command `position` [m], moving at `velocity` [m/s]."""
        raise NotImplementedError

    def hard_stop(self) -> np.ndarray:
        """Stop where the arm is now; returns that position."""
        raise NotImplementedError


class OscJogger(Jogger):
    """Streams pose + twist targets to osc_pd_controller, keeping the start orientation."""

    def __init__(self, robot):
        from arm_client.robot import Pose, Twist

        from arm_client import CONFIG_DIR

        self._Pose, self._Twist, self._config_dir = Pose, Twist, CONFIG_DIR
        self.robot = robot
        self.orientation = None

    def start(self) -> np.ndarray:
        self.robot.controller_switcher_client.switch_controller("osc_pd_controller")
        self.robot.osc_pd_controller_parameters_client.load_param_config(
            file_path=self._config_dir / "controllers" / "osc_pd" / "default.yaml"
        )
        self.robot.set_target_streaming(True)
        self.orientation = self.robot.end_effector_pose.orientation
        return self.hard_stop()

    def step(self, position: np.ndarray, velocity: np.ndarray) -> None:
        self.robot.publish_target(
            pose=self._Pose(position, self.orientation), twist=self._Twist(velocity, np.zeros(3))
        )

    def hard_stop(self) -> np.ndarray:
        position = self.robot.end_effector_pose.position
        self.step(position, np.zeros(3))
        return position


class LinkDrift:
    """Orientation change of a link since the start [deg], from the joint positions (pyroki FK)."""

    def __init__(self, robot, link_name: str):
        import jax
        import jaxlie

        self._jaxlie = jaxlie
        self.robot = robot
        self._pk_robot = robot.pyroki_robot
        self._fk = jax.jit(self._pk_robot.forward_kinematics)
        self._link_idx = self._pk_robot.links.names.index(link_name)
        self._start = self._rotation()  # Also compiles the FK, before the control loop

    def _rotation(self):
        q = np.asarray(self.robot.q)[: self._pk_robot.joints.num_actuated_joints]
        if q.shape[0] < self._pk_robot.joints.num_actuated_joints:  # Gripper fingers in the URDF
            q = np.concatenate([q, np.zeros(self._pk_robot.joints.num_actuated_joints - q.shape[0])])
        wxyz_xyz = self._fk(q)[self._link_idx]
        return self._jaxlie.SO3(wxyz_xyz[:4])

    def degrees(self) -> float:
        delta = self._start.inverse() @ self._rotation()
        return float(np.rad2deg(np.linalg.norm(np.asarray(delta.log()))))


def run(jogger: Jogger, touch: TouchSource, cfg: BodyDemoConfig = CFG, drift: LinkDrift | None = None) -> None:
    """Jog until Ctrl+C (or `q` in the terminal input)."""
    lower, upper = np.array(cfg.box_lower), np.array(cfg.box_upper)
    logic = JogLogic(cfg)
    position = jogger.start()
    if np.any(position < lower) or np.any(position > upper):
        raise RuntimeError(f"Start position {position} is outside the bounding box [{lower}, {upper}]")

    period = 1.0 / cfg.rate_hz
    t_last = t_next = time.monotonic()
    t_drift = t_last
    drift_warned = False
    while not (isinstance(touch, TerminalTouchSource) and touch.quit_requested.is_set()):
        t_next += period
        time.sleep(max(0.0, t_next - time.monotonic()))
        now = time.monotonic()
        dt, t_last = min(now - t_last, cfg.max_dt_s), now

        frame = touch.latest()
        if frame is None or now - frame.t > cfg.touch_timeout_s:
            if np.any(logic.velocity):
                print("No touch data: stopping")
                logic.stop()
                position = jogger.hard_stop()
            continue

        moving = bool(np.any(logic.velocity))
        if logic.update(dt, frame.covered):
            position = jogger.hard_stop()
            print(f"STOP (touched {sorted(logic.directions(frame.covered))}) at {np.round(position, 3)}")
            continue
        if not np.any(logic.velocity):
            continue
        if not moving:
            print(f"Moving: {sorted(logic.directions(frame.covered))}")

        new_position = position + logic.velocity * dt
        clipped = np.clip(new_position, lower, upper)
        if not np.allclose(clipped, new_position):
            logic.stop()
            jogger.step(clipped, np.zeros(3))
            position = clipped
            print(f"Bounding box reached at {np.round(position, 3)}: stopped")
            continue
        position = new_position
        jogger.step(position, logic.velocity)

        if drift is not None and now - t_drift > cfg.drift_print_s:
            t_drift = now
            degrees = drift.degrees()
            if degrees > cfg.link_drift_warn_deg and not drift_warned:
                print(f"Warning: {cfg.link_name} rotated {degrees:.1f} deg, the hole mapping may be off")
                drift_warned = True
            print(f"v = {np.round(logic.velocity, 3)} m/s, {cfg.link_name} drift {degrees:.1f} deg")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", choices=["terminal", "waveguide"], default="waveguide")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port of the Teensy (default: {DEFAULT_PORT})")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--no-drift", action="store_true", help=f"Do not monitor the {CFG.link_name} orientation")
    args = parser.parse_args()

    if args.input == "waveguide":
        try:
            touch = TeensyTouchSource(args.port, args.baudrate)
        except FileNotFoundError as e:
            parser.exit(1, f"Error: {e}\n")
    else:
        touch = TerminalTouchSource(aliases=CFG.direction_holes)

    from arm_client.robot import Robot

    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    jogger = OscJogger(robot)
    try:
        print("Going to home position...")
        robot.home()
        drift = None if args.no_drift else LinkDrift(robot, CFG.link_name)
        touch.start()
        run(jogger, touch, CFG, drift)
    except KeyboardInterrupt:
        print()
    except RuntimeError as e:
        print(f"Stopping: {e}")
    finally:
        if jogger.orientation is not None:
            jogger.hard_stop()  # The robot holds this target
        touch.close()
        robot.shutdown()


if __name__ == "__main__":
    main()
