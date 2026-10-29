"""Waveguide Demo 3 - Robot body: move the arm by touching it

The waveguide is wrapped around link 5. Each hole has a direction: the axis through it, pointing out of the
link, along which touching it moves the arm. The end-effector orientation is kept constant.

- Holding holes accelerates the arm along their direction (their mean, for neighbouring holes covered by one
  finger); on release, the arm keeps its velocity
- Touching a hole whose direction is more than `stop_angle_deg` from the velocity stops the arm immediately
  ("collision detection"). It stays stopped until every hole is released
- The arm moves within an absolute bounding box (base frame): reaching it stops the arm

Hole directions:
- Calibration (`--calibrate`, with the arm at home): the script asks to touch all the holes of each base axis
  (+x, -x, ...) in turn, and saves them with the link orientation (`BodyDemoConfig.calibration_path`) for
  the next runs. Without one, the placeholder `BodyDemoConfig.direction_holes` is used
- `interpolate_holes`: the holes between two calibrated ones get a direction interpolated between theirs,
  in proportion to their index (e.g. 10: +y, 14: +z -> 11 is +y rotated 22.5 deg about x, 12 45 deg, 13 67.5 deg)
- `fixed_mapping`: True, the directions stay as calibrated, in the base frame (valid while link 5 does not
  rotate much: its drift is printed). False, they follow the rotation of link 5 since the calibration

The Teensy (TEENSY_BLUETOOTH firmware) detects the holes on board, see `touch.TeensyConfig`; its sensor
calibration (`--sensor-calibration`) is uploaded on the first run after a power cycle (about a minute over BLE).

The arm streams pose + velocity targets to osc_pd_controller (`OscJogger`); fr3_pose_controller's jog
mode can replace it once available (same `Jogger` interface).

`--live-plot` (`body_live_plot.py`): the touches, the holes coloured by their direction, the arm in its bounding
box, and the calibration of the mapping from the plot (⚙ tab). `--no-robot` simulates the arm (no ROS), to try
the sensor and the plot on the bench.

    pixi run -e wg python experiments/waveguide/wg_body_demo.py                    # waveguide, Teensy over BLE (--link serial)
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --live-plot        # same, with the live plot (calibration: ⚙ tab)
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --calibrate        # calibrate the hole -> direction mapping first
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --input terminal   # covered holes typed in the terminal
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --live-plot --no-robot --input terminal   # no arm, no sensor
"""

import argparse
import json
import shutil
import threading
import time
from collections import Counter
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
from touch import TeensyTouchSource, TerminalTouchSource, TouchFrame, TouchSource, add_teensy_args, teensy_config

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
    # Holes of each base axis, with the arm at home. Placeholder, used without a calibration
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
    interpolate_holes: bool = True  # Directions of the holes between two calibrated ones (module docstring)
    fixed_mapping: bool = True  # True: directions fixed in the base frame; False: they follow the link rotation
    acceleration: float = 0.05  # m/s^2, along the held direction
    max_speed: float = 0.10  # m/s
    stop_angle_deg: float = 120.0  # A touch further than this from the velocity stops the arm (in (90, 180))
    # Bounding box of the end-effector position, absolute (base frame) [m]. TODO: set for the setup
    box_lower: tuple[float, float, float] = (0.30, -0.25, 0.20)
    box_upper: tuple[float, float, float] = (0.60, 0.25, 0.60)

    rate_hz: float = 100.0  # Control loop
    max_dt_s: float = 0.05  # A stalled loop step integrates at most this, so the target does not jump
    touch_timeout_s: float = 0.5  # No frame from the sensor for this long: stop
    link_name: str = "fr3_link5"  # Link the waveguide is on
    link_drift_warn_deg: float = 10.0  # fixed_mapping: warn once its orientation drifted this much from the start
    drift_print_s: float = 1.0

    # Calibration (--calibrate)
    calibration_path: Path = Path(__file__).resolve().parent / "calibration" / "body_direction_holes.json"
    calibration_min_frames: int = 3  # A hole must be covered in this many frames to count (noise)
    calibration_poll_s: float = 0.001


CFG = BodyDemoConfig()


class DirectionCalibration:
    """Hole -> direction mapping from touches: each hole goes to the direction it was covered most during."""

    def __init__(self, min_frames: int = CFG.calibration_min_frames):
        self.min_frames = min_frames
        self.counts: dict[str, Counter] = {}

    def add(self, direction: str, frame: TouchFrame) -> None:
        self.counts.setdefault(direction, Counter()).update(frame.covered)

    def direction_holes(self) -> dict[str, tuple[int, ...]]:
        best: dict[int, tuple[int, str]] = {}  # hole -> (count, direction)
        for direction, counts in self.counts.items():
            for hole, n in counts.items():
                if n >= self.min_frames and n > best.get(hole, (0, ""))[0]:
                    best[hole] = (n, direction)
        return {d: tuple(sorted(h for h, (_, hd) in best.items() if hd == d)) for d in self.counts}

    def conflicts(self) -> dict[int, list[str]]:
        """Holes counted in more than one direction (each was assigned to its most frequent one)."""
        seen: dict[int, list[str]] = {}
        for direction, counts in self.counts.items():
            for hole, n in counts.items():
                if n >= self.min_frames:
                    seen.setdefault(hole, []).append(direction)
        return {h: ds for h, ds in sorted(seen.items()) if len(ds) > 1}


def _wait_enter(prompt: str, done: threading.Event) -> None:
    input(prompt)
    done.set()


@dataclass(frozen=True)
class HoleMapping:
    """Holes of each base axis, and the link rotation (base <- link, 3x3) when they were recorded (None: unknown)."""

    direction_holes: dict[str, tuple[int, ...]]
    link_rotation: np.ndarray | None = None


def calibrate(touch: TouchSource, cfg: BodyDemoConfig = CFG, link: "LinkOrientation | None" = None) -> HoleMapping:
    """Ask to touch the holes of each direction in turn; returns (and saves) the mapping.

    `link` records the link orientation, needed by `fixed_mapping=False`.
    """
    calibration = DirectionCalibration(cfg.calibration_min_frames)
    print("Calibration of the hole -> direction mapping, with the arm at home.")
    link_rotation = None if link is None else link.rotation()
    for direction in DIRECTIONS:
        input(f"\nPress Enter, then touch all the holes moving the arm along {direction} (one by one or a swipe)...")
        done = threading.Event()
        threading.Thread(target=_wait_enter, args=("Recording, press Enter when done: ", done), daemon=True).start()
        calibration.add(direction, TouchFrame())  # The direction exists even if nothing is touched
        last_t = None
        while not done.is_set():
            frame = touch.latest()
            if frame is not None and frame.t != last_t:  # Count each frame once
                last_t = frame.t
                calibration.add(direction, frame)
            time.sleep(cfg.calibration_poll_s)
        print(f"{direction}: holes {sorted(h for h, n in calibration.counts[direction].items() if n >= cfg.calibration_min_frames)}")

    direction_holes = calibration.direction_holes()
    for hole, directions in calibration.conflicts().items():
        print(f"Warning: hole {hole} was touched for {directions}, assigned to the most frequent")
    for direction, holes in direction_holes.items():
        if not holes:
            print(f"Warning: no hole for {direction}")
    mapping = HoleMapping(direction_holes, link_rotation)
    save_mapping(mapping, cfg.calibration_path)
    print(f"Saved to {cfg.calibration_path}")
    return mapping


def save_mapping(mapping: HoleMapping, path: Path, backup: bool = False) -> Path | None:
    """Write the mapping to `path`. With `backup`, the previous file is kept as <name>_backup_<date>_<time>
    (returned, None if there was none)."""
    backup_path = None
    if backup and path.exists():
        backup_path = path.with_name(f"{path.stem}_backup_{time.strftime('%Y_%m_%d_%H%M%S')}{path.suffix}")
        shutil.copy2(path, backup_path)
    data = {"direction_holes": {d: list(h) for d, h in mapping.direction_holes.items()}}
    if mapping.link_rotation is not None:
        data["link_rotation"] = np.asarray(mapping.link_rotation).tolist()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    return backup_path


def load_mapping(path: Path) -> HoleMapping | None:
    """The saved mapping, or None if there is no calibration yet."""
    if not path.exists():
        return None
    data = json.loads(path.read_text())
    if "direction_holes" not in data:  # First format: the holes only
        data = {"direction_holes": data}
    direction_holes = {d: tuple(int(h) for h in holes) for d, holes in data["direction_holes"].items()}
    link_rotation = np.array(data["link_rotation"]) if "link_rotation" in data else None
    return HoleMapping(direction_holes, link_rotation)


def slerp(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    """Unit vector at fraction `s` of the rotation from unit vector `a` to `b` (not opposite)."""
    angle = float(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0)))
    if angle < 1e-9:
        return a.copy()
    return (np.sin((1.0 - s) * angle) * a + np.sin(s * angle) * b) / np.sin(angle)


def hole_vectors(direction_holes: dict[str, tuple[int, ...]], interpolate: bool = True) -> dict[int, np.ndarray]:
    """Unit direction of each hole, in the base frame of the calibration.

    With `interpolate`, the holes between two mapped ones get the direction interpolated (slerp) between
    theirs, in proportion to their index. Not between opposite directions: the rotation is undefined.
    """
    vectors: dict[int, np.ndarray] = {}
    names: dict[int, str] = {}
    for name, holes in direction_holes.items():
        if name not in DIRECTIONS:
            raise ValueError(f"Unknown direction '{name}', expected one of {list(DIRECTIONS)}")
        for hole in holes:
            if hole in vectors:
                raise ValueError(f"Hole {hole} is mapped to both {names[hole]} and {name}")
            vectors[hole], names[hole] = DIRECTIONS[name], name

    if interpolate:
        mapped = sorted(vectors)
        for lo, hi in zip(mapped, mapped[1:]):
            if hi - lo < 2:
                continue
            a, b = vectors[lo], vectors[hi]
            if np.dot(a, b) < -1.0 + 1e-6:
                print(f"Warning: holes {lo + 1}-{hi - 1} are between opposite directions ({names[lo]}, {names[hi]}): left unmapped")
                continue
            for hole in range(lo + 1, hi):
                vectors[hole] = slerp(a, b, (hole - lo) / (hi - lo))
    return vectors


class JogLogic:
    """Touches -> end-effector velocity. Pure logic, no ROS: see the module docstring for the behavior."""

    def __init__(self, cfg: BodyDemoConfig = CFG):
        self.cfg = cfg
        self.vectors = hole_vectors(cfg.direction_holes, cfg.interpolate_holes)
        self._cos_stop = float(np.cos(np.deg2rad(cfg.stop_angle_deg)))
        self.velocity = np.zeros(3)
        self.stopped = False  # After a stop, until every hole is released

    def touched(self, covered: frozenset[int]) -> list[int]:
        """The covered holes that have a direction."""
        return sorted(h for h in covered if h in self.vectors)

    def directions(self, covered: frozenset[int], rotation: np.ndarray | None = None) -> list[np.ndarray]:
        """Base-frame directions of the covered holes; `rotation` (3x3) rotates them from the calibration."""
        vectors = [self.vectors[h] for h in self.touched(covered)]
        return vectors if rotation is None else [rotation @ v for v in vectors]

    def stop(self) -> None:
        self.velocity = np.zeros(3)
        self.stopped = True

    def update(self, dt: float, covered: frozenset[int], rotation: np.ndarray | None = None) -> bool:
        """Update `velocity` from the holes covered during the last `dt` [s]. Returns True on a stop.

        `rotation` (3x3): rotation of the link since the calibration (`fixed_mapping=False`), None: none.
        """
        directions = self.directions(covered, rotation)
        if self.stopped:
            self.stopped = bool(directions)
            return False
        if not directions:
            return False
        speed = float(np.linalg.norm(self.velocity))
        if speed > 0.0 and any(np.dot(d, self.velocity) < self._cos_stop * speed for d in directions):
            self.stop()
            return True
        push = np.mean(directions, axis=0)
        norm = float(np.linalg.norm(push))
        if norm < 1e-6:  # Opposite holes at once: no direction
            return False
        self.velocity = self.velocity + self.cfg.acceleration * dt * push / norm
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


class SimJogger(Jogger):
    """No arm (--no-robot): the commanded position is taken as reached. Starts in the middle of the bounding box."""

    def __init__(self, cfg: BodyDemoConfig = CFG):
        self.position = (np.array(cfg.box_lower) + np.array(cfg.box_upper)) / 2

    def start(self) -> np.ndarray:
        return self.position.copy()

    def step(self, position: np.ndarray, velocity: np.ndarray) -> None:
        self.position = np.array(position, dtype=float)

    def hard_stop(self) -> np.ndarray:
        return self.position.copy()


class DemoMonitor:
    """What `run` does, for the live plot, and the plot's controls over it. Thread-safe.

    State: "starting", "idle", "moving", "stop" (collision), "box" (bounding box reached) or "nodata", with its
    message. Controls: `paused` keeps the arm stopped (calibrating the mapping), `set_mapping` replaces the hole
    mapping, `stop_event` ends `run`.
    """

    MAX_EVENTS = 100

    def __init__(self):
        self._lock = threading.Lock()
        self.paused = threading.Event()
        self.stop_event = threading.Event()
        self._mapping: dict[str, tuple[int, ...]] | None = None
        self._position: np.ndarray | None = None
        self._velocity = np.zeros(3)
        self._state, self._message = "starting", "Starting..."
        self._events: list[tuple[float, str]] = []  # (CLOCK_MONOTONIC, state) of the stops and box hits

    def publish(self, position: np.ndarray, velocity: np.ndarray) -> None:
        with self._lock:
            self._position, self._velocity = np.array(position, dtype=float), np.array(velocity, dtype=float)

    def set_state(self, state: str, message: str) -> None:
        with self._lock:
            self._state, self._message = state, message
            if state in ("stop", "box"):
                self._events = [*self._events[-self.MAX_EVENTS + 1 :], (time.monotonic(), state)]

    def snapshot(self) -> dict:
        with self._lock:
            return dict(
                position=self._position,
                velocity=self._velocity,
                state=self._state,
                message=self._message,
                events=list(self._events),
            )

    def set_mapping(self, direction_holes: dict[str, tuple[int, ...]]) -> None:
        with self._lock:
            self._mapping = dict(direction_holes)

    def take_mapping(self) -> dict[str, tuple[int, ...]] | None:
        """The mapping set since the last call, or None."""
        with self._lock:
            mapping, self._mapping = self._mapping, None
            return mapping


class LinkOrientation:
    """Orientation of a link, from the joint positions (pyroki FK), and its change since the start."""

    def __init__(self, robot, link_name: str):
        import jax
        import jaxlie

        self._jaxlie = jaxlie
        self.robot = robot
        self._pk_robot = robot.pyroki_robot
        self._fk = jax.jit(self._pk_robot.forward_kinematics)
        self._link_idx = self._pk_robot.links.names.index(link_name)
        self.start = self._so3()  # Also compiles the FK, before the control loop

    def _so3(self):
        q = np.asarray(self.robot.q)[: self._pk_robot.joints.num_actuated_joints]
        if q.shape[0] < self._pk_robot.joints.num_actuated_joints:  # Gripper fingers in the URDF
            q = np.concatenate([q, np.zeros(self._pk_robot.joints.num_actuated_joints - q.shape[0])])
        wxyz_xyz = self._fk(q)[self._link_idx]
        return self._jaxlie.SO3(wxyz_xyz[:4])

    def rotation(self) -> np.ndarray:
        """Current orientation (base <- link, 3x3)."""
        return np.asarray(self._so3().as_matrix())

    def start_rotation(self) -> np.ndarray:
        return np.asarray(self.start.as_matrix())

    def drift_degrees(self) -> float:
        """Rotation since the start [deg]."""
        delta = self.start.inverse() @ self._so3()
        return float(np.rad2deg(np.linalg.norm(np.asarray(delta.log()))))


def run(
    jogger: Jogger,
    touch: TouchSource,
    cfg: BodyDemoConfig = CFG,
    link: LinkOrientation | None = None,
    calibration_rotation: np.ndarray | None = None,
    monitor: DemoMonitor | None = None,
) -> None:
    """Jog until Ctrl+C (or `q` in the terminal input, or `monitor.stop_event`).

    `link`: prints its drift; with `fixed_mapping=False`, needed with its orientation at the calibration,
    `calibration_rotation` (base <- link, 3x3). `monitor`: the live plot's view of the loop, and its controls.
    """
    if not cfg.fixed_mapping and (link is None or calibration_rotation is None):
        raise ValueError("fixed_mapping=False needs the link and its orientation at the calibration")

    def say(state: str, message: str) -> None:
        print(message)
        if monitor is not None:
            monitor.set_state(state, message)

    lower, upper = np.array(cfg.box_lower), np.array(cfg.box_upper)
    logic = JogLogic(cfg)
    position = jogger.start()
    if np.any(position < lower) or np.any(position > upper):
        raise RuntimeError(f"Start position {position} is outside the bounding box [{lower}, {upper}]")
    say("idle", "Ready: touch the waveguide")

    period = 1.0 / cfg.rate_hz
    t_last = t_next = time.monotonic()
    t_drift = t_last
    drift_warned = False
    while not (isinstance(touch, TerminalTouchSource) and touch.quit_requested.is_set()):
        if monitor is not None and monitor.stop_event.is_set():
            break
        t_next += period
        time.sleep(max(0.0, t_next - time.monotonic()))
        now = time.monotonic()
        dt, t_last = min(now - t_last, cfg.max_dt_s), now

        if monitor is not None:
            monitor.publish(position, logic.velocity)
            mapping = monitor.take_mapping()
            if mapping is not None:  # Calibrated from the plot
                logic = JogLogic(replace(cfg, direction_holes=mapping))
                position = jogger.hard_stop()
                say("idle", f"New hole mapping: {mapping}")
            if monitor.paused.is_set():
                if np.any(logic.velocity):
                    logic.stop()
                    position = jogger.hard_stop()
                    say("idle", "Paused: stopped")
                continue

        frame = touch.latest()
        if frame is None or now - frame.t > cfg.touch_timeout_s:
            if np.any(logic.velocity):
                say("nodata", "No touch data: stopping")
                logic.stop()
                position = jogger.hard_stop()
            continue

        rotation = None
        if not cfg.fixed_mapping and logic.touched(frame.covered):  # FK only when needed
            rotation = link.rotation() @ calibration_rotation.T

        moving, was_stopped = bool(np.any(logic.velocity)), logic.stopped
        if logic.update(dt, frame.covered, rotation):
            position = jogger.hard_stop()
            say("stop", f"STOP (touched holes {logic.touched(frame.covered)}) at {np.round(position, 3)}")
            continue
        if was_stopped and not logic.stopped:
            say("idle", "Released: ready")
        if not np.any(logic.velocity):
            continue
        if not moving:
            say("moving", f"Moving: holes {logic.touched(frame.covered)}, v = {np.round(logic.velocity, 3)}")

        new_position = position + logic.velocity * dt
        clipped = np.clip(new_position, lower, upper)
        if not np.allclose(clipped, new_position):
            logic.stop()
            jogger.step(clipped, np.zeros(3))
            position = clipped
            say("box", f"Bounding box reached at {np.round(position, 3)}: stopped")
            continue
        position = new_position
        jogger.step(position, logic.velocity)

        if link is not None and now - t_drift > cfg.drift_print_s:
            t_drift = now
            degrees = link.drift_degrees()
            if cfg.fixed_mapping and degrees > cfg.link_drift_warn_deg and not drift_warned:
                print(f"Warning: {cfg.link_name} rotated {degrees:.1f} deg, the hole mapping may be off")
                drift_warned = True
            print(f"v = {np.round(logic.velocity, 3)} m/s, {cfg.link_name} drift {degrees:.1f} deg")


def run_with_live_plot(
    jogger: Jogger,
    touch: TouchSource,
    cfg: BodyDemoConfig,
    link: LinkOrientation | None,
    calibration_rotation: np.ndarray | None,
    sensor_calibration: Path,
) -> None:
    """`run` in a background thread, with the live plot (`body_live_plot.show`) in this one (Qt needs the main
    thread). Ends when the window is closed (or Ctrl+C) or `run` ends."""
    import body_live_plot  # Qt, only needed here

    monitor, errors = DemoMonitor(), []

    def control() -> None:
        try:
            run(jogger, touch, cfg, link, calibration_rotation, monitor)
        except Exception as e:  # noqa: BLE001, raised again below
            errors.append(e)
        finally:
            monitor.stop_event.set()

    thread = threading.Thread(target=control, name="body_demo_control", daemon=True)
    thread.start()
    try:
        hole_ids, hole_cm = body_live_plot.hole_layout(sensor_calibration, cfg.direction_holes)
        body_live_plot.show(touch, monitor, cfg, hole_ids, hole_cm, link=link)
    finally:
        monitor.stop_event.set()
        thread.join(timeout=2.0)
    if errors:
        raise errors[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", choices=["terminal", "waveguide"], default="waveguide")
    add_teensy_args(parser)
    parser.add_argument(
        "--no-drift",
        action="store_true",
        help=f"Do not monitor the {CFG.link_name} orientation (needed with BodyDemoConfig.fixed_mapping=False)",
    )
    parser.add_argument("--calibrate", action="store_true", help="Calibrate the hole -> direction mapping first")
    parser.add_argument("--live-plot", action="store_true", help="Live plot, with the mapping calibration (⚙ tab)")
    parser.add_argument("--no-robot", action="store_true", help="No arm (no ROS): its motion is simulated")
    args = parser.parse_args()
    if args.calibrate and args.input != "waveguide":
        parser.error("--calibrate needs --input waveguide")
    if args.calibrate and args.live_plot:
        parser.error("--calibrate: with --live-plot, calibrate from the plot (⚙ tab)")
    if (args.no_drift or args.no_robot) and not CFG.fixed_mapping:
        parser.error("--no-drift / --no-robot: the link orientation is needed with BodyDemoConfig.fixed_mapping=False")

    cfg = CFG
    mapping = load_mapping(cfg.calibration_path)
    if mapping is not None:
        cfg = replace(cfg, direction_holes=mapping.direction_holes)
    elif not args.calibrate:
        print(f"No calibration at {cfg.calibration_path} (--calibrate): using the placeholder mapping")

    if args.input == "waveguide":
        try:
            touch = TeensyTouchSource(teensy_config(args))
        except (FileNotFoundError, ValueError) as e:
            parser.exit(1, f"Error: {e}\n")
    else:
        touch = TerminalTouchSource(aliases=cfg.direction_holes)

    robot, link = None, None
    if args.no_robot:
        jogger = SimJogger(cfg)
    else:
        from arm_client.robot import Robot

        robot = Robot(namespace="fr3")
        robot.wait_until_ready()
        jogger = OscJogger(robot)
    try:
        if robot is not None:
            print("Going to home position...")
            robot.home()
            link = None if args.no_drift else LinkOrientation(robot, cfg.link_name)
        touch.start()
        if args.calibrate:
            mapping = calibrate(touch, cfg, link)
            cfg = replace(cfg, direction_holes=mapping.direction_holes)
        print(f"Hole mapping: {cfg.direction_holes}")

        calibration_rotation = None if mapping is None else mapping.link_rotation
        if not cfg.fixed_mapping and calibration_rotation is None:
            # Placeholder or calibrated without the link orientation: assume it was at home, as now
            print(f"No {cfg.link_name} orientation in the calibration: using the current one (at home)")
            calibration_rotation = link.start_rotation()
        if args.live_plot:
            run_with_live_plot(jogger, touch, cfg, link, calibration_rotation, args.sensor_calibration)
        else:
            run(jogger, touch, cfg, link, calibration_rotation)
    except KeyboardInterrupt:
        print()
    except (RuntimeError, TimeoutError) as e:
        print(f"Stopping: {e}")
    finally:
        if robot is not None and jogger.orientation is not None:
            jogger.hard_stop()  # The robot holds this target
        touch.close()
        if robot is not None:
            robot.shutdown()


if __name__ == "__main__":
    main()
