"""Shared harness for osc_controller benchmarks: setup, 1 kHz recording, metrics, saving.

Every run records the controller's own 1 kHz topics (not Robot properties) and saves, next to
the data, a YAML with the full live controller parameters, the CLI arguments, the git commit and
the metrics. Two runs are comparable when their YAMLs agree on everything but what you varied.
"""

from __future__ import annotations

import argparse
import gc
import subprocess
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from arm_client.robot import Pose, Robot, Twist
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from scipy.signal import butter, sosfiltfilt, welch
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from arm_client import CONFIG_DIR

CONTROLLER = "osc_controller"
OSC_CONFIG_DIR = CONFIG_DIR / "controllers" / "osc"
RESULTS_DIR = Path(__file__).parent / "results"
FS = 1000.0  # controller rate (Hz)
CHATTER_HP_HZ = 15.0  # motion content of the benchmarks is below this; torque content above is chatter


# =======================
# MARK: Setup
# =======================


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--config",
        help="Gain preset: a name in configs/controllers/osc/ (e.g. 'default') or a YAML path. Default: keep the live parameters.",
    )
    parser.add_argument(
        "--set",
        metavar="NAME=VALUE",
        action="append",
        default=[],
        help="Parameter override applied after --config, e.g. --set gains.k_pos_x=1000 (repeatable)",
    )
    parser.add_argument("--home", action="store_true", help="Home with the JTC before switching to osc_controller")
    parser.add_argument("--tag", default="", help="Free label stored in the result file name and metadata")
    parser.add_argument("--keep", action="store_true", help="Keep the --config/--set parameters after the run (default: restore)")


def _resolve_config(config: str) -> Path:
    path = Path(config)
    if path.suffix not in (".yaml", ".yml"):
        path = OSC_CONFIG_DIR / f"{config}.yaml"
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def _parse_overrides(items: list[str]) -> list[tuple[str, Any]]:
    overrides = []
    for item in items:
        name, _, value = item.partition("=")
        if not value:
            raise ValueError(f"--set expects NAME=VALUE, got {item!r}")
        parsed = yaml.safe_load(value)
        if isinstance(parsed, int) and not isinstance(parsed, bool):
            parsed = float(parsed)  # gains are doubles; rclpy rejects an int for a double parameter
        overrides.append((name.strip(), parsed))
    return overrides


def snapshot_params(robot: Robot) -> dict[str, Any]:
    """All live osc_controller parameters except the URDF."""
    client = robot.osc_controller_parameters_client
    names = sorted(n for n in client.list_parameters() if n != "robot_description")
    return {n: _plain(v) for n, v in zip(names, client.get_parameters(names))}


def setup(args: argparse.Namespace) -> tuple[Robot, dict[str, Any]]:
    """Connect, optionally home, switch to osc_controller, apply gains. Returns (robot, params)."""
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()
    if args.home:
        robot.home()

    # Re-seed the republished target from the measured pose so the controller we switch to never
    # receives a stale one
    robot.set_target(pose=robot.end_effector_pose.copy())
    if CONTROLLER not in robot.controller_switcher_client.get_active_controllers():
        robot.controller_switcher_client.switch_controller(CONTROLLER)

    client = robot.osc_controller_parameters_client
    client.wait_until_ready()
    before = snapshot_params(robot)
    if args.config:
        client.load_param_config(file_path=_resolve_config(args.config))
    overrides = _parse_overrides(args.set)
    if overrides:
        client.set_parameters(overrides)

    params = snapshot_params(robot)
    robot._bench_restore = [] if args.keep else [(n, before[n]) for n in params if params[n] != before.get(n)]
    print_gains(params)
    time.sleep(0.5)  # let the new gains settle before recording
    return robot, params


def teardown(robot: Robot) -> None:
    """Restore the parameters changed by `setup` (unless --keep) and shut down."""
    restore = getattr(robot, "_bench_restore", [])
    if restore:
        robot.osc_controller_parameters_client.set_parameters(restore)
        print(f"Restored {len(restore)} parameter(s): {', '.join(n for n, _ in restore)}")
    robot.shutdown()


def print_gains(params: dict[str, Any]) -> None:
    def g(prefix):
        return " ".join(f"{params.get(f'gains.{prefix}_{a}', '?'):g}" for a in "xyz")

    print(
        f"[{CONTROLLER}] inertia_decoupling={params.get('control.inertia_decoupling')} "
        f"partial={params.get('control.partial_inertia_decoupling')} | k_pos {g('k_pos')} | d_pos {g('d_pos')} | "
        f"k_rot {g('k_rot')} | d_rot {g('d_rot')} | null k={params.get('nullspace.stiffness')}"
    )


# =======================
# MARK: Recording
# =======================


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _row_ee(m: Odometry):
    p, q, v, w = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear, m.twist.twist.angular
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]


def _row_twist(m: TwistStamped):
    t = m.twist
    return [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]


def _row_wrench(m: WrenchStamped):
    w = m.wrench
    return [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]


def _row_state(m: FrankaRobotState):
    js = m.measured_joint_state
    return [*js.position[:7], *js.velocity[:7], *js.effort[:7], *m.tau_ext_hat_filtered.effort[:7]]


class _Buffer:
    """Growable float array of rows [arrival time, stamp, values...].

    Samples go into a preallocated array: keeping ~7 kHz of Python objects alive instead made the
    garbage collector stall the main thread for up to 95 ms (seen as gaps in streamed targets).
    """

    def __init__(self, width: int, capacity: int = 60_000):
        self._data = np.empty((capacity, 2 + width))
        self.n = 0

    def append(self, t: float, stamp: float, values) -> None:
        if self.n == len(self._data):
            self._data = np.concatenate([self._data, np.empty_like(self._data)])
        row = self._data[self.n]
        row[0] = t
        row[1] = stamp
        row[2:] = values
        self.n += 1

    def view(self) -> np.ndarray:
        return self._data[: self.n]


class Recorder:
    """Records the controller and robot-state topics at 1 kHz on the Robot's node.

    Each stream ``<k>`` yields ``<k>`` (values), ``<k>_t`` (local arrival time, comparable with
    `mark` times) and ``<k>_stamp`` (publisher header stamp, for derivatives and spectra).

    Streams: ee [pos(3) quat_xyzw(4) lin(3) ang(3)], err [lin(3) ang(3)], wrench [F(3) T(3)],
    tau (commanded torque, 7), state [q dq tau_J tau_ext] (4 x 7).
    """

    def __init__(self, robot: Robot):
        streams = {
            "ee": (Odometry, "/fr3/osc/ee_state", _row_ee, 13),
            "err": (TwistStamped, "/fr3/osc/task_error", _row_twist, 6),
            "wrench": (WrenchStamped, "/fr3/osc/task_wrench", _row_wrench, 6),
            "tau": (JointState, "/fr3/osc/joint_torques", lambda m: m.effort[:7], 7),
            "state": (FrankaRobotState, robot.config.franka_robot_state_topic, _row_state, 28),
        }
        self._buffers = {k: _Buffer(width) for k, (_, _, _, width) in streams.items()}
        self._last_arrival: dict[str, float] = {k: 0.0 for k in streams}
        self._recording = False
        self.events: list[tuple[float, str]] = []
        for key, (msg_type, topic, row, _) in streams.items():
            robot.node.create_subscription(msg_type, topic, self._callback(key, row), qos_profile_sensor_data)
        time.sleep(0.5)  # discovery

    def _callback(self, key, row):
        buffer = self._buffers[key]

        def cb(msg):
            now = time.time()
            self._last_arrival[key] = now
            if self._recording:
                buffer.append(now, _stamp(msg), row(msg))

        return cb

    def check_live(self, max_age: float = 0.1) -> None:
        """Raise if any stream has been silent for more than max_age (controller inactive, server down)."""
        now = time.time()
        stale = {k: now - t for k, t in self._last_arrival.items() if now - t > max_age}
        if stale:
            ages = ", ".join(f"{k} {'never' if a > 1e6 else f'{a:.2f} s ago'}" for k, a in stale.items())
            raise RuntimeError(f"Streams not live (is {CONTROLLER} active?): {ages}")

    def start(self) -> None:
        """Start recording; raises if the streams are not live, before any motion is commanded."""
        self.check_live()
        # Move everything allocated so far (imports, JAX, the URDF...) out of the collector's reach
        # so that a full collection during the run stays short
        gc.collect()
        gc.freeze()
        self._recording = True

    def stop(self) -> None:
        self._recording = False

    def mark(self, label: str) -> float:
        t = time.time()
        self.events.append((t, label))
        return t

    def arrays(self) -> dict[str, np.ndarray]:
        out = {}
        for key, buffer in self._buffers.items():
            rows = buffer.view()
            if not len(rows):
                raise RuntimeError(f"No samples recorded on stream '{key}'")
            out[f"{key}_t"] = rows[:, 0].copy()
            out[f"{key}_stamp"] = rows[:, 1].copy()
            out[key] = rows[:, 2:].copy()
        out["event_t"] = np.array([e[0] for e in self.events])
        out["event_label"] = np.array([e[1] for e in self.events])
        return out


# =======================
# MARK: Metrics
# =======================


def stream_health(data: dict[str, np.ndarray]) -> dict[str, Any]:
    """Received rate and largest gap per stream, from header stamps."""
    health = {}
    for key in ("ee", "err", "wrench", "tau", "state"):
        s = data[f"{key}_stamp"]
        span = s[-1] - s[0]
        health[key] = {"rate_hz": len(s) / span if span > 0 else 0.0, "max_gap_ms": float(np.diff(s).max() * 1e3)}
    return health


def chatter(x: np.ndarray, fs: float = FS, f_hp: float = CHATTER_HP_HZ) -> dict[str, np.ndarray]:
    """High-frequency content of each column of x (N x d): RMS above f_hp and the dominant frequency there."""
    x = np.asarray(x, dtype=float)
    if len(x) < 64:
        return {"rms_hf": np.full(x.shape[1], np.nan), "peak_hz": np.full(x.shape[1], np.nan)}
    sos = butter(4, f_hp, btype="highpass", fs=fs, output="sos")
    hf = sosfiltfilt(sos, x, axis=0)
    f, p = welch(x - x.mean(0), fs=fs, nperseg=min(1024, len(x)), axis=0)
    band = f >= f_hp
    peak = f[band][np.argmax(p[band], axis=0)]
    return {"rms_hf": np.sqrt((hf**2).mean(0)), "peak_hz": peak}


def chatter_metrics(data: dict[str, np.ndarray], t0: float, t1: float) -> dict[str, Any]:
    """Chatter of commanded torque, measured joint velocity and measured torque within [t0, t1)."""
    m_tau = (data["tau_t"] >= t0) & (data["tau_t"] < t1)
    m_st = (data["state_t"] >= t0) & (data["state_t"] < t1)
    tau_cmd = chatter(data["tau"][m_tau])
    dq = chatter(data["state"][m_st, 7:14])
    tau_j = chatter(data["state"][m_st, 14:21])
    return {
        "tau_cmd_rms_hf_Nm": tau_cmd["rms_hf"],
        "tau_cmd_peak_hz": tau_cmd["peak_hz"],
        "dq_rms_hf_mrad_s": dq["rms_hf"] * 1e3,
        "dq_peak_hz": dq["peak_hz"],
        "tau_J_rms_hf_Nm": tau_j["rms_hf"],
        "tau_cmd_rms_hf_max_Nm": float(np.nanmax(tau_cmd["rms_hf"])),
        "dq_rms_hf_max_mrad_s": float(np.nanmax(dq["rms_hf"]) * 1e3),
    }


def orientation_error_mrad(quats_xyzw: np.ndarray, ref: Rotation) -> np.ndarray:
    return (Rotation.from_quat(quats_xyzw) * ref.inv()).magnitude() * 1e3


# =======================
# MARK: Saving
# =======================


def _plain(v: Any) -> Any:
    """Convert numpy containers and scalars to YAML-safe Python types."""
    if isinstance(v, dict):
        return {str(k): _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    if isinstance(v, np.ndarray):
        return [_plain(x) for x in v.tolist()]
    if isinstance(v, np.generic):
        return v.item()
    if isinstance(v, float):
        return round(v, 6)
    return v


def _git_commit() -> str:
    try:
        sha = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=Path(__file__).parent, text=True)
        dirty = subprocess.call(["git", "diff", "--quiet"], cwd=Path(__file__).parent) != 0
        return sha.strip() + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def save(name: str, args: argparse.Namespace, params: dict, data: dict, metrics: dict) -> Path:
    """Save data (.npz) and metadata + metrics (.yaml) under results/. Returns the YAML path."""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    stem = "_".join(s for s in (name, args.tag, time.strftime("%Y%m%d_%H%M%S")) if s)
    np.savez_compressed(RESULTS_DIR / f"{stem}.npz", **data)
    meta = {
        "benchmark": name,
        "tag": args.tag,
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "git": _git_commit(),
        "args": _plain(vars(args)),
        "controller": CONTROLLER,
        "params": params,
        "stream_health": _plain(stream_health(data)),
        "metrics": _plain(metrics),
    }
    path = RESULTS_DIR / f"{stem}.yaml"
    with open(path, "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    print(f"Saved -> {path} (+ .npz)")
    try:
        from plots import plot_run

        for fig_path in plot_run(path):
            print(f"Plot  -> {fig_path}")
    except Exception as e:  # a plotting bug must never lose a run: redraw later with plots.py
        print(f"Plotting failed ({type(e).__name__}: {e}); redraw with: python plots.py {path}")
    return path


def fmt(v: np.ndarray, scale: float = 1.0, digits: int = 2) -> str:
    return "[" + " ".join(f"{x * scale:.{digits}f}" for x in np.asarray(v, dtype=float)) + "]"


__all__ = [
    "Pose",
    "Twist",
    "Recorder",
    "add_common_args",
    "chatter_metrics",
    "fmt",
    "orientation_error_mrad",
    "save",
    "setup",
    "teardown",
]
