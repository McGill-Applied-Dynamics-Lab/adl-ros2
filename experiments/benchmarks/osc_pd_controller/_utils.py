"""Shared utilities for OSC PD controller benchmarks."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np
from arm_client.robot import Pose, Robot
from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR

RESULTS_DIR = Path(__file__).parent / "results"
OSC_DEFAULT_CONFIG = CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
OSC_RIM_CONFIG = CONFIG_DIR / "controllers" / "osc_pd" / "rim_controller.yaml"


def setup_robot(config_path: Path = OSC_DEFAULT_CONFIG) -> tuple[Robot, Pose]:
    """Connect, home, switch to osc_pd_controller, load config. Returns (robot, home_pose)."""
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()
    robot.home()
    robot.controller_switcher_client.switch_controller("osc_pd_controller")
    robot.osc_pd_controller_parameters_client.load_param_config(file_path=config_path)
    home_pose = robot.end_effector_pose.copy()
    print(f"Home pose: {home_pose}")
    return robot, home_pose


def orientation_error_rad(q_ref: Rotation, q_cur: Rotation) -> float:
    """Geodesic angle between two orientations (radians)."""
    return (q_ref.inv() * q_cur).magnitude()


def sample_state(robot: Robot, ref_orientation: Rotation) -> dict:
    """One-shot snapshot of the robot state."""
    pose = robot.end_effector_pose
    try:
        force = robot.end_effector_external_wrench["force"].copy()
    except RuntimeError:
        force = np.zeros(3)
    return {
        "ts": time.time(),
        "position": pose.position.copy(),
        "orientation_quat": pose.orientation.as_quat(),  # xyzw
        "ori_err_rad": orientation_error_rad(ref_orientation, pose.orientation),
        "q": robot.q.copy(),
        "force": force,
    }


def collect(
    robot: Robot,
    ref_orientation: Rotation,
    duration: float,
    rate_hz: float = 200.0,
) -> dict[str, np.ndarray]:
    """Fixed-duration data collection loop at rate_hz. Returns stacked arrays."""
    dt = 1.0 / rate_hz
    samples: list[dict] = []
    next_tick = time.perf_counter()
    t_end = time.perf_counter() + duration
    while time.perf_counter() < t_end:
        samples.append(sample_state(robot, ref_orientation))
        next_tick += dt
        time.sleep(max(0.0, next_tick - time.perf_counter()))
    return {k: np.array([s[k] for s in samples]) for k in samples[0]}


def save(data: dict[str, Any], name: str) -> Path:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = RESULTS_DIR / f"{name}_{ts}.npz"
    saveable = {k: np.array(v) if not isinstance(v, np.ndarray) else v for k, v in data.items()}
    np.savez(path, **saveable)
    print(f"Saved → {path}")
    return path


def print_summary(label: str, data: dict[str, np.ndarray], home_pos: np.ndarray) -> None:
    pos = data["position"]
    ori = np.degrees(data["ori_err_rad"])
    tracking_err = np.linalg.norm(pos - home_pos, axis=1) if "target" not in data else None
    print(f"\n[{label}]")
    print(f"  Orientation error — mean: {ori.mean():.2f}°  max: {ori.max():.2f}°")
    if tracking_err is not None:
        print(
            f"  Position error (norm) — mean: {tracking_err.mean() * 1e3:.1f}mm  max: {tracking_err.max() * 1e3:.1f}mm"
        )
