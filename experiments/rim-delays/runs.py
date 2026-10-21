"""Loading a saved run (sim or robot) into one shape, and the plot style, for analyze.py and pretests/.

A run is ``<run_dir>/metadata.json`` + ``<run_dir>/samples.mcap`` (``fr3_teleop --save`` or
``sim_teleop.py --save``). [load_run][] returns its streams with the robot side normalized to a
``robot`` table, whichever backend wrote it:

| column | meaning |
|---|---|
| ``ts`` | run time [s] (the clock of the ``haptic`` stream) |
| ``x, y, z`` | tool point position, base frame [m] |
| ``fx, fy, fz`` | OSC task force on the robot [N] (sim: the PD + feedforward; robot: ``task_wrench``) |
| ``f_table`` | table force on the robot, up [N] (sim: the contact model; robot: ``-fz`` clipped at 0, quasi-static) |
| ``qx, qy, qz, qw`` | tool orientation quaternion, base frame (robot runs that logged it; absent in sim) |

[orientation_error][] and [tip_error][] turn the orientation into what a demo shows: how far the
tool has turned from the held orientation, and how far that moves the tip of a tool.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from experiment_logger import load_streams  # noqa: E402

METHODS = ("proxy-rim", "proxy-fixed-mass", "proxy-god-object", "zoh", "tdpa-zoh")
LABELS = {"proxy-rim": "RIM (ours)", "zoh": "ZOH", "tdpa-zoh": "TDPA", "proxy-fixed-mass": "Fixed mass",
          "proxy-god-object": "God-object"}
COLORS = {"proxy-rim": "#2a78d6", "zoh": "#eb6834", "tdpa-zoh": "#1baf7a", "proxy-fixed-mass": "#eda100",
          "proxy-god-object": "#e87ba4"}  # categorical slots 1-5
INK, INK_2, INK_3, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2, "ytick.color": INK_2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11, "axes.titlecolor": INK,
    "legend.frameon": False, "lines.linewidth": 2.0,
})


def load_run(run_dir: Path, streams: tuple[str, ...] = ("haptic", "plant", "sim", "command")) -> dict | None:
    """``{"meta", "haptic", "robot", "command", "dir"}`` for a run, or ``None`` if it is incomplete.
    ``robot`` (see the module docstring) and ``command`` may be ``None``."""
    run_dir = Path(run_dir)
    meta_path = run_dir / "metadata.json"
    if not meta_path.exists() or not (run_dir / "samples.mcap").exists():
        return None
    meta = json.loads(meta_path.read_text())
    meta = meta.get("metadata", meta)
    s = load_streams(run_dir, list(streams))
    robot = None
    if "sim" in s:
        d = s["sim"]
        robot = pd.DataFrame({"ts": d.ts, "x": d.x_x, "y": d.x_y, "z": d.x_z, "vx": d.v_x, "vy": d.v_y, "vz": d.v_z,
                              "fx": d.f_task_x, "fy": d.f_task_y, "fz": d.f_task_z, "f_table": d.f_contact_z})
    elif "plant" in s and "ee_position_0" in s["plant"]:
        d = s["plant"]
        robot = pd.DataFrame({"ts": d.ts, "x": d.ee_position_0, "y": d.ee_position_1, "z": d.ee_position_2,
                              "fx": d.task_force_0, "fy": d.task_force_1, "fz": d.task_force_2,
                              "f_table": (-d.task_force_2).clip(lower=0.0)})
        if "ee_orientation_0" in d:
            for k, c in enumerate(("qx", "qy", "qz", "qw")):
                robot[c] = d[f"ee_orientation_{k}"]
        dt = np.gradient(robot.ts.to_numpy())
        for a in "xyz":
            robot[f"v{a}"] = np.gradient(robot[a].to_numpy()) / np.where(dt > 0, dt, np.nan)
    return {"meta": meta, "haptic": s.get("haptic"), "robot": robot, "command": s.get("command"), "dir": run_dir}


def find(meta: dict, key: str, default=None):
    """``key`` from the metadata, wherever it sits (top level or under ``args``)."""
    if key in meta and meta[key] is not None:
        return meta[key]
    return meta.get("args", {}).get(key, default)


def surface_of(meta: dict) -> float | None:
    """Table height of a run [m]: the script's, else ``--contact-surface``, else the sim table."""
    script = meta.get("script") or {}
    for value in (script.get("surface_z"), find(meta, "contact_surface"), meta.get("table_z")):
        if value is not None:
            return float(value)
    return None


def orientation_error(q_xyzw: np.ndarray, q_hold_xyzw) -> np.ndarray:
    """``(n, 3)`` rotation vector from the held orientation to the measured one, base-frame axes
    (roll, pitch, yaw about x, y, z) [rad]."""
    from scipy.spatial.transform import Rotation

    return (Rotation.from_quat(q_xyzw) * Rotation.from_quat(q_hold_xyzw).inv()).as_rotvec()


def tip_error(q_xyzw: np.ndarray, q_hold_xyzw, length: float) -> np.ndarray:
    """``(n,)`` displacement [m] of a tool tip ``length`` beyond the end-effector point (along the
    end-effector z axis) caused by the orientation error alone."""
    from scipy.spatial.transform import Rotation

    tip = np.array([0.0, 0.0, length])
    return np.linalg.norm(Rotation.from_quat(q_xyzw).apply(tip) - Rotation.from_quat(q_hold_xyzw).apply(tip), axis=1)


def orientation_summary(q_xyzw: np.ndarray, q_hold_xyzw, length: float) -> dict:
    """RMS and max orientation error per axis [mrad], and the tip error for a tool of ``length`` [mm]."""
    e = 1e3 * orientation_error(q_xyzw, q_hold_xyzw)
    tip = 1e3 * tip_error(q_xyzw, q_hold_xyzw, length)
    out = {}
    for k, name in enumerate(("roll", "pitch", "yaw")):
        out[f"{name} error RMS / max [mrad]"] = f"{np.sqrt(np.mean(e[:, k] ** 2)):.1f} / {np.max(np.abs(e[:, k])):.1f}"
    out[f"tip error ({1e3 * length:g} mm tool) RMS / max [mm]"] = f"{np.sqrt(np.mean(tip ** 2)):.2f} / {tip.max():.2f}"
    return out


def save(fig, out: Path, name: str) -> list[Path]:
    """Save ``fig`` as PNG and PDF in ``out``; returns the paths."""
    out.mkdir(parents=True, exist_ok=True)
    paths = [out / f"{name}.{ext}" for ext in ("png", "pdf")]
    for p in paths:
        fig.savefig(p, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return paths
