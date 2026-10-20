"""Shared runner and metrics for the interactive pre-tests (T2-T5).

A test is a preset of ``fr3_teleop`` flags plus the metrics that matter for it. [run_test][] starts
the run (``fr3_teleop`` on the robot, ``sim_teleop.py`` with ``--sim``), you play with it on the
Inverse3 until Ctrl-C, then it loads the saved run, prints the metrics and saves a plot next to it.
Flags after the test's own are passed through and override the preset (the last value wins).

Runs go to ``~/data/franka/rim-delays/<robot|sim>/pretests/<test>_<time>/``.
"""

from __future__ import annotations

import argparse
import datetime
import signal
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
REPO = HERE.parents[2]

import matplotlib.pyplot as plt  # noqa: E402
from runs import COLORS, INK, INK_2, INK_3, find, load_run, save, surface_of  # noqa: E402

OUT = Path("~/data/franka/rim-delays").expanduser()
CONTACT_BAND = 0.002  # robot within this of the table counts as in contact [m]
FREE_MARGIN = 0.005  # leader this far above the table counts as free motion [m]


# ------------------------------------------------------------------ running


def to_argv(flags: dict) -> list[str]:
    """``{"interface_axis": "xyz", "force": True}`` → ``["--interface-axis", "xyz", "--force"]``."""
    argv = []
    for key, value in flags.items():
        flag = "--" + key.replace("_", "-")
        if value is None or value is False:
            continue
        if value is True:
            argv.append(flag)
        elif isinstance(value, (list, tuple)):
            argv += [flag, *map(str, value)]
        else:
            argv += [flag, str(value)]
    return argv


def parse(description: str, argv: list[str] | None) -> tuple[argparse.Namespace, list[str]]:
    """The pre-test's own flags (``--sim``, ``--analyse``) and the rest, passed to the run."""
    p = argparse.ArgumentParser(description=description, epilog="Other flags go to fr3_teleop / sim_teleop.py.")
    p.add_argument("--sim", action="store_true", help="simulated FR3 (sim_teleop.py) instead of the robot")
    p.add_argument("--analyse", type=Path, default=None, help="analyse this saved run instead of starting one")
    return p.parse_known_args(argv)


def run_test(name: str, preset: dict, args: argparse.Namespace, rest: list[str]) -> Path:
    """Start the run with ``preset`` (then ``rest``), wait for it to end (Ctrl-C), return its directory."""
    if args.analyse is not None:
        return args.analyse
    backend = "sim" if args.sim else "robot"
    out_dir = OUT / backend / "pretests"
    run_name = f"{name}_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    exe = [sys.executable, str(HERE.parent / "sim_teleop.py")] if args.sim else ["fr3_teleop"]
    cmd = exe + ["--conf", str(REPO / "python/fr3_haptic/configs/teleop.yaml")] + to_argv(preset) + rest
    cmd += ["--save", "--output-dir", str(out_dir), "--run-name", run_name]
    print(f"[{name}] {' '.join(cmd)}\n[{name}] Ctrl-C ends the run; the metrics follow.")
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)  # the child handles Ctrl-C and saves
    try:
        subprocess.run(cmd, cwd=REPO)
    finally:
        signal.signal(signal.SIGINT, previous)
    return out_dir / run_name


# ------------------------------------------------------------------ the run, as arrays


class Run:
    """The streams a pre-test needs, on the haptic clock, z = the contact axis."""

    def __init__(self, run_dir: Path) -> None:
        run = load_run(run_dir)
        if run is None or run["haptic"] is None:
            raise SystemExit(f"no complete run in {run_dir}")
        self.dir = Path(run_dir)
        self.meta = run["meta"]
        h = run["haptic"]
        self.ca = int(self.meta.get("contact_axis", 0))
        self.dim = sum(1 for c in h.columns if c.startswith("x_l_"))
        self.surface = surface_of(self.meta)
        self.K = float(find(self.meta, "kv", 500.0))
        self.force_gain = float(find(self.meta, "force_gain", 0.1))
        self.method = find(self.meta, "method", "?")
        self.t = h.ts.to_numpy()
        self.xl = h[[f"x_l_{k}" for k in range(self.dim)]].to_numpy()
        self.f = h[[f"f_{k}" for k in range(self.dim)]].to_numpy()
        self.energy = h.energy_j.to_numpy() if "energy_j" in h else None
        self.robot = run["robot"]
        c = run["command"]
        self.cmd_t = self.cmd_x = None
        if c is not None and len(c):
            cols = [f"x_{k}" for k in range(self.dim)] if f"x_0" in c else ["x"]
            self.cmd_t, self.cmd_x = c.ts.to_numpy(), c[cols].to_numpy()
        delays = self.meta.get("delays") or {}
        self.T_c = 1e-3 * float((delays.get("command") or {}).get("base_ms", 0.0) or 0.0)
        self.T_f = 1e-3 * float((delays.get("feedback") or {}).get("base_ms", 0.0) or 0.0)

    @property
    def zl(self) -> np.ndarray:
        return self.xl[:, self.ca]

    @property
    def fz(self) -> np.ndarray:
        return self.f[:, self.ca]

    def robot_at(self, t: np.ndarray, col: str) -> np.ndarray:
        r = self.robot
        return np.interp(t, r.ts, r[col]) if r is not None else np.full(len(t), np.nan)


# ------------------------------------------------------------------ metrics


def felt_stiffness(run: Run) -> tuple[float, int]:
    """Slope of the rendered force along z on the leader's penetration below the table, while it is
    below [N/m, world], and the number of samples used."""
    if run.surface is None:
        return np.nan, 0
    pen = run.surface - run.zl
    on = pen > 0.001
    if on.sum() < 50:
        return np.nan, int(on.sum())
    A = np.stack([pen[on], np.ones(on.sum())], 1)
    (k, _), *_ = np.linalg.lstsq(A, run.fz[on], rcond=None)
    return float(k), int(on.sum())


def force_matching(run: Run) -> dict:
    """Felt force along z vs. the table force on the robot, the robot shifted by the command delay
    (what the operator feels now acts on the robot ``T_c`` later)."""
    if run.robot is None or run.surface is None:
        return {}
    t = run.t
    table = run.robot_at(t + run.T_c, "f_table")
    z_r = run.robot_at(t + run.T_c, "z")
    on = (z_r < run.surface + CONTACT_BAND) & (table > 0.5)
    if on.sum() < 50:
        return {"contact_samples": int(on.sum())}
    felt = run.fz[on]
    ratio = float(np.sum(felt * table[on]) / np.sum(table[on] ** 2))
    return {"contact_samples": int(on.sum()), "felt_over_table": ratio,
            "felt_minus_table_rms": float(np.sqrt(np.mean((felt - table[on]) ** 2)))}


def free_drag(run: Run) -> float:
    """RMS of the rendered force while the leader is well above the table [N] (0 is perfect)."""
    if run.surface is None:
        return float(np.sqrt(np.mean(np.sum(run.f ** 2, 1))))
    free = run.zl > run.surface + FREE_MARGIN
    return float(np.sqrt(np.mean(np.sum(run.f[free] ** 2, 1)))) if free.any() else np.nan


def contact_oscillation(run: Run, min_hz: float = 5.0) -> tuple[float, float]:
    """Dominant frequency [Hz] and RMS [N] of the rendered z force above ``min_hz`` while the leader
    is below the table — a buzz or limit cycle at the wall. ``(nan, 0)`` without contact."""
    if run.surface is None:
        return np.nan, 0.0
    on = run.zl < run.surface
    if on.sum() < 500:
        return np.nan, 0.0
    dt = float(np.median(np.diff(run.t)))
    best, rms = np.nan, 0.0
    # contiguous contact segments, each detrended
    idx = np.flatnonzero(on)
    splits = np.flatnonzero(np.diff(idx) > 1) + 1
    power = None
    for seg in np.split(idx, splits):
        if len(seg) < 256:
            continue
        y = run.fz[seg] - np.convolve(run.fz[seg], np.ones(51) / 51, mode="same")  # high-pass, ~20 Hz
        y = y[25:-25]
        spec = np.abs(np.fft.rfft(y * np.hanning(len(y)))) ** 2
        freqs = np.fft.rfftfreq(len(y), dt)
        band = freqs >= min_hz
        if band.any():
            k = np.argmax(spec[band])
            if power is None or spec[band][k] > power:
                power, best = spec[band][k], float(freqs[band][k])
        rms = max(rms, float(np.sqrt(np.mean(y ** 2))))
    return best, rms


def prediction_error(run: Run) -> dict:
    """The commanded target (proxy) vs. where the robot is ``T_c`` later, per axis [mm], split into
    free motion and contact."""
    if run.cmd_t is None or run.robot is None:
        return {}
    t = run.cmd_t
    out = {}
    names = "xyz"[: run.cmd_x.shape[1]] if run.cmd_x.shape[1] == 3 else "z"
    for k, a in enumerate(names):
        e = 1e3 * (run.robot_at(t + run.T_c, a) - run.cmd_x[:, k])
        out[f"{a}_rms_mm"] = float(np.sqrt(np.nanmean(e ** 2)))
    if run.surface is not None and "z" in names:
        k = names.index("z")
        e = 1e3 * (run.robot_at(t + run.T_c, "z") - run.cmd_x[:, k])
        contact = run.cmd_x[:, k] < run.surface + CONTACT_BAND
        if contact.any():
            out["z_contact_rms_mm"] = float(np.sqrt(np.nanmean(e[contact] ** 2)))
        if (~contact).any():
            out["z_free_rms_mm"] = float(np.sqrt(np.nanmean(e[~contact] ** 2)))
    return out


def common_metrics(run: Run) -> dict:
    k, n = felt_stiffness(run)
    hz, rms = contact_oscillation(run)
    m = {
        "method": run.method,
        "K (coupling) [N/m]": run.K,
        "felt stiffness, world [N/m]": k,
        "felt stiffness, at the handle [N/m]": k * run.force_gain,
        "free-motion force RMS [N]": free_drag(run),
        "contact oscillation [Hz]": hz,
        "contact oscillation RMS [N]": rms,
        "passivity energy min [mJ]": 1e3 * float(np.nanmin(run.energy)) if run.energy is not None else np.nan,
        "stop reason": run.meta.get("stop_reason", ""),
    }
    for key, v in force_matching(run).items():
        m[f"force matching: {key}"] = v
    return m


def print_metrics(name: str, metrics: dict) -> str:
    width = max(len(k) for k in metrics)
    lines = [f"[{name}] results"]
    for k, v in metrics.items():
        if isinstance(v, float):
            v = f"{v:.3g}"
        lines.append(f"  {k:<{width}}  {v}")
    text = "\n".join(lines)
    print(text)
    return text


# ------------------------------------------------------------------ plot


def plot_run(run: Run, name: str, title: str) -> list[Path]:
    """Height and force along z over time: operator, commanded target, robot."""
    color = COLORS.get(run.method, "#2a78d6")
    fig, axs = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    a = axs[0]
    if run.surface is not None:
        a.axhline(run.surface * 1e3, color=INK_3, linewidth=1, linestyle="--")
    a.plot(run.t, run.zl * 1e3, color=color, label="operator (leader)")
    if run.cmd_t is not None:
        k = run.ca if run.cmd_x.shape[1] > 1 else 0
        a.plot(run.cmd_t, run.cmd_x[:, k] * 1e3, color=INK_3, linewidth=1.2, label="target sent")
    if run.robot is not None:
        a.plot(run.robot.ts, run.robot.z * 1e3, color=INK_2, linewidth=1.4, label="robot")
    a.set_ylabel("height z [mm]")
    a.legend(loc="upper right", fontsize=8, ncol=3)
    a = axs[1]
    a.plot(run.t, run.fz, color=color, label="felt (world)")
    if run.robot is not None:
        a.plot(run.robot.ts, run.robot.f_table, color=INK_2, linewidth=1.4, label="table force on the robot")
    a.set_ylabel("force along z [N]")
    a.set_xlabel("time [s]")
    a.legend(loc="upper right", fontsize=8, ncol=2)
    fig.suptitle(title, x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout()
    return save(fig, run.dir, name)


def finish(name: str, title: str, run_dir: Path, extra=None) -> None:
    """Load the run, print the common metrics (plus ``extra(run)``), save the plot and the report."""
    run = Run(run_dir)
    metrics = common_metrics(run)
    if extra is not None:
        metrics.update(extra(run))
    text = print_metrics(name, metrics)
    (run.dir / f"{name}_report.txt").write_text(text + "\n")
    paths = plot_run(run, name, title)
    print(f"[{name}] -> {run.dir} ({paths[0].name})")
