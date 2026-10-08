"""Figures for osc_controller benchmark runs.

Every benchmark calls `plot_run` after saving, writing PNGs into the run directory. To redraw
existing runs, or to overlay several runs (the way to compare gains):

    python plots.py results/step_response/*_k400                     # per-run figures
    python plots.py --compare results/step_response/2026*            # -> results/step_response/compare/<timestamp>/

Overlays need runs of the same benchmark. Each run is labelled with its tag and key gains.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from _layout import DATA_FILE, META_FILE, new_compare_dir, resolve_run  # noqa: E402
from scipy.signal import butter, sosfiltfilt, welch  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

FS = 1000.0
CHATTER_HP_HZ = 15.0
JOINT_COLORS = plt.cm.viridis(np.linspace(0.0, 0.9, 7))
AXES = {"x": 0, "y": 1, "z": 2}


# =======================
# MARK: Loading and helpers
# =======================


def load(run: str | Path) -> tuple[dict, dict]:
    """(meta, data) of a run, given its directory or its meta.yaml."""
    run = resolve_run(run)
    meta = yaml.safe_load((run / META_FILE).read_text())
    meta["_dir"] = run
    with np.load(run / DATA_FILE) as npz:
        data = {k: npz[k] for k in npz.files}
    return meta, data


def gains_label(params: dict) -> str:
    def axes3(prefix):
        vals = [params.get(f"gains.{prefix}_{a}") for a in "xyz"]
        return f"{vals[0]:g}" if len(set(vals)) == 1 else "/".join(f"{v:g}" for v in vals)

    def damping(block):
        # Negative d_* = from the damping ratio (servers without the ratio parameter: critical, 1)
        if any(params.get(f"gains.d_{block}_{a}", -1.0) < 0 for a in "xyz"):
            return f"ζ{params.get(f'gains.damping_ratio_{block}', 1.0):g}"
        return axes3(f"d_{block}")

    dec = "dec" if params.get("control.inertia_decoupling") else "nodec"
    if params.get("control.partial_inertia_decoupling"):
        dec += "-partial"
    return f"kp {axes3('k_pos')} d {damping('pos')} | kr {axes3('k_rot')} d {damping('rot')} | {dec}"


def run_label(meta: dict) -> str:
    tag = meta.get("tag") or meta["_dir"].name
    return f"{tag}: {gains_label(meta['params'])}"


def highpass(x: np.ndarray, f_hp: float = CHATTER_HP_HZ) -> np.ndarray:
    sos = butter(4, f_hp, btype="highpass", fs=FS, output="sos")
    return sosfiltfilt(sos, x, axis=0)


def moving_rms(x: np.ndarray, window: int = 50) -> np.ndarray:
    kernel = np.ones(window) / window
    return np.sqrt(np.convolve(x**2, kernel, mode="same"))


def psd(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return welch(x - x.mean(0), fs=FS, nperseg=min(1024, len(x)), axis=0)


def rot_err_mrad(quats: np.ndarray, ref: Rotation) -> np.ndarray:
    return (Rotation.from_quat(quats) * ref.inv()).magnitude() * 1e3


def window(data: dict, key: str, t0: float, t1: float) -> np.ndarray:
    t = data[f"{key}_t"]
    return (t >= t0) & (t < t1)


def step_segments(meta: dict, data: dict) -> list[dict]:
    """Step windows with their commanded start/end positions (stored, or rebuilt from the event marks)."""
    if "step_t0" in data:
        return [
            {"label": str(lbl), "t0": t0, "t1": t1, "p_from": pf, "p_to": pt}
            for lbl, t0, t1, pf, pt in zip(data["step_label"], data["step_t0"], data["step_t1"], data["step_from"], data["step_to"])
        ]
    # Older runs: steps are '<axis>+' then '<axis>-' around the start position
    amplitude = meta["metrics"]["amplitude_m"]
    start = data["ee"][0, :3]
    t_end = data["ee_t"][-1]
    segs = []
    labels, times = list(data["event_label"]), list(data["event_t"])
    for i, (lbl, t0) in enumerate(zip(labels, times)):
        lbl = str(lbl)
        if len(lbl) != 2 or lbl[0] not in AXES:
            continue
        out = start.copy()
        out[AXES[lbl[0]]] += amplitude
        p_from, p_to = (start, out) if lbl[1] == "+" else (out, start)
        t1 = times[i + 1] if i + 1 < len(times) else t_end
        segs.append({"label": lbl, "t0": t0, "t1": t1, "p_from": p_from, "p_to": p_to})
    return segs


def step_traces(data: dict, seg: dict, ref_rot: Rotation) -> dict:
    """Time series of one step, relative to the step command."""
    m = window(data, "ee", seg["t0"], seg["t1"])
    t = data["ee_t"][m] - seg["t0"]
    pos = data["ee"][m, :3]
    length = np.linalg.norm(seg["p_to"] - seg["p_from"])
    axis = (seg["p_to"] - seg["p_from"]) / length
    s = (pos - seg["p_from"]) @ axis
    off = pos - seg["p_from"] - np.outer(s, axis)
    mt = window(data, "tau", seg["t0"], seg["t1"])
    ms = window(data, "state", seg["t0"], seg["t1"])
    tau = data["tau"][mt]
    return {
        "t": t,
        "s_mm": s * 1e3,
        "length_mm": length * 1e3,
        "off_mm": np.linalg.norm(off, axis=1) * 1e3,
        "rot_mrad": rot_err_mrad(data["ee"][m, 3:7], ref_rot),
        "speed_mm_s": np.linalg.norm(data["ee"][m, 7:10], axis=1) * 1e3,
        "tau_t": data["tau_t"][mt] - seg["t0"],
        "tau": tau,
        "tau_hf_rms": moving_rms(np.linalg.norm(highpass(tau), axis=1)) if len(tau) > 64 else np.zeros(len(tau)),
        "state_t": data["state_t"][ms] - seg["t0"],
        "dq": data["state"][ms, 7:14],
    }


def _save(fig, meta: dict, kind: str) -> Path:
    path = meta["_dir"] / f"{kind}.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def _joint_legend(ax) -> None:
    ax.legend([f"j{i + 1}" for i in range(7)], fontsize=7, ncol=7, loc="upper right", handlelength=1.0)


# =======================
# MARK: Per-run figures
# =======================


def plot_spectra(meta: dict, data: dict, t0: float | None = None, t1: float | None = None) -> Path:
    """PSD of commanded torque, measured joint velocity and measured torque, per joint."""
    t0 = data["tau_t"][0] if t0 is None else t0
    t1 = data["tau_t"][-1] + 1e-3 if t1 is None else t1
    tau = data["tau"][window(data, "tau", t0, t1)]
    st = data["state"][window(data, "state", t0, t1)]
    fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, x, name, unit in (
        (axs[0], tau, "commanded torque", "Nm²/Hz"),
        (axs[1], st[:, 7:14], "measured joint velocity", "(rad/s)²/Hz"),
        (axs[2], st[:, 14:21], "measured joint torque", "Nm²/Hz"),
    ):
        f, p = psd(x)
        for j in range(7):
            ax.semilogy(f, p[:, j], color=JOINT_COLORS[j], lw=0.9)
        ax.axvline(CHATTER_HP_HZ, color="k", ls=":", lw=0.8)
        ax.set_ylabel(unit)
        ax.set_title(f"PSD {name}", fontsize=10, loc="left")
        ax.grid(True, which="both", alpha=0.3)
    _joint_legend(axs[0])
    axs[-1].set_xlabel("frequency [Hz]")
    axs[-1].set_xlim(0, FS / 2)
    fig.suptitle(f"{meta['benchmark']} — {run_label(meta)}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return _save(fig, meta, "spectra")


def plot_step_run(meta: dict, data: dict) -> list[Path]:
    segs = step_segments(meta, data)
    ref_rot = Rotation.from_quat(data["ee"][0, 3:7])
    fig, axs = plt.subplots(len(segs), 4, figsize=(17, 2.6 * len(segs) + 1), squeeze=False)
    for r, seg in enumerate(segs):
        tr = step_traces(data, seg, ref_rot)
        a = axs[r]
        a[0].plot(tr["t"], tr["s_mm"], color="C0", label="measured")
        a[0].axhline(tr["length_mm"], color="k", ls="--", lw=0.8, label="target")
        a[0].set_ylabel(f"{seg['label']}\nalong step [mm]")
        a[1].plot(tr["t"], tr["off_mm"], color="C1", label="off-axis [mm]")
        a[1].plot(tr["t"], tr["rot_mrad"] / 10.0, color="C3", label="rot err / 10 [mrad]")
        for j in range(7):
            a[2].plot(tr["tau_t"], tr["tau"][:, j], color=JOINT_COLORS[j], lw=0.8)
            a[3].plot(tr["state_t"], tr["dq"][:, j], color=JOINT_COLORS[j], lw=0.8)
        a[2].set_ylabel("τ_cmd [Nm]")
        a[3].set_ylabel("dq [rad/s]")
        for ax in a:
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, tr["t"][-1] if len(tr["t"]) else 1)
        if r == 0:
            a[0].legend(fontsize=7)
            a[1].legend(fontsize=7)
            _joint_legend(a[2])
            a[0].set_title("displacement along the step")
            a[1].set_title("off-axis deviation / orientation error")
            a[2].set_title("commanded torque (excl. gravity)")
            a[3].set_title("measured joint velocity")
    for ax in axs[-1]:
        ax.set_xlabel("time since step command [s]")
    fig.suptitle(f"step response {meta['metrics']['amplitude_m'] * 1e3:.0f} mm — {run_label(meta)}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return [_save(fig, meta, "steps"), plot_spectra(meta, data)]


def plot_hold_run(meta: dict, data: dict) -> list[Path]:
    t0 = data["tau_t"][0]
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    err_t = data["err_t"] - t0
    axs[0].plot(err_t, np.linalg.norm(data["err"][:, :3], axis=1) * 1e3, label="|position error| [mm]")
    axs[0].plot(err_t, np.linalg.norm(data["err"][:, 3:], axis=1), label="|orientation error| [mrad]")
    axs[0].legend(fontsize=8)
    for j in range(7):
        axs[1].plot(data["tau_t"] - t0, data["tau"][:, j], color=JOINT_COLORS[j], lw=0.8)
        axs[2].plot(data["state_t"] - t0, data["state"][:, 7 + j], color=JOINT_COLORS[j], lw=0.8)
    axs[1].set_ylabel("τ_cmd [Nm]")
    axs[2].set_ylabel("dq [rad/s]")
    _joint_legend(axs[1])
    axs[2].set_xlabel("time [s]")
    for ax in axs:
        ax.grid(True, alpha=0.3)
    fig.suptitle(f"hold — {run_label(meta)}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return [_save(fig, meta, "hold"), plot_spectra(meta, data)]


def tracking_traces(data: dict) -> dict:
    ref_t, ref_pos = data["ref_t"], data["ref_pos"]
    m = (data["ee_t"] >= ref_t[0]) & (data["ee_t"] <= ref_t[-1])
    meas = np.column_stack([np.interp(ref_t, data["ee_t"][m], data["ee"][m, k]) for k in range(3)])
    err = meas - ref_pos
    return {"t": ref_t - ref_t[0], "ref": ref_pos, "meas": meas, "err_mm": err * 1e3, "speed": np.linalg.norm(data["ref_vel"], axis=1)}


def plot_tracking_run(meta: dict, data: dict) -> list[Path]:
    tr = tracking_traces(data)
    t0 = data["ref_t"][0]
    fig = plt.figure(figsize=(15, 11))
    gs = fig.add_gridspec(4, 2, width_ratios=[1, 1.6])
    ax_xy = fig.add_subplot(gs[:, 0])
    # Path in the plane of the two axes it moves along most (xy for the eight, xz for a plunge)
    a, b = sorted(np.argsort(np.ptp(tr["ref"], axis=0))[-2:])
    ax_xy.plot(tr["ref"][:, b], tr["ref"][:, a], "k--", lw=0.9, label="target")
    ax_xy.plot(tr["meas"][:, b], tr["meas"][:, a], color="C0", lw=1.0, label="measured")
    ax_xy.set_xlabel(f"{'xyz'[b]} [m]")
    ax_xy.set_ylabel(f"{'xyz'[a]} [m]")
    ax_xy.set_aspect("equal")
    ax_xy.legend(fontsize=8)
    ax_xy.grid(True, alpha=0.3)
    ax_e = fig.add_subplot(gs[0, 1])
    for k, name in enumerate("xyz"):
        ax_e.plot(tr["t"], tr["err_mm"][:, k], lw=0.9, label=f"e_{name}")
    ax_e.plot(tr["t"], np.linalg.norm(tr["err_mm"], axis=1), "k", lw=0.9, label="|e|")
    ax_e.set_ylabel("error [mm]")
    ax_e.legend(fontsize=7, ncol=4)
    ax_v = ax_e.twinx()
    ax_v.plot(tr["t"], tr["speed"] * 1e3, color="0.6", lw=0.7, ls=":")
    ax_v.set_ylabel("target speed [mm/s]", color="0.5")
    ax_r = fig.add_subplot(gs[1, 1], sharex=ax_e)
    # The controller's own orientation error (rotation vector of q_target q^-1, base frame)
    me = (data["err_t"] >= t0) & (data["err_t"] <= data["ref_t"][-1])
    for k, name in enumerate("xyz"):
        ax_r.plot(data["err_t"][me] - t0, data["err"][me, 3 + k] * 1e3, lw=0.9, label=f"about {name}")
    ax_r.set_ylabel("orientation err [mrad]")
    ax_r.legend(fontsize=7, ncol=3)
    ax_q = fig.add_subplot(gs[2, 1], sharex=ax_e)
    ms = (data["state_t"] >= t0) & (data["state_t"] <= data["ref_t"][-1])
    for j in range(7):
        ax_q.plot(data["state_t"][ms] - t0, data["state"][ms, 7 + j], color=JOINT_COLORS[j], lw=0.7)
    ax_q.set_ylabel("dq [rad/s]")
    ax_t = fig.add_subplot(gs[3, 1], sharex=ax_e)
    mt = (data["tau_t"] >= t0) & (data["tau_t"] <= data["ref_t"][-1])
    for j in range(7):
        ax_t.plot(data["tau_t"][mt] - t0, data["tau"][mt, j], color=JOINT_COLORS[j], lw=0.7)
    ax_t.set_ylabel("τ_cmd [Nm]")
    ax_t.set_xlabel("time [s]")
    _joint_legend(ax_t)
    for ax in (ax_e, ax_r, ax_q, ax_t):
        ax.grid(True, alpha=0.3)
    fig.suptitle(f"tracking — {run_label(meta)}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return [_save(fig, meta, "tracking"), plot_spectra(meta, data, t0, data["ref_t"][-1])]


PLOTTERS = {"step_response": plot_step_run, "hold": plot_hold_run, "tracking": plot_tracking_run}


def plot_run(run: str | Path) -> list[Path]:
    meta, data = load(run)
    return PLOTTERS[meta["benchmark"]](meta, data)


# =======================
# MARK: Comparison overlays
# =======================


def _overlay_spectra(axs, runs: list[tuple[dict, dict]], windows: list[tuple[float, float]]) -> None:
    """Joint-summed PSD of commanded torque and joint velocity, one line per run."""
    for i, ((meta, data), (t0, t1)) in enumerate(zip(runs, windows)):
        tau = data["tau"][window(data, "tau", t0, t1)]
        dq = data["state"][window(data, "state", t0, t1), 7:14]
        for ax, x in ((axs[0], tau), (axs[1], dq)):
            f, p = psd(x)
            ax.semilogy(f, p.sum(1), color=f"C{i}", lw=0.9, label=run_label(meta))
    axs[0].set_ylabel("Σ PSD τ_cmd [Nm²/Hz]")
    axs[1].set_ylabel("Σ PSD dq [(rad/s)²/Hz]")
    for ax in axs:
        ax.axvline(CHATTER_HP_HZ, color="k", ls=":", lw=0.8)
        ax.set_xlim(0, FS / 2)
        ax.grid(True, which="both", alpha=0.3)
        ax.set_xlabel("frequency [Hz]")
    axs[0].legend(fontsize=7)


def compare_steps(runs: list[tuple[dict, dict]], out: Path) -> list[Path]:
    labels = [s["label"] for s in step_segments(*runs[0])]
    fig, axs = plt.subplots(len(labels), 4, figsize=(17, 2.6 * len(labels) + 1), squeeze=False)
    for i, (meta, data) in enumerate(runs):
        ref_rot = Rotation.from_quat(data["ee"][0, 3:7])
        segs = {s["label"]: s for s in step_segments(meta, data)}
        for r, lbl in enumerate(labels):
            if lbl not in segs:
                continue
            tr = step_traces(data, segs[lbl], ref_rot)
            c = f"C{i}"
            axs[r, 0].plot(tr["t"], tr["s_mm"] / tr["length_mm"], color=c, lw=1.0, label=run_label(meta))
            axs[r, 1].plot(tr["t"], tr["rot_mrad"], color=c, lw=1.0)
            axs[r, 2].plot(tr["t"], tr["off_mm"], color=c, lw=1.0)
            axs[r, 3].plot(tr["tau_t"], tr["tau_hf_rms"], color=c, lw=1.0)
    for r, lbl in enumerate(labels):
        axs[r, 0].axhline(1.0, color="k", ls="--", lw=0.8)
        axs[r, 0].set_ylabel(f"{lbl}\nnormalized")
        axs[r, 0].set_ylim(-0.1, 1.25)
        for ax in axs[r]:
            ax.grid(True, alpha=0.3)
    titles = ("displacement / step size", "orientation error [mrad]", "off-axis deviation [mm]", "τ_cmd chatter: moving RMS of |τ_hf| [Nm]")
    for ax, title in zip(axs[0], titles):
        ax.set_title(title, fontsize=10)
    axs[0, 0].legend(fontsize=7)
    for ax in axs[-1]:
        ax.set_xlabel("time since step command [s]")
    fig.suptitle("step response comparison", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    paths = [out / "steps.png"]
    fig.savefig(paths[0], dpi=110)
    plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(15, 5))
    _overlay_spectra(axs, runs, [(d["tau_t"][0], d["tau_t"][-1] + 1e-3) for _, d in runs])
    fig.suptitle("spectra over the whole step sequence", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    paths.append(out / "spectra.png")
    fig.savefig(paths[1], dpi=110)
    plt.close(fig)
    return paths


def compare_tracking(runs: list[tuple[dict, dict]], out: Path) -> list[Path]:
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(2, 3)
    ax_xy = fig.add_subplot(gs[:, 0])
    ax_e = fig.add_subplot(gs[0, 1:])
    ax_s = [fig.add_subplot(gs[1, 1]), fig.add_subplot(gs[1, 2])]
    for i, (meta, data) in enumerate(runs):
        tr = tracking_traces(data)
        if i == 0:
            # Path in the plane of the two axes it moves along most (xy for the eight, xz for a plunge)
            a, b = sorted(np.argsort(np.ptp(tr["ref"], axis=0))[-2:])
            ax_xy.plot(tr["ref"][:, b], tr["ref"][:, a], "k--", lw=0.9, label="target")
        ax_xy.plot(tr["meas"][:, b], tr["meas"][:, a], color=f"C{i}", lw=1.0, label=run_label(meta))
        ax_e.plot(tr["t"], np.linalg.norm(tr["err_mm"], axis=1), color=f"C{i}", lw=0.9, label=run_label(meta))
    ax_xy.set_aspect("equal")
    ax_xy.set_xlabel(f"{'xyz'[b]} [m]")
    ax_xy.set_ylabel(f"{'xyz'[a]} [m]")
    ax_xy.legend(fontsize=7)
    ax_e.set_ylabel("|position error| [mm]")
    ax_e.set_xlabel("time [s]")
    for ax in (ax_xy, ax_e):
        ax.grid(True, alpha=0.3)
    _overlay_spectra(ax_s, runs, [(d["ref_t"][0], d["ref_t"][-1]) for _, d in runs])
    fig.suptitle("tracking comparison", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    path = out / "tracking.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return [path]


def compare_hold(runs: list[tuple[dict, dict]], out: Path) -> list[Path]:
    fig, axs = plt.subplots(1, 2, figsize=(15, 5))
    _overlay_spectra(axs, runs, [(d["tau_t"][0], d["tau_t"][-1] + 1e-3) for _, d in runs])
    fig.suptitle("hold comparison", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    path = out / "spectra.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return [path]


COMPARERS = {"step_response": compare_steps, "tracking": compare_tracking, "hold": compare_hold}


def compare(run_paths: list[str | Path]) -> list[Path]:
    runs = [load(p) for p in run_paths]
    benches = {m["benchmark"] for m, _ in runs}
    if len(benches) != 1:
        raise ValueError(f"Overlay runs of one benchmark at a time, got {sorted(benches)}")
    bench = benches.pop()
    out = new_compare_dir(bench)
    (out / "runs.yaml").write_text(yaml.safe_dump([str(m["_dir"]) for m, _ in runs]))
    return COMPARERS[bench](runs, out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", help="Run directories (or their meta.yaml)")
    parser.add_argument("--compare", action="store_true", help="Overlay the runs instead of per-run figures")
    args = parser.parse_args()
    paths = compare(args.runs) if args.compare else [p for r in args.runs for p in plot_run(r)]
    for p in paths:
        print(p)


if __name__ == "__main__":
    main()
