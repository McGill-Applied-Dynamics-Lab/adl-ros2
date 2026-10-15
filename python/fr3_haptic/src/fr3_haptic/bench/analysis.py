"""Load and visualize haptic stability-bench runs.

Diagnostic plots for debugging buzz/instability:
  - position tracking (leader vs virtual mass)
  - velocity signals (raw leader velocity reveals sensor noise)
  - rendered coupling force
  - loop period (is the configured rate actually achieved?)
  - force power spectrum (broadband high-freq energy -> velocity noise;
    a sharp peak -> a limit cycle / ZOH passivity-boundary buzz)
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .haptic_bench import SAMPLE_COLUMNS


def load_run(run_dir: str | Path) -> dict[str, Any]:
    """Load a bench run directory into a dict of named columns + metadata."""
    run_dir = Path(run_dir)
    data = np.load(run_dir / "samples.npz", allow_pickle=True)
    samples = data["samples"]
    out: dict[str, Any] = {name: samples[:, i] for i, name in enumerate(SAMPLE_COLUMNS)}
    out["samples"] = samples

    meta_path = run_dir / "metadata.json"
    out["metadata"] = json.loads(meta_path.read_text()) if meta_path.exists() else {}

    t = out["t"]
    dt = np.diff(t)
    out["dt"] = dt
    out["effective_rate_hz"] = float(1.0 / np.median(dt)) if dt.size else float("nan")
    return out


def _force_psd(force: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    """Power spectral density of the force signal (Welch if available, else FFT)."""
    f = force - np.mean(force)
    try:
        from scipy.signal import welch

        nperseg = int(min(len(f), max(256, fs // 2)))
        return welch(f, fs=fs, nperseg=nperseg)
    except Exception:
        spec = np.abs(np.fft.rfft(f)) ** 2 / len(f)
        freqs = np.fft.rfftfreq(len(f), d=1.0 / fs)
        return freqs, spec


def plot_run(run_dir: str | Path, save_path: str | Path | None = None, show: bool = True):
    """Render the diagnostic panel for a bench run. Returns the matplotlib Figure."""
    import matplotlib.pyplot as plt

    run = load_run(run_dir)
    t = run["t"]
    cfg = run["metadata"].get("config", {})
    fs = run["effective_rate_hz"]
    rate_cfg = cfg.get("rate_hz", fs)

    fig, axes = plt.subplots(3, 2, figsize=(13, 9))
    title = run["metadata"].get("notes") or Path(run_dir).name
    fig.suptitle(
        f"{title}\nK={cfg.get('stiffness')} D={cfg.get('damping')} m={cfg.get('mass')} "
        f"| rate cfg={rate_cfg}Hz, achieved≈{fs:.0f}Hz | vel_filter_alpha={cfg.get('vel_filter_alpha')}",
        fontsize=10,
    )

    # Position tracking
    ax = axes[0, 0]
    ax.plot(t, run["x_leader"], label="leader", lw=0.8)
    ax.plot(t, run["x_mass"], label="virtual mass", lw=0.8)
    ax.set(ylabel="position [m]", title="Position tracking")
    ax.legend(fontsize=8)

    # Coupling lag
    ax = axes[0, 1]
    ax.plot(t, (run["x_mass"] - run["x_leader"]) * 1000, lw=0.8, color="tab:red")
    ax.set(ylabel="x_mass − x_leader [mm]", title="Coupling lag")

    # Velocities (raw leader velocity = noise smoking gun)
    ax = axes[1, 0]
    ax.plot(t, run["v_leader"], label="leader (raw)", lw=0.6, alpha=0.8)
    ax.plot(t, run["v_mass"], label="virtual mass", lw=0.8)
    ax.set(ylabel="velocity [m/s]", title="Velocities (raw leader vel reveals sensor noise)")
    ax.legend(fontsize=8)

    # Force
    ax = axes[1, 1]
    ax.plot(t, run["force"], lw=0.6, color="tab:purple")
    ax.set(ylabel="coupling force [N]", title="Rendered force")

    # Loop period
    ax = axes[2, 0]
    dt_ms = run["dt"] * 1000
    ax.plot(dt_ms, lw=0.5)
    if rate_cfg:
        ax.axhline(1000.0 / rate_cfg, color="k", ls="--", lw=0.8, label=f"target {1000.0/rate_cfg:.2f} ms")
        ax.legend(fontsize=8)
    ax.set(xlabel="sample", ylabel="loop period [ms]", title="Loop timing (jitter -> chatter)")

    # Force PSD
    ax = axes[2, 1]
    freqs, psd = _force_psd(run["force"], fs if np.isfinite(fs) else float(rate_cfg or 1000.0))
    ax.semilogy(freqs, psd, lw=0.8, color="tab:purple")
    ax.set(xlabel="frequency [Hz]", ylabel="force PSD", title="Force spectrum (peak=limit cycle, broadband=noise)")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    if save_path is not None:
        fig.savefig(save_path, dpi=120)
    if show:
        plt.show()
    return fig
