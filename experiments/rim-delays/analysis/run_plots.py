"""The figures of one run, for `check_run.ipynb` (and the experiment scripts).

Each function takes a loaded run ([runs.load_run][]) and returns a matplotlib figure. Interface
axes are the base-frame x, y, z for ``--interface-axis xyz``; ``i0, i1, ...`` otherwise.
Forces compare the felt ``f`` with ``-F_osc`` (the robot's force, at the time the operator's action
reached it): they overlap when the operator feels what the robot exerts, and e_F = f + F_osc is
their gap.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import metrics as M  # noqa: E402
from runs import GRID, INK_2, INK_3, surface_of  # noqa: E402

# categorical slots in fixed order (runs.COLORS uses the same palette), one per role
LEADER, PROXY, TARGET, ROBOT = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
FELT, ROBOT_FORCE, GAP = "#2a78d6", "#eb6834", "#52514e"


def axis_names(run: dict) -> list[str]:
    B = M.basis(run)
    if B.shape[1] == 3 and np.allclose(np.abs(B), np.eye(3)):
        return list("xyz")
    return [f"i{k}" for k in range(B.shape[1])]


def _surface_on(run: dict, k: int, ax) -> None:
    """The table, dashed, on the contact axis."""
    s = surface_of(run["meta"])
    if s is not None and k == int(run["meta"].get("contact_axis", 0)):
        ax.axhline(s, color=INK_3, lw=1, ls="--", label="table")


def positions(run: dict):
    """Per axis: leader, proxy, target published to the robot, robot measured [m]."""
    hs = M.haptic_side(run)
    h, tg = run["haptic"], run["target"]
    rb = M.robot_at(run, hs["t"])
    names = axis_names(run)
    fig, axes = plt.subplots(len(names), 1, figsize=(10, 2.3 * len(names)), sharex=True)
    for k, (ax, a) in enumerate(zip(np.atleast_1d(axes), names)):
        ax.plot(hs["t"], hs["x_l"][:, k], color=LEADER, lw=1.5, label="leader")
        if f"proxy_x_{k}" in h and np.isfinite(h[f"proxy_x_{k}"]).any():
            ax.plot(hs["t"], h[f"proxy_x_{k}"], color=PROXY, lw=1.5, label="proxy")
        if tg is not None:
            ax.plot(tg.ts, tg[f"x_{k}"], color=TARGET, lw=1.5, label="target at robot")
        ax.plot(hs["t"], rb["x"][:, k], color=ROBOT, lw=1.5, label="robot")
        _surface_on(run, k, ax)
        ax.set_ylabel(f"{a} [m]")
    np.atleast_1d(axes)[0].legend(loc="upper right", ncol=5, fontsize=8)
    np.atleast_1d(axes)[-1].set_xlabel("time [s]")
    fig.suptitle("Position", x=0.01, ha="left")
    fig.tight_layout()
    return fig


def forces(run: dict):
    """Per axis: felt f vs robot −F_osc (left), e_F (right) [N]."""
    e = M.unfelt_force(run)
    names = axis_names(run)
    fig, axes = plt.subplots(len(names), 2, figsize=(12, 2.3 * len(names)), sharex=True,
                             gridspec_kw={"width_ratios": (3, 2)})
    axes = np.atleast_2d(axes)
    for k, a in enumerate(names):
        left, right = axes[k]
        left.plot(e["t"], e["f"][:, k], color=FELT, lw=1.5, label="felt f")
        left.plot(e["t"], -e["F_osc"][:, k], color=ROBOT_FORCE, lw=1.5, label="robot −F_osc (at t_apply)")
        left.set_ylabel(f"{a} [N]")
        right.plot(e["t"], e["e_F"][:, k], color=GAP, lw=1.2)
        right.axhline(0, color=GRID, lw=1)
        right.set_ylabel(f"e_F {a} [N]")
        right.text(0.01, 0.92, f"RMS {M.rms(e['e_F'][:, k]):.2f} N", transform=right.transAxes, color=INK_2, fontsize=9,
                   va="top")
    axes[0, 0].legend(loc="upper right", fontsize=8)
    axes[-1, 0].set_xlabel("time [s]")
    axes[-1, 1].set_xlabel("time [s]")
    fig.suptitle("Force: felt vs robot, and the unfelt force e_F = f + F_osc", x=0.01, ha="left")
    fig.tight_layout()
    return fig


def timing(run: dict):
    """Delays (haptic → robot lag, robot → haptic plant age), haptic period, lost controller ticks."""
    h, r = run["haptic"], run["robot"]
    e = M.unfelt_force(run)
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 6.5), sharex=True)
    a1.plot(e["t"], 1e3 * e["lag"], color=LEADER, lw=1.2, label="haptic → robot (measured lag)")
    a1.plot(h.ts, 1e3 * h.plant_age_s, color=PROXY, lw=1.2, label="robot → haptic (plant sample age)")
    a1.set_ylabel("delay [ms]")
    a1.legend(loc="upper right", fontsize=8)
    if "mono_s" in h:
        a2.plot(h.ts.to_numpy()[1:], 1e3 * np.diff(h.mono_s.to_numpy()), color=INK_2, lw=1)
    a2.set_ylabel("haptic period [ms]")
    if "t_ctrl" in r:
        stamp = np.sort(r.t_ctrl.to_numpy())
        gap = np.round(np.diff(stamp) * 1e3).astype(int) - 1  # controller ticks missing before each sample
        t = r.ts.to_numpy()[np.argsort(r.t_ctrl.to_numpy())][1:]
        a3.vlines(t[gap > 0], 0, gap[gap > 0], color=ROBOT, lw=1)
        lost = gap[gap > 0].sum() / max(1, len(stamp) + gap[gap > 0].sum())
        a3.text(0.01, 0.9, f"{100 * lost:.1f} % of controller ticks lost", transform=a3.transAxes, color=INK_2,
                fontsize=9, va="top")
    a3.set_ylabel("ticks lost")
    a3.set_xlabel("time [s]")
    fig.suptitle("Timing", x=0.01, ha="left")
    fig.tight_layout()
    return fig


def energy_and_model(run: dict):
    """Energy balance E(t), and for proxy-rim the eigenvalues of the model's Λ over time."""
    t, E = M.energy_balance(run)
    m = run.get("model")
    has_model = m is not None and len(m) and "M_eff_8" in m
    fig, axes = plt.subplots(2 if has_model else 1, 1, figsize=(10, 4.6 if has_model else 2.6), sharex=True)
    axes = np.atleast_1d(axes)
    axes[0].plot(t, E, color=INK_2, lw=1.5)
    axes[0].axhline(0, color=GRID, lw=1)
    axes[0].set_ylabel("E [J]")
    axes[0].set_title("Energy balance E = −∫(f·v_l + F_osc·v) dt  (< 0: the system generated energy)", loc="left",
                      fontsize=10)
    if has_model:
        lam = m[[f"M_eff_{k}" for k in range(9)]].to_numpy().reshape(-1, 3, 3)
        eig = np.linalg.eigvalsh(lam)
        for k, (label, color) in enumerate((("lightest", LEADER), ("middle", PROXY), ("heaviest", TARGET))):
            axes[1].plot(m.ts, eig[:, k], color=color, lw=1.5, label=label)
        axes[1].set_ylabel("Λ eigenvalue [kg]")
        axes[1].legend(loc="upper right", fontsize=8, ncol=3)
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    return fig
