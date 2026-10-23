"""Metrics of EXP_PLAN.md on a loaded run ([runs.load_run][]), shared by the experiment scripts.

Forces and velocities are in interface coordinates (``meta["interface_basis"]``), base-frame axes for
``--interface-axis xyz``. ``f`` is the force the operator felt (the haptic log's ``f``: after the
clamp, ramp and watchdog); ``F_osc`` is the OSC task force on the robot (``task_wrench``).

- [unfelt_force][] (e_F): ``f(t) + F_osc(t_apply)``, where ``t_apply`` is when the operator's action
  at ``t`` reached the robot: the first target published from haptic tick ``k`` or later (the
  ``target`` stream carries the tick). Measured, not the nominal ``T_c``.
- [energy_balance][]: ``E(t) = -∫ (f·v_l + F_osc·v) dt``; negative = the system generated energy.

Only the haptic log's span counts: the command loop keeps sending the last tick after the haptic
loop stops.
"""

from __future__ import annotations

import numpy as np


def basis(run: dict) -> np.ndarray:
    """``(3, dim)`` interface basis in the base frame."""
    return np.asarray(run["meta"]["interface_basis"], dtype=float)


def _vectors(df, name: str, dim: int) -> np.ndarray:
    return df[[f"{name}_{k}" for k in range(dim)]].to_numpy(dtype=float)


def haptic_side(run: dict) -> dict[str, np.ndarray]:
    """``t`` [s], leader ``x_l``, ``v_l``, felt force ``f``, ``(n, dim)``, one row per haptic tick."""
    h = run["haptic"]
    dim = basis(run).shape[1]
    return {"t": h.ts.to_numpy(dtype=float), "x_l": _vectors(h, "x_l", dim), "v_l": _vectors(h, "v_l", dim),
            "f": _vectors(h, "f", dim)}


def robot_at(run: dict, t: np.ndarray) -> dict[str, np.ndarray]:
    """Robot ``x``, ``v``, ``F_osc`` in interface coordinates, interpolated at times ``t`` [s]
    (NaN outside the robot log)."""
    r, B = run["robot"], basis(run)
    rt = r.ts.to_numpy(dtype=float)

    def interp(cols):
        return np.stack([np.interp(t, rt, r[c].to_numpy(dtype=float), left=np.nan, right=np.nan) for c in cols], 1) @ B

    return {"x": interp(("x", "y", "z")), "v": interp(("vx", "vy", "vz")), "F_osc": interp(("fx", "fy", "fz"))}


def apply_times(run: dict) -> np.ndarray:
    """Per haptic tick, when its action reached the robot [s]: the first target published from that
    tick or a later one. NaN past the last target."""
    tg = run["target"]
    n = len(run["haptic"])
    tick = tg.tick.to_numpy().astype(int)
    ts = tg.ts.to_numpy(dtype=float)
    keep = tick >= 0  # a freeze carries no tick
    tick, ts = tick[keep], ts[keep]
    # first publication of each tick (the loop resends the latest tick until a new one arrives)
    tick, first = np.unique(tick, return_index=True)
    ts = ts[first]
    idx = np.searchsorted(tick, np.arange(n), side="left")
    out = np.full(n, np.nan)
    ok = idx < len(tick)
    out[ok] = ts[idx[ok]]
    return out


def unfelt_force(run: dict) -> dict[str, np.ndarray]:
    """e_F per haptic tick: ``t``, ``e_F`` ``(n, dim)``, the felt ``f``, the robot ``F_osc`` at
    ``t_apply``, and ``lag`` = ``t_apply - t`` [s] (the measured command-side delay)."""
    hs = haptic_side(run)
    t_apply = apply_times(run)
    F = robot_at(run, t_apply)["F_osc"]
    return {"t": hs["t"], "e_F": hs["f"] + F, "f": hs["f"], "F_osc": F, "lag": t_apply - hs["t"]}


def energy_balance(run: dict) -> tuple[np.ndarray, np.ndarray]:
    """``(t, E)``: ``E(t) = -∫ (f·v_l + F_osc·v) dt`` [J], on the haptic clock."""
    hs = haptic_side(run)
    rb = robot_at(run, hs["t"])
    power = np.sum(hs["f"] * hs["v_l"], 1) + np.nan_to_num(np.sum(rb["F_osc"] * rb["v"], 1))
    dt = np.diff(hs["t"], prepend=hs["t"][0])
    return hs["t"], -np.cumsum(power * dt)


def rms(x: np.ndarray, axis=0) -> np.ndarray:
    """RMS ignoring NaN rows (per column for a ``(n, dim)`` array)."""
    x = np.asarray(x, dtype=float)
    return np.sqrt(np.nanmean(x**2, axis=axis))
