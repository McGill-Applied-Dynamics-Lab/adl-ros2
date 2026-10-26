"""Per-experiment metrics (EXP_PLAN.md), for the experiment scripts and their notebooks.

Each ``eN(run)`` takes a loaded run (``runs.load_run``) and returns a flat dict: scalars (one
table column each in the notebooks) and a few lists (per phase, per path bin, per contact) the
notebooks plot. The scripted windows come from the run's own script (``metrics.script_of``), so a
metric covers exactly the motion it is about.

Signs: ``f`` is the force the operator felt (world axes; in contact with the table it pushes up,
``f_z > 0``); ``F_osc`` the OSC's force on the robot; ``e_F = f + F_osc`` taken when the operator's
action reached the robot (``metrics.unfelt_force``). The robot's table force is ``-F_osc_z`` (quasi-static).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import metrics as M  # noqa: E402
from runs import surface_of  # noqa: E402

CONTACT_N = 1.0  # felt force above its pre-contact level that counts as contact onset [N]


def _segment(run, kind: str):
    """The first script segment of class ``kind`` (``_Oscillation``, ``_Circle``, ``_Cruise``)."""
    script = M.script_of(run)
    if script is None:
        return None
    return next((s for s in script.segments if type(s).__name__ == kind), None)


def _window(t, t0, t1):
    return (t >= t0) & (t <= t1)


def _common(run) -> dict:
    e = M.unfelt_force(run)
    t, E = M.energy_balance(run)
    return {"e": e, "E_end_J": float(E[-1]), "E_min_J": float(np.nanmin(E))}


# ------------------------------------------------------------------ E1


def e1(run) -> dict:
    """Free-motion transparency on a sine along ``n``: e_F (along n and its norm), the felt force,
    the robot's force, over the full-amplitude cycles."""
    seg = _segment(run, "_Oscillation")
    c = _common(run)
    e = c["e"]
    if seg is None:
        return {"E_end_J": c["E_end_J"]}
    w = _window(e["t"], seg.t0 + seg.ramp_s, seg.t0 + seg.duration - seg.ramp_s)
    n = seg.direction
    cfg = run["meta"]["script"]
    om = 2 * np.pi * cfg["frequency_hz"]
    return {
        "frequency_hz": cfg["frequency_hz"],
        "amplitude_m": cfg["amplitude"],
        "accel_peak": cfg["amplitude"] * om**2,
        "direction": n.tolist(),
        "eF_rms": float(M.rms(np.linalg.norm(e["e_F"][w], axis=1))),
        "eF_n_rms": float(M.rms(e["e_F"][w] @ n)),
        "f_n_rms": float(M.rms(e["f"][w] @ n)),
        "Fosc_n_rms": float(M.rms(e["F_osc"][w] @ n)),
        "lag_ms": float(1e3 * np.nanmedian(e["lag"][w])),
        "E_end_J": c["E_end_J"],
    }


def _half_range(y: np.ndarray) -> float:
    """Half the 1–99 percentile span: a sine's amplitude, robust to spikes."""
    return float(np.percentile(y, 99) - np.percentile(y, 1)) / 2


def _first_harmonic(t: np.ndarray, y: np.ndarray, w: float) -> complex:
    """``y``'s phasor at ``w`` [rad/s] (least squares on cos, sin and an offset)."""
    B = np.c_[np.cos(w * t), np.sin(w * t), np.ones_like(t)]
    (c, s, _), *_ = np.linalg.lstsq(B, y, rcond=None)
    return complex(c, -s)


def e1_checks(run) -> dict:
    """Diagnostics of an E1 run, over ``e1``'s window (the robot's shifted by the measured lag):
    stop reason, OSC stiffness and coupling K, time with the chatter guard tripped; the amplitude
    along n of the leader, the proxy (M3–M5) and the robot [m]; and the robot's first-harmonic
    impedance ``Z = F_osc / x`` along n at the drive frequency. ``Re Z = k − m ω²`` gives the
    robot's effective mass over the frequencies; ``Im Z`` is friction: for Coulomb friction F_c,
    ``F_c ≈ π Im Z |X| / 4`` (constant in frequency), for viscous friction ``c = Im Z / ω``."""
    meta = run["meta"]
    out = {"stop_reason": meta.get("stop_reason"), "osc_k_pos_x": (meta.get("controller_params") or {}).get("gains.k_pos_x"),
           "K": meta.get("args", {}).get("kv")}
    seg = _segment(run, "_Oscillation")
    if seg is None:
        return out
    e = M.unfelt_force(run)
    lag = float(np.nanmedian(e["lag"]))
    n = seg.direction
    t0, t1 = seg.t0 + seg.ramp_s, seg.t0 + seg.duration - seg.ramp_s
    h, rb = run["haptic"], run["robot"]
    th, tr = h.ts.to_numpy(float), rb.ts.to_numpy(float)
    wh, wr = _window(th, t0, t1), _window(tr, t0 + lag, t1 + lag)
    x_l = h[[f"x_l_{i}" for i in range(3)]].to_numpy(float) @ n
    x_r = rb[["x", "y", "z"]].to_numpy(float) @ n
    F = rb[["fx", "fy", "fz"]].to_numpy(float) @ n
    w = 2 * np.pi * run["meta"]["script"]["frequency_hz"]
    X, Fh = _first_harmonic(tr[wr], x_r[wr], w), _first_harmonic(tr[wr], F[wr], w)
    Z = Fh / X
    out.update({
        "guard_s": float(h.guard_tripped.to_numpy(float).sum() * np.median(np.diff(th))) if "guard_tripped" in h else 0.0,
        "A_leader": _half_range(x_l[wh]),
        "A_proxy": _half_range((h[[f"proxy_x_{i}" for i in range(3)]].to_numpy(float) @ n)[wh])
        if "proxy_x_0" in h and np.isfinite(h.proxy_x_0).any() else np.nan,
        "A_robot": _half_range(x_r[wr]),
        "X1": abs(X), "F1": abs(Fh), "ReZ": Z.real, "ImZ": Z.imag, "coulomb_N": np.pi * Z.imag * abs(X) / 4,
    })
    return out


# ------------------------------------------------------------------ E2


def e2(run, bins: int = 36) -> dict:
    """Horizontal circle: e_F split along the circle's inward radial direction (inertia) and the
    leader's tangent (friction), and binned against the circle phase; the angle in the plane from
    the centripetal direction to the felt force and to the robot's force."""
    seg = _segment(run, "_Circle")
    c = _common(run)
    e = c["e"]
    if seg is None:
        return {"E_end_J": c["E_end_J"]}
    w = _window(e["t"], seg.t0 + seg.ramp_s, seg.t0 + seg.duration - seg.ramp_s)
    hs = M.haptic_side(run)
    rel = hs["x_l"][w, :2] - seg.center[:2]
    phase = np.arctan2(rel[:, 1], rel[:, 0])
    inward = -rel / np.linalg.norm(rel, axis=1, keepdims=True)
    # direction of motion: the circle's angle grows with omega, so for omega > 0 the leader moves
    # counter-clockwise, along (-sin φ, cos φ) = (inward_y, -inward_x)
    tangent = np.stack([inward[:, 1], -inward[:, 0]], 1) * np.sign(seg.omega)
    eF = e["e_F"][w]
    radial, tangential = np.sum(eF[:, :2] * inward, 1), np.sum(eF[:, :2] * tangent, 1)

    def angle_to_inward(F):  # signed angle in the plane from the centripetal direction to F [deg]
        a = np.arctan2(F[:, 1], F[:, 0]) - np.arctan2(inward[:, 1], inward[:, 0])
        return np.degrees(np.angle(np.exp(1j * a)))

    edges = np.linspace(-np.pi, np.pi, bins + 1)
    k = np.clip(np.digitize(phase, edges) - 1, 0, bins - 1)
    per_bin = lambda x: [float(np.nanmean(x[k == i])) if np.any(k == i) else None for i in range(bins)]  # noqa: E731
    return {
        "period_s": run["meta"]["script"]["period_s"],
        "radius_m": run["meta"]["script"]["radius"],
        "eF_rms": float(M.rms(np.linalg.norm(eF, axis=1))),
        "eF_radial_mean": float(np.mean(radial)), "eF_radial_rms": float(M.rms(radial)),
        "eF_tangential_mean": float(np.mean(tangential)), "eF_tangential_rms": float(M.rms(tangential)),
        "eF_z_rms": float(M.rms(eF[:, 2])),
        # radial forces: the outward reaction felt while the proxy is pulled inward (the rendered
        # inertia), and the robot's centripetal force; eF_radial = robot - felt
        "felt_radial_mean": float(np.mean(np.sum(-e["f"][w, :2] * inward, 1))),
        "robot_radial_mean": float(np.nanmean(np.sum(e["F_osc"][w, :2] * inward, 1))),
        "angle_f_deg": float(np.nanmean(angle_to_inward(-e["f"][w, :2]))),  # felt: the proxy pulls the hand inward
        "angle_Fosc_deg": float(np.nanmean(angle_to_inward(e["F_osc"][w, :2]))),
        "phase_deg": np.degrees(0.5 * (edges[:-1] + edges[1:])).tolist(),
        "eF_radial_by_phase": per_bin(radial), "eF_tangential_by_phase": per_bin(tangential),
        "E_end_J": c["E_end_J"],
    }


def _mean_and_2f(phase: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    """Mean, amplitude and phase [deg] of ``y``'s component at twice the circle frequency."""
    ok = np.isfinite(y)
    p = phase[ok]
    (c0, c, s_), *_ = np.linalg.lstsq(np.c_[np.ones_like(p), np.cos(2 * p), np.sin(2 * p)], y[ok], rcond=None)
    return float(c0), float(np.hypot(c, s_)), float(np.degrees(np.arctan2(s_, c)) / 2)


def e2_checks(run, bins: int = 36) -> dict:
    """Diagnostics of an E2 run, over ``e2``'s window: stop reason, OSC stiffness, RIM reduction
    (orientation held or free), M4's mass, chatter-guard time; the felt radial force (the outward
    reaction the proxy renders), the robot's radial and tangential force (``F_osc``), each binned
    against the circle phase with its mean and its component at twice the circle frequency (2f);
    and the robot's own circle radius (its stream, shifted by the measured lag) [m]."""
    meta = run["meta"]
    args = meta.get("args", {})
    out = {"stop_reason": meta.get("stop_reason"), "osc_k_pos_x": (meta.get("controller_params") or {}).get("gains.k_pos_x"),
           "hold_orientation": args.get("rim_hold_orientation"), "proxy_mass": args.get("proxy_mass")}
    seg = _segment(run, "_Circle")
    if seg is None:
        return out
    e = M.unfelt_force(run)
    w = _window(e["t"], seg.t0 + seg.ramp_s, seg.t0 + seg.duration - seg.ramp_s)
    hs = M.haptic_side(run)
    rel = hs["x_l"][w, :2] - seg.center[:2]
    phase = np.arctan2(rel[:, 1], rel[:, 0])
    inward = -rel / np.linalg.norm(rel, axis=1, keepdims=True)
    tangent = np.stack([inward[:, 1], -inward[:, 0]], 1) * np.sign(seg.omega)
    signals = {
        "felt_radial": np.sum(-e["f"][w, :2] * inward, 1),
        "robot_radial": np.sum(e["F_osc"][w, :2] * inward, 1),
        "robot_tangential": np.sum(e["F_osc"][w, :2] * tangent, 1),
    }
    edges = np.linspace(-np.pi, np.pi, bins + 1)
    k = np.clip(np.digitize(phase, edges) - 1, 0, bins - 1)
    out["phase_deg"] = np.degrees(0.5 * (edges[:-1] + edges[1:])).tolist()
    for name, y in signals.items():
        out[f"{name}_mean"], out[f"{name}_2f"], out[f"{name}_2f_phase_deg"] = _mean_and_2f(phase, y)
        out[f"{name}_by_phase"] = [float(np.nanmean(y[k == i])) if np.any(k == i) else None for i in range(bins)]
    rb, lag = run["robot"], float(np.nanmedian(e["lag"]))
    tr = rb.ts.to_numpy(float)
    wr = _window(tr, seg.t0 + seg.ramp_s + lag, seg.t0 + seg.duration - seg.ramp_s + lag)
    radius = np.linalg.norm(rb[["x", "y"]].to_numpy(float)[wr] - seg.center[:2], axis=1)
    h = run["haptic"]
    out.update({
        "guard_s": float(h.guard_tripped.to_numpy(float).sum() * np.median(np.diff(h.ts.to_numpy(float))))
        if "guard_tripped" in h else 0.0,
        "robot_radius_mean": float(radius.mean()), "robot_radius_half_range": _half_range(radius),
        "accel": float(seg.radius * seg.omega**2),
    })
    return out


# ------------------------------------------------------------------ contact (E3, E5, E6)


def contacts(run, crossings: np.ndarray, window_s: float = 1.5) -> list[dict]:
    """Per contact, from the times the leader crossed the table going down: the felt force at the
    crossing (pre-contact drag) and whether it is already at the force clamp (``saturated``: then
    the contact cannot show as an onset), force-onset latency (the felt force rising ``CONTACT_N``
    above its level just before the crossing), leader penetration at onset, felt overshoot (peak /
    settled), settling time, bounces (leader re-entries), robot peak table force, e_F_z and HF force
    RMS (> 8 Hz) over the contact; and the time to the hold force (``hold_latency_ms``: from the
    crossing until the felt force is within ±20 % of its settled value for good). The onset needs a
    rise; M1 under delay drags more before contact than it feels in it, so for M1 the time to the
    hold force is the one to read. ``window_s`` should cover the leader's time at depth."""
    hs = M.haptic_side(run)
    e = M.unfelt_force(run)
    t, z, fz = hs["t"], hs["x_l"][:, 2], hs["f"][:, 2]
    s = surface_of(run["meta"])
    rb = M.robot_at(run, t)
    a = run["meta"].get("args", {})
    clamp = float((run["meta"].get("haptic_step") or {}).get(
        "max_force_world", a.get("max_i3_force", np.inf) / a.get("force_gain", 1.0)))  # the force clamp [world N]
    out = []
    for tc in crossings:
        pre = _window(t, tc - 0.1, tc - 0.01)  # just before: clear of the approach's speed ramp
        base = float(np.median(fz[pre])) if pre.any() else 0.0
        saturated = abs(base) >= 0.95 * clamp
        # the hold force (the last 0.5 s at depth) and when the felt force reached it for good
        held = _window(t, tc + window_s - 0.5, tc + window_s) & (z < s)
        settled = float(np.median(fz[held])) if held.any() else float("nan")
        band = np.abs(fz - settled) > 0.2 * max(abs(settled), CONTACT_N)  # outside ±20 % of it
        off = np.flatnonzero(_window(t, tc, tc + window_s) & band) if np.isfinite(settled) else []
        hold_latency = float(1e3 * (t[off[-1]] - tc)) if len(off) else 0.0
        after = _window(t, tc - 0.01, tc + window_s)
        on_idx = np.flatnonzero(after & (fz - base > CONTACT_N))
        if not len(on_idx):
            out.append({"t_cross": float(tc), "hold_latency_ms": hold_latency, "felt_settled_N": settled,
                        "felt_at_crossing_N": base, "saturated": saturated, "onset_latency_ms": float("nan")})
            continue
        i0 = on_idx[0]
        inside = _window(t, t[i0], tc + window_s) & (z < s)
        late = np.flatnonzero(inside & band)
        reentries = np.flatnonzero(np.diff((z[inside] < s).astype(int)) == 1) if inside.any() else []
        out.append({
            "t_cross": float(tc),
            "hold_latency_ms": hold_latency,
            "felt_at_crossing_N": base,
            "saturated": saturated,
            "onset_latency_ms": float(1e3 * (t[i0] - tc)),
            "penetration_at_onset_mm": float(1e3 * (s - z[i0])),
            "felt_peak_N": float(np.nanmax(fz[inside])) if inside.any() else float("nan"),
            "felt_settled_N": settled,
            "overshoot": float(np.nanmax(fz[inside]) / settled) if inside.any() and settled > CONTACT_N else float("nan"),
            "settling_ms": float(1e3 * (t[late[-1]] - t[i0])) if len(late) else 0.0,
            "bounces": int(len(reentries)),
            "robot_peak_table_N": float(np.nanmax(-rb["F_osc"][inside, 2])) if inside.any() else float("nan"),
            "eF_z_mean": float(np.nanmean(e["e_F"][inside, 2])) if inside.any() else float("nan"),
            "hf_rms_N": M.hf_rms(fz[inside], t[inside]) if inside.sum() > 30 else float("nan"),
        })
    return out


def _leader_crossings(run) -> np.ndarray:
    hs = M.haptic_side(run)
    z, s = hs["x_l"][:, 2], surface_of(run["meta"])
    i = np.flatnonzero((z[:-1] >= s) & (z[1:] < s))
    t = hs["t"][i + 1]
    keep = np.r_[True, np.diff(t) > 0.3] if len(t) else np.zeros(0, bool)  # one per contact, not per re-entry
    return t[keep]


def _summarize(rows: list[dict]) -> dict:
    keys = [k for k in (rows[0] if rows else {}) if k != "t_cross"]
    out = {"contacts": len(rows)}
    for k in keys:
        v = np.array([r.get(k, np.nan) for r in rows], dtype=float)
        out[f"{k}_mean"] = float(np.nanmean(v)) if np.isfinite(v).any() else float("nan")
        out[f"{k}_std"] = float(np.nanstd(v)) if np.isfinite(v).any() else float("nan")
    out["per_contact"] = rows
    return out


# ------------------------------------------------------------------ E3


def e3(run) -> dict:
    """Contact onset at a constant approach speed: the contact metrics, one per approach, averaged."""
    c = _common(run)
    cfg = run["meta"].get("script") or {}
    # the leader's time at depth after the crossing: the rest of the approach, then the hold
    at_depth = cfg.get("depth", 0.01) / max(cfg.get("approach_speed") or 0.05, 1e-3) + cfg.get("hold_s", 2.0)
    rows = contacts(run, _leader_crossings(run), window_s=at_depth)
    # osc_controller's contact force limiter (run time of each event): a contact is limited if an
    # event falls between its leader crossing and 5 s later (the robot reaches the table T_c later)
    limit = run["meta"].get("contact_limit") or {}
    times = np.asarray(limit.get("times_s", []), dtype=float)
    for r in rows:
        r["contact_limited"] = bool(np.any((times >= r["t_cross"]) & (times <= r["t_cross"] + 5.0)))
    return {"approach_speed": cfg.get("approach_speed"), **_summarize(rows), "E_end_J": c["E_end_J"],
            "E_min_J": c["E_min_J"], "contact_limit_events": limit.get("events")}


# ------------------------------------------------------------------ E4


def _harmonic(t: np.ndarray, y: np.ndarray, w: float) -> complex:
    """``y``'s phasor at ``w`` [rad/s], with an offset and a linear trend (a slow line underneath)."""
    B = np.c_[np.cos(w * t), np.sin(w * t), t - t.mean(), np.ones_like(t)]
    (c, s_, *_), *_ = np.linalg.lstsq(B, y, rcond=None)
    return complex(c, -s_)


def e4(run, bins: int = 20, mass_bins: int = 7) -> dict:
    """Reach sweep: e_F against the position along the line (binned), the model's Λ eigenvalues
    there (proxy-rim runs), and, per position (``mass_bins`` bins), the masses along the dither
    direction n, phase-separated at the dither frequency ω (friction, in phase with the velocity,
    drops out of the in-phase part; X, F are first-harmonic phasors):

    - ``m_robot``: the robot's effective mass, ``−Re(F_osc / x) / ω²`` on the robot's own motion;
    - ``m_felt``: the mass the method renders, ``Re(f / x_p) / ω²`` on the proxy's motion;
    - ``m_err``: the mass the operator was not shown, ``−Re(e_F / x_p) / ω²`` (≈ m_robot − m_felt
      when the robot tracks the proxy);
    - ``tracking``: robot / proxy amplitude, and ``coulomb_N``: the robot's Coulomb friction, ``π Im(F/x)|x|/4``;
    - ``m_model``: nᵀΛn of the RIM model (proxy-rim runs).
    """
    seg = _segment(run, "_Oscillation")
    c = _common(run)
    e = c["e"]
    if seg is None:
        return {"E_end_J": c["E_end_J"]}
    w = _window(e["t"], seg.t0, seg.t0 + seg.duration)
    start, end = seg.base.start, seg.base.end
    line = (end - start) / np.linalg.norm(end - start)
    pos = M.robot_at(run, e["t"])["x"]
    s = (pos[w] - start) @ line
    eF = np.linalg.norm(e["e_F"][w], axis=1)
    length = float(np.linalg.norm(end - start))
    edges = np.linspace(0.0, length, bins + 1)
    k = np.clip(np.digitize(s, edges) - 1, 0, bins - 1)
    out = {"line_length_m": length, "eF_rms": float(M.rms(eF)),
           "s_m": (0.5 * (edges[:-1] + edges[1:])).tolist(),
           "eF_rms_by_s": [float(M.rms(eF[k == i])) if np.any(k == i) else None for i in range(bins)],
           "E_end_J": c["E_end_J"]}
    m = run.get("model")
    lam = None
    if m is not None and len(m) and "M_eff_8" in m:
        lam = m[[f"M_eff_{j}" for j in range(9)]].to_numpy().reshape(-1, 3, 3)
        mt = m.ts.to_numpy()
        ms = (M.robot_at(run, mt)["x"] - start) @ line
        mk = np.clip(np.digitize(ms, edges) - 1, 0, bins - 1)
        eig = np.linalg.eigvalsh(lam)
        inside = (mt >= seg.t0) & (mt <= seg.t0 + seg.duration)
        out["lambda_eig_by_s"] = [np.nanmean(eig[inside & (mk == i)], axis=0).tolist() if np.any(inside & (mk == i)) else None
                                  for i in range(bins)]

    # masses along the dither, per position along the line
    n, om = seg.direction, 2 * np.pi * seg.frequency_hz
    lag = float(np.nanmedian(e["lag"]))
    h, rb = run["haptic"], run["robot"]
    t, tr = e["t"], rb.ts.to_numpy(float)
    x_p = h[[f"proxy_x_{j}" for j in range(3)]].to_numpy(float) @ n if "proxy_x_0" in h else M.haptic_side(run)["x_l"] @ n
    x_r, F_r = rb[["x", "y", "z"]].to_numpy(float), rb[["fx", "fy", "fz"]].to_numpy(float) @ n
    s_r = (x_r - start) @ line
    s_h = (M.robot_at(run, t + lag)["x"] - start) @ line  # where the robot was when each haptic tick's command arrived
    edges = np.linspace(0.0, length, mass_bins + 1)
    rows = []
    for i in range(mass_bins):
        on_r = (s_r >= edges[i]) & (s_r < edges[i + 1]) & _window(tr, seg.t0 + lag, seg.t0 + seg.duration + lag)
        on_h = (s_h >= edges[i]) & (s_h < edges[i + 1]) & _window(t, seg.t0, seg.t0 + seg.duration)
        if on_r.sum() < 2 * 1000 / seg.frequency_hz or on_h.sum() < 2 * 1000 / seg.frequency_hz:  # two cycles at least
            rows.append(None)
            continue
        Xr, Fr = _harmonic(tr[on_r], (x_r @ n)[on_r], om), _harmonic(tr[on_r], F_r[on_r], om)
        Xp = _harmonic(t[on_h], x_p[on_h], om)
        Xt = _harmonic(t[on_h], M.robot_at(run, t[on_h] + lag)["x"] @ n, om)
        row = {"s_m": float(0.5 * (edges[i] + edges[i + 1])),
               "m_robot": float(-(Fr / Xr).real / om**2),
               "m_felt": float((_harmonic(t[on_h], e["f"][on_h] @ n, om) / Xp).real / om**2),
               "m_err": float(-(_harmonic(t[on_h], e["e_F"][on_h] @ n, om) / Xp).real / om**2),
               "tracking": float(abs(Xt / Xp)), "coulomb_N": float(np.pi * (Fr / Xr).imag * abs(Xr) / 4),
               "A_proxy_m": float(abs(Xp))}
        if lam is not None:
            mt = m.ts.to_numpy()
            ms = (M.robot_at(run, mt)["x"] - start) @ line
            sel = (ms >= edges[i]) & (ms < edges[i + 1]) & (mt >= seg.t0) & (mt <= seg.t0 + seg.duration)
            row["m_model"] = float(np.mean(np.einsum("i,kij,j->k", n, lam[sel], n))) if sel.any() else None
        rows.append(row)
    out["mass_by_s"] = rows
    errs = np.array([r["m_err"] for r in rows if r is not None])
    out["m_err_rms"] = float(np.sqrt(np.mean(errs**2))) if len(errs) else float("nan")
    return out


# ------------------------------------------------------------------ E5, E6 (Inverse3)


def e5(run, hf_threshold_n: float) -> dict:
    """One K step: HF force RMS (> 8 Hz) of the felt force while the leader is in contact, and the
    verdict against the threshold fixed before running."""
    hs = M.haptic_side(run)
    z, s = hs["x_l"][:, 2], surface_of(run["meta"])
    inside = z < s
    hf = M.hf_rms(np.linalg.norm(hs["f"][inside], axis=1), hs["t"][inside]) if inside.sum() > 30 else float("nan")
    # the chatter guard zeroes the force when it trips (which also hides the HF content): a trip is unstable
    g = run["haptic"].guard_tripped.to_numpy(float) if "guard_tripped" in run["haptic"] else np.zeros(1)
    trips = int(np.sum(np.diff(g) > 0))
    # P-P under delay goes unstable slowly (~1/(2T)): the HF RMS misses it; a strongly negative energy
    # balance (energy generated by the loop) shows it, for the operator's verdict
    c = _common(run)
    return {"K": float(run["meta"]["args"]["kv"]), "contact_s": float(inside.sum() * np.median(np.diff(hs["t"]))),
            "hf_rms_N": hf, "hf_threshold_N": hf_threshold_n, "guard_trips": trips, "E_min_J": c["E_min_J"],
            "stable": bool(np.isfinite(hf) and hf < hf_threshold_n and trips == 0),
            "stop_reason": run["meta"].get("stop_reason", "")}


def e6(run) -> dict:
    """Taps and holds on the Inverse3: the contact and stability metrics per tap, averaged, and the
    energy balance."""
    c = _common(run)
    rows = contacts(run, _leader_crossings(run), window_s=1.5)
    return {**_summarize(rows), "E_end_J": c["E_end_J"], "E_min_J": c["E_min_J"]}
