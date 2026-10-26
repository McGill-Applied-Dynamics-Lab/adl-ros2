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
    tangent = np.stack([-inward[:, 1], inward[:, 0]], 1) * np.sign(seg.omega)  # direction of motion
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
        "angle_f_deg": float(np.nanmean(angle_to_inward(-e["f"][w, :2]))),  # felt: the proxy pulls the hand inward
        "angle_Fosc_deg": float(np.nanmean(angle_to_inward(e["F_osc"][w, :2]))),
        "phase_deg": np.degrees(0.5 * (edges[:-1] + edges[1:])).tolist(),
        "eF_radial_by_phase": per_bin(radial), "eF_tangential_by_phase": per_bin(tangential),
        "E_end_J": c["E_end_J"],
    }


# ------------------------------------------------------------------ contact (E3, E5, E6)


def contacts(run, crossings: np.ndarray, window_s: float = 1.5) -> list[dict]:
    """Per contact, from the times the leader crossed the table going down: the felt force at the
    crossing (pre-contact drag) and whether it is already at the force clamp (``saturated``: then
    the contact cannot show as an onset), force-onset latency (the felt force rising ``CONTACT_N``
    above its level just before the crossing), leader penetration at onset, felt overshoot (peak /
    settled), settling time, bounces (leader re-entries), robot peak table force, e_F_z and HF force
    RMS (> 8 Hz) over the contact."""
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
        after = _window(t, tc - 0.01, tc + window_s)
        on_idx = np.flatnonzero(after & (fz - base > CONTACT_N))
        if not len(on_idx):
            out.append({"t_cross": float(tc), "felt_at_crossing_N": base, "saturated": saturated,
                        "onset_latency_ms": float("nan")})
            continue
        i0 = on_idx[0]
        inside = _window(t, t[i0], tc + window_s) & (z < s)
        settled = float(np.median(fz[_window(t, tc + window_s - 0.5, tc + window_s) & (z < s)])) if inside.any() else float("nan")
        band = np.abs(fz - settled) > 0.2 * max(abs(settled), CONTACT_N)
        late = np.flatnonzero(inside & band)
        reentries = np.flatnonzero(np.diff((z[inside] < s).astype(int)) == 1) if inside.any() else []
        out.append({
            "t_cross": float(tc),
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
    rows = contacts(run, _leader_crossings(run), window_s=1.5)
    cfg = run["meta"].get("script") or {}
    return {"approach_speed": cfg.get("approach_speed"), **_summarize(rows), "E_end_J": c["E_end_J"],
            "E_min_J": c["E_min_J"]}


# ------------------------------------------------------------------ E4


def e4(run, bins: int = 20) -> dict:
    """Reach sweep: e_F against the position along the line (binned), and the model's Λ eigenvalues
    there (proxy-rim runs)."""
    seg = _segment(run, "_Oscillation")
    c = _common(run)
    e = c["e"]
    if seg is None:
        return {"E_end_J": c["E_end_J"]}
    w = _window(e["t"], seg.t0, seg.t0 + seg.duration)
    start, end = seg.base.start, seg.base.end
    n = (end - start) / np.linalg.norm(end - start)
    pos = M.robot_at(run, e["t"])["x"]
    s = (pos[w] - start) @ n
    eF = np.linalg.norm(e["e_F"][w], axis=1)
    edges = np.linspace(0.0, float(np.linalg.norm(end - start)), bins + 1)
    k = np.clip(np.digitize(s, edges) - 1, 0, bins - 1)
    out = {"line_length_m": float(edges[-1]), "eF_rms": float(M.rms(eF)),
           "s_m": (0.5 * (edges[:-1] + edges[1:])).tolist(),
           "eF_rms_by_s": [float(M.rms(eF[k == i])) if np.any(k == i) else None for i in range(bins)],
           "E_end_J": c["E_end_J"]}
    m = run.get("model")
    if m is not None and len(m) and "M_eff_8" in m:
        lam = m[[f"M_eff_{j}" for j in range(9)]].to_numpy().reshape(-1, 3, 3)
        mt = m.ts.to_numpy()
        mpos = M.robot_at(run, mt)["x"]
        ms = (mpos - start) @ n
        mk = np.clip(np.digitize(ms, edges) - 1, 0, bins - 1)
        eig = np.linalg.eigvalsh(lam)
        inside = (mt >= seg.t0) & (mt <= seg.t0 + seg.duration)
        out["lambda_eig_by_s"] = [np.nanmean(eig[inside & (mk == i)], axis=0).tolist() if np.any(inside & (mk == i)) else None
                                  for i in range(bins)]
    return out


# ------------------------------------------------------------------ E5, E6 (Inverse3)


def e5(run, hf_threshold_n: float) -> dict:
    """One K step: HF force RMS (> 8 Hz) of the felt force while the leader is in contact, and the
    verdict against the threshold fixed before running."""
    hs = M.haptic_side(run)
    z, s = hs["x_l"][:, 2], surface_of(run["meta"])
    inside = z < s
    hf = M.hf_rms(np.linalg.norm(hs["f"][inside], axis=1), hs["t"][inside]) if inside.sum() > 30 else float("nan")
    return {"K": float(run["meta"]["args"]["kv"]), "contact_s": float(inside.sum() * np.median(np.diff(hs["t"]))),
            "hf_rms_N": hf, "hf_threshold_N": hf_threshold_n, "stable": bool(np.isfinite(hf) and hf < hf_threshold_n),
            "stop_reason": run["meta"].get("stop_reason", "")}


def e6(run) -> dict:
    """Taps and holds on the Inverse3: the contact and stability metrics per tap, averaged, and the
    energy balance."""
    c = _common(run)
    rows = contacts(run, _leader_crossings(run), window_s=1.5)
    return {**_summarize(rows), "E_end_J": c["E_end_J"], "E_min_J": c["E_min_J"]}
