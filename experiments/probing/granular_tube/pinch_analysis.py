"""Analyze pinch_100_set_*_nylon.pkl runs and produce per-sensor scatter plots.

For every pinch in every loaded set we compute:

    max_top  = max(pressures_top  - mean(baselines_top))
    max_bot  = max(pressures_bottom - mean(baselines_bottom))
    mean_max = (max_top + max_bot) / 2

    signal_top    = (max_top - mean_max) / mean_max     (= -signal_bottom)
    signal_bottom = (max_bot - mean_max) / mean_max

Geometry: an upright cylinder (radius = TUBE_DIAMETER / 2). Each pinch produces
two contacts 180° apart on the tube surface at the pinch height z_cyl. Two
sensors live at the end caps:

    Base (bottom) sensor: azimuth   0°, R_sensor from axis, z = 0
    Top sensor          : azimuth 135°, R_sensor from axis, z = TOP_SENSOR_Z

Pinch heights stored in pinch_main.py are in the gripper frame; we map them
into the cylinder frame with z_cyl = pinch_height + Z_OFFSET.

For each pinch there are 4 distances (2 contacts × 2 sensors). We plot each
sensor's `signal` against the two distances from that sensor to the two
contacts (so each pinch contributes 2 points to each of the two scatters).

Per-set angle correction (target_angles is in radians, raw range ≈ ±π/4):
    The mapping from a set's index to its correction rule lives in
    `SET_ANGLE_RULES` near the top of this file. Available rules:
        "add_90"      — every angle += 90°
        "neg_add_180" — negative angles += 180°; positives unchanged
        "passthrough" — raw values, no correction
    Add new sets to `SET_ANGLE_RULES` as you collect them.
"""

import pickle
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

# ===================== Geometry =====================
# Match pinch_main.py.
TUBE_DIAMETER = 0.040  # m
TUBE_RADIUS = TUBE_DIAMETER / 2.0

# Each end-cap sensor sits 17 mm from the cylinder axis.
SENSOR_RADIUS = 0.017  # m

BASE_SENSOR_ANGLE_DEG = 0.0
BASE_SENSOR_Z = -0.0040
TOP_SENSOR_ANGLE_DEG = 135.0
TOP_SENSOR_Z = 0.1040  # m

# Vertical offset between the gripper-frame z stored in the pickle and the
# cylinder frame used here (z=0 at the base sensor). Tweak as needed.
Z_OFFSET = 0.006  # m

# Fraction of each signal's peak used as the rise-time threshold. With
# RISE_FRACTION = 0.5 we measure time-to-half-peak, the standard choice.
RISE_FRACTION = 0.90


# ===================== File layout =====================
PROJECT_ROOT = Path(__file__).resolve().parent
RESULTS_DIR = PROJECT_ROOT / "results"
DATA_GLOB = "pinch_100_set_*_nylon.pkl"
PLOT_OUT = RESULTS_DIR / "pinch_analysis.png"
PLOT_OUT_RISE = RESULTS_DIR / "pinch_analysis_risetime.png"

_SET_RE = re.compile(r"pinch_100_set_(\d+)_nylon\.pkl$")

# Per-set angle-correction rule. To bring in a new set, just add an entry here
# (or extend ANGLE_RULES if a brand-new correction is needed). No need to
# touch fix_angles for new files of an existing rule type.
#
# Available rules:
#   "add_90"      — every angle += 90°            (raw [-45°, 45°] → [45°, 135°])
#   "neg_add_180" — negative angles += 180°       (raw [-45°, 45°] → [0°, 45°] ∪ [135°, 180°])
#   "passthrough" — leave the raw values as-is
SET_ANGLE_RULES: dict[int, str] = {
    1: "add_90",
    2: "neg_add_180",
    3: "neg_add_180",
    # 3: "add_90",           # add entries for new sets as they are collected
    # 4: "neg_add_180",
}

ANGLE_RULES = {
    "add_90": lambda a: a + np.pi / 2.0,
    "neg_add_180": lambda a: np.where(a >= 0.0, a, a + np.pi),
    "passthrough": lambda a: a,
}


# ===================== Helpers =====================
def fix_angles(angles_rad: np.ndarray, set_idx: int) -> np.ndarray:
    """Map raw target_angles (radians) into the [0, π] cylinder-frame convention.

    The rule for each set is configured in `SET_ANGLE_RULES`. Sets without an
    entry fall back to `passthrough` and emit a warning so they're easy to
    spot in the run log.
    """
    rule = SET_ANGLE_RULES.get(set_idx)
    if rule is None:
        print(
            f"[warn] no entry in SET_ANGLE_RULES for set {set_idx}; using raw "
            f'values. Add `{set_idx}: "add_90"` (or another rule) to fix.'
        )
        rule = "passthrough"
    if rule not in ANGLE_RULES:
        raise ValueError(
            f"Unknown angle rule {rule!r} for set {set_idx}. "
            f"Valid rules: {sorted(ANGLE_RULES)}"
        )
    return ANGLE_RULES[rule](np.asarray(angles_rad, dtype=np.float64))


def sensor_xyz(angle_deg: float, z: float) -> np.ndarray:
    a = np.deg2rad(angle_deg)
    return np.array(
        [SENSOR_RADIUS * np.cos(a), SENSOR_RADIUS * np.sin(a), z], dtype=np.float64
    )


def contacts_xyz(theta_rad: float, z_cyl: float) -> tuple[np.ndarray, np.ndarray]:
    """Two contact points 180° apart on the tube surface at height z_cyl."""
    cx = TUBE_RADIUS * np.cos(theta_rad)
    cy = TUBE_RADIUS * np.sin(theta_rad)
    c1 = np.array([cx, cy, z_cyl], dtype=np.float64)
    c2 = np.array([-cx, -cy, z_cyl], dtype=np.float64)
    return c1, c2


def per_probe_max(pressure, baseline) -> float:
    """max(pressure - mean(baseline)). Returns NaN if no pressure samples."""
    p = np.asarray(pressure, dtype=np.float64)
    b = np.asarray(baseline, dtype=np.float64)
    if p.size == 0:
        return np.nan
    bl_mean = float(np.mean(b)) if b.size > 0 else 0.0
    return float(np.max(p - bl_mean))


def per_probe_rise_time(
    pressure, baseline, timestamps, fraction: float = RISE_FRACTION
) -> float:
    """First-crossing time of `fraction * max(signal)` for the baseline-subtracted signal.

    Timestamps come straight from the pickle (anchored at 0 = first pressure
    sample of the pinch stream, which is roughly the start of the close
    motion). Returns NaN if the signal has no positive deflection or if the
    timestamp array doesn't line up with the pressure array.
    """
    p = np.asarray(pressure, dtype=np.float64)
    b = np.asarray(baseline, dtype=np.float64)
    t = np.asarray(timestamps, dtype=np.float64)
    if p.size == 0 or t.size != p.size:
        return np.nan
    bl_mean = float(np.mean(b)) if b.size > 0 else 0.0
    sig = p - bl_mean
    max_sig = float(np.max(sig))
    if max_sig <= 0.0:
        return np.nan
    threshold = fraction * max_sig
    idx = int(np.argmax(sig >= threshold))  # first index above threshold
    return float(t[idx])


def load_set(path: Path, set_idx: int) -> dict:
    with open(path, "rb") as f:
        d = pickle.load(f)

    angles = fix_angles(d["target_angles"], set_idx)
    heights = np.asarray(d["target_heights"], dtype=np.float64) + Z_OFFSET

    pressure_timestamps = d.get("pressure_timestamps", [])

    # Trim to the shortest list so a partially-aborted run still loads cleanly.
    n = min(
        len(angles),
        len(heights),
        len(d["pressures_top"]),
        len(d["pressures_bottom"]),
        len(d["baselines_top"]),
        len(d["baselines_bottom"]),
        len(pressure_timestamps) if pressure_timestamps else len(angles),
    )
    return {
        "angles": angles[:n],
        "heights": heights[:n],
        "pressures_top": d["pressures_top"][:n],
        "pressures_bottom": d["pressures_bottom"][:n],
        "baselines_top": d["baselines_top"][:n],
        "baselines_bottom": d["baselines_bottom"][:n],
        "pressure_timestamps": pressure_timestamps[:n] if pressure_timestamps else None,
    }


# ===================== Main =====================
def main():
    sensor_base = sensor_xyz(BASE_SENSOR_ANGLE_DEG, BASE_SENSOR_Z)
    sensor_top = sensor_xyz(TOP_SENSOR_ANGLE_DEG, TOP_SENSOR_Z)

    files = sorted(RESULTS_DIR.glob(DATA_GLOB))
    if not files:
        raise SystemExit(f"No data files matched: {RESULTS_DIR}/{DATA_GLOB}")

    sig_top = []
    sig_bot = []
    rise_top = []  # top-sensor rise time minus per-probe mean rise time
    rise_bot = []
    d_top_c1 = []
    d_top_c2 = []
    d_base_c1 = []
    d_base_c2 = []
    set_tag = []  # for the peak-signal scatter (matches sig_*)
    rise_set_tag = []  # parallel tag for the rise-time scatter (subset of probes)
    rise_d_top_c1 = []  # distances paired with rise-time samples (subset of probes)
    rise_d_top_c2 = []
    rise_d_base_c1 = []
    rise_d_base_c2 = []

    for path in files:
        m = _SET_RE.search(path.name)
        if not m:
            continue
        set_idx = int(m.group(1))
        print(f"Loading {path.name} (set {set_idx})")

        s = load_set(path, set_idx)
        ts_list = s["pressure_timestamps"]
        for i in range(len(s["angles"])):
            mt = per_probe_max(s["pressures_top"][i], s["baselines_top"][i])
            mb = per_probe_max(s["pressures_bottom"][i], s["baselines_bottom"][i])
            if not (np.isfinite(mt) and np.isfinite(mb)):
                continue
            mean_max = 0.5 * (mt + mb)
            if mean_max == 0.0:
                continue  # avoid divide-by-zero in the normalized signal

            theta = float(s["angles"][i])
            z_cyl = float(s["heights"][i])
            c1, c2 = contacts_xyz(theta, z_cyl)

            dtc1 = float(np.linalg.norm(sensor_top - c1))
            dtc2 = float(np.linalg.norm(sensor_top - c2))
            dbc1 = float(np.linalg.norm(sensor_base - c1))
            dbc2 = float(np.linalg.norm(sensor_base - c2))

            sig_top.append((mt - mean_max) / mean_max)
            sig_bot.append((mb - mean_max) / mean_max)
            d_top_c1.append(dtc1)
            d_top_c2.append(dtc2)
            d_base_c1.append(dbc1)
            d_base_c2.append(dbc2)
            set_tag.append(set_idx)

            # Rise time (only computed if pressure_timestamps are present and
            # both signals show a positive deflection above baseline).
            if ts_list is None:
                continue
            t_arr = ts_list[i]
            rt = per_probe_rise_time(
                s["pressures_top"][i], s["baselines_top"][i], t_arr
            )
            rb = per_probe_rise_time(
                s["pressures_bottom"][i], s["baselines_bottom"][i], t_arr
            )
            if not (np.isfinite(rt) and np.isfinite(rb)):
                continue
            mean_rise = 0.5 * (rt + rb)
            rise_top.append(rt - mean_rise)
            rise_bot.append(rb - mean_rise)
            rise_d_top_c1.append(dtc1)
            rise_d_top_c2.append(dtc2)
            rise_d_base_c1.append(dbc1)
            rise_d_base_c2.append(dbc2)
            rise_set_tag.append(set_idx)

    if not sig_top:
        raise SystemExit("No valid probes found across loaded files.")

    sig_top = np.asarray(sig_top)
    sig_bot = np.asarray(sig_bot)
    d_top_c1 = np.asarray(d_top_c1)
    d_top_c2 = np.asarray(d_top_c2)
    d_base_c1 = np.asarray(d_base_c1)
    d_base_c2 = np.asarray(d_base_c2)
    set_tag = np.asarray(set_tag)

    rise_top = np.asarray(rise_top)
    rise_bot = np.asarray(rise_bot)
    rise_d_top_c1 = np.asarray(rise_d_top_c1)
    rise_d_top_c2 = np.asarray(rise_d_top_c2)
    rise_d_base_c1 = np.asarray(rise_d_base_c1)
    rise_d_base_c2 = np.asarray(rise_d_base_c2)
    rise_set_tag = np.asarray(rise_set_tag)

    # Two points per probe per sensor (contact 1 and contact 2). Stack so each
    # scatter is just (distance, signal); sensor-specific signal is repeated
    # because both contacts share the same pressure trace.
    d_top_all = np.concatenate([d_top_c1, d_top_c2]) * 1000.0  # mm
    d_base_all = np.concatenate([d_base_c1, d_base_c2]) * 1000.0  # mm
    sig_top_all = np.concatenate([sig_top, sig_top])
    sig_bot_all = np.concatenate([sig_bot, sig_bot])
    set_tag_all = np.concatenate([set_tag, set_tag])

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    set_colors = {1: "tab:blue", 2: "tab:orange"}
    for set_idx in np.unique(set_tag_all):
        m = set_tag_all == set_idx
        c = set_colors.get(int(set_idx), "tab:gray")
        axes[0].scatter(
            d_top_all[m],
            sig_top_all[m],
            s=18,
            alpha=0.6,
            color=c,
            label=f"set {int(set_idx)}",
        )
        axes[1].scatter(
            d_base_all[m],
            sig_bot_all[m],
            s=18,
            alpha=0.6,
            color=c,
            label=f"set {int(set_idx)}",
        )

    axes[0].axhline(0, color="k", linewidth=0.7, alpha=0.5)
    axes[0].set_xlabel("Distance (mm) — Top sensor → contact")
    axes[0].set_ylabel(
        "(max(top) − mean(max(top), max(bottom))) / mean(max(top), max(bottom))"
    )
    axes[0].set_title(f"Top sensor (n = {len(sig_top_all)})")
    axes[0].grid(True, linestyle="--", alpha=0.5)
    axes[0].legend(loc="best")

    axes[1].axhline(0, color="k", linewidth=0.7, alpha=0.5)
    axes[1].set_xlabel("Distance (mm) — Bottom sensor → contact")
    axes[1].set_ylabel(
        "(max(bottom) − mean(max(top), max(bottom))) / mean(max(top), max(bottom))"
    )
    axes[1].set_title(f"Bottom sensor (n = {len(sig_bot_all)})")
    axes[1].grid(True, linestyle="--", alpha=0.5)
    axes[1].legend(loc="best")

    plt.tight_layout()
    PLOT_OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(PLOT_OUT, dpi=150)
    print(f"Saved: {PLOT_OUT}")

    # ----- Rise-time figure (parallel layout, same per-set coloring) -----
    if rise_top.size == 0:
        print(
            "[warn] no rise-time samples (no pressure_timestamps in the loaded "
            "pickles, or no probes had a positive deflection); skipping "
            "rise-time figure."
        )
    else:
        rise_d_top_all = np.concatenate([rise_d_top_c1, rise_d_top_c2]) * 1000.0
        rise_d_base_all = np.concatenate([rise_d_base_c1, rise_d_base_c2]) * 1000.0
        rise_top_all = np.concatenate([rise_top, rise_top])
        rise_bot_all = np.concatenate([rise_bot, rise_bot])
        rise_set_tag_all = np.concatenate([rise_set_tag, rise_set_tag])

        fig2, axes2 = plt.subplots(1, 2, figsize=(13, 5))
        for set_idx in np.unique(rise_set_tag_all):
            m = rise_set_tag_all == set_idx
            c = set_colors.get(int(set_idx), "tab:gray")
            axes2[0].scatter(
                rise_d_top_all[m],
                rise_top_all[m] * 1000.0,
                s=18,
                alpha=0.6,
                color=c,
                label=f"set {int(set_idx)}",
            )
            axes2[1].scatter(
                rise_d_base_all[m],
                rise_bot_all[m] * 1000.0,
                s=18,
                alpha=0.6,
                color=c,
                label=f"set {int(set_idx)}",
            )

        pct = int(round(RISE_FRACTION * 100))
        axes2[0].axhline(0, color="k", linewidth=0.7, alpha=0.5)
        axes2[0].set_xlabel("Distance (mm) — Top sensor → contact")
        axes2[0].set_ylabel(
            f"t_top − mean(t_top, t_bottom)  [ms]  (rise to {pct}% of peak)"
        )
        axes2[0].set_title(f"Top sensor rise-time deviation (n = {len(rise_top_all)})")
        axes2[0].grid(True, linestyle="--", alpha=0.5)
        axes2[0].legend(loc="best")

        axes2[1].axhline(0, color="k", linewidth=0.7, alpha=0.5)
        axes2[1].set_xlabel("Distance (mm) — Bottom sensor → contact")
        axes2[1].set_ylabel(
            f"t_bottom − mean(t_top, t_bottom)  [ms]  (rise to {pct}% of peak)"
        )
        axes2[1].set_title(
            f"Bottom sensor rise-time deviation (n = {len(rise_bot_all)})"
        )
        axes2[1].grid(True, linestyle="--", alpha=0.5)
        axes2[1].legend(loc="best")

        plt.tight_layout()
        fig2.savefig(PLOT_OUT_RISE, dpi=150)
        print(f"Saved: {PLOT_OUT_RISE}")

    plt.show()


if __name__ == "__main__":
    main()
