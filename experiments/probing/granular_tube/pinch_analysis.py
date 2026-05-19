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
from scipy.signal import medfilt
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

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
RISE_FRACTION = 0.85

# Optional outlier clipping for the rise-time figure: keep only points whose
# normalized rise-time deviation falls within ±N·std of the per-axis mean.
# Set to None to disable. Applied independently to the top- and bottom-sensor
# scatters.
RISE_STD_CUTOFF = 3

# Median-filter window (in samples) applied to the baseline-subtracted signal
# *before* feature extraction (max, rise time). Set to 1 (or anything <= 1) to
# disable filtering. Must be odd; an even value is bumped up by 1.
# At ~10 kHz sample rate, 51 samples ≈ 5 ms — robust to single-sample spikes
# without smearing the close/open transients.
MEDIAN_FILTER_WINDOW = 91


# ===================== File layout =====================
PROJECT_ROOT = Path(__file__).resolve().parent
RESULTS_DIR = PROJECT_ROOT / "results"
DATA_GLOB = "pinch_100_set_*_hard.pkl"
PLOT_OUT = RESULTS_DIR / "pinch_analysis.png"
PLOT_OUT_VS_HEIGHT = RESULTS_DIR / "pinch_analysis_vs_height.png"
PLOT_OUT_VS_ANGLE = RESULTS_DIR / "pinch_analysis_vs_angle.png"
PLOT_OUT_DIFF_VS_ANGLE = RESULTS_DIR / "pinch_analysis_diff_vs_angle.png"

# Restrict which set indices get loaded into the scatter. Set to None to plot
# every file that matches DATA_GLOB. Examples:
#     SETS_TO_PLOT = None        # plot all sets found on disk
#     SETS_TO_PLOT = [1, 2]      # only sets 1 and 2
#     SETS_TO_PLOT = [3]         # only set 3
SETS_TO_PLOT: list[int] | None = None
SETS_TO_PLOT = [1, 2, 3, 4, 5]

_SET_RE = re.compile(r"pinch_100_set_(\d+)_hard\.pkl$")

# Per-set angle-correction rule. To bring in a new set, just add an entry here
# (or extend ANGLE_RULES if a brand-new correction is needed). No need to
# touch fix_angles for new files of an existing rule type.
#
# Available rules:
#   "add_90"      — every angle += 90°            (raw [-45°, 45°] → [45°, 135°])
#   "neg_add_180" — negative angles += 180°       (raw [-45°, 45°] → [0°, 45°] ∪ [135°, 180°])
#   "passthrough" — leave the raw values as-is
SET_ANGLE_RULES: dict[int, str] = {
    # 1: "add_90",
    # 2: "neg_add_180",
    # 3: "neg_add_180",
    # 4: "add_90",
    # 5: "add_90",
    1: "add_90",  # add entries for new sets as they are collected
    2: "add_90",
    3: "add_90",
    4: "add_90",
    5: "neg_add_180",
}

ANGLE_RULES = {
    "add_90": lambda a: a + np.pi / 2.0,
    "neg_add_180": lambda a: np.where(a >= 0.0, a, a + np.pi),
    "passthrough": lambda a: a,
}

# Per-set colormap. Within a set, the markers are shaded by the probe's index
# in the file: pinch 1 = lightest end of the colormap, last pinch = darkest.
# All markers in one set share the same hue. Sequential matplotlib colormaps
# work best ("Blues", "Oranges", "Greens", "Reds", "Purples", "Greys"...).
SET_CMAPS: dict[int, str] = {
    1: "Blues",
    2: "Oranges",
    3: "Greens",
    4: "Greys",
    5: "Purples",
}
DEFAULT_CMAP = "Greys"
# Trim the colormap range to avoid invisible-light and pitch-black extremes.
COLOR_RANGE = (0.30, 0.95)


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


def _median_filter(signal: np.ndarray, window: int) -> np.ndarray:
    """1-D median filter; returns input unchanged for window <= 1 or undersized signals.

    `medfilt` requires an odd kernel size; we silently round even values up.
    """
    if window is None or window <= 1:
        return signal
    if window % 2 == 0:
        window += 1
    if signal.size < window:
        return signal
    return medfilt(signal, kernel_size=window)


def per_probe_max(pressure, baseline) -> float:
    """max(median_filter(pressure - mean(baseline))). Returns NaN if no pressure samples."""
    p = np.asarray(pressure, dtype=np.float64)
    b = np.asarray(baseline, dtype=np.float64)
    if p.size == 0:
        return np.nan
    bl_mean = float(np.mean(b)) if b.size > 0 else 0.0
    sig = _median_filter(p - bl_mean, MEDIAN_FILTER_WINDOW)
    return float(np.max(sig))


def per_probe_rise_time(
    pressure, baseline, timestamps, fraction: float = RISE_FRACTION
) -> float:
    """First-crossing time of `fraction * max(signal)` for the baseline-subtracted,
    median-filtered signal.

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
    sig = _median_filter(p - bl_mean, MEDIAN_FILTER_WINDOW)
    max_sig = float(np.max(sig))
    if max_sig <= 0.0:
        return np.nan
    threshold = fraction * max_sig
    idx = int(np.argmax(sig >= threshold))  # first index above threshold
    return float(t[idx])


def _diff_trace(pressure_top, baseline_top, pressure_bot, baseline_bot) -> np.ndarray:
    """Baseline-subtracted, median-filtered (top − bottom) pressure trace.
    Returns an empty array if the two inputs cannot be paired sample-by-sample.
    """
    pt = np.asarray(pressure_top, dtype=np.float64)
    pb = np.asarray(pressure_bot, dtype=np.float64)
    if pt.size == 0 or pt.size != pb.size:
        return np.empty(0, dtype=np.float64)
    bt_mean = float(np.mean(baseline_top)) if len(baseline_top) > 0 else 0.0
    bb_mean = float(np.mean(baseline_bot)) if len(baseline_bot) > 0 else 0.0
    return _median_filter((pt - bt_mean) - (pb - bb_mean), MEDIAN_FILTER_WINDOW)


def per_probe_diff_max(pressure_top, baseline_top, pressure_bot, baseline_bot) -> float:
    """Signed peak of the (top − bottom) trace. Positive ⇒ top read higher
    than bottom at the most extreme sample; negative ⇒ vice versa.
    """
    diff = _diff_trace(pressure_top, baseline_top, pressure_bot, baseline_bot)
    if diff.size == 0:
        return np.nan
    return float(diff[int(np.argmax(np.abs(diff)))])


def per_probe_diff_rise_time(
    pressure_top,
    baseline_top,
    pressure_bot,
    baseline_bot,
    timestamps,
    fraction: float = RISE_FRACTION,
) -> float:
    """First-crossing time of `fraction · signed_peak` for the (top − bottom)
    trace. Sign of the threshold follows the sign of the peak.
    """
    diff = _diff_trace(pressure_top, baseline_top, pressure_bot, baseline_bot)
    t = np.asarray(timestamps, dtype=np.float64)
    if diff.size == 0 or t.size != diff.size:
        return np.nan
    peak = float(diff[int(np.argmax(np.abs(diff)))])
    if peak == 0.0:
        return np.nan
    threshold = fraction * peak
    crossed = diff >= threshold if peak > 0 else diff <= threshold
    return float(t[int(np.argmax(crossed))])


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

    # Optional: restrict to a chosen subset of set indices.
    if SETS_TO_PLOT is not None:
        wanted = {int(s) for s in SETS_TO_PLOT}
        kept = []
        for path in files:
            m = _SET_RE.search(path.name)
            if m and int(m.group(1)) in wanted:
                kept.append(path)
        if not kept:
            available = sorted(
                int(_SET_RE.search(p.name).group(1))
                for p in files
                if _SET_RE.search(p.name)
            )
            raise SystemExit(
                f"SETS_TO_PLOT = {SETS_TO_PLOT} matched no files. "
                f"Available set indices on disk: {available}"
            )
        files = kept
        print(f"SETS_TO_PLOT filter active; using sets {sorted(wanted)}")

    sig_top = []
    sig_bot = []
    rise_top = []  # top-sensor rise time minus per-probe mean rise time
    rise_bot = []
    d_top_c1 = []
    d_top_c2 = []
    d_base_c1 = []
    d_base_c2 = []
    set_tag = []  # for the peak-signal scatter (matches sig_*)
    set_pos = []  # probe's index within its source set (for cmap shading)
    theta_pinch = []  # per-pinch angle (rad), paired with sig_top/sig_bot
    height_pinch = (
        []
    )  # per-pinch cylinder-frame height (m), paired with sig_top/sig_bot
    rise_set_tag = []  # parallel tag for the rise-time scatter (subset of probes)
    rise_set_pos = []  # parallel position within set for the rise-time scatter
    rise_d_top_c1 = []  # distances paired with rise-time samples (subset of probes)
    rise_d_top_c2 = []
    rise_d_base_c1 = []
    rise_d_base_c2 = []
    rise_theta_pinch = []  # per-pinch angle (rad), paired with rise_top/rise_bot
    rise_height_pinch = []  # per-pinch height (m), paired with rise_top/rise_bot
    diff_peak = []  # signed peak of (top − bottom) trace, one per pinch
    diff_theta = []  # per-pinch angle (rad), paired with diff_peak
    diff_set_tag = []
    diff_set_pos = []
    diff_rise = []  # rise time of (top − bottom) trace to fraction·signed-peak
    diff_rise_theta = []  # paired angles for diff_rise
    diff_rise_set_tag = []
    diff_rise_set_pos = []

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
            set_pos.append(i)
            theta_pinch.append(theta)
            height_pinch.append(z_cyl)

            dpk = per_probe_diff_max(
                s["pressures_top"][i],
                s["baselines_top"][i],
                s["pressures_bottom"][i],
                s["baselines_bottom"][i],
            )
            if np.isfinite(dpk):
                diff_peak.append(dpk)
                diff_theta.append(theta)
                diff_set_tag.append(set_idx)
                diff_set_pos.append(i)

            # Rise time (only computed if pressure_timestamps are present and
            # both signals show a positive deflection above baseline).
            if ts_list is None:
                continue
            t_arr = ts_list[i]
            drt = per_probe_diff_rise_time(
                s["pressures_top"][i],
                s["baselines_top"][i],
                s["pressures_bottom"][i],
                s["baselines_bottom"][i],
                t_arr,
            )
            if np.isfinite(drt):
                diff_rise.append(drt)
                diff_rise_theta.append(theta)
                diff_rise_set_tag.append(set_idx)
                diff_rise_set_pos.append(i)
            rt = per_probe_rise_time(
                s["pressures_top"][i], s["baselines_top"][i], t_arr
            )
            rb = per_probe_rise_time(
                s["pressures_bottom"][i], s["baselines_bottom"][i], t_arr
            )
            if not (np.isfinite(rt) and np.isfinite(rb)):
                continue
            mean_rise = 0.5 * (rt + rb)
            if mean_rise == 0.0:
                continue  # avoid divide-by-zero in the normalized rise time
            rise_top.append((rt - mean_rise) / mean_rise)
            rise_bot.append((rb - mean_rise) / mean_rise)
            rise_d_top_c1.append(dtc1)
            rise_d_top_c2.append(dtc2)
            rise_d_base_c1.append(dbc1)
            rise_d_base_c2.append(dbc2)
            rise_set_tag.append(set_idx)
            rise_set_pos.append(i)
            rise_theta_pinch.append(theta)
            rise_height_pinch.append(z_cyl)

    if not sig_top:
        raise SystemExit("No valid probes found across loaded files.")

    sig_top = np.asarray(sig_top)
    sig_bot = np.asarray(sig_bot)
    d_top_c1 = np.asarray(d_top_c1)
    d_top_c2 = np.asarray(d_top_c2)
    d_base_c1 = np.asarray(d_base_c1)
    d_base_c2 = np.asarray(d_base_c2)
    set_tag = np.asarray(set_tag)
    set_pos = np.asarray(set_pos)
    theta_pinch = np.asarray(theta_pinch)
    height_pinch = np.asarray(height_pinch)

    rise_top = np.asarray(rise_top)
    rise_bot = np.asarray(rise_bot)
    rise_d_top_c1 = np.asarray(rise_d_top_c1)
    rise_d_top_c2 = np.asarray(rise_d_top_c2)
    rise_d_base_c1 = np.asarray(rise_d_base_c1)
    rise_d_base_c2 = np.asarray(rise_d_base_c2)
    rise_set_tag = np.asarray(rise_set_tag)
    rise_set_pos = np.asarray(rise_set_pos)
    rise_theta_pinch = np.asarray(rise_theta_pinch)
    rise_height_pinch = np.asarray(rise_height_pinch)
    diff_peak = np.asarray(diff_peak)
    diff_theta = np.asarray(diff_theta)
    diff_set_tag = np.asarray(diff_set_tag)
    diff_set_pos = np.asarray(diff_set_pos)
    diff_rise = np.asarray(diff_rise)
    diff_rise_theta = np.asarray(diff_rise_theta)
    diff_rise_set_tag = np.asarray(diff_rise_set_tag)
    diff_rise_set_pos = np.asarray(diff_rise_set_pos)

    def std_mask(values: np.ndarray, cutoff: float | None) -> np.ndarray:
        """Per-axis ±cutoff·σ keep-mask. Falls through to a no-op when the
        cutoff is None, the sample is too small, or the std is zero."""
        keep = np.ones(values.shape, dtype=bool)
        if cutoff is None or values.size < 2:
            return keep
        std = float(np.std(values))
        if std == 0.0:
            return keep
        mean = float(np.mean(values))
        return np.abs(values - mean) <= cutoff * std

    def colors_for_set(set_idx: int, positions: np.ndarray) -> np.ndarray:
        """Map per-probe positions within `set_idx` to RGBA colors via SET_CMAPS.
        First probe → light end of the cmap; last probe → dark end.
        """
        cmap = plt.get_cmap(SET_CMAPS.get(int(set_idx), DEFAULT_CMAP))
        if positions.size == 0:
            return np.empty((0, 4))
        pmax = float(positions.max())
        if pmax == 0:
            t = np.full(positions.shape, 0.5, dtype=np.float64)
        else:
            t = positions.astype(np.float64) / pmax
        lo, hi = COLOR_RANGE
        return cmap(lo + (hi - lo) * t)

    def scatter_with_outline(ax, x, y, tags, positions, is_c2, *, keep=None):
        """Per-set colored scatter with c2 (θ + π) contacts outlined in black."""
        if keep is None:
            keep = np.ones(tags.shape, dtype=bool)
        handles = []
        for set_idx in np.unique(tags):
            m = (tags == set_idx) & keep
            for sub_mask, edge, lw in (
                (m & ~is_c2, "none", 0.0),
                (m & is_c2, "black", 0.6),
            ):
                if not sub_mask.any():
                    continue
                ax.scatter(
                    x[sub_mask],
                    y[sub_mask],
                    s=18,
                    alpha=0.6,
                    c=colors_for_set(int(set_idx), positions[sub_mask]),
                    edgecolors=edge,
                    linewidths=lw,
                )
            cmap = plt.get_cmap(SET_CMAPS.get(int(set_idx), DEFAULT_CMAP))
            handles.append(
                Patch(
                    color=cmap(COLOR_RANGE[1]),
                    label=f"set {int(set_idx)} (light → dark = pinch order)",
                )
            )
        return handles

    # Two points per probe per sensor (contact 1 and contact 2). Stack so each
    # scatter is just (distance, signal); sensor-specific signal is repeated
    # because both contacts share the same pressure trace. After fix_angles
    # puts the pinch angle θ in [0, π], c1 sits at azimuth θ and c2 at θ + π,
    # so c2 is always the larger-angle contact and we outline it in black.
    d_top_all = np.concatenate([d_top_c1, d_top_c2]) * 1000.0  # mm
    d_base_all = np.concatenate([d_base_c1, d_base_c2]) * 1000.0  # mm
    sig_top_all = np.concatenate([sig_top, sig_top])
    sig_bot_all = np.concatenate([sig_bot, sig_bot])
    set_tag_all = np.concatenate([set_tag, set_tag])
    set_pos_all = np.concatenate([set_pos, set_pos])
    is_c2_all = np.concatenate(
        [np.zeros(len(sig_top), dtype=bool), np.ones(len(sig_top), dtype=bool)]
    )

    pct = int(round(RISE_FRACTION * 100))
    cutoff_note = f"  [±{RISE_STD_CUTOFF}σ clip]" if RISE_STD_CUTOFF is not None else ""

    # ----- Combined distance figure: peak signal (top row) + rise time (bottom row)
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    handles_dist = scatter_with_outline(
        axes[0, 0], d_top_all, sig_top_all, set_tag_all, set_pos_all, is_c2_all
    )
    scatter_with_outline(
        axes[0, 1], d_base_all, sig_bot_all, set_tag_all, set_pos_all, is_c2_all
    )

    if rise_top.size > 0:
        rise_d_top_all = np.concatenate([rise_d_top_c1, rise_d_top_c2]) * 1000.0
        rise_d_base_all = np.concatenate([rise_d_base_c1, rise_d_base_c2]) * 1000.0
        rise_top_all = np.concatenate([rise_top, rise_top])
        rise_bot_all = np.concatenate([rise_bot, rise_bot])
        rise_set_tag_all = np.concatenate([rise_set_tag, rise_set_tag])
        rise_set_pos_all = np.concatenate([rise_set_pos, rise_set_pos])
        rise_is_c2_all = np.concatenate(
            [np.zeros(len(rise_top), dtype=bool), np.ones(len(rise_top), dtype=bool)]
        )
        top_keep = std_mask(rise_top_all, RISE_STD_CUTOFF)
        bot_keep = std_mask(rise_bot_all, RISE_STD_CUTOFF)
        if RISE_STD_CUTOFF is not None:
            print(
                f"RISE_STD_CUTOFF = {RISE_STD_CUTOFF}: dropped "
                f"{int((~top_keep).sum())}/{len(top_keep)} top and "
                f"{int((~bot_keep).sum())}/{len(bot_keep)} bottom points"
            )
        scatter_with_outline(
            axes[1, 0],
            rise_d_top_all,
            rise_top_all,
            rise_set_tag_all,
            rise_set_pos_all,
            rise_is_c2_all,
            keep=top_keep,
        )
        scatter_with_outline(
            axes[1, 1],
            rise_d_base_all,
            rise_bot_all,
            rise_set_tag_all,
            rise_set_pos_all,
            rise_is_c2_all,
            keep=bot_keep,
        )
        n_top_keep = int(top_keep.sum())
        n_bot_keep = int(bot_keep.sum())
    else:
        print(
            "[warn] no rise-time samples (no pressure_timestamps in the loaded "
            "pickles, or no probes had a positive deflection); rise-time row "
            "will be empty."
        )
        n_top_keep = 0
        n_bot_keep = 0

    handles_dist.append(
        Line2D(
            [0],
            [0],
            marker="o",
            linestyle="none",
            markerfacecolor="lightgray",
            markeredgecolor="black",
            markeredgewidth=0.6,
            markersize=6,
            label=r"outlined: contact at $\theta + \pi$",
        )
    )

    rise_top_ylabel = (
        r"$\left(t_{\mathrm{top}} - \overline{t}\right) / \overline{t}$"
        f"   (rise to {pct}% of peak)"
    )
    rise_bot_ylabel = (
        r"$\left(t_{\mathrm{bot}} - \overline{t}\right) / \overline{t}$"
        f"   (rise to {pct}% of peak)"
    )
    subplot_specs = [
        (
            axes[0, 0],
            "Distance (mm) — Top sensor → contact",
            r"$\left(p^{\max}_{\mathrm{top}} - \overline{p^{\max}}\right) / \overline{p^{\max}}$",
            f"Top peak signal (n = {len(sig_top_all)})",
        ),
        (
            axes[0, 1],
            "Distance (mm) — Bottom sensor → contact",
            r"$\left(p^{\max}_{\mathrm{bot}} - \overline{p^{\max}}\right) / \overline{p^{\max}}$",
            f"Bottom peak signal (n = {len(sig_bot_all)})",
        ),
        (
            axes[1, 0],
            "Distance (mm) — Top sensor → contact",
            rise_top_ylabel,
            f"Top rise-time deviation (n = {n_top_keep}){cutoff_note}",
        ),
        (
            axes[1, 1],
            "Distance (mm) — Bottom sensor → contact",
            rise_bot_ylabel,
            f"Bottom rise-time deviation (n = {n_bot_keep}){cutoff_note}",
        ),
    ]
    for ax, xl, yl, title in subplot_specs:
        ax.axhline(0, color="k", linewidth=0.7, alpha=0.5)
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.grid(True, linestyle="--", alpha=0.5)
    axes[0, 0].legend(handles=handles_dist, loc="best", fontsize=8)

    plt.tight_layout()
    PLOT_OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(PLOT_OUT, dpi=150)
    print(f"Saved: {PLOT_OUT}")

    # ----- Pressure/time vs height and vs angle (2×2 grids, one point per pinch) -----
    def scatter_by_set(ax, x, y, tags, positions, *, keep=None):
        """Per-set colored scatter; returns the legend handles produced."""
        handles = []
        for set_idx in np.unique(tags):
            sm = tags == set_idx
            if keep is not None:
                sm = sm & keep
            if not sm.any():
                continue
            ax.scatter(
                x[sm],
                y[sm],
                s=18,
                alpha=0.6,
                c=colors_for_set(int(set_idx), positions[sm]),
            )
            cmap = plt.get_cmap(SET_CMAPS.get(int(set_idx), DEFAULT_CMAP))
            handles.append(
                Patch(
                    color=cmap(COLOR_RANGE[1]),
                    label=f"set {int(set_idx)} (light → dark = pinch order)",
                )
            )
        return handles

    rise_top_keep_1d = std_mask(rise_top, RISE_STD_CUTOFF)
    rise_bot_keep_1d = std_mask(rise_bot, RISE_STD_CUTOFF)
    sig_top_label = r"$\left(p^{\max}_{\mathrm{top}} - \overline{p^{\max}}\right) / \overline{p^{\max}}$"
    sig_bot_label = r"$\left(p^{\max}_{\mathrm{bot}} - \overline{p^{\max}}\right) / \overline{p^{\max}}$"
    rise_top_label = rise_top_ylabel
    rise_bot_label = rise_bot_ylabel

    def finalize_subplot(ax, title, xlabel, ylabel, handles):
        ax.axhline(0, color="k", linewidth=0.7, alpha=0.5)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle="--", alpha=0.5)
        if handles:
            ax.legend(handles=handles, loc="best", fontsize=8)

    # ---- vs height ----
    heights_mm = height_pinch * 1000.0
    rise_heights_mm = rise_height_pinch * 1000.0

    fig3, axes3 = plt.subplots(2, 2, figsize=(13, 9))
    h_top = scatter_by_set(axes3[0, 0], heights_mm, sig_top, set_tag, set_pos)
    scatter_by_set(axes3[0, 1], heights_mm, sig_bot, set_tag, set_pos)
    scatter_by_set(
        axes3[1, 0],
        rise_heights_mm,
        rise_top,
        rise_set_tag,
        rise_set_pos,
        keep=rise_top_keep_1d,
    )
    scatter_by_set(
        axes3[1, 1],
        rise_heights_mm,
        rise_bot,
        rise_set_tag,
        rise_set_pos,
        keep=rise_bot_keep_1d,
    )

    n_top_h = int(rise_top_keep_1d.sum())
    n_bot_h = int(rise_bot_keep_1d.sum())
    finalize_subplot(
        axes3[0, 0],
        f"Top peak signal (n = {len(sig_top)})",
        "Pinch height z (mm)",
        sig_top_label,
        h_top,
    )
    finalize_subplot(
        axes3[0, 1],
        f"Bottom peak signal (n = {len(sig_bot)})",
        "Pinch height z (mm)",
        sig_bot_label,
        [],
    )
    finalize_subplot(
        axes3[1, 0],
        f"Top rise-time deviation (n = {n_top_h}){cutoff_note}",
        "Pinch height z (mm)",
        rise_top_label,
        [],
    )
    finalize_subplot(
        axes3[1, 1],
        f"Bottom rise-time deviation (n = {n_bot_h}){cutoff_note}",
        "Pinch height z (mm)",
        rise_bot_label,
        [],
    )
    plt.tight_layout()
    fig3.savefig(PLOT_OUT_VS_HEIGHT, dpi=150)
    print(f"Saved: {PLOT_OUT_VS_HEIGHT}")

    # ---- vs angle ----
    angles_deg = np.rad2deg(theta_pinch)
    rise_angles_deg = np.rad2deg(rise_theta_pinch)

    fig4, axes4 = plt.subplots(2, 2, figsize=(13, 9))
    h_top_a = scatter_by_set(axes4[0, 0], angles_deg, sig_top, set_tag, set_pos)
    scatter_by_set(axes4[0, 1], angles_deg, sig_bot, set_tag, set_pos)
    scatter_by_set(
        axes4[1, 0],
        rise_angles_deg,
        rise_top,
        rise_set_tag,
        rise_set_pos,
        keep=rise_top_keep_1d,
    )
    scatter_by_set(
        axes4[1, 1],
        rise_angles_deg,
        rise_bot,
        rise_set_tag,
        rise_set_pos,
        keep=rise_bot_keep_1d,
    )

    finalize_subplot(
        axes4[0, 0],
        f"Top peak signal (n = {len(sig_top)})",
        r"Pinch angle $\theta$ (deg)",
        sig_top_label,
        h_top_a,
    )
    finalize_subplot(
        axes4[0, 1],
        f"Bottom peak signal (n = {len(sig_bot)})",
        r"Pinch angle $\theta$ (deg)",
        sig_bot_label,
        [],
    )
    finalize_subplot(
        axes4[1, 0],
        f"Top rise-time deviation (n = {n_top_h}){cutoff_note}",
        r"Pinch angle $\theta$ (deg)",
        rise_top_label,
        [],
    )
    finalize_subplot(
        axes4[1, 1],
        f"Bottom rise-time deviation (n = {n_bot_h}){cutoff_note}",
        r"Pinch angle $\theta$ (deg)",
        rise_bot_label,
        [],
    )
    plt.tight_layout()
    fig4.savefig(PLOT_OUT_VS_ANGLE, dpi=150)
    print(f"Saved: {PLOT_OUT_VS_ANGLE}")

    # ----- Difference signal (top − bottom) vs angle -----
    if diff_peak.size == 0:
        print("[warn] no difference-signal samples; skipping diff-vs-angle figure.")
    else:
        diff_angles_deg = np.rad2deg(diff_theta)
        diff_rise_angles_deg = np.rad2deg(diff_rise_theta)
        diff_rise_keep = std_mask(diff_rise, RISE_STD_CUTOFF)
        if RISE_STD_CUTOFF is not None and diff_rise.size > 0:
            print(
                f"diff rise-time clip: dropped "
                f"{int((~diff_rise_keep).sum())}/{len(diff_rise_keep)} points"
            )

        fig5, axes5 = plt.subplots(1, 2, figsize=(13, 5))
        h_diff = scatter_by_set(
            axes5[0], diff_angles_deg, diff_peak, diff_set_tag, diff_set_pos
        )
        if diff_rise.size > 0:
            scatter_by_set(
                axes5[1],
                diff_rise_angles_deg,
                diff_rise * 1000.0,
                diff_rise_set_tag,
                diff_rise_set_pos,
                keep=diff_rise_keep,
            )

        finalize_subplot(
            axes5[0],
            f"(Top − Bottom) peak signal (n = {len(diff_peak)})",
            r"Pinch angle $\theta$ (deg)",
            r"$\max\,|p_{\mathrm{top}} - p_{\mathrm{bot}}|$  (signed)",
            h_diff,
        )
        finalize_subplot(
            axes5[1],
            f"(Top − Bottom) rise time (n = {int(diff_rise_keep.sum())}){cutoff_note}",
            r"Pinch angle $\theta$ (deg)",
            f"Time to {pct}% of signed peak  (ms)",
            [],
        )
        plt.tight_layout()
        fig5.savefig(PLOT_OUT_DIFF_VS_ANGLE, dpi=150)
        print(f"Saved: {PLOT_OUT_DIFF_VS_ANGLE}")

    plt.show()


if __name__ == "__main__":
    main()
