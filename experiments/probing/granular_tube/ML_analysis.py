"""ML_analysis.py — predict pinch angle and height from the pressure traces.

Loads every `pinch_100_set_*_hard.pkl` run that has a corresponding entry in
`pinch_analysis.SET_ANGLE_RULES`, builds a per-pinch feature vector from the
baseline-subtracted, median-filtered, stride-downsampled top + bottom pressure
traces, applies the per-set angle corrections, and benchmarks three simple
regression models predicting `(angle_deg, height_mm)` jointly.

Models:
    1. Random Forest         — multi-output natively.
    2. Gradient Boosting     — one regressor per output via MultiOutputRegressor.
    3. SVR (RBF kernel)      — scaled features inside a pipeline.

Notes:
    * Angles are predicted in plain degrees in [0, 180]. There's a soft wrap
      ambiguity at 0°/180° (the cylinder is symmetric under θ → θ + π) — for
      data sets whose corrected angles cluster away from the boundary this is
      harmless; for sets that straddle it, expect inflated MAE near the edges.
    * Each pinch's pressure trace is baseline-subtracted, median-filtered, and
      then linearly interpolated to a fixed `RESAMPLE_LEN` samples — so pinches
      of different durations all map onto the same grid without truncation or
      drops. The full event from each pinch is retained.
"""

import pickle
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from sklearn.ensemble import GradientBoostingRegressor, RandomForestRegressor
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVR

from pinch_analysis import (
    MEDIAN_FILTER_WINDOW,
    RESULTS_DIR,
    SET_ANGLE_RULES,
    Z_OFFSET,
    _median_filter,
    fix_angles,
)

# ===================== Config =====================
DATA_GLOB = "pinch_100_set_*_hard.pkl"
_SET_RE = re.compile(r"pinch_100_set_(\d+)_hard\.pkl$")

# Resample each (filtered, baseline-subtracted) pressure trace to this fixed
# number of samples via linear interpolation. Every pinch contributes its full
# event, mapped onto a common grid regardless of original duration.
RESAMPLE_LEN = 50

# Sets to include. None ⇒ use every set in SET_ANGLE_RULES that exists on disk.
SETS_TO_USE = None

TEST_SIZE = 0.50
RANDOM_STATE = 42

PLOT_OUT = RESULTS_DIR / "ML_predictions.png"


# ===================== Feature extraction =====================
def preprocess_trace(pressure, baseline) -> np.ndarray | None:
    """Baseline-subtract, median-filter, linearly resample to RESAMPLE_LEN.

    Returns None when the input trace has fewer than 2 samples (can't interp).
    """
    p = np.asarray(pressure, dtype=np.float64)
    if p.size < 2:
        return None
    bl = float(np.mean(baseline)) if len(baseline) > 0 else 0.0
    sig = np.asarray(_median_filter(p - bl, MEDIAN_FILTER_WINDOW), dtype=np.float64)
    x_old = np.linspace(0.0, 1.0, sig.size)
    x_new = np.linspace(0.0, 1.0, RESAMPLE_LEN)
    return np.interp(x_new, x_old, sig)


def build_dataset() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (X, Y, groups):
    X       — (N, 2 * RESAMPLE_LEN) per-pinch feature matrix
    Y       — (N, 2) targets [angle_deg, height_mm]
    groups  — (N,) source set index for each sample
    """
    files = sorted(RESULTS_DIR.glob(DATA_GLOB))
    wanted = set(SET_ANGLE_RULES.keys()) if SETS_TO_USE is None else set(SETS_TO_USE)

    feats: list[np.ndarray] = []
    angles: list[float] = []
    heights: list[float] = []
    groups: list[int] = []
    n_skipped = 0

    for path in files:
        m = _SET_RE.search(path.name)
        if not m:
            continue
        set_idx = int(m.group(1))
        if set_idx not in wanted:
            continue
        with open(path, "rb") as f:
            d = pickle.load(f)

        target_angles = fix_angles(d["target_angles"], set_idx)
        target_heights = np.asarray(d["target_heights"], dtype=np.float64) + Z_OFFSET
        n = min(
            len(target_angles),
            len(target_heights),
            len(d["pressures_top"]),
            len(d["pressures_bottom"]),
            len(d["baselines_top"]),
            len(d["baselines_bottom"]),
        )

        before = len(feats)
        for i in range(n):
            pt = preprocess_trace(d["pressures_top"][i], d["baselines_top"][i])
            pb = preprocess_trace(d["pressures_bottom"][i], d["baselines_bottom"][i])
            if pt is None or pb is None:
                n_skipped += 1
                continue
            feats.append(np.concatenate([pt, pb]))
            angles.append(float(np.rad2deg(target_angles[i])))
            heights.append(float(target_heights[i] * 1000.0))  # mm
            groups.append(set_idx)
        print(f"  set {set_idx}: +{len(feats) - before} samples")

    if not feats:
        raise SystemExit(
            f"No usable samples in {RESULTS_DIR}/{DATA_GLOB} for sets "
            f"{sorted(wanted)}."
        )

    X = np.asarray(feats)
    Y = np.column_stack([np.asarray(angles), np.asarray(heights)])
    groups_arr = np.asarray(groups, dtype=int)
    if n_skipped:
        print(f"[warn] skipped {n_skipped} pinches (trace had <2 samples)")
    return X, Y, groups_arr


# ===================== Models =====================
def candidate_models() -> dict:
    return {
        "RandomForest": RandomForestRegressor(
            n_estimators=300, random_state=RANDOM_STATE, n_jobs=-1
        ),
        "GradientBoosting": MultiOutputRegressor(
            GradientBoostingRegressor(random_state=RANDOM_STATE)
        ),
        "SVR-RBF": Pipeline(
            [
                ("scale", StandardScaler()),
                (
                    "svr",
                    MultiOutputRegressor(SVR(kernel="rbf", C=10.0, gamma="scale")),
                ),
            ]
        ),
    }


def evaluate(model, X_train, X_test, Y_train, Y_test) -> tuple[dict, np.ndarray]:
    model.fit(X_train, Y_train)
    yhat = model.predict(X_test)
    metrics = {
        "angle_mae": float(mean_absolute_error(Y_test[:, 0], yhat[:, 0])),
        "angle_r2": float(r2_score(Y_test[:, 0], yhat[:, 0])),
        "height_mae": float(mean_absolute_error(Y_test[:, 1], yhat[:, 1])),
        "height_r2": float(r2_score(Y_test[:, 1], yhat[:, 1])),
    }
    return metrics, yhat


def plot_predictions(Y_test: np.ndarray, predictions: dict, output_path: Path) -> None:
    """True-vs-predicted scatter per (model, target) with y=x and line-of-best-fit.

    Layout: rows = models, columns = (angle, height). Each subplot annotates
    R², MAE, and the fitted slope/intercept.
    """
    target_specs = [("Angle", "deg", 0), ("Height", "mm", 1)]
    n_models = len(predictions)
    fig, axes = plt.subplots(n_models, 2, figsize=(11, 4 * n_models), squeeze=False)
    for row, (name, yhat) in enumerate(predictions.items()):
        for col, (label, unit, idx) in enumerate(target_specs):
            ax = axes[row, col]
            yt = Y_test[:, idx]
            yp = yhat[:, idx]
            ax.scatter(yt, yp, s=22, alpha=0.6, edgecolors="none")

            lo = float(min(yt.min(), yp.min()))
            hi = float(max(yt.max(), yp.max()))
            pad = 0.03 * (hi - lo) if hi > lo else 1.0
            xs = np.array([lo - pad, hi + pad])
            ax.plot(
                xs,
                xs,
                color="k",
                linestyle="--",
                linewidth=0.8,
                alpha=0.6,
                label="y = x",
            )

            slope, intercept = np.polyfit(yt, yp, 1)
            ax.plot(
                xs,
                slope * xs + intercept,
                color="C3",
                linewidth=1.4,
                label=f"fit: y = {slope:.2f}·x + {intercept:.2f}",
            )

            r2 = r2_score(yt, yp)
            mae = mean_absolute_error(yt, yp)
            ax.text(
                0.04,
                0.96,
                f"$R^2$ = {r2:.3f}\nMAE = {mae:.2f} {unit}",
                transform=ax.transAxes,
                va="top",
                ha="left",
                fontsize=10,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.7),
            )
            ax.set_xlabel(f"True {label.lower()} ({unit})")
            ax.set_ylabel(f"Predicted {label.lower()} ({unit})")
            ax.set_title(f"{name} — {label}")
            ax.set_xlim(xs[0], xs[1])
            ax.set_ylim(xs[0], xs[1])
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, linestyle="--", alpha=0.4)
            ax.legend(loc="lower right", fontsize=8)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")


# ===================== Entry point =====================
def main() -> None:
    print(f"Loading dataset from {RESULTS_DIR}/{DATA_GLOB}")
    X, Y, groups = build_dataset()
    print(
        f"\nDataset: X={X.shape}, Y={Y.shape}, "
        f"sets={sorted(set(int(g) for g in groups))}"
    )
    print(f"  angle range : {Y[:, 0].min():6.1f}° → {Y[:, 0].max():6.1f}°")
    print(f"  height range: {Y[:, 1].min():6.1f} mm → {Y[:, 1].max():6.1f} mm")

    X_train, X_test, Y_train, Y_test = train_test_split(
        X, Y, test_size=TEST_SIZE, random_state=RANDOM_STATE
    )
    print(f"  train: n={len(X_train)} | test: n={len(X_test)}")

    print(
        f"\n{'Model':<18}{'angle MAE':>12}{'angle R²':>10}"
        f"{'height MAE':>14}{'height R²':>10}"
    )
    print("-" * 64)
    predictions: dict[str, np.ndarray] = {}
    for name, model in candidate_models().items():
        m, yhat = evaluate(model, X_train, X_test, Y_train, Y_test)
        predictions[name] = yhat
        print(
            f"{name:<18}"
            f"{m['angle_mae']:>10.2f}°"
            f"{m['angle_r2']:>10.3f}"
            f"{m['height_mae']:>11.2f} mm"
            f"{m['height_r2']:>10.3f}"
        )

    plot_predictions(Y_test, predictions, PLOT_OUT)
    plt.show()


if __name__ == "__main__":
    main()
