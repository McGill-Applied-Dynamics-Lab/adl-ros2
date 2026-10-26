"""Plot helpers shared by the experiment notebooks: the delay axis, the method colours, a run's
full metrics (lists included)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runs import COLORS, INK_2, LABELS, save  # noqa: E402,F401

DELAYS = ["0", "5g", "850", "2600"]  # round trips, in plot order
METHOD_ORDER = ["zoh", "proxy-god-object", "proxy-fixed-mass", "proxy-rim-free", "proxy-rim"]
LABELS = {**LABELS, "zoh": "M1 P-P (ZOH)", "proxy-god-object": "M3 God-object", "proxy-fixed-mass": "M4 Fixed mass",
          "proxy-rim": "M5 RIM (orientation held)", "proxy-rim-free": "M5 RIM (rotation free)"}
COLORS = {**COLORS, "proxy-rim-free": "#008300"}  # categorical slot 6 (slots 1-5 are the other methods)


def delay_key(rtt) -> str:
    s = str(rtt).lower()
    return s if s == "5g" else f"{int(round(float(s)))}"


def delay_axis(ax, present) -> dict:
    """Categorical x axis for the delays present, in order; returns {delay: x}."""
    order = [d for d in DELAYS if d in set(present)] + sorted(set(present) - set(DELAYS))
    ax.set_xticks(range(len(order)))
    ax.set_xticklabels(["5G" if d == "5g" else f"{d} ms" for d in order])
    ax.set_xlabel("round-trip delay T")
    return {d: i for i, d in enumerate(order)}


def methods_in(df) -> list[str]:
    present = list(dict.fromkeys(df.method))
    return [m for m in METHOD_ORDER if m in present] + [m for m in present if m not in METHOD_ORDER]


def color(method: str) -> str:
    return COLORS.get(method, INK_2)


def metrics_of(run_dir) -> dict:
    """A run's ``analysis/metrics.json`` (lists included)."""
    return json.loads((Path(run_dir) / "analysis" / "metrics.json").read_text())


def by_delay(ax, df, y: str, label_methods: bool = True):
    """``y`` against the delay, one line per method (mean over repeats, min-max as a band)."""
    df = df.assign(delay=df.rtt.map(delay_key))
    xs = delay_axis(ax, df.delay)
    for m in methods_in(df):
        d = df[df.method == m].groupby("delay")[y].agg(["mean", "min", "max"])
        d = d.loc[[k for k in xs if k in d.index]]
        x = [xs[k] for k in d.index]
        ax.plot(x, d["mean"], "o-", color=color(m), label=LABELS.get(m, m) if label_methods else None)
        ax.fill_between(x, d["min"], d["max"], color=color(m), alpha=0.15, lw=0)
    return xs
