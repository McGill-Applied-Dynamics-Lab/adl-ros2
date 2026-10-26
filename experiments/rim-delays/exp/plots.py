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


def contact_panel(ax_z, ax_f, run: dict, contact: dict, method: str, window_s: float, before_s: float = 0.5) -> None:
    """One contact of an E3 run, on two axes sharing time from the leader's crossing [s]:
    ``ax_z`` heights above the table [mm] (leader, proxy, robot), ``ax_f`` the felt force and the
    robot's table force ``−F_osc,z`` [N], each at its own side's time (the robot's on the robot
    clock, so the felt force shows its contact a feedback delay later). Drawn on the plot: the
    crossing, the force onset, the time to the hold force and the hold force; a box with the
    contact's metrics (``contact``: one of ``exp_metrics.e3``'s ``per_contact`` rows)."""
    from runs import surface_of

    s = surface_of(run["meta"])
    tc = contact["t_cross"]
    h, rb = run["haptic"], run["robot"]
    th = h.ts.to_numpy(float) - tc
    tr = rb.ts.to_numpy(float) - tc
    wh, wr = (th > -before_s) & (th < window_s + 0.3), (tr > -before_s) & (tr < window_s + 0.3)
    c = color(method)
    ax_z.plot(th[wh], 1e3 * (h.x_l_2.to_numpy(float)[wh] - s), color=c, lw=2, label="leader")
    if "proxy_x_2" in h and np.isfinite(h.proxy_x_2).any():
        ax_z.plot(th[wh], 1e3 * (h.proxy_x_2.to_numpy(float)[wh] - s), color=c, lw=1.5, ls=":", label="proxy")
    ax_z.plot(tr[wr], 1e3 * (rb.z.to_numpy(float)[wr] - s), color=INK_2, lw=1.5, ls="--", label="robot")
    ax_z.axhline(0, color="0.6", lw=1, zorder=0)
    ax_f.plot(th[wh], h.f_2.to_numpy(float)[wh], color=c, lw=2, label="felt f_z")
    ax_f.plot(tr[wr], -rb.fz.to_numpy(float)[wr], color=INK_2, lw=1.5, ls="--", label="robot table force −F_osc,z")
    for ax in (ax_z, ax_f):
        ax.axvline(0, color="0.6", lw=1, zorder=0)
    settled = contact.get("felt_settled_N", np.nan)
    if np.isfinite(settled):
        ax_f.axhline(settled, color=c, lw=1, ls=":", zorder=0)
    hold = contact.get("hold_latency_ms", np.nan)
    if np.isfinite(hold):
        ax_f.axvline(hold / 1e3, color=c, lw=1.5, ls="--")
    onset = contact.get("onset_latency_ms", np.nan)
    if np.isfinite(onset) and onset >= 0:
        k = np.argmin(np.abs(th - onset / 1e3))
        ax_f.plot(onset / 1e3, h.f_2.to_numpy(float)[k], "o", ms=8, color=c, mec="white", mew=1.5)
    lines = [f"drag at crossing {contact.get('felt_at_crossing_N', np.nan):.1f} N",
             f"time to hold force {hold:.0f} ms", f"hold force {settled:.1f} N"]
    if np.isfinite(onset) and onset >= 0:
        lines.append(f"onset {onset:.0f} ms, overshoot {contact.get('overshoot', np.nan):.2f}")
    if np.isfinite(contact.get("robot_peak_table_N", np.nan)):
        lines.append(f"robot peak {contact['robot_peak_table_N']:.1f} N")
    if contact.get("contact_limited"):
        lines.append("contact force limit hit")
    ax_f.text(0.98, 0.97, "\n".join(lines), transform=ax_f.transAxes, ha="right", va="top", fontsize=8,
              bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="0.8", alpha=0.9))
