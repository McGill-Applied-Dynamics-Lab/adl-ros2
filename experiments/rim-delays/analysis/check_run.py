r"""Sanity check of a saved run (robot or sim): is it complete, healthy and aligned?

Prints the run's configuration, every stream's size and rate, the haptic loop's health (watchdog,
guard, period, plant age), the measured haptic → robot lag against the configured command delay,
e_F and the energy balance (`metrics.py`), and Λ₀ at the start joints; then a list of warnings.

Saves the report and the run's figures (`run_plots.py`) next to its data, to compare runs:
``<run dir>/analysis/check.md`` (the report, the figures embedded) and ``<run dir>/analysis/*.png``.

    pixi run -e humble python experiments/rim-delays/analysis/check_run.py <run dir> [<run dir> ...]
    pixi run -e humble python experiments/rim-delays/analysis/check_run.py <run dir> --show-plot   # also open the figures
    pixi run -e humble python experiments/rim-delays/analysis/check_run.py <run dir> --no-lambda   # skip the Pinocchio model
"""

from __future__ import annotations

import argparse
import datetime
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "tools"))

import matplotlib.pyplot as plt  # noqa: E402
import metrics as M  # noqa: E402
import run_plots as P  # noqa: E402
from runs import find, load_run  # noqa: E402

HAPTIC_HZ_MIN = 990.0  # below this the haptic loop missed ticks
PLANT_HZ_MIN = 900.0  # robot side: every controller tick should be recorded (1 kHz)
LAG_TOLERANCE_MS = 5.0  # measured lag minus configured command delay, median
FIGURES = {
    "positions": "Position",
    "forces": "Force: felt vs robot, and e_F",
    "timing": "Timing",
    "energy_and_model": "Energy and model",
}  # run_plots function -> title in check.md


def rate(ts) -> float:
    """Mean rate of a timestamp column [Hz]."""
    ts = np.asarray(ts, dtype=float)
    return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else float("nan")


class Report:
    """The report's lines, printed and written to ``check.md`` the same way."""

    def __init__(self) -> None:
        self.lines: list[str] = []

    def head(self, text: str) -> None:
        self.lines.append(f"  {text}")

    def line(self, label: str, value: str) -> None:
        self.lines.append(f"  {label:<28} {value}")


def configured_command_delay_ms(meta: dict) -> float | None:
    """The command link's constant delay [ms]; ``None`` for a variable one (jitter or trace)."""
    c = (meta.get("delays") or {}).get("command")
    if c is None:
        return 0.0
    if c.get("jitter_ms", 0) or c.get("trace"):
        return None
    return float(c.get("base_ms", 0.0))


def lambda0(run: dict, use_model: bool) -> np.ndarray | None:
    """Λ at the run start: the first logged RIM model, else Pinocchio at ``q0``."""
    m = run.get("model")
    if m is not None and len(m) and "M_eff_8" in m:
        return m[[f"M_eff_{k}" for k in range(9)]].to_numpy()[0].reshape(3, 3)
    q0 = run["meta"].get("q0")
    if not use_model or q0 is None:
        return None
    from lambda_map import Inertia

    return Inertia(find(run["meta"], "tool_tip_offset")).at(q0)["Lambda"]


def check(run_dir: Path, use_model: bool, run: dict | None = None) -> tuple[list[str], list[str]]:
    """The report for one run, as ``(lines, warnings)``. ``run``: the loaded run, if at hand."""
    report = Report()
    line = report.line
    warn: list[str] = []
    run = load_run(run_dir) if run is None else run
    if run is None:
        return [f"  {run_dir}: incomplete (no metadata.json or samples.mcap)"], [f"{run_dir.name}: incomplete"]
    meta, h = run["meta"], run["haptic"]
    np.set_printoptions(precision=3, suppress=True)

    # -- configuration
    delays = meta.get("delays") or {}
    def delay_text(link):
        d = delays.get(link)
        if d is None:
            return "none"
        text = f"{d.get('base_ms', 0):g} ms"
        if d.get("jitter_ms"):
            text += f" ± {d['jitter_ms']:g} ({d.get('jitter_dist')})"
        if d.get("trace"):
            text += f" + trace (median {d['trace']['median_ms']:.1f} ms, {d.get('trace_mode')})"
        return text + f", seed {d.get('seed')}"

    line("backend", "sim" if meta.get("sim") else "robot")
    line("method", f"{find(meta, 'method')}  (K {find(meta, 'kv')} N/m, D {find(meta, 'dv')} N·s/m)")
    line("delay feedback / command", f"{delay_text('feedback')} / {delay_text('command')}")
    line("script", str((meta.get("script") or {}).get("profile", "none (Inverse3)")) + ("  + hand" if meta.get("hand") else ""))
    line("stop reason", str(meta.get("stop_reason", "")))
    if meta.get("q0") is not None:
        line("q0 [rad]", str(np.round(meta["q0"], 3).tolist()))
    params = meta.get("controller_params") or {}
    if params:
        gains = {k.split(".", 1)[1]: v for k, v in params.items() if k.startswith("gains.") and ("pos" in k or "ratio" in k)}
        line("OSC gains (read back)", ", ".join(f"{k} {v:g}" for k, v in gains.items()))
    elif not meta.get("sim"):
        warn.append("no controller_params in the metadata (run recorded before they were logged)")

    # -- streams
    report.head("streams:")
    streams = {"haptic": h, "robot": run["robot"], "command": run["command"], "target": run["target"], "model": run["model"]}
    for name, df in streams.items():
        if df is None or not len(df):
            line(f"  {name}", "missing")
            continue
        line(f"  {name}", f"{len(df):7d} rows, {rate(df.ts):7.1f} Hz, {df.ts.iloc[0]:6.2f} → {df.ts.iloc[-1]:6.2f} s")
    if h is None or not len(h):
        warn.append("no haptic log")
        return report.lines, warn
    for name in ("target",):
        if streams[name] is None:
            warn.append(f"no {name} stream (run recorded before it was logged): no e_F")
    if find(meta, "method") == "proxy-rim" and run["model"] is None:
        warn.append("proxy-rim without a model stream")
    robot_hz = rate(run["robot"].ts) if run["robot"] is not None else float("nan")
    r = run["robot"]
    if r is not None and "t_ctrl" in r and not meta.get("sim"):
        stamp = r.t_ctrl.to_numpy()
        order = np.argsort(stamp)
        missing = int(np.sum(np.maximum(np.round(np.diff(stamp[order]) * 1e3) - 1, 0)))
        lost = missing / (len(stamp) + missing)
        latency = 1e3 * (r.ts.to_numpy() - stamp)  # receive time minus controller stamp, up to a clock offset
        latency -= latency.min()
        line("  controller ticks lost", f"{100 * lost:.1f} %")
        line("  receive latency p50/p99/max", f"{np.median(latency):.1f} / {np.percentile(latency, 99):.1f} / "
             f"{latency.max():.1f} ms (above the fastest sample)")
    new_format = h is not None and "mono_s" in h  # older runs recorded only the decimated plant samples
    if not meta.get("sim") and new_format and not robot_hz >= PLANT_HZ_MIN:
        warn.append(f"robot recorded at {robot_hz:.0f} Hz (< {PLANT_HZ_MIN:g}): controller ticks lost")

    # -- haptic loop health
    report.head("haptic loop:")
    if "mono_s" in h:
        period_ms = 1e3 * np.diff(h.mono_s.to_numpy())
        line("  period p50 / p99 / max", f"{np.median(period_ms):.2f} / {np.percentile(period_ms, 99):.2f} / {period_ms.max():.2f} ms")
        hz = 1e3 / np.mean(period_ms)
        if hz < HAPTIC_HZ_MIN:
            warn.append(f"haptic loop at {hz:.0f} Hz")
    else:
        warn.append("no mono_s in the haptic log (run recorded before it was logged): haptic clock approximate")
    age = 1e3 * h.plant_age_s.to_numpy()
    age = age[np.isfinite(age)]
    if len(age):
        line("  plant age p50 / p99 / max", f"{np.median(age):.1f} / {np.percentile(age, 99):.1f} / {age.max():.1f} ms")
    wd = h.watchdog_gain.to_numpy()
    after_ramp = wd[int(0.5 * len(wd)) :]  # the watchdog ramps in at the start
    line("  watchdog gain min", f"{np.nanmin(after_ramp):.2f} (second half of the run)")
    if np.nanmin(after_ramp) < 1.0:
        warn.append("watchdog faded the force out during the run (stale plant samples)")
    trips = int(np.sum(np.diff(h.guard_tripped.to_numpy()) > 0))
    line("  guard trips", str(trips))
    if trips:
        warn.append(f"{trips} guard trip(s)")
    if "watchdog" in str(meta.get("stop_reason", "")):
        warn.append(f"stopped by the watchdog: {meta['stop_reason']}")
    limit = meta.get("contact_limit") or {}
    line("  contact force limit", "not logged" if not limit else
         f"{limit['events']} event(s)" + (f" at {', '.join(f'{t:.1f}' for t in limit['times_s'])} s" if limit["events"] else ""))
    if limit.get("events"):
        warn.append(f"contact force limit reached {limit['events']} time(s): the robot's force was capped")

    if run["target"] is None or run["robot"] is None:
        return report.lines, warn

    # -- alignment: when each haptic tick's action reached the robot
    e = M.unfelt_force(run)
    lag = 1e3 * e["lag"][np.isfinite(e["lag"])]
    report.head("haptic → robot:")
    if len(lag):
        line("  measured lag p1 / p50 / p99", f"{np.percentile(lag, 1):.1f} / {np.median(lag):.1f} / {np.percentile(lag, 99):.1f} ms")
        expected = configured_command_delay_ms(meta)
        period = 1e3 / float((meta.get("rates") or {}).get("command_hz") or 1000.0)
        if expected is not None:
            line("  configured command delay", f"{expected:g} ms (+ up to {period:.1f} ms command period)")
            if abs(np.median(lag) - expected - period / 2) > LAG_TOLERANCE_MS + period / 2:
                warn.append(f"median lag {np.median(lag):.1f} ms vs configured {expected:g} ms")
    else:
        warn.append("no haptic tick reached the robot")

    # -- metrics
    report.head("metrics:")
    axes = "xyz" if e["e_F"].shape[1] == 3 else [f"i{k}" for k in range(e["e_F"].shape[1])]
    line("  e_F RMS per axis", "  ".join(f"{a} {v:.2f}" for a, v in zip(axes, M.rms(e["e_F"]))) + " N")
    line("  felt |f| RMS / max", f"{M.rms(np.linalg.norm(e['f'], axis=1)):.2f} / {np.nanmax(np.linalg.norm(e['f'], axis=1)):.2f} N")
    line("  robot |F_osc| max", f"{np.nanmax(np.linalg.norm(e['F_osc'], axis=1)):.2f} N")
    _, E = M.energy_balance(run)
    line("  energy E end / min", f"{E[-1]:+.3f} / {E.min():+.3f} J")

    # -- task inertia at the start
    lam = lambda0(run, use_model)
    if lam is not None:
        w, v = np.linalg.eigh(lam)
        line("Λ₀ eigenvalues", f"{w} kg (heaviest {v[:, -1]}, lightest {v[:, 0]})")
    return report.lines, warn


def verdict(warnings: list[str]) -> list[str]:
    return ["  OK"] if not warnings else ["  WARNINGS:"] + [f"    - {w}" for w in warnings]


def save_figures(run: dict, out: Path, show: bool) -> list[str]:
    """Save every figure of the run as ``<out>/<name>.png``; returns the names saved. A figure that
    fails (a stream missing) is skipped with a note."""
    saved = []
    for name in FIGURES:
        try:
            fig = getattr(P, name)(run)
        except Exception as e:  # noqa: BLE001 - one missing stream must not lose the other figures
            print(f"[check_run] {name}: not plotted ({e!r})")
            continue
        fig.savefig(out / f"{name}.png", dpi=150, bbox_inches="tight")
        saved.append(name)
        if not show:
            plt.close(fig)
    return saved


def write_markdown(path: Path, run_dir: Path, lines: list[str], warnings: list[str], figures: list[str]) -> None:
    text = [
        f"# Check: {run_dir.name}",
        "",
        f"`{run_dir}` — checked {datetime.datetime.now():%Y-%m-%d %H:%M}",
        "",
        "**OK**" if not warnings else "**Warnings**\n\n" + "\n".join(f"- {w}" for w in warnings),
        "",
        "## Report",
        "",
        "```text",
        *[ln[2:] if ln.startswith("  ") else ln for ln in lines],
        "```",
    ]
    for name in figures:
        text += ["", f"## {FIGURES[name]}", "", f"![{name}]({name}.png)"]
    path.write_text("\n".join(text) + "\n")


def use_gui_backend() -> None:
    """``runs.py`` selects the non-interactive Agg backend; switch to a GUI one to show figures."""
    for backend in ("QtAgg", "TkAgg"):
        try:
            plt.switch_backend(backend)
            return
        except Exception:  # noqa: BLE001 - try the next one
            continue
    print("[check_run] no GUI backend (Qt, Tk): figures are saved, not shown")


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("runs", type=Path, nargs="+")
    p.add_argument("--no-lambda", action="store_true", help="do not compute Λ₀ from q0 (no Pinocchio model)")
    p.add_argument("--show-plot", action="store_true", help="also open the figures (they are saved either way)")
    args = p.parse_args(argv)
    if args.show_plot:
        use_gui_backend()
    for run_dir in args.runs:
        run_dir = run_dir.expanduser()
        run = load_run(run_dir)
        lines, warnings = check(run_dir, use_model=not args.no_lambda, run=run)
        print(f"\n[check_run] {run_dir}")
        print("\n".join(lines + verdict(warnings)))
        if run is None:
            continue
        out = run_dir / "analysis"
        out.mkdir(exist_ok=True)
        figures = save_figures(run, out, args.show_plot)
        write_markdown(out / "check.md", run_dir, lines, warnings, figures)
        print(f"[check_run] saved {out / 'check.md'} and {len(figures)} figures")
    if args.show_plot:
        plt.show()


if __name__ == "__main__":
    main()
