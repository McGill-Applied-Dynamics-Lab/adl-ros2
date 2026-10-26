"""Shared runner for the experiment scripts (``e1_*.py`` … ``e6_*.py``).

An experiment script sets its parameters at the top and calls [run_experiment][] with a list of
[Condition][]s (method, round-trip delay, extra ``fr3_teleop`` flags). For each condition this:

1. asks before running on the robot (Enter: run, ``s``: skip, ``q``: quit; ``--yes`` does not ask);
2. runs ``fr3_teleop`` (``--sim``: ``sim_teleop.py``) with ``configs/teleop.yaml``, the experiment's
   base flags, the condition's flags and its delay; the run is saved as
   ``<OUT>/<robot|sim>/<exp>/<YYYYmmdd_HHMMSS>_<exp>_<condition>``, so nothing is ever overwritten;
3. analyses it: ``analysis/check_run.py`` (``<run>/analysis/check.md`` + figures) and the
   experiment's own metrics (``<run>/analysis/metrics.json``);
4. appends the run to the session file ``<OUT>/<robot|sim>/<exp>/sessions/<start time>.txt``, the
   list the experiment's notebook reads.

Delays are round trips (``rtt_ms``), split evenly between the links, with one seed per delay
condition so every method sees the same delay sequence. ``"5g"`` replays the lab's 5G trace.

Common flags of every script: ``--sim``, ``--dry-run`` (print the commands), ``--yes``,
``--only-method m …``, ``--only-rtt r …``, ``--analyse <run dir> …`` (re-run the analysis only).
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RIM = HERE.parent
sys.path[:0] = [str(RIM), str(RIM / "analysis"), str(RIM / "tools")]
REPO = RIM.parents[1]
CONF = REPO / "python/fr3_haptic/configs/teleop.yaml"
OUT = Path("~/data/franka/rim-delays").expanduser()
TRACE_RTT_MS = {"5g": 45.0}  # rough mean round trip of a replayed trace, for the run length [ms]
# Frozen feedback rates (tools/pilot_fm.py on pilots/fm_circle2): robot state 500 Hz (M1 ripple
# K|v|/f_m 0.72 N), RIM model 25 Hz (Λ change 2.1 % per update, 0.12 N RMS from holding); targets
# at f_t = f_h = 1 kHz. Every run gets them.
RATES = {"model_update_hz": 500.0, "rim_update_hz": 25.0, "command_hz": 1000.0}


@dataclass
class Condition:
    """One run: a method, a round-trip delay [ms] (or ``"5g"``), extra ``fr3_teleop`` flags (keys
    are flag names without the dashes, ``True`` for a bare flag), and a short label for the name."""

    method: str
    rtt: float | str = 0.0
    flags: dict = field(default_factory=dict)
    label: str = ""

    def name(self, exp: str) -> str:
        parts = [exp, self.method, f"rtt{rtt_label(self.rtt)}"] + ([self.label] if self.label else [])
        return "_".join(parts)


def rtt_label(rtt) -> str:
    return str(rtt).lower() if str(rtt).lower() in TRACE_RTT_MS else f"{int(round(float(rtt)))}"


def grid(methods, rtts, variants=({},), labels=None) -> list[Condition]:
    """Every method × delay × variant (a variant is a dict of extra flags, ``labels`` names them)."""
    labels = labels or [""] * len(variants)
    return [Condition(m, r, dict(v), lab) for v, lab in zip(variants, labels) for m in methods for r in rtts]


def delay_flags(rtt) -> dict:
    """The delay flags for a round trip, with one fixed seed per delay condition."""
    label = rtt_label(rtt)
    seed = sum(ord(c) for c in label)  # same for every method at this delay
    if label in TRACE_RTT_MS:
        return {"feedback_delay_ms": 0, "command_delay_ms": 0, "delay_trace": label, "delay_trace_mode": "sequential",
                "delay_seed": seed}
    half = float(rtt) / 2
    return {"feedback_delay_ms": half, "command_delay_ms": half, "delay_seed": seed}


def to_argv(flags: dict) -> list[str]:
    """``{"interface_axis": "xyz", "force": True, "start_position": [0.4, 0, 0.2]}`` → CLI arguments."""
    argv = []
    for key, value in flags.items():
        flag = "--" + key.replace("_", "-")
        if value is None or value is False:
            continue
        if value is True:
            argv.append(flag)
        elif isinstance(value, (list, tuple, np.ndarray)):
            argv += [flag, *(f"{float(v):g}" for v in value)]
        else:
            argv += [flag, str(value)]
    return argv


def script_seconds(flags: dict) -> float | None:
    """How long the scripted leader of these flags takes [s] (``None`` for the Inverse3)."""
    if flags.get("device") != "scripted":
        return None
    from fr3_haptic.teleop import build_parser, script_config
    from fr3_haptic.scripted import Script

    args = build_parser().parse_args(to_argv({k: v for k, v in flags.items() if k.startswith("script") or k in (
        "device", "contact_surface")}))
    origin = np.asarray(flags.get("start_position", (0.4, 0.0, 0.2)), dtype=float)
    return Script(script_config(args), origin).duration_s


# ------------------------------------------------------------------ the model at the start


def rim_holds_orientation() -> bool:
    """``rim_hold_orientation`` from ``configs/teleop.yaml``: whether M5 reduces with the end
    effector's orientation held (then M4's mass and E1's directions come from the held Λ too)."""
    import yaml

    def find(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == "rim_hold_orientation":
                    return v
                got = find(v)
                if got is not None:
                    return got
        return None

    return bool(find(yaml.safe_load(CONF.read_text())))


def start_inertia(start_position, hold_orientation: bool | None = None) -> dict:
    """RIM's model at the joints ``fr3_teleop --home --start-position`` reaches (IK from home, as the
    robot's move does): ``Lambda`` (rotation free), ``Lambda_held`` (orientation held),
    ``Lambda_rim`` (the one M5 uses: ``hold_orientation``, default ``rim_hold_orientation`` in
    teleop.yaml) and its eigen-decomposition (``eigvals``, ``eigvecs``)."""
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    from lambda_map import Inertia, line_joints

    if hold_orientation is None:
        hold_orientation = rim_holds_orientation()
    inertia = Inertia()
    q = line_joints(inertia, np.asarray(inertia.fr3.home_config, float), list(start_position), 40)[-1]
    at = inertia.at(q)
    at["Lambda_rim"] = at["Lambda_held"] if hold_orientation else at["Lambda"]
    w, v = np.linalg.eigh(at["Lambda_rim"])
    v = v * np.sign(v[np.argmax(np.abs(v), axis=0), range(3)])  # each eigenvector's largest component > 0
    return {**at, "q": q, "eigvals": w, "eigvecs": v, "hold_orientation": bool(hold_orientation)}


def mass_along(lam: np.ndarray, n) -> float:
    """``nᵀΛn`` for a unit ``n``: M4's best-case mass along ``n``."""
    n = np.asarray(n, dtype=float)
    n = n / np.linalg.norm(n)
    return float(n @ lam @ n)


# ------------------------------------------------------------------ running


def parse(description: str, argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--sim", action="store_true", help="simulated FR3 (sim_teleop.py)")
    p.add_argument("--dry-run", action="store_true", help="print the commands, run nothing")
    p.add_argument("--yes", action="store_true", help="do not ask before each robot run")
    p.add_argument("--only-method", nargs="+", default=None)
    p.add_argument("--only-rtt", nargs="+", default=None, help="round trips [ms], or 5g")
    p.add_argument("--analyse", type=Path, nargs="+", default=None, help="re-run the analysis of these runs only")
    return p.parse_args(argv)


def analyse_run(run_dir: Path, analyse) -> dict:
    """``check_run`` (report + figures in ``<run>/analysis``) and the experiment's ``analyse(run)``
    metrics, saved as ``<run>/analysis/metrics.json``; returns the metrics."""
    from check_run import check, save_figures, verdict, write_markdown
    from runs import load_run

    run = load_run(run_dir)
    if run is None:
        print(f"[exp] {run_dir}: no complete run")
        return {}
    lines, warnings = check(run_dir, use_model=False, run=run)
    print("\n".join(lines + verdict(warnings)))
    out = run_dir / "analysis"
    out.mkdir(exist_ok=True)
    write_markdown(out / "check.md", run_dir, lines, warnings, save_figures(run, out, show=False))
    metrics = analyse(run) if analyse is not None else {}
    metrics = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in metrics.items()}
    metrics["warnings"] = warnings
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, default=float))
    print("[exp] metrics: " + ", ".join(f"{k} {v:.3g}" if isinstance(v, float) else f"{k} {v}"
                                        for k, v in metrics.items() if not isinstance(v, (list, dict))))
    return metrics


def run_experiment(exp: str, conditions: list[Condition], base_flags: dict, analyse=None, args=None,
                   margin_s: float = 3.0) -> list[Path]:
    """Run (or ``--analyse``) the conditions; see the module docstring. Returns the run directories."""
    args = args or parse(exp)
    if args.analyse:
        for run_dir in args.analyse:
            print(f"\n[exp] {run_dir}")
            analyse_run(run_dir.expanduser(), analyse)
        return list(args.analyse)

    todo = [c for c in conditions
            if (not args.only_method or c.method in args.only_method)
            and (not args.only_rtt or rtt_label(c.rtt) in {rtt_label(r) for r in args.only_rtt})]
    backend = "sim" if args.sim else "robot"
    out_dir = OUT / backend / exp
    session = out_dir / "sessions" / f"{datetime.datetime.now():%Y%m%d_%H%M%S}.txt"
    exe = [sys.executable, str(RIM / "sim_teleop.py")] if args.sim else ["fr3_teleop"]
    print(f"[exp] {exp}: {len(todo)} runs -> {out_dir}  (session list: {session})")
    done: list[Path] = []
    for i, c in enumerate(todo, 1):
        flags = {**RATES, **base_flags, **c.flags, "method": c.method, **delay_flags(c.rtt)}
        if "seconds" not in flags:
            script_s = script_seconds(flags)
            if script_s is None:
                raise ValueError(f"{c.name(exp)}: an Inverse3 run needs 'seconds' in its flags")
            label = rtt_label(c.rtt)
            rtt_s = (TRACE_RTT_MS[label] if label in TRACE_RTT_MS else float(c.rtt)) / 1e3
            flags["seconds"] = round(script_s + rtt_s + margin_s, 1)
        print(f"\n[exp] {i}/{len(todo)} {c.name(exp)}  ({flags['seconds']:.0f} s)")
        if args.dry_run:
            print("  " + " ".join(exe + ["--conf", str(CONF)] + to_argv(flags)))
            continue
        if not args.sim and not args.yes:
            answer = input("  Enter: run | s: skip | q: quit > ").strip().lower()
            if answer == "q":
                break
            if answer == "s":
                continue
        name = f"{datetime.datetime.now():%Y%m%d_%H%M%S}_{c.name(exp)}"
        cmd = exe + ["--conf", str(CONF)] + to_argv(flags) + [
            "--save", "--output-dir", str(out_dir), "--run-name", name, "--notes", f"{exp} {c.name(exp)}"]
        subprocess.run(cmd, cwd=REPO)
        run_dir = out_dir / name
        if not (run_dir / "metadata.json").exists():
            print(f"[exp] {name}: no run saved")
            continue
        meta_path = run_dir / "experiment.json"
        meta_path.write_text(json.dumps({"exp": exp, "method": c.method, "rtt": c.rtt, "label": c.label,
                                         "flags": flags}, indent=2, default=float))
        analyse_run(run_dir, analyse)
        session.parent.mkdir(parents=True, exist_ok=True)
        with session.open("a") as fh:
            fh.write(f"{run_dir}\n")
        done.append(run_dir)
    if done:
        print(f"\n[exp] {len(done)} runs; the notebook input: {session}")
    return done


# ------------------------------------------------------------------ notebooks


def collect(sources) -> list[Path]:
    """Run directories from a mix of run directories, session files (one run per line) and
    directories of runs."""
    runs: list[Path] = []
    for s in sources:
        s = Path(s).expanduser()
        if s.is_file():
            runs += [Path(line.strip()) for line in s.read_text().splitlines() if line.strip()]
        elif (s / "metadata.json").exists():
            runs.append(s)
        elif s.is_dir():
            runs += sorted(p.parent for p in s.glob("*/metadata.json"))
    return runs


def method_key(run_dir: Path, method: str | None) -> str | None:
    """The method as the notebooks group it: ``proxy-rim`` (orientation held) or ``proxy-rim-free``
    (rotation free, also every run from before ``--rim-hold-orientation`` existed), from the run's
    metadata; other methods unchanged."""
    if method != "proxy-rim":
        return method
    meta_path = Path(run_dir) / "metadata.json"
    if not meta_path.exists():
        return method
    meta = json.loads(meta_path.read_text())
    meta = meta.get("metadata", meta)
    return "proxy-rim" if meta.get("args", {}).get("rim_hold_orientation") else "proxy-rim-free"


def load_table(runs, analyse=None):
    """One row per run: its condition (``experiment.json``) and metrics (``analysis/metrics.json``,
    recomputed with ``analyse`` when given or missing)."""
    import pandas as pd
    from runs import load_run

    rows = []
    for r in runs:
        r = Path(r)
        cond = json.loads((r / "experiment.json").read_text()) if (r / "experiment.json").exists() else {}
        mpath = r / "analysis" / "metrics.json"
        if analyse is not None or not mpath.exists():
            run = load_run(r)
            metrics = analyse(run) if (analyse is not None and run is not None) else {}
        else:
            metrics = json.loads(mpath.read_text())
        rows.append({"run": r.name, "dir": str(r), "method": method_key(r, cond.get("method")), "rtt": cond.get("rtt"),
                     "label": cond.get("label"), **{k: v for k, v in metrics.items() if not isinstance(v, (list, dict))}})
    return pd.DataFrame(rows)
