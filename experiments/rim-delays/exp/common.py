"""Shared runner for the experiment scripts (``e1_*.py`` … ``e6_*.py``).

An experiment script sets its parameters at the top and calls [run_experiment][] with a list of
[Condition][]s (method, round-trip delay, extra ``fr3_teleop`` flags). For each condition this:

0. skips it if it already ran: a run of the same condition, with the same flags, that ended normally
   (``--rerun`` runs it again); the earlier run goes in the session file;
1. asks before an Inverse3 run on the robot (run, skip or quit; ``--yes`` does not ask); scripted
   runs follow each other without asking. Every prompt takes a word or its first letter, and
   Enter picks the default (in capitals);
2. runs ``fr3_teleop`` (``--sim``: ``sim_teleop.py``) with ``configs/teleop.yaml``, the experiment's
   base flags, the condition's flags and its delay; the run is saved as
   ``<OUT>/<robot|sim>/<exp>/<YYYYmmdd_HHMMSS>_<exp>_<condition>``, so nothing is ever overwritten.
   ``--home`` (if the experiment sets it) is passed to the session's first run only, and again
   after a server crash; the other runs only move to the start position. If ``fr3_teleop`` cannot
   reach the robot or the haptic watchdog stops the run (the server crashed: [SERVER_DOWN][],
   [WATCHDOG][]), it asks to retry the same condition (a stopped run is kept and analysed, not
   listed); any other failure ends the session. A crashed haptic process ([HAPTIC_ERROR][]) is
   retried [AUTO_RETRIES][] times without asking. The haptic status lines are off;
3. analyses it: ``analysis/check_run.py`` (``<run>/analysis/check.md`` + figures) and the
   experiment's own metrics (``<run>/analysis/metrics.json``);
4. appends the run to the session file ``<OUT>/<robot|sim>/<exp>/sessions/<start time>.txt``, the
   list the experiment's notebook reads.

Delays are round trips (``rtt_ms``), split evenly between the links, with one seed per delay
condition so every method sees the same delay sequence. ``"5g"`` replays the lab's 5G trace.

Common flags of every script: ``--sim``, ``--dry-run`` (print the commands), ``--yes``, ``--rerun``,
``--only-method m …``, ``--only-rtt r …``, ``--analyse <run dir> …`` (re-run the analysis only),
``--no-save`` (try it out: nothing saved, analysed or skipped as done).
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
OUT = Path(os.environ.get("RIM_DELAYS_DATA", "~/data/franka/rim-delays")).expanduser()  # the runs; override on another machine
TRACE_RTT_MS = {"5g": 45.0}  # rough mean round trip of a replayed trace, for the run length [ms]
# Frozen feedback rates (tools/pilot_fm.py on pilots/fm_circle2): robot state 500 Hz (M1 ripple
# K|v|/f_m 0.72 N), RIM model 25 Hz (Λ change 2.1 % per update, 0.12 N RMS from holding); targets
# at f_t = f_h = 1 kHz. Every run gets them.
RATES = {"model_update_hz": 500.0, "rim_update_hz": 25.0, "command_hz": 1000.0}
# fr3_teleop's errors when the robot server is not there (crashed): the condition can be retried.
SERVER_DOWN = ("Timeout waiting for", "Timed out waiting for")
WATCHDOG = "watchdog"  # in a run's stop reason: the robot's samples stopped arriving mid-run
RUN_ENDED = "run ended"  # the stop reason of a run that went to its end
HAPTIC_ERROR = "haptic process error"  # the stop reason when the haptic loop crashed (no haptic log)
AUTO_RETRIES = 2  # a crashed haptic process is retried this many times before asking
QUIET = {"print_every_s": 0}  # no haptic status lines
# fr3_teleop's exit codes when the operator skips / quits at its --start-prompt (nothing saved)
START_SKIPPED, START_QUIT = 10, 11


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


def held_mass_with_armature(start_position, directions: dict, armatures) -> "pd.DataFrame":
    """nᵀΛn with the orientation held, at the start joints, with a uniform reflected rotor
    inertia (armature, kg·m²) added to the mass matrix's diagonal: what the RIM model would say
    with the motors' inertia, which the URDF leaves out. ``directions``: name → unit vector."""
    import pandas as pd
    import pinocchio as pin

    from lambda_map import Inertia

    q = start_inertia(start_position)["q"]
    inertia = Inertia()
    inertia.at(q)  # fills the adapter's data at q
    a = inertia.adapter
    J6 = pin.computeFrameJacobian(a._model, a._data, q, a._ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
    M = a._data.M.copy()
    rows = []
    for ia in armatures:
        lam = np.linalg.inv(J6 @ np.linalg.inv(M + ia * np.eye(len(q))) @ J6.T)[:3, :3]
        rows.append({"armature": ia, **{k: mass_along(lam, n) for k, n in directions.items()}})
    return pd.DataFrame(rows).set_index("armature")


def mass_along(lam: np.ndarray, n) -> float:
    """``nᵀΛn`` for a unit ``n``: M4's best-case mass along ``n``."""
    n = np.asarray(n, dtype=float)
    n = n / np.linalg.norm(n)
    return float(n @ lam @ n)


# ------------------------------------------------------------------ running


def parse(description: str, argv=None, extra=None) -> argparse.Namespace:
    """The common flags (module docstring); ``extra(parser)`` adds a script's own."""
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--sim", action="store_true", help="simulated FR3 (sim_teleop.py)")
    p.add_argument("--dry-run", action="store_true", help="print the commands, run nothing")
    p.add_argument("--yes", action="store_true", help="do not ask before each robot run")
    p.add_argument("--rerun", action="store_true", help="run every condition, even those already done")
    p.add_argument("--no-save", action="store_true",
                   help="try it out (e.g. a visitor): nothing is saved or analysed, and no condition is skipped as done")
    p.add_argument("--only-method", nargs="+", default=None)
    p.add_argument("--only-rtt", nargs="+", default=None, help="round trips [ms], or 5g")
    p.add_argument("--analyse", type=Path, nargs="+", default=None, help="re-run the analysis of these runs only")
    if extra is not None:
        extra(p)
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
    if run.get("haptic") is None:  # the haptic process crashed: nothing to compute the metrics on
        print(f"[exp] {run_dir.name}: no haptic log, no metrics")
        return {}
    metrics = analyse(run) if analyse is not None else {}
    metrics = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in metrics.items()}
    metrics["warnings"] = warnings
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, default=float))
    print("[exp] metrics: " + ", ".join(f"{k} {v:.3g}" if isinstance(v, float) else f"{k} {v}"
                                        for k, v in metrics.items() if not isinstance(v, (list, dict))))
    return metrics


def stop_reason(run_dir: Path) -> str | None:
    """Why a saved run ended (``None``: no run saved)."""
    path = run_dir / "metadata.json"
    if not path.exists():
        return None
    meta = json.loads(path.read_text())
    return meta.get("metadata", meta).get("stop_reason", "")


def same_flags(a: dict, b: dict) -> bool:
    """Whether two runs' flags are the same condition (``home`` and output flags aside, floats to 1e-6)."""
    def norm(f):
        text = json.dumps({k: v for k, v in f.items() if k not in ("home", "start_prompt", *QUIET)}, default=float)
        return json.loads(text, parse_float=lambda x: round(float(x), 6))

    return norm(a) == norm(b)


def previous_run(out_dir: Path, exp: str, c: Condition, flags: dict) -> Path | None:
    """The latest run of this condition with these flags that ended normally and was analysed, if any."""
    for d in sorted(out_dir.glob(f"*_{c.name(exp)}"), reverse=True):
        cond_path = d / "experiment.json"
        if (d.name[16:] != c.name(exp) or not cond_path.exists() or stop_reason(d) != RUN_ENDED
                or not (d / "analysis" / "metrics.json").exists()):
            continue
        if same_flags(json.loads(cond_path.read_text())["flags"], flags):
            return d
    return None


def _tty() -> bool:
    return sys.stdout.isatty()


def ask(prompt: str, options: list[str]) -> str:
    """Ask until the answer is one of ``options`` (a word or its first letter, any case); Enter
    picks the first, the default. Returns the option."""
    menu = " / ".join(f"[{o[0].upper()}]{o[1:]}" if i == 0 else f"({o[0]}){o[1:]}" for i, o in enumerate(options))
    while True:
        answer = input(f"{prompt}  {menu} > ").strip().lower()
        if not answer:
            return options[0]
        for o in options:
            if answer in (o, o[0]):
                return o
        print(f"  ? {answer!r}: type one of {', '.join(options)} (or its first letter)")


def header(text: str) -> None:
    """A condition's header: a rule across the terminal, then the text (bold on a terminal)."""
    rule = "═" * 100
    print(f"\n{rule}\n\033[1m{text}\033[0m" if _tty() else f"\n{rule}\n{text}", flush=True)


def alert(message: str) -> None:
    """A message that stands out: a bold red banner (on a terminal) and the terminal bell."""
    bar = "!" * 78
    text = f"\n{bar}\n!!  {message}\n{bar}"
    if _tty():
        text = f"\033[1;31m{text}\033[0m\a"
    print(text, flush=True)


def run_teleop(cmd: list[str]) -> tuple[int, str]:
    """Run ``fr3_teleop``, its stderr shown live and kept; returns the exit code and the stderr."""
    proc = subprocess.Popen(cmd, cwd=REPO, stderr=subprocess.PIPE, text=True, bufsize=1)
    err = []
    try:
        for line in proc.stderr:
            sys.stderr.write(line)
            err.append(line)
    except KeyboardInterrupt:  # Ctrl-C reaches fr3_teleop too: let it stop the robot and save
        proc.wait()
        raise
    return proc.wait(), "".join(err)


def write_condition(run_dir: Path, exp: str, c: Condition, flags: dict) -> None:
    """``<run>/experiment.json``: the experiment, the condition and the flags it ran with."""
    (run_dir / "experiment.json").write_text(json.dumps({"exp": exp, "method": c.method, "rtt": c.rtt,
                                                         "label": c.label, "flags": flags}, indent=2, default=float))


def run_experiment(exp: str, conditions: list[Condition], base_flags: dict, analyse=None, args=None,
                   margin_s: float = 3.0, home: bool | None = None, session: Path | None = None,
                   progress: str | None = None) -> list[Path]:
    """Run (or ``--analyse``) the conditions; see the module docstring. Returns the run directories.

    For a script that calls it once per run (E5's ladders): ``home`` whether the first run homes
    (default: the base flags' ``home``), ``session`` the session file to append to (default: a new
    one), ``progress`` the step label for the header (default ``i/n`` of these conditions).

    A condition with a ``start_prompt`` flag shows it in fr3_teleop once the robot is ready, and
    waits for Enter before the forces start (instead of the "ready?" prompt before launching); a skip
    there skips the condition, a quit ends the session."""
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
    single = session is not None  # called once per run: the caller numbers and reports
    session = session or out_dir / "sessions" / f"{datetime.datetime.now():%Y%m%d_%H%M%S}.txt"
    exe = [sys.executable, str(RIM / "sim_teleop.py")] if args.sim else ["fr3_teleop"]
    no_save = bool(getattr(args, "no_save", False))
    if not single:
        print(f"[exp] {exp}: {len(todo)} runs, NOT SAVED (--no-save)" if no_save
              else f"[exp] {exp}: {len(todo)} runs -> {out_dir}  (session list: {session})")
    done: list[Path] = []
    home = bool(base_flags.get("home")) if home is None else home  # the first run only (and after a server crash)
    for i, c in enumerate(todo, 1):
        flags = {**RATES, **QUIET, **base_flags, **c.flags, "method": c.method, **delay_flags(c.rtt), "home": home}
        if "seconds" not in flags:
            script_s = script_seconds(flags)
            if script_s is None:
                raise ValueError(f"{c.name(exp)}: an Inverse3 run needs 'seconds' in its flags")
            label = rtt_label(c.rtt)
            rtt_s = (TRACE_RTT_MS[label] if label in TRACE_RTT_MS else float(c.rtt)) / 1e3
            flags["seconds"] = round(script_s + rtt_s + margin_s, 1)
        prior = None if args.rerun or no_save else previous_run(out_dir, exp, c, flags)
        if prior is not None:
            print(f"[exp] {progress or f'{i}/{len(todo)}'} {c.name(exp)}: done before ({prior.name}), skipped")
            if not args.dry_run:
                session.parent.mkdir(parents=True, exist_ok=True)
                with session.open("a") as fh:
                    fh.write(f"{prior}\n")
            done.append(prior)
            continue
        header(f"[exp] {progress or f'{i}/{len(todo)}'} {c.name(exp)}  ({flags['seconds']:.0f} s)")
        if args.dry_run:
            print("  " + " ".join(exe + ["--conf", str(CONF)] + to_argv(flags)))
            home = False
            continue
        if flags.get("device") != "scripted" and not flags.get("start_prompt") and not args.sim and not args.yes:
            answer = ask("[exp] ready?", ["run", "skip", "quit"])
            if answer == "quit":
                break
            if answer == "skip":
                continue
        auto_retries = 0
        while True:
            name = f"{datetime.datetime.now():%Y%m%d_%H%M%S}_{c.name(exp)}"
            cmd = exe + ["--conf", str(CONF)] + to_argv(flags) + ["--notes", f"{exp} {c.name(exp)}"]
            if not no_save:
                cmd += ["--save", "--output-dir", str(out_dir), "--run-name", name]
            code, err = run_teleop(cmd)
            if code in (START_SKIPPED, START_QUIT):  # the operator, at fr3_teleop's start prompt
                break
            run_dir = out_dir / name
            reason = stop_reason(run_dir)
            stopped = reason is not None and WATCHDOG in reason
            crashed = reason == HAPTIC_ERROR
            if crashed and auto_retries < AUTO_RETRIES:
                auto_retries += 1
                write_condition(run_dir, exp, c, flags)
                print(f"[exp] {name}: the haptic process crashed; retrying ({auto_retries}/{AUTO_RETRIES})")
                continue
            if not stopped and not crashed and (code == 0 or not any(s in err for s in SERVER_DOWN)):
                break
            if stopped or crashed:  # kept for diagnosis, not listed in the session
                write_condition(run_dir, exp, c, flags)
                analyse_run(run_dir, analyse)
            if crashed:
                alert(f"HAPTIC PROCESS CRASHED {AUTO_RETRIES + 1} TIMES: {c.name(exp)}\n!!  "
                      "See the traceback above.")
            else:
                alert(f"ROBOT SERVER {'STOPPED MID-RUN (watchdog)' if stopped else 'NOT RESPONDING'}: "
                      f"{c.name(exp)}\n!!  Restart the server on the mini-PC, then retry.")
            if ask("[exp]", ["retry", "quit"]) == "quit":
                raise SystemExit(f"[exp] stopped at {i}/{len(todo)} {c.name(exp)}; session list: {session}")
            flags["home"] = bool(base_flags.get("home"))  # the server restarted: home again
        if code == START_QUIT:  # ends the whole session, also for a caller looping over runs (E5)
            raise SystemExit(f"[exp] quit at {c.name(exp)}; session list: {session}")
        if code == START_SKIPPED:
            print(f"[exp] {c.name(exp)}: skipped")
            home = False  # the robot did move to the start
            continue
        home = False
        if no_save:  # nothing to analyse or list; a failed run does not end the session
            print(f"[exp] {c.name(exp)}: {'done' if code == 0 else f'fr3_teleop exit {code}'} (not saved)")
            continue
        saved = reason is not None
        if code != 0 or not saved:
            if saved:  # keep what was recorded before the failure
                analyse_run(run_dir, analyse)
            raise SystemExit(f"[exp] {name}: fr3_teleop failed (exit {code}"
                             f"{', run saved' if saved else ', no run saved'}); stopped at {i}/{len(todo)}. "
                             f"Session list: {session}")
        if reason != RUN_ENDED:
            print(f"[exp] {name}: stopped early ({reason}); it will run again next time")
        meta = json.loads((run_dir / "metadata.json").read_text())
        limit = meta.get("metadata", meta).get("contact_limit") or {}
        if limit.get("events"):  # kept: the run is valid, its robot force was capped (check.md says when)
            alert(f"CONTACT FORCE LIMIT reached {limit['events']} time(s): {c.name(exp)}\n!!  "
                  "The robot's force was capped (osc_controller); the run is kept and flagged.")
        write_condition(run_dir, exp, c, flags)
        analyse_run(run_dir, analyse)
        session.parent.mkdir(parents=True, exist_ok=True)
        with session.open("a") as fh:
            fh.write(f"{run_dir}\n")
        done.append(run_dir)
    if done and not single:
        print(f"\n[exp] {len(done)} runs; the notebook input: {session}")
    return done


# ------------------------------------------------------------------ notebooks


def latest_session(exp: str, backend: str = "robot") -> Path:
    """The experiment's latest session file: after a resumed session it lists every condition once
    (the skipped runs too), so it is the complete, current set of runs."""
    sessions = sorted((OUT / backend / exp / "sessions").glob("*.txt"))
    if not sessions:
        raise FileNotFoundError(f"no session file in {OUT / backend / exp / 'sessions'}")
    return sessions[-1]


def relocate(path) -> Path:
    """A run path as recorded (absolute, on the machine that ran it), under ``OUT`` when it is not
    there: session files and ``kmax.csv`` store absolute paths, the data may be copied elsewhere."""
    path = Path(path).expanduser()
    if path.exists() or "rim-delays" not in path.parts:
        return path
    parts = path.parts
    return OUT.joinpath(*parts[len(parts) - parts[::-1].index("rim-delays"):])


def collect(sources) -> list[Path]:
    """Run directories from a mix of run directories, session files (one run per line) and
    directories of runs."""
    runs: list[Path] = []
    for s in sources:
        s = Path(s).expanduser()
        if s.is_file():
            runs += [relocate(line.strip()) for line in s.read_text().splitlines() if line.strip()]
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
