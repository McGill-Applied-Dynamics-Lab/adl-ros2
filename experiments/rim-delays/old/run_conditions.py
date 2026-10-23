r"""Run the delay experiment: every (task, method, round-trip delay) cell, ``repeats`` times.

Reads ``conditions.yaml`` (next to this script by default). Each run is one ``fr3_teleop``
(``--backend robot``) or ``sim_teleop.py`` (``--backend sim``) process, saved as
``<output_dir>/<backend>/<session>/<task>_<method>_rtt<ms>_r<k>`` (``output_dir`` default
``~/data/franka/rim-delays``). Within each repeat the
(task, method, delay) order is shuffled (seeded), so drift — a warming motor, a tiring
operator — does not line up with one condition. Runs already on disk are skipped, so an
interrupted session resumes where it stopped.

    pixi run -e humble python experiments/rim-delays/run_conditions.py --backend sim --dry-run
    pixi run -e humble python experiments/rim-delays/run_conditions.py --backend sim
    pixi run -e humble python experiments/rim-delays/run_conditions.py --backend robot --session day1 \
        --only-task press --only-rtt 0 850

On the robot every run waits for Enter (``s`` skips it, ``q`` stops); ``--yes`` does not wait.
"""

from __future__ import annotations

import argparse
import datetime
import random
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from fr3_haptic.scripted import Script, ScriptConfig

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SCRIPT_KEYS = {
    "script": "profile", "script_depth": "depth", "script_speed": "speed", "script_hold_s": "hold_s",
    "script_rest_s": "rest_s", "script_repeats": "repeats", "script_radius": "radius",
    "script_period_s": "period_s", "script_laps": "laps",
}


TRACES = {"5g": 400.0}  # round-trip delay conditions replayed from a trace, and a bound on their delay [ms]


def rtt_label(rtt) -> str:
    """``0`` → ``0000``, ``850`` → ``0850``, ``"5g"`` → ``5g``."""
    return str(rtt).lower() if str(rtt).lower() in TRACES else f"{int(round(float(rtt))):04d}"


@dataclass
class Run:
    task: str
    method: str
    rtt: float | str
    rep: int
    flags: dict
    seconds: float

    @property
    def name(self) -> str:
        return f"{self.task}_{self.method}_rtt{rtt_label(self.rtt)}_r{self.rep}"


def to_argv(flags: dict) -> list[str]:
    """``{"interface_axis": "xyz", "hand": True, "start_position": [0.4, 0, 0.2]}`` → CLI arguments."""
    argv = []
    for key, value in flags.items():
        flag = "--" + key.replace("_", "-")
        if value is None or value is False:
            continue
        if value is True:
            argv.append(flag)
        elif isinstance(value, (list, tuple)):
            argv += [flag, *map(str, value)]
        else:
            argv += [flag, str(value)]
    return argv


def script_seconds(flags: dict) -> float:
    """How long the scripted operator takes for these flags [s] (0 for the Inverse3)."""
    if flags.get("device") != "scripted":
        return 0.0
    surface = flags.get("script_surface_z", flags.get("contact_surface"))
    kwargs = {SCRIPT_KEYS[k]: v for k, v in flags.items() if k in SCRIPT_KEYS}
    origin = np.asarray(flags.get("start_position", (0.4, 0.0, 0.2)), dtype=float)
    return Script(ScriptConfig(surface_z=surface, **kwargs), origin).duration_s


def plan(cfg: dict, backend: str, only_task=None, only_method=None, only_rtt=None) -> list[Run]:
    """Every run of the session, in the order to run them."""
    rng = random.Random(cfg.get("seed", 0))
    runs = []
    for rep in range(cfg["repeats"]):
        block = []
        for task, task_flags in cfg["tasks"].items():
            if only_task and task not in only_task:
                continue
            for method in cfg["methods"]:
                if only_method and method not in only_method:
                    continue
                for rtt in cfg["rtt_ms"]:
                    if only_rtt and rtt_label(rtt) not in {rtt_label(r) for r in only_rtt}:
                        continue
                    flags = {**cfg.get("common", {}), **(cfg.get(backend) or {}), **task_flags}
                    flags.update(method=method)
                    trace = str(rtt).lower()
                    if trace in TRACES:
                        flags.update(feedback_delay_ms=0, command_delay_ms=0, delay_trace=trace)
                        rtt_s = TRACES[trace] * 1e-3
                    else:
                        flags.update(feedback_delay_ms=float(rtt) / 2, command_delay_ms=float(rtt) / 2)
                        rtt_s = float(rtt) * 1e-3
                    seconds = flags.pop("seconds", None)
                    if seconds is None:
                        seconds = script_seconds(flags) + rtt_s + cfg.get("margin_s", 3.0)
                    block.append(Run(task, method, rtt, rep, flags, float(seconds)))
        rng.shuffle(block)
        runs += block
    return runs


def command(run: Run, cfg: dict, backend: str, out_dir: Path) -> list[str]:
    exe = [sys.executable, str(HERE.parent / "sim_teleop.py")] if backend == "sim" else ["fr3_teleop"]
    conf = cfg.get("conf")
    head = ["--conf", str(REPO / conf)] if conf else []
    tail = ["--seconds", f"{run.seconds:.1f}", "--save", "--output-dir", str(out_dir), "--run-name", run.name,
            "--notes", f"rim-delays {run.name}"]
    return exe + head + to_argv(run.flags) + tail


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--conditions", type=Path, default=HERE / "conditions.yaml")
    p.add_argument("--backend", choices=("sim", "robot"), default="sim")
    p.add_argument("--session", default=None, help="session directory name; default today's date")
    p.add_argument("--only-task", nargs="+", default=None)
    p.add_argument("--only-method", nargs="+", default=None)
    p.add_argument("--only-rtt", nargs="+", default=None, help="round trips [ms], or 5g")
    p.add_argument("--repeats", type=int, default=None, help="override the conditions file")
    p.add_argument("--dry-run", action="store_true", help="print the commands, run nothing")
    p.add_argument("--yes", action="store_true", help="robot: do not wait for Enter before each run")
    args = p.parse_args(argv)

    cfg = yaml.safe_load(args.conditions.read_text())
    if args.repeats is not None:
        cfg["repeats"] = args.repeats
    session = args.session or datetime.date.today().isoformat()
    out_dir = Path(cfg.get("output_dir", "~/data/franka/rim-delays")).expanduser() / args.backend / session
    runs = plan(cfg, args.backend, args.only_task, args.only_method, args.only_rtt)
    todo = [r for r in runs if not (out_dir / r.name).exists()]
    total_s = sum(r.seconds for r in todo)
    print(f"[rim-delays] {len(runs)} runs, {len(runs) - len(todo)} already done; {len(todo)} to go, "
          f"~{total_s / 60:.0f} min of run time -> {out_dir}")

    wait = args.backend == "robot" and not args.yes
    for i, run in enumerate(todo, 1):
        cmd = command(run, cfg, args.backend, out_dir)
        print(f"\n[rim-delays] {i}/{len(todo)} {run.name} ({run.seconds:.0f} s)")
        if args.dry_run:
            print("  " + " ".join(cmd))
            continue
        if wait:
            answer = input("  Enter: run | s: skip | q: quit > ").strip().lower()
            if answer == "q":
                break
            if answer == "s":
                continue
        result = subprocess.run(cmd, cwd=REPO)
        if result.returncode != 0:
            print(f"[rim-delays] {run.name} exited with {result.returncode}")
            if not (out_dir / run.name).exists() and not wait:
                print("[rim-delays] stopping: the run failed before saving")
                break


if __name__ == "__main__":
    main()
