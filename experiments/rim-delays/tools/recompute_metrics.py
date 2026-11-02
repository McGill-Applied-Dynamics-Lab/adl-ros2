r"""Recompute ``analysis/metrics.json`` of an experiment's session runs from their recordings.

Loads each run once, calls the experiment's ``exp_metrics`` function and merges the result into the
run's existing ``metrics.json`` (the ``warnings`` and any other keys stay). Unlike
``eN_*.py --analyse`` it does not rerun ``check_run`` or touch ``check.md`` and the figures. The old
file is kept once as ``metrics.json.bak``; a run whose metrics did not change is not rewritten.

    pixi run -e analysis python experiments/rim-delays/tools/recompute_metrics.py e3 [--dry-run]

Set ``RIM_DELAYS_DATA`` first if the data is not in ``~/data/franka/rim-delays``.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")
HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path[:0] = [str(HERE.parent / "exp"), str(HERE.parent)]
for src in (REPO / "python/fr3_haptic/src", REPO / "external/adl-python/packages/experiment_logger/src"):
    sys.path.append(str(src))

import numpy as np  # noqa: E402

import exp_metrics  # noqa: E402
from common import OUT, collect  # noqa: E402
from runs import load_run  # noqa: E402

ANALYSE = {"e1": exp_metrics.e1, "e2": exp_metrics.e2, "e3": exp_metrics.e3, "e4": exp_metrics.e4}


def _plain(metrics: dict) -> dict:
    """The metrics as ``common.analyse_run`` saves them, through a JSON round trip."""
    metrics = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in metrics.items()}
    return json.loads(json.dumps(metrics, default=float))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("exp", choices=sorted(ANALYSE))
    p.add_argument("--session", type=Path, default=None, help="session file (default: the latest)")
    p.add_argument("--dry-run", action="store_true", help="report what would change, write nothing")
    args = p.parse_args()

    session = args.session or sorted((OUT / f"robot/{args.exp}/sessions").glob("*.txt"))[-1]
    runs = collect([session])
    print(f"{args.exp}: {session.name}, {len(runs)} runs{' (dry run)' if args.dry_run else ''}")
    changed = 0
    for i, r in enumerate(runs, 1):
        path = r / "analysis" / "metrics.json"
        old = json.loads(path.read_text())
        run = load_run(r)
        if run is None or run.get("haptic") is None:
            print(f"[{i}/{len(runs)}] {r.name}: no haptic log, skipped")
            continue
        new = {**old, **_plain(ANALYSE[args.exp](run))}
        added = sorted(set(new) - set(old))
        if new == old:
            print(f"[{i}/{len(runs)}] {r.name}: unchanged")
            continue
        diff = [k for k in new if k in old and new[k] != old[k]]
        print(f"[{i}/{len(runs)}] {r.name}: new {added}, changed {diff}")
        changed += 1
        if not args.dry_run:
            bak = path.with_name("metrics.json.bak")
            if not bak.exists():
                shutil.copy2(path, bak)
            path.write_text(json.dumps(new, indent=2, default=float))
    print(f"{changed} of {len(runs)} runs {'would change' if args.dry_run else 'rewritten'}")


if __name__ == "__main__":
    main()
