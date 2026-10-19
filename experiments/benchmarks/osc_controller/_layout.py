"""On-disk layout of benchmark results (no ROS imports, shared by the benchmarks, plots and compare).

results/<benchmark>/<timestamp>[_<tag>]/      one run
    meta.yaml                                 args, git commit, live parameters, stream health, metrics
    data.npz                                  raw streams
    <kind>.png                                figures
results/<benchmark>/compare/<timestamp>/      overlays made by `plots.py --compare`
"""

from __future__ import annotations

import time
from pathlib import Path

RESULTS_DIR = Path(__file__).parent / "results"
META_FILE = "meta.yaml"
DATA_FILE = "data.npz"
COMPARE_DIR = "compare"


def _timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def new_run_dir(benchmark: str, tag: str = "") -> Path:
    path = RESULTS_DIR / benchmark / "_".join(s for s in (_timestamp(), tag) if s)
    path.mkdir(parents=True)
    return path


def new_compare_dir(benchmark: str) -> Path:
    path = RESULTS_DIR / benchmark / COMPARE_DIR / _timestamp()
    path.mkdir(parents=True, exist_ok=True)
    return path


def run_dirs(benchmark: str) -> list[Path]:
    """Run directories of a benchmark, oldest first (names start with the timestamp)."""
    return sorted(p.parent for p in (RESULTS_DIR / benchmark).glob(f"*/{META_FILE}"))


def resolve_run(path: str | Path) -> Path:
    """Run directory from a run directory or a path to its meta.yaml."""
    path = Path(path)
    run = path.parent if path.name == META_FILE else path
    if not (run / META_FILE).exists():
        raise FileNotFoundError(f"Not a run directory (no {META_FILE}): {path}")
    return run
