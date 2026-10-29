r"""Pack the runs the talk figures use, to analyse them on another machine.

Takes the latest session of E1–E4 and E6 (every condition once) and E5's ``kmax.csv`` steps, and
writes a ``.tar.gz`` holding only what the analysis reads, relative to the data root:
``robot/<exp>/<run>/{experiment.json, metadata.json, samples.mcap, analysis/metrics.json}``, the
session files and ``kmax.csv``. Extract it anywhere and point ``RIM_DELAYS_DATA`` at it (default
``~/data/franka/rim-delays``); the recorded absolute paths are re-rooted there (``common.relocate``).

    pixi run -e analysis python experiments/rim-delays/tools/pack_analysis_data.py [out.tar.gz]
"""

from __future__ import annotations

import csv
import sys
import tarfile
from pathlib import Path

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / "exp")]
from common import OUT, collect, relocate  # noqa: E402

EXPERIMENTS = ("e1", "e2", "e3", "e4", "e6")
FILES = ("experiment.json", "metadata.json", "samples.mcap", "analysis/metrics.json")


def runs_to_pack() -> tuple[list[Path], list[Path]]:
    """The run directories and the extra files (session files, kmax.csv) the figures read."""
    runs, extra = [], []
    for exp in EXPERIMENTS:
        session = sorted((OUT / f"robot/{exp}/sessions").glob("*.txt"))[-1]
        runs += collect([session])
        extra.append(session)
    kmax = OUT / "robot/e5/kmax.csv"
    if kmax.exists():
        with kmax.open(newline="") as fh:
            runs += [relocate(row["run"]) for row in csv.DictReader(fh) if row["grip"] == "guided"]
        extra.append(kmax)
    return runs, extra


def main() -> None:
    dest = Path(sys.argv[1] if len(sys.argv) > 1 else "rim-delays-analysis-data.tar.gz").expanduser()
    runs, extra = runs_to_pack()
    missing = [r for r in runs if not all((r / f).exists() for f in FILES)]
    if missing:
        raise SystemExit(f"incomplete runs: {[r.name for r in missing]}")
    paths = [r / f for r in runs for f in FILES] + extra
    size = sum(p.stat().st_size for p in paths)
    print(f"{len(runs)} runs, {len(paths)} files, {size / 1e9:.2f} GB -> {dest}")
    with tarfile.open(dest, "w:gz") as tar:
        for p in paths:
            tar.add(p, arcname=str(p.relative_to(OUT)))
    print(f"done: {dest.stat().st_size / 1e9:.2f} GB. On the other machine: tar xzf {dest.name} -C <dir>, "
          f"then RIM_DELAYS_DATA=<dir> (or extract into ~/data/franka/rim-delays)")


if __name__ == "__main__":
    main()
