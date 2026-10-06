"""Compare osc_controller benchmark runs side by side.

Usage:
    python compare.py [benchmark] [--last N] [--tag SUBSTR]

benchmark: hold | step_response | tracking (default: all). Prints, per run, the gains that differ
between the listed runs and the headline metrics of that benchmark.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

RESULTS_DIR = Path(__file__).parent / "results"

HEADLINE = {
    "hold": ["err_pos_mean_mm", "err_rot_mean_mrad", "drift_mm", "tau_cmd_rms_hf_max_Nm", "dq_rms_hf_max_mrad_s"],
    "step_response": [
        "rise_ms_mean",
        "overshoot_mm_max",
        "settle_ms_mean",
        "ss_error_mm_mean",
        "rot_err_max_mrad_max",
        "tau_cmd_rms_hf_max_Nm_max",
        "dq_rms_hf_max_mrad_s_max",
    ],
    "tracking": [
        "err_rms_mm",
        "err_max_mm",
        "lag_ms",
        "err_rms_after_lag_mm",
        "rot_err_max_mrad",
        "tau_cmd_rms_hf_max_Nm",
        "dq_rms_hf_max_mrad_s",
    ],
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("benchmark", nargs="?", choices=sorted(HEADLINE))
    parser.add_argument("--last", type=int, default=10, help="Show the N most recent runs per benchmark")
    parser.add_argument("--tag", default="", help="Only runs whose tag contains this")
    args = parser.parse_args()

    for bench in [args.benchmark] if args.benchmark else sorted(HEADLINE):
        runs = []
        for path in sorted(RESULTS_DIR.glob(f"{bench}_*.yaml")):
            meta = yaml.safe_load(path.read_text())
            if meta.get("benchmark") == bench and args.tag in (meta.get("tag") or ""):
                runs.append((path.stem, meta))
        runs = runs[-args.last :]
        if not runs:
            continue

        # Parameters that differ between the listed runs
        keys = sorted({k for _, m in runs for k in m["params"]})
        varying = [k for k in keys if len({repr(m["params"].get(k)) for _, m in runs}) > 1]
        cols = varying + HEADLINE[bench]

        print(f"\n### {bench}  ({len(runs)} runs)")
        header = ["run"] + [c.replace("gains.", "").replace("control.", "") for c in cols]
        rows = []
        for stem, meta in runs:
            row = [stem.removeprefix(f"{bench}_")]
            for c in cols:
                v = meta["params"].get(c) if c in varying else meta["metrics"].get(c)
                row.append(f"{v:.3g}" if isinstance(v, float) else str(v))
            rows.append(row)
        widths = [max(len(r[i]) for r in rows + [header]) for i in range(len(header))]
        for r in [header] + rows:
            print("  ".join(s.rjust(w) if i else s.ljust(w) for i, (s, w) in enumerate(zip(r, widths))))


if __name__ == "__main__":
    main()
