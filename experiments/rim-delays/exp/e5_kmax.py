r"""E5 — Maximum stable coupling stiffness (a Z-width measurement), on the Inverse3.

One run per K step, for each (method, delay, grip): tap and hold against the table for
``SECONDS``. After each run the HF force RMS (> 8 Hz) while in contact is compared with
``HF_THRESHOLD_N`` (fixed before running); you confirm or override the verdict, and the ladder
stops at the first unstable step. Every step is appended to ``<OUT>/<robot|sim>/e5/kmax.csv``;
K_max of a ladder is its last stable K.

Grips: ``held`` (hold the handle), ``released`` (drop the handle onto the wall: the worst case).

    pixi run -e humble python experiments/rim-delays/exp/e5_kmax.py [--only-method ...] [--only-rtt ...]
    pixi run -e humble python experiments/rim-delays/exp/e5_kmax.py --only-rtt 0   # parameter selection: K_max at T = 0
"""

from __future__ import annotations

import csv
import json

import exp_metrics
from common import OUT, Condition, ask, mass_along, parse, run_experiment, start_inertia

METHODS = ["zoh", "proxy-rim"]  # + one of proxy-god-object / proxy-fixed-mass as a check
RTTS = [0, "5g", 850, 2600]  # [ms]
GRIPS = ["held", "released"]
K_LADDER = [300, 500, 750, 1000, 1500, 2000, 3000, 4000]  # coupling stiffness steps [N/m]
HF_THRESHOLD_N = 0.5  # instability: HF RMS (> 8 Hz) of the felt force in contact above this [N]
SECONDS = 30.0
START = [0.4, 0.0, 0.2]
BASE = {
    "interface_axis": "xyz",
    "contact_surface": 0.083,
    "home": True,
    "start_position": START,
    "device": "inverse3",
    "force": True,
    "seconds": SECONDS,
}


def analyse(run):
    return exp_metrics.e5(run, HF_THRESHOLD_N)


def main() -> None:
    args = parse(__doc__.split("\n")[0])
    if args.analyse:
        run_experiment("e5", [], BASE, analyse, args)
        return
    m4 = None
    table = OUT / ("sim" if args.sim else "robot") / "e5" / "kmax.csv"
    for method in [m for m in METHODS if not args.only_method or m in args.only_method]:
        if method == "proxy-fixed-mass" and m4 is None:
            m4 = round(mass_along(start_inertia(START)["Lambda_rim"], [0.0, 0.0, 1.0]), 3)
        for rtt in RTTS:
            for grip in GRIPS:
                print(f"\n[e5] ===== {method}, rtt {rtt}, grip {grip}: ladder {K_LADDER}")
                k_max = None
                for k in K_LADDER:
                    flags = {"kv": k, **({"proxy_mass": m4} if method == "proxy-fixed-mass" else {})}
                    print(
                        f"[e5] K = {k} N/m — {'hold the handle' if grip == 'held' else 'drop the handle onto the table'}, tap and hold"
                    )
                    runs = run_experiment("e5", [Condition(method, rtt, flags, f"{grip}_K{k}")], BASE, analyse, args)
                    if args.dry_run:
                        continue
                    if not runs:
                        break
                    metrics = json.loads((runs[0] / "analysis" / "metrics.json").read_text())
                    stable = metrics["stable"]
                    answer = ask(
                        f"[e5] HF RMS {metrics['hf_rms_N']:.2f} N vs {HF_THRESHOLD_N} N -> "
                        f"{'stable' if stable else 'UNSTABLE'}.",
                        ["accept", "flip"],
                    )
                    if answer == "flip":
                        stable = not stable
                    table.parent.mkdir(parents=True, exist_ok=True)
                    new = not table.exists()
                    with table.open("a", newline="") as fh:
                        w = csv.writer(fh)
                        if new:
                            w.writerow(["method", "rtt", "grip", "K", "hf_rms_N", "stable", "run"])
                        w.writerow([method, rtt, grip, k, metrics["hf_rms_N"], stable, runs[0]])
                    if not stable:
                        break
                    k_max = k
                print(f"[e5] K_max ({method}, rtt {rtt}, {grip}) = {k_max} N/m  (table: {table})")


if __name__ == "__main__":
    main()
