r"""E5 — Maximum stable coupling stiffness (a Z-width measurement), on the Inverse3: the E5-lite.

One run per K step of each ladder in ``PLAN`` (method, delay, grip, K steps, search): tap and hold
against the table for ``SECONDS``. After each run the verdict is proposed from the felt force — its
HF RMS (> 8 Hz) in contact against ``HF_THRESHOLD_N``, and the chatter guard (a trip is unstable) —
with the energy balance shown: under delay P-P goes unstable slowly (~1/(2T), well below 8 Hz), which
the HF RMS does not see, so you confirm or flip the verdict by feel. Every step is appended to
``<OUT>/<robot|sim>/e5/kmax.csv``.

Searches: ``down`` runs from the top step and stops at the first stable one (K_max = that step);
``up`` stops at the first unstable one (K_max = the last stable step).

E5-lite (2026-10-26) instead of the full grid (2 methods x 4 delays x 2 grips x 8 steps, ~1.5-2 h):
K and D are frozen (K 1000, D 70, β 0.2), T = 0 is known stable for M1 and 5G behaves like T = 0. What
E5 still gives: M1's K_max at 850 ms and 2.6 s (the K it uses in E6, ``K_M1`` in ``e6_taps.py``) and
M5 staying stable at 2.6 s where M1 is not. Grips: ``held`` (hold the handle), ``released`` (drop the
handle onto the table: the worst case).

    pixi run -e humble python experiments/rim-delays/exp/e5_kmax.py [--only-method ...] [--only-rtt ...]
"""

from __future__ import annotations

import csv
import json

import exp_metrics
from common import OUT, Condition, ask, parse, run_experiment, rtt_label

PLAN = [
    # (method, round trip [ms], grip, K steps [N/m], search)
    ("zoh", 850, "held", [1000, 500, 250, 125], "down"),
    ("zoh", 2600, "held", [1000, 500, 250, 125], "down"),
    ("proxy-rim", 2600, "held", [1000, 2000], "up"),
    ("proxy-rim", 2600, "released", [1000, 2000], "up"),  # optional: the worst case
]
HF_THRESHOLD_N = 0.5  # instability: HF RMS (> 8 Hz) of the felt force in contact above this [world N]
SECONDS = 20.0
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
    table = OUT / ("sim" if args.sim else "robot") / "e5" / "kmax.csv"
    ladders = [p for p in PLAN if (not args.only_method or p[0] in args.only_method)
               and (not args.only_rtt or rtt_label(p[1]) in {rtt_label(r) for r in args.only_rtt})]
    for method, rtt, grip, steps, search in ladders:
        print(f"\n[e5] ===== {method}, rtt {rtt}, grip {grip}: K {steps} ({search})")
        k_max = None
        for k in steps:
            print(f"[e5] K = {k} N/m — {'hold the handle' if grip == 'held' else 'drop the handle onto the table'}, tap and hold")
            runs = run_experiment("e5", [Condition(method, rtt, {"kv": k}, f"{grip}_K{k}")], BASE, analyse, args)
            if args.dry_run:
                continue
            if not runs:
                break
            metrics = json.loads((runs[0] / "analysis" / "metrics.json").read_text())
            stable = metrics["stable"]
            answer = ask(f"[e5] HF RMS {metrics['hf_rms_N']:.2f} N (limit {HF_THRESHOLD_N}), guard trips {metrics['guard_trips']}, "
                         f"energy min {metrics['E_min_J']:.1f} J -> {'stable' if stable else 'UNSTABLE'}. By feel?",
                         ["accept", "flip"])
            if answer == "flip":
                stable = not stable
            table.parent.mkdir(parents=True, exist_ok=True)
            new = not table.exists()
            with table.open("a", newline="") as fh:
                w = csv.writer(fh)
                if new:
                    w.writerow(["method", "rtt", "grip", "K", "search", "hf_rms_N", "guard_trips", "E_min_J", "stable", "run"])
                w.writerow([method, rtt, grip, k, search, metrics["hf_rms_N"], metrics["guard_trips"], metrics["E_min_J"],
                            stable, runs[0]])
            if search == "down" and stable:
                k_max = k
                break
            if search == "up":
                if not stable:
                    break
                k_max = k
        print(f"[e5] K_max ({method}, rtt {rtt}, {grip}) = {k_max} N/m  (table: {table})")


if __name__ == "__main__":
    main()
