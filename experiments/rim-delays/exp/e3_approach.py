r"""E3 — Contact onset over approach velocity: model-based vs P-P, and the inertial transient with speed.

Scripted leader (no hand): a vertical approach at a constant speed through the table, a hold at a
fixed leader depth, and a retract; ``REPEATS`` approaches per run. Two sweeps: the approach speeds at
``SPEED_SWEEP_RTT``, and the delays at ``DELAY_SWEEP_SPEED``. Reports the contact metrics per run
(onset latency, penetration at onset, felt overshoot, robot peak table force, e_F during the hold).

Check the FR3's collision reflex thresholds before the fast approaches.

    pixi run -e humble python experiments/rim-delays/exp/e3_approach.py [--sim] [--only-method ...] [--only-rtt ...]
"""

from __future__ import annotations

from common import Condition, mass_along, parse, run_experiment, start_inertia

import exp_metrics

METHODS = ["zoh", "proxy-god-object", "proxy-fixed-mass", "proxy-rim"]
SPEEDS = [0.01, 0.05, 0.10, 0.20]  # approach speeds [m/s]
SPEED_SWEEP_RTT = 850  # [ms]
RTTS = [0, "5g", 850, 2600]  # [ms]
DELAY_SWEEP_SPEED = 0.05  # [m/s]
REPEATS = 3
START = [0.4, 0.0, 0.2]
BASE = {
    "interface_axis": "xyz",
    "contact_surface": 0.083,
    "home": True,
    "start_position": START,
    "device": "scripted",
    "script": "approach",
    "script_depth": 0.01,  # leader depth below the table during the hold [m]
    "script_pre_height": 0.03,  # the constant speed starts this high above the table [m]
    "script_hold_s": 2.0,
    "script_repeats": REPEATS,
}


def conditions() -> list[Condition]:
    lam0 = start_inertia(START)
    m4 = {"proxy_mass": round(mass_along(lam0["Lambda_rim"], [0.0, 0.0, 1.0]), 3)}
    pairs = [(SPEED_SWEEP_RTT, v) for v in SPEEDS] + [(r, DELAY_SWEEP_SPEED) for r in RTTS]
    pairs = list(dict.fromkeys((str(r), v) for r, v in pairs))  # the shared point once
    out = []
    for rtt, v in pairs:
        rtt = rtt if rtt == "5g" else float(rtt)
        for m in METHODS:
            flags = {"script_approach_speed": v, **(m4 if m == "proxy-fixed-mass" else {})}
            out.append(Condition(m, rtt, flags, f"v{100 * v:g}cm"))
    return out


if __name__ == "__main__":
    args = parse(__doc__.split("\n")[0])
    run_experiment("e3", [] if args.analyse else conditions(), BASE, exp_metrics.e3, args)
