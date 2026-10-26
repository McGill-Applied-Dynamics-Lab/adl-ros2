r"""E6 — Contact taps and holds on the Inverse3 (and the talk video).

You hold the handle and tap and hold on the table, ``TAPS`` times, during ``SECONDS``. Reports the
contact metrics per tap (onset latency, penetration at onset, felt overshoot, settling, bounces,
robot peak table force, HF force RMS) averaged per run, and the energy balance. Start the camera by
hand before each run.

M1 uses the common coupling K; where it is unstable, set its K per delay in ``K_M1`` (70 % of its
K_max(T) from E5) and report it.

    pixi run -e humble python experiments/rim-delays/exp/e6_taps.py [--only-method ...] [--only-rtt ...]
"""

from __future__ import annotations

from common import Condition, mass_along, parse, run_experiment, start_inertia

import exp_metrics

METHODS = ["zoh", "proxy-god-object", "proxy-fixed-mass", "proxy-rim"]
RTTS = [0, "5g", 850, 2600]  # [ms]
K_M1 = {}  # e.g. {2600: 300}: M1's coupling K per delay where the common K is unstable [N/m]
TAPS = 5
SECONDS = 60.0
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


def conditions() -> list[Condition]:
    m4 = round(mass_along(start_inertia(START)["Lambda_rim"], [0.0, 0.0, 1.0]), 3)
    out = []
    for m in METHODS:
        for rtt in RTTS:
            flags = {"proxy_mass": m4} if m == "proxy-fixed-mass" else {}
            if m == "zoh" and rtt in K_M1:
                flags["kv"] = K_M1[rtt]
            out.append(Condition(m, rtt, flags, f"{TAPS}taps"))
    return out


if __name__ == "__main__":
    args = parse(__doc__.split("\n")[0])
    run_experiment("e6", [] if args.analyse else conditions(), BASE, exp_metrics.e6, args)
