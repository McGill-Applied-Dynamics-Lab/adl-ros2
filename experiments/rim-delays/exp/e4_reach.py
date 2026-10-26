r"""E4 — Reach sweep: a mass tuned at one configuration is wrong elsewhere.

Scripted leader (no hand): a slow line from ``START`` to ``LINE_END`` with a small dither along
it, so inertial force is present throughout. M4's mass is tuned at the start of the line (nᵀΛ₀n
along the line). Reports e_F against the position along the line and, for RIM, the model's Λ
eigenvalues there. Choose the line with ``tools/lambda_map.py --line``; stay well short of the
singularity.

    pixi run -e humble python experiments/rim-delays/exp/e4_reach.py [--sim] [--only-method ...] [--only-rtt ...]
"""

from __future__ import annotations

import numpy as np
from common import Condition, mass_along, parse, run_experiment, start_inertia

import exp_metrics

METHODS = ["proxy-fixed-mass", "proxy-rim"]
RTTS = [0, 2600]  # [ms]
START = [0.35, 0.0, 0.25]  # [m]
LINE_END = [0.70, 0.0, 0.25]  # [m]
SPEED = 0.02  # peak speed along the line [m/s]
DITHER_AMPLITUDE = 0.01  # [m]
DITHER_HZ = 1.5
BASE = {
    "interface_axis": "xyz",
    "contact_surface": 0.083,
    "home": True,
    "start_position": START,
    "device": "scripted",
    "script": "line",
    "script_line_end": LINE_END,
    "script_speed": SPEED,
    "script_dither_amplitude": DITHER_AMPLITUDE,
    "script_dither_hz": DITHER_HZ,
}


def conditions() -> list[Condition]:
    lam0 = start_inertia(START)
    n = np.subtract(LINE_END, START)
    out = []
    for m in METHODS:
        flags = {"proxy_mass": round(mass_along(lam0["Lambda_rim"], n), 3)} if m == "proxy-fixed-mass" else {}
        out += [Condition(m, rtt, flags) for rtt in RTTS]
    return out


if __name__ == "__main__":
    args = parse(__doc__.split("\n")[0])
    run_experiment("e4", [] if args.analyse else conditions(), BASE, exp_metrics.e4, args)
