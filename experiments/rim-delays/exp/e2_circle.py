r"""E2 — Horizontal circle (anisotropy): a fixed mass cannot follow an inertia that varies with the direction.

Scripted leader (no hand) on a circle in the air: radius 10 cm, period 2 s, centered at
(0.5, 0, 0.25) m, from its own start (0.5, 0, 0.3) m (EXP_PLAN: the common start folds joint 4 too
far for a fast circle). Reports e_F split into its radial (inertia) and tangential (friction)
parts, per phase, and the angle of the felt and robot forces to the centripetal direction. M4's
mass is nᵀΛ₀n along the initial motion direction (+y).

    pixi run -e humble python experiments/rim-delays/exp/e2_circle.py [--sim] [--only-method ...] [--only-rtt ...]
"""

from __future__ import annotations

from common import Condition, mass_along, parse, run_experiment, start_inertia

import exp_metrics

METHODS = ["proxy-god-object", "proxy-fixed-mass", "proxy-rim"]
RTTS = [0, 2600]  # [ms]
START = [0.5, 0.0, 0.3]  # [m]
CIRCLE_Z = 0.25  # [m]
RADIUS = 0.1  # [m]
PERIOD_S = 2.0
LAPS = 4
BASE = {
    "interface_axis": "xyz",
    "contact_surface": 0.083,
    "home": True,
    "start_position": START,
    "device": "scripted",
    "script": "circle",
    "script_surface_z": CIRCLE_Z + 0.01,  # the circle runs at surface - depth
    "script_depth": 0.01,
    "script_radius": RADIUS,
    "script_period_s": PERIOD_S,
    "script_laps": LAPS,
}


def conditions() -> list[Condition]:
    lam0 = start_inertia(START)
    out = []
    for m in METHODS:
        flags = {"proxy_mass": round(mass_along(lam0["Lambda_rim"], [0.0, 1.0, 0.0]), 3)} if m == "proxy-fixed-mass" else {}
        out += [Condition(m, rtt, flags) for rtt in RTTS]
    return out


if __name__ == "__main__":
    args = parse(__doc__.split("\n")[0])
    run_experiment("e2", [] if args.analyse else conditions(), BASE, exp_metrics.e2, args)
