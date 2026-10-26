r"""E4 — Reach sweep: a mass tuned at one configuration is wrong elsewhere.

Scripted leader (no hand): a slow line from ``START`` to ``LINE_END`` with a dither along
``DITHER_DIRECTION``, so inertial force is present throughout. M4's mass is tuned at the start of
the line (nᵀΛ₀n along the dither). Reports, per position along the line, the robot's effective mass
along the dither, the mass each method renders and its error, phase-separated at the dither
frequency (friction, in phase with the velocity, drops out), and the model's Λ there.

Design (2026-10-26, label ``zdither``): outward along x at z = 0.15 m, where the held Λ along z
falls from ~16 to ~7 kg, with a vertical dither of 3 cm at 0.7 Hz — well below the proxy's
coupling resonance (√(K/m)/2π, 1.3–1.9 Hz along the line), where the robot tracks the proxy. The
first design (x 0.35 → 0.70 m at z = 0.25 m, 1 cm at 1.5 Hz along the line) sat on that resonance:
the proxy swung 1.7× the leader, the robot followed half of it, and Λ along x hardly changed
(12.7 → 9.5 → 10.9 kg).

    pixi run -e humble python experiments/rim-delays/exp/e4_reach.py [--sim] [--only-method ...] [--only-rtt ...]
"""

from __future__ import annotations

import exp_metrics
from common import Condition, mass_along, parse, run_experiment, start_inertia

METHODS = ["proxy-fixed-mass", "proxy-rim"]
RTTS = [0, 2600]  # [ms]
START = [0.30, 0.0, 0.15]  # [m]
LINE_END = [0.68, 0.0, 0.15]  # [m]
SPEED = 0.01  # peak speed along the line [m/s]: ~70 s out, ~50 dither cycles
RETURN_SPEED = 0.05  # back to the start [m/s]
DITHER_DIRECTION = [0.0, 0.0, 1.0]
DITHER_AMPLITUDE = 0.03  # [m]; the table is 3.7 cm below the lowest point
DITHER_HZ = 0.7
LABEL = "zdither"
BASE = {
    "interface_axis": "xyz",
    "contact_surface": 0.083,
    "home": True,
    "start_position": START,
    "device": "scripted",
    "chatter_hz": 0,  # no chatter guard: no device feels the force; on 5G it zeroed f for 3 s (E1, ZOH)
    "script": "line",
    "script_line_end": LINE_END,
    "script_speed": SPEED,
    "script_return_speed": RETURN_SPEED,
    "script_dither_direction": DITHER_DIRECTION,
    "script_dither_amplitude": DITHER_AMPLITUDE,
    "script_dither_hz": DITHER_HZ,
}


def conditions() -> list[Condition]:
    lam0 = start_inertia(START)
    out = []
    for m in METHODS:
        flags = {"proxy_mass": round(mass_along(lam0["Lambda_rim"], DITHER_DIRECTION), 3)} if m == "proxy-fixed-mass" else {}
        out += [Condition(m, rtt, flags, LABEL) for rtt in RTTS]
    return out


if __name__ == "__main__":
    args = parse(__doc__.split("\n")[0])
    run_experiment("e4", [] if args.analyse else conditions(), BASE, exp_metrics.e4, args)
