r"""E1 — Free-motion transparency: sinusoids along the heaviest and lightest directions of Λ₀.

Scripted leader (no hand). Reports RMS e_F (along the motion and its norm) per run; the notebook
plots RMS e_F against the delay and against the frequency. M4's mass is retuned per direction to
nᵀΛ₀n (its best case). Λ₀ is the Λ M5 uses (orientation held or free, per rim_hold_orientation in
teleop.yaml) at the start joints (IK from home, as the robot's move).

    pixi run -e humble python experiments/rim-delays/exp/e1_free_motion.py            # every condition
    pixi run -e humble python experiments/rim-delays/exp/e1_free_motion.py --sim       # simulated FR3
    pixi run -e humble python experiments/rim-delays/exp/e1_free_motion.py --only-method proxy-rim --only-rtt 0
    pixi run -e humble python experiments/rim-delays/exp/e1_free_motion.py --dry-run   # the commands only
    pixi run -e humble python experiments/rim-delays/exp/e1_free_motion.py --analyse <run dir> ...

Mind the limits: the inertial force nᵀΛ₀n · A (2πf)² must stay under the felt-force clamp and the
feedforward cap, or e_F measures the clamps, not the methods; the script warns. E1 raises both
(``FELT_CLAMP_N``, ``FEEDFORWARD_CAP_N``): the leader is scripted, so no handle feels the felt force
(the clamp only cuts the logged f and M4/M5's f_ff = −f), and there is no contact. The feedforward
is a real force on the robot, under the OSC's 80 N limit.
"""

from __future__ import annotations

import exp_metrics
import numpy as np
import yaml
from common import CONF, Condition, mass_along, parse, run_experiment, start_inertia

CONF_VALUES = {k: v for group in yaml.safe_load(CONF.read_text()).values() if isinstance(group, dict)
               for k, v in group.items()}

# ------------------------------------------------------------------ the experiment
METHODS = ["zoh", "proxy-god-object", "proxy-fixed-mass", "proxy-rim"]
RTTS = {"zoh": [0, "5g", 850, 2600], "default": [0, 2600]}  # round trips [ms]; M3-M5 are flat in T
DIRECTIONS = ["heavy", "light"]  # eigenvectors of Λ₀
FREQUENCIES = [0.25, 0.5, 1.0]  # [Hz]
AMPLITUDE = 0.05  # [m]
CYCLES = 5
START = [0.4, 0.0, 0.2]  # [m]
FELT_CLAMP_N = 150.0  # clamp on the felt force f [world N] (ZOH at 2.6 s, 1 Hz: up to K · 2A = 100 N)
FEEDFORWARD_CAP_N = 50.0  # |f_ff| sent to the robot (M4, M5) [N]; peak Λa ~29 N (heavy, 1 Hz)
BASE = {
    "interface_axis": "xyz",
    "contact_surface": 0.083,  # the table, far below the motion
    "home": True,
    "start_position": START,
    "device": "scripted",
    "script": "sine",
    "script_amplitude": AMPLITUDE,
    "script_cycles": CYCLES,
    "max_i3_force": FELT_CLAMP_N * CONF_VALUES["force_gain"],  # the clamp in handle N
    "feedforward_cap": FEEDFORWARD_CAP_N,
}


def conditions() -> list[Condition]:
    lam0 = start_inertia(START)
    vecs = {"heavy": lam0["eigvecs"][:, 2], "light": lam0["eigvecs"][:, 0]}
    print(
        f"[e1] Λ₀ eigenvalues {lam0['eigvals'].round(2)} kg; heavy {vecs['heavy'].round(3)}, light {vecs['light'].round(3)}"
    )
    limit = min(FELT_CLAMP_N, FEEDFORWARD_CAP_N)
    for d in DIRECTIONS:
        for f in FREQUENCIES:
            peak = mass_along(lam0["Lambda_rim"], vecs[d]) * AMPLITUDE * (2 * np.pi * f) ** 2
            if peak > limit:
                print(
                    f"[e1] WARNING: {d}, {f:g} Hz: inertial force ~{peak:.0f} N > felt clamp {FELT_CLAMP_N:.0f} N / "
                    f"feedforward cap {FEEDFORWARD_CAP_N:.0f} N: lower AMPLITUDE or raise them"
                )
    out = []
    for d in DIRECTIONS:
        n = vecs[d]
        for f in FREQUENCIES:
            for m in METHODS:
                flags = {"script_direction": n.tolist(), "script_frequency": f}
                if m == "proxy-fixed-mass":
                    flags["proxy_mass"] = round(mass_along(lam0["Lambda_rim"], n), 3)
                for rtt in RTTS.get(m, RTTS["default"]):
                    out.append(Condition(m, rtt, flags, f"{d}_f{f:g}"))
    return out


if __name__ == "__main__":
    args = parse(__doc__.split("\n")[0])
    run_experiment("e1", [] if args.analyse else conditions(), BASE, exp_metrics.e1, args)
