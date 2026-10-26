r"""E3 — Contact onset over approach velocity: model-based vs P-P, and the inertial transient with speed.

Scripted leader (no hand): a vertical approach at a constant speed through the table, a hold at a
fixed leader depth, and a retract; ``REPEATS`` approaches per run. Two sweeps: the approach speeds at
``SPEED_SWEEP_RTT``, and the delays at ``DELAY_SWEEP_SPEED``. Reports the contact metrics per run
(onset latency, penetration at onset, felt overshoot, robot peak table force, e_F during the hold).

Check the FR3's collision reflex thresholds before the fast approaches.

    pixi run -e humble python experiments/rim-delays/exp/e3_approach.py [--sim] [--only-method ...] [--only-rtt ...]
"""

from __future__ import annotations

import exp_metrics
import yaml
from common import CONF, Condition, mass_along, parse, run_experiment, start_inertia

CONF_VALUES = {k: v for group in yaml.safe_load(CONF.read_text()).values() if isinstance(group, dict)
               for k, v in group.items()}

METHODS = ["zoh", "proxy-god-object", "proxy-fixed-mass", "proxy-rim"]
SPEEDS = [0.01, 0.05, 0.10, 0.20]  # approach speeds [m/s]
SPEED_SWEEP_RTT = 850  # [ms]
RTTS = [0, "5g", 850, 2600]  # [ms]
DELAY_SWEEP_SPEED = 0.05  # [m/s]
REPEATS = 3
# At long delays the hold and the rest at the pre-height must outlast the round trip: ZOH's contact
# reaches the operator ~T + 0.1 s after the leader crosses (2.7 s at 2.6 s, after a 2 s hold), and
# the robot must be back up before the next approach. Applied for T >= 2 s; 2 s / 1 s otherwise.
LONG_DELAY = {"script_hold_s": 4.0, "script_rest_s": 3.0}
# Steady-approach sweep, M3–M5 only (label "steady"): in the main sweeps the leader reaches its speed
# 3 cm above the table, and the M4/M5 proxy (a mass on the coupling, ζ ≈ 0.3, decay ~0.34 s) is still
# catching up at contact. Here the pre-height gives >= 1 s at constant speed before contact (3 decay
# times at 20 cm/s), so the proxies meet the table at the leader's speed. M4's mass is nᵀΛn at the
# contact (its best case at impact). Moves at 10 cm/s to keep the 20 cm retracts short.
STEADY_METHODS = ["proxy-god-object", "proxy-fixed-mass", "proxy-rim"]
STEADY_SPEEDS = [0.05, 0.10, 0.20]  # [m/s]
STEADY_RTT = 850  # [ms]; M3–M5 are flat in the delay
STEADY_START = [0.4, 0.0, 0.30]  # [m]
STEADY = {"script_pre_height": 0.20, "script_speed": 0.10, "start_position": STEADY_START}
START = [0.4, 0.0, 0.12]  # just above the pre-height (0.083 + 0.03 m): a short move down to it
# Clamp on the felt force f [world N]. No handle feels it (scripted leader): it only cut what the
# metrics read. M1 drags with K·v·T before contact: 170 N at 20 cm/s and 850 ms, 130 N at 5 cm/s and
# 2.6 s. The feedforward cap stays the yaml's (30 N): here f_ff pushes the robot into the table.
FELT_CLAMP_N = 250.0
BASE = {
    "interface_axis": "xyz",
    "contact_surface": 0.083,
    "home": True,
    "start_position": START,
    "device": "scripted",
    "chatter_hz": 0,  # no chatter guard: no device feels the force; on 5G it zeroed f for 3 s (E1, ZOH)
    "script": "approach",
    "script_depth": 0.01,  # leader depth below the table during the hold [m]
    "script_pre_height": 0.03,  # the constant speed starts this high above the table [m]
    "script_hold_s": 2.0,
    "script_repeats": REPEATS,
    "max_i3_force": FELT_CLAMP_N * CONF_VALUES["force_gain"],  # the clamp in handle N
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
            flags = {"script_approach_speed": v, **(m4 if m == "proxy-fixed-mass" else {}),
                     **(LONG_DELAY if rtt != "5g" and rtt >= 2000 else {})}
            out.append(Condition(m, rtt, flags, f"v{100 * v:g}cm"))
    m4_contact = {"proxy_mass": round(mass_along(start_inertia([START[0], START[1], BASE["contact_surface"] + 0.005])["Lambda_rim"],
                                                 [0.0, 0.0, 1.0]), 3)}
    for v in STEADY_SPEEDS:
        for m in STEADY_METHODS:
            flags = {"script_approach_speed": v, **STEADY, **(m4_contact if m == "proxy-fixed-mass" else {})}
            out.append(Condition(m, float(STEADY_RTT), flags, f"v{100 * v:g}cm_steady"))
    return out


if __name__ == "__main__":
    args = parse(__doc__.split("\n")[0])
    run_experiment("e3", [] if args.analyse else conditions(), BASE, exp_metrics.e3, args)
