r"""T5 — two-port TDPA (position–position) under delay: when and how hard do the passivity controllers act?

Free motion and contact, with a delay (default: 850 ms round trip). On top of the common metrics,
per end of the two-port: the observer's minimum, how often its controller fired, its mean and
max damping, and each end's input / output energy.

    pixi run -e humble python experiments/rim-delays/pretests/t5_tdpa.py --contact-surface 0.105
    pixi run -e humble python experiments/rim-delays/pretests/t5_tdpa.py --sim --feedback-delay-ms 1300 --command-delay-ms 1300
"""

from __future__ import annotations

import numpy as np
from common import finish, parse, run_test
from experiment_logger import load_streams

NAME = "t5_tdpa"
PRESET = {
    "method": "tdpa2-zoh",
    "interface_axis": "xyz",
    "feedback_delay_ms": 425,
    "command_delay_ms": 425,
    "force": True,
    "plant_max_age_ms": 100,
    "seconds": 600,
}


def ends(run) -> dict:
    h = load_streams(run.dir, ["haptic"])["haptic"]
    out = {}
    if "tdpa_energy_j" in h and np.isfinite(h.tdpa_energy_j).any():
        w, a = h.tdpa_energy_j.to_numpy(), h.tdpa_damping.to_numpy()
        fired = a > 0
        out.update({
            "operator end: observer min [mJ]": 1e3 * float(np.nanmin(w)),
            "operator end: PC active [% of ticks]": 100 * float(np.mean(fired)),
            "operator end: damping mean / max when active [N·s/m]":
                f"{np.mean(a[fired]):.3g} / {np.max(a[fired]):.3g}" if fired.any() else "0 / 0",
        })
    r = load_streams(run.dir, ["robot_port"]).get("robot_port")
    if r is not None and len(r):
        fired = r.damping > 0
        out.update({
            "robot end: observer min [mJ]": 1e3 * float((r.remote_energy_in - r.energy_out).min()),
            "robot end: PC active [% of commands]": 100 * float(fired.mean()),
            "robot end: damping mean / max when active [N·s/m]":
                f"{r.damping[fired].mean():.3g} / {r.damping[fired].max():.3g}" if fired.any() else "0 / 0",
            "robot end: energy in / out [J]": f"{r.energy_in.iloc[-1]:.3g} / {r.energy_out.iloc[-1]:.3g}",
        })
    return out


def main(argv=None) -> None:
    args, rest = parse(__doc__.split("\n")[0], argv)
    run_dir = run_test(NAME, PRESET, args, rest)
    finish(NAME, "T5 — two-port TDPA (position–position)", run_dir, extra=ends)


if __name__ == "__main__":
    main()
