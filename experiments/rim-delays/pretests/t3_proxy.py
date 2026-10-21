r"""T3 — proxy without the robot's model: fixed mass (default) or god-object (``--method proxy-god-object``).

Free motion, then contact. Reports the felt stiffness, how far the commanded proxy goes into the
table (should be 0: the proxy stops at the wall), the free-motion force, the buzz at the wall, and
whether the robot presses the table with the force the operator feels.

    pixi run -e humble python experiments/rim-delays/pretests/t3_proxy.py --contact-surface 0.105
    pixi run -e humble python experiments/rim-delays/pretests/t3_proxy.py --sim --proxy-mass 2.0
    pixi run -e humble python experiments/rim-delays/pretests/t3_proxy.py --method proxy-god-object --contact-surface 0.105
"""

from __future__ import annotations

import numpy as np
from common import finish, parse, run_test

NAME = "t3_proxy"
PRESET = {
    "method": "proxy-fixed-mass",
    "interface_axis": "xyz",
    "feedback_delay_ms": 0,
    "command_delay_ms": 0,
    "force": True,
    "seconds": 600,
    # "contact_surface": 0.41858,
}


def proxy_penetration(run) -> dict:
    if run.cmd_t is None or run.surface is None:
        return {}
    k = run.ca if run.cmd_x.shape[1] > 1 else 0
    depth = 1e3 * float(max(0.0, run.surface - run.cmd_x[:, k].min()))
    return {
        "proxy below the table, max [mm]": depth,
        "leader below the table, max [mm]": 1e3 * float(max(0.0, run.surface - np.min(run.zl))),
    }


def main(argv=None) -> None:
    args, rest = parse(__doc__.split("\n")[0], argv)
    run_dir = run_test(NAME, PRESET, args, rest)
    finish(NAME, "T3 — proxy (no robot model), no delay", run_dir, extra=proxy_penetration, tool_length=args.tool_length)


if __name__ == "__main__":
    main()
