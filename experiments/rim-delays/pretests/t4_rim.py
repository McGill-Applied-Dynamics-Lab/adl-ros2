r"""T4 — RIM: does the proxy predict the robot, and does the robot press with the force felt?

Free motion, then contact. On top of T3's metrics, the **prediction error**: the target sent to the
robot (the proxy) against where the robot actually is one command delay later, per axis, in free
motion and in contact. Add a delay to see that the prediction holds under it.

    pixi run -e humble python experiments/rim-delays/pretests/t4_rim.py --contact-surface 0.105
    pixi run -e humble python experiments/rim-delays/pretests/t4_rim.py --sim --feedback-delay-ms 425 --command-delay-ms 425
"""

from __future__ import annotations

from common import finish, parse, prediction_error, run_test
from t3_proxy import proxy_penetration

NAME = "t4_rim"
PRESET = {
    "method": "proxy-rim",
    "interface_axis": "xyz",
    "feedback_delay_ms": 0,
    "command_delay_ms": 0,
    "force": True,
    "seconds": 600,
}


def extra(run) -> dict:
    out = proxy_penetration(run)
    out.update({f"prediction error: {k}": v for k, v in prediction_error(run).items()})
    return out


def main(argv=None) -> None:
    args, rest = parse(__doc__.split("\n")[0], argv)
    run_dir = run_test(NAME, PRESET, args, rest)
    finish(NAME, "T4 — RIM (arm only)", run_dir, extra=extra)


if __name__ == "__main__":
    main()
