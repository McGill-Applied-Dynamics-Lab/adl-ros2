r"""T2 — ZOH with no delay: the reference every method is compared to.

Tap and press the table, move in free space. Reports the felt wall stiffness (world and at the
handle), free-motion force, a buzz or limit cycle at the wall, the passivity energy, and how the
felt force matches the table force on the robot.

    pixi run -e humble python experiments/rim-delays/pretests/t2_zoh.py --contact-surface 0.105
    pixi run -e humble python experiments/rim-delays/pretests/t2_zoh.py --sim --kv 1500     # any fr3_teleop flag
    pixi run -e humble python experiments/rim-delays/pretests/t2_zoh.py --analyse <run dir>
"""

from __future__ import annotations

from common import finish, parse, run_test

NAME = "t2_zoh"
PRESET = {
    "method": "zoh",
    "interface_axis": "xyz",
    "feedback_delay_ms": 0,
    "command_delay_ms": 0,
    "force": True,
    "seconds": 600,
    "contact_surface": 0.41858,
}


def main(argv=None) -> None:
    args, rest = parse(__doc__.split("\n")[0], argv)
    run_dir = run_test(NAME, PRESET, args, rest)
    finish(NAME, "T2 — ZOH, no delay", run_dir, tool_length=args.tool_length)


if __name__ == "__main__":
    main()
