r"""E6 — Contact taps and holds on the Inverse3 (and the talk video).

You hold the handle and follow the guide window (``fr3_teleop --guide``): a reference height that
scrolls in from the right, over the table, your hand and (M3–M5) the proxy, all local and undelayed.
``GUIDE`` alternates taps (fast, no hold) and holds (``HOLD_S`` at ``DEPTH_MM`` below the table, about
K × depth of force for every method), so every condition gets the same motion. Reports the contact
metrics per contact (from the leader's crossing to when it left the table: onset latency,
penetration at onset, time to the hold force, felt overshoot, settling, bounces, robot peak table
force, HF force RMS after the impact, whether the force arrived before the lift), averaged per run
and per kind (``tap_*``, ``hold_*``), how closely the hand followed the guide, and the energy balance.

Every method uses the frozen coupling (K 1000, D 70), M1 at every delay too: an unstable M1 run is
a result, kept. M4's mass is nᵀΛ₀n along z at the contact (the table under the start position), as
in E3. The run starts with the hand at the guide's hover height (the start position).

Only z is teleoperated: the leader's x, y are locked at the start (``fr3_teleop --lock-axes xy``), so
every contact lands on the same spot, and a light spring centres the handle there. The interface
stays xyz, so the methods and M5's 3-D model are unchanged: the proxy's x, y are held by the
coupling to the fixed leader, and any sideways motion the arm's coupled inertia causes is predicted.

Video: ``--video`` runs the conditions in ``VIDEO`` only, with a prompt to start the camera, as clips
for the talk rather than data: a shorter guide (``VIDEO_GUIDE``), from higher (``VIDEO_HOVER_MM``, so
the impacts show), each clip with its own hold; saved as their own experiment (``e6_video``: its own runs and sessions, so the E6 data and its notebook are
untouched; a take already done is skipped, ``--rerun`` for another). Render
the leader / proxy / robot positions next to or over the clip with ``tools/contact_video.py``.

    pixi run -e humble python experiments/rim-delays/exp/e6_taps.py [--only-method ...] [--only-rtt ...]

To let someone try it, nothing saved: ``--no-save`` (e.g. ``--no-save --only-rtt 2600``).
"""

from __future__ import annotations

import exp_metrics
from common import Condition, mass_along, parse, run_experiment, start_inertia
from fr3_haptic.guide import Guide, GuideConfig

METHODS = ["zoh", "proxy-god-object", "proxy-fixed-mass", "proxy-rim"]
RTTS = [0, "5g", 850, 2600]  # [ms]
# --video: clips cut for the talk, not compared with each other (E6's runs carry the numbers): M1 at
# 0 and 2.6 s in the intro (the problem), M5 at 2.6 s as the closing demo
VIDEO = {  # (method, rtt): the hold of its clip [s]
    ("zoh", 0): 1.5,  # the reference: P-P with no delay
    ("zoh", 2600): 4.0,  # the problem: the hold must outlast the round trip for the table to be felt at all
    ("proxy-rim", 2600): 1.5,  # the demo: contact at once despite the delay
}
VIDEO_EXP = "e6_video"  # their own folder and sessions (the E6 data untouched)
VIDEO_GUIDE = "tap,hold,tap"  # shorter
VIDEO_HOVER_MM = 100.0  # start and hover higher: the descent and the impact show on camera [mm]
GUIDE = "tap,hold,tap,hold,tap,hold"
HOLD_S = 4.0  # at depth, for a hold: M1 at 2.6 s still feels ~1.4 s of it (at 2 s it never did) [s]
DEPTH_MM = 10.0  # below the table: ~10 N at K = 1000 [mm]
HOVER_MM = 30.0  # between contacts; the start height [mm]
TAP_CMS = 8.0  # descent of a tap: what an Inverse3 hand follows (20 cm/s was too fast) [cm/s]
TAP_DWELL_S = 0.3  # at depth, for a tap [s]
# shown once the robot is ready, before the forces start (fr3_teleop --start-prompt)
INSTRUCTIONS = (
    "E6 {method}, T = {rtt} — follow the guide window: taps and holds, handle held\n"
    "{camera}Keep the blue dot (your hand) on the green line; the next seconds come in from the right.\n"
    "Same grip for every condition. Click back on this terminal, then Enter."
)
CAMERA = "START THE CAMERA (side view: the tool and the table in frame).\n"
SURFACE = 0.083  # table height [m]
START = [0.4, 0.0, SURFACE + 1e-3 * HOVER_MM]
TAIL_S = 1.0  # after the guide (its last 2 s pause included) and the command delay [s]
CENTER_K, CENTER_D = 150.0, 10.0  # handle spring on x, y [world N/m, N·s/m]: 30 N/m, 2 N·s/m at the handle (β 0.2)
BASE = {
    "interface_axis": "xyz",
    "lock_axes": "xy",  # the leader stays at the start in x, y: taps on one spot, z only commanded
    "free_axis_stiffness": CENTER_K,
    "free_axis_damping": CENTER_D,
    "contact_surface": SURFACE,
    "home": True,  # start with the robot at the home pose (the handle is at the start position)
    "start_position": START,
    "device": "inverse3",
    "force": True,
    "guide": GUIDE,
    "guide_hold_s": HOLD_S,
    "guide_depth_mm": DEPTH_MM,
    "guide_hover_mm": HOVER_MM,
    "guide_tap_cms": TAP_CMS,
    "guide_tap_dwell_s": TAP_DWELL_S,
}


def guide_seconds(pattern: str = GUIDE, hold_s: float = HOLD_S, hover_mm: float = HOVER_MM) -> float:
    """How long the guide takes [s]."""
    cfg = GuideConfig(pattern=tuple(pattern.split(",")), hold_s=hold_s, depth=1e-3 * DEPTH_MM, hover=1e-3 * hover_mm,
                      tap_speed=1e-2 * TAP_CMS, tap_dwell_s=TAP_DWELL_S)
    return Guide(cfg, SURFACE).duration_s


GUIDE_S = guide_seconds()


def seconds(rtt, guide_s: float = GUIDE_S) -> float:
    """Run length: the guide, then the robot's last lift (``T_c`` later) and ``TAIL_S``; the hand
    waits 3-4.3 s after its last lift."""
    one_way = 0.0 if rtt == "5g" else float(rtt) / 2e3  # 5G: ~20 ms
    return round(guide_s + one_way + TAIL_S, 1)


def conditions(video: bool = False) -> list[Condition]:
    """Every method × delay; ``video``: the ``VIDEO`` ones only, with the camera prompt and the clip's
    own guide (``VIDEO_GUIDE``, its hold, from ``VIDEO_HOVER_MM``)."""
    # M4's best case where the contact happens: at the table under the start position
    m4 = round(mass_along(start_inertia([START[0], START[1], SURFACE])["Lambda_rim"], [0.0, 0.0, 1.0]), 3)
    out = []
    for m in METHODS:
        for rtt in RTTS:
            if video and (m, rtt) not in VIDEO:
                continue
            flags = {"proxy_mass": m4} if m == "proxy-fixed-mass" else {}
            flags["seconds"] = seconds(rtt)
            if video:
                hold = VIDEO[(m, rtt)]
                flags.update({
                    "guide": VIDEO_GUIDE, "guide_hold_s": hold, "guide_hover_mm": VIDEO_HOVER_MM,
                    "start_position": [START[0], START[1], SURFACE + 1e-3 * VIDEO_HOVER_MM],
                    "seconds": seconds(rtt, guide_seconds(VIDEO_GUIDE, hold, VIDEO_HOVER_MM)),
                })
            flags["start_prompt"] = INSTRUCTIONS.format(
                method=m, rtt=rtt if rtt == "5g" else f"{rtt} ms", camera=CAMERA if video else ""
            )
            out.append(Condition(m, rtt, flags, "guided"))
    return out


if __name__ == "__main__":
    args = parse(__doc__.split("\n")[0], extra=lambda p: p.add_argument(
        "--video", action="store_true", help=f"the camera takes: the VIDEO conditions, saved under {VIDEO_EXP}/"))
    exp = VIDEO_EXP if args.video else "e6"
    run_experiment(exp, [] if args.analyse else conditions(video=args.video), BASE, exp_metrics.e6, args)
