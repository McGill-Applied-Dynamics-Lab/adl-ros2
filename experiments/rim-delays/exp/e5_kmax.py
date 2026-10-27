r"""E5 — Maximum stable coupling stiffness (a Z-width measurement), on the Inverse3: the E5-lite.

One run per K step of each ladder in ``PLAN`` (method, delay, grip, K steps, search): tap and hold
against the table for ``SECONDS[grip]``. After each run the verdict is proposed from the felt force — its
HF RMS (> 8 Hz) in contact against ``HF_THRESHOLD_N``, and the chatter guard (a trip is unstable) —
with the energy balance shown: under delay P-P goes unstable slowly (~1/(2T), well below 8 Hz), which
the HF RMS does not see, so you confirm or flip the verdict by feel. With the ``dropped`` grip the
slow instability is measured: the run is also unstable if the loop threw the free handle more than
``MAX_REBOUND_MM`` above the table, from ``RELEASE_S`` after its first touch (hands off) to the end. Every step is appended to ``<OUT>/<robot|sim>/e5/kmax.csv``.

Searches: ``down`` runs from the top step and stops at the first stable one (K_max = that step);
``up`` stops at the first unstable one (K_max = the last stable step).

E5-lite (2026-10-26) instead of the full grid (2 methods x 4 delays x 2 grips x 8 steps, ~1.5-2 h):
K and D are frozen (K 1000, D 70, β 0.2), T = 0 is known stable for M1 and 5G behaves like T = 0. What
E5 still gives: M1's K_max at 850 ms and 2.6 s (the K it uses in E6, ``K_M1`` in ``e6_taps.py``) and
M5 staying stable at 2.6 s where M1 is not. Grips: ``held`` (hold the handle), ``released`` (drop the
handle onto the table and pick it up, 3 times: the first E5-lite session), ``dropped`` (let go of it
once, hands off for the rest of the run: the worst case, no hand damping).

Dropped ladders (added 2026-10-27): held, M1 was stable at K = 1000 at both delays — the hand damps
P-P's slow oscillation, and the HF RMS cannot see it. Dropping the handle removes the hand, so the
comparison is M1 vs M5 at the same K, delay and grip. Ladders already done are skipped (their recorded
verdicts are reused), so running the whole plan again runs only the new ones.

The free handle needs a faster velocity filter (``VEL_FILTER_HZ``, 2026-10-27): at the frozen 8 Hz the
coupling damper D acts on a velocity lagging ~100° at 17 Hz, i.e. as a ~560 N/m spring with slightly
negative damping, and the released handle (~0.06 kg) rings at ~18 Hz before any contact, whatever the
method or delay (phase margin ~0° with the device's ~2–3 ms latency; ~30° at 50 Hz). The hand hides
it in held runs. The first dropped step, M5 at T = 0, checks the free handle before any delay.

    pixi run -e humble python experiments/rim-delays/exp/e5_kmax.py [--only-method ...] [--only-rtt ...]
"""

from __future__ import annotations

import csv
import datetime
import json

import exp_metrics
from common import OUT, Condition, ask, parse, run_experiment, rtt_label

PLAN = [
    # (method, round trip [ms], grip, K steps [N/m], search)
    ("zoh", 850, "held", [1000, 500, 250, 125], "down"),
    ("zoh", 2600, "held", [1000, 500, 250, 125], "down"),
    ("proxy-rim", 2600, "held", [1000, 2000], "up"),
    ("proxy-rim", 2600, "released", [1000, 2000], "up"),  # optional: the worst case
    # the worst case, hands off: the free handle at T = 0 first (device check), M5 (the reference), then M1
    ("proxy-rim", 0, "dropped", [1000], "up"),
    ("proxy-rim", 2600, "dropped", [1000], "up"),
    ("zoh", 850, "dropped", [1000, 500, 250, 125], "down"),
    ("zoh", 2600, "dropped", [1000, 500, 250, 125], "down"),
]
# shown once the robot is ready, before the forces start (fr3_teleop --start-prompt)
INSTRUCTIONS = {
    "held": "E5 {method}, T = {rtt} ms, K = {k} N/m — HANDLE HELD\n"
            "Hold the handle. Tap down onto the table and hold ~3 s, about 3 times in {seconds:.0f} s.\n"
            "Hold it as you would to work: firm, not stiff. If it starts to oscillate, keep holding ~2 s, then lift.",
    "released": "E5 {method}, T = {rtt} ms, K = {k} N/m — HANDLE RELEASED (worst case)\n"
                "Bring the handle down onto the table, then let go of it, about 3 times in {seconds:.0f} s.\n"
                "Watch whether it settles on the table or bounces/oscillates; catch it if it runs away.",
    "dropped": "E5 {method}, T = {rtt} ms, K = {k} N/m — HANDLE DROPPED, HANDS OFF (worst case)\n"
               "Bring the handle down onto the table and let go of it ONCE, within 1 s of touching it.\n"
               "Then hands off for the rest of the {seconds:.0f} s: do not lift it at the end, the run ends by itself.\n"
               "Keep your hand near it; catch it only if it hits the workspace edge (then flip the verdict to unstable).",
}
HF_THRESHOLD_N = 0.5  # instability: HF RMS (> 8 Hz) of the felt force in contact above this [world N]
MAX_REBOUND_MM = 10.0  # dropped grip: unstable if the free handle was thrown higher above the table [mm]
VEL_FILTER_HZ = {"dropped": 50.0}  # leader velocity low-pass per grip [Hz] (default: teleop.yaml's 8 Hz)
RELEASE_S = 2.0  # dropped grip: hands off from this long after the handle's first touch of the table [s]
SECONDS = {"held": 20.0, "released": 20.0, "dropped": 30.0}  # dropped: ~5 periods of P-P's ~1/(2T) at 2.6 s
START = [0.4, 0.0, 0.2]
BASE = {
    "interface_axis": "xyz",
    "contact_surface": 0.083,
    "home": True,
    "start_position": START,
    "device": "inverse3",
    "force": True,
}


def verdicts(table) -> dict[str, bool]:
    """``run dir -> stable`` for the steps already in ``kmax.csv`` (the operator's verdict)."""
    if not table.exists():
        return {}
    with table.open(newline="") as fh:
        return {row["run"]: row["stable"] == "True" for row in csv.DictReader(fh)}


def analyse(run):
    return exp_metrics.e5(run, HF_THRESHOLD_N, MAX_REBOUND_MM, RELEASE_S)


def main() -> None:
    args = parse(__doc__.split("\n")[0])
    if args.analyse:
        run_experiment("e5", [], BASE, analyse, args)
        return
    out = OUT / ("sim" if args.sim else "robot") / "e5"
    table = out / "kmax.csv"
    started = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    session = out / "sessions" / f"{started}.txt"  # one for the whole E5 run
    ladders = [p for p in PLAN if (not args.only_method or p[0] in args.only_method)
               and (not args.only_rtt or rtt_label(p[1]) in {rtt_label(r) for r in args.only_rtt})]
    total = sum(len(p[3]) for p in ladders)  # at most: a ladder stops early
    step, homed = 0, False
    print(f"[e5] {len(ladders)} ladders, at most {total} runs -> {out}  (session list: {session})")
    for method, rtt, grip, steps, search in ladders:
        print(f"\n[e5] ===== {method}, rtt {rtt}, grip {grip}: K {steps} ({search})")
        k_max = None
        for k in steps:
            step += 1
            prompt = INSTRUCTIONS[grip].format(method=method, rtt=rtt, k=k, seconds=SECONDS[grip])
            flags = {"kv": k, "seconds": SECONDS[grip], "start_prompt": prompt}
            if grip in VEL_FILTER_HZ:
                flags["vel_filter_hz"] = VEL_FILTER_HZ[grip]
            runs = run_experiment("e5", [Condition(method, rtt, flags, f"{grip}_K{k}")], BASE,
                                  analyse, args, home=not homed, session=session, progress=f"{step}/{total} (max)")
            homed = homed or any(r.name[:15] >= started for r in runs)  # home on the first run actually launched
            if args.dry_run:
                continue
            if not runs:
                break
            metrics = json.loads((runs[0] / "analysis" / "metrics.json").read_text())
            recorded = verdicts(table).get(str(runs[0]))
            if recorded is not None:  # done before: keep its verdict, do not ask or log it again
                print(f"[e5] K = {k}: recorded {'stable' if recorded else 'UNSTABLE'} ({runs[0].name})")
                stable = recorded
            if recorded is None:
                stable = metrics["stable"]
                dropped = (f", thrown {metrics['rebound_mm']:.0f} mm above the table (limit {MAX_REBOUND_MM:g}), "
                           f"{metrics['rebounds']} rebound(s)") if grip == "dropped" else ""
                answer = ask(f"[e5] HF RMS {metrics['hf_rms_N']:.2f} N (limit {HF_THRESHOLD_N}), guard trips {metrics['guard_trips']}, "
                             f"energy min {metrics['E_min_J']:.1f} J{dropped} -> {'stable' if stable else 'UNSTABLE'}. By feel?",
                             ["accept", "flip"])
                if answer == "flip":
                    stable = not stable
                table.parent.mkdir(parents=True, exist_ok=True)
                new = not table.exists()
                with table.open("a", newline="") as fh:
                    w = csv.writer(fh)
                    if new:
                        w.writerow(["method", "rtt", "grip", "K", "search", "hf_rms_N", "guard_trips", "E_min_J", "stable", "run"])
                    w.writerow([method, rtt, grip, k, search, metrics["hf_rms_N"], metrics["guard_trips"], metrics["E_min_J"],
                                stable, runs[0]])
            if search == "down" and stable:
                k_max = k
                break
            if search == "up":
                if not stable:
                    break
                k_max = k
        print(f"[e5] K_max ({method}, rtt {rtt}, {grip}) = {k_max} N/m  (table: {table})")


if __name__ == "__main__":
    main()
