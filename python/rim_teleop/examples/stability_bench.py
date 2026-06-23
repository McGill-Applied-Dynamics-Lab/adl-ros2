"""Run one haptic stability-bench trial (CH-62) — robot-free.

Couples the Inverse3 to a constant 1-DoF virtual mass via a spring/damper and
records the response. Sweep `--stiffness` / `--rate` across runs to map the
stability envelope; hold the device still (or let it rest) and watch for the
onset of buzzing/limit-cycle as gains increase.

Usage:
    pixi run -e humble python python/rim_teleop/examples/stability_bench.py \
        --mass 1.0 --stiffness 1000 --damping 90 --rate 1000 --duration 10 \
        --notes "z-axis, baseline gains"
"""

from __future__ import annotations

import argparse

import numpy as np
from rim_teleop.bench import HapticBench, HapticBenchConfig, plot_run, save_bench_run


def main() -> None:
    p = argparse.ArgumentParser(description="Haptic stability bench (Inverse3 + virtual mass)")
    p.add_argument("--mass", type=float, default=1.0, help="Virtual mass [kg]")
    p.add_argument("--stiffness", type=float, default=1000.0, help="Coupling stiffness K [N/m]")
    p.add_argument("--damping", type=float, default=90.0, help="Coupling damping D [Ns/m]")
    p.add_argument("--rate", type=float, default=1000.0, help="Feedback loop rate [Hz]")
    p.add_argument("--duration", type=float, default=10.0, help="Run length [s]")
    p.add_argument("--force-cap", type=float, default=12.0, help="Max force to device [N]")
    p.add_argument("--vel-filter-alpha", type=float, default=1.0, help="IIR alpha on leader velocity; 1.0 = off")
    p.add_argument("--uri", default="ws://localhost:10001", help="Inverse3 websocket URI")
    p.add_argument("--notes", default="", help="Free-text purpose, saved to metadata")
    p.add_argument("--no-save", action="store_true", help="Run without writing a data directory")
    p.add_argument("--plot", default=True, action="store_true", help="Show diagnostic plots after the run")
    args = p.parse_args()

    cfg = HapticBenchConfig(
        mass=args.mass,
        stiffness=args.stiffness,
        damping=args.damping,
        rate_hz=args.rate,
        duration_s=args.duration,
        force_cap=args.force_cap,
        vel_filter_alpha=args.vel_filter_alpha,
        uri=args.uri,
        notes=args.notes,
    )

    print(
        f"[bench] mass={cfg.mass} K={cfg.stiffness} D={cfg.damping} "
        f"rate={cfg.rate_hz}Hz duration={cfg.duration_s}s — hold the device and feel for instability."
    )

    bench = HapticBench(cfg)
    samples = bench.run()  # connects to the real Inverse3; zeroes force on exit

    # Quick summary so the operator gets immediate feedback on stability.
    if samples.shape[0] > 0:
        force = samples[:, -1]
        lag = np.abs(samples[:, 3] - samples[:, 1])  # |x_mass - x_leader|
        print(
            f"[bench] {samples.shape[0]} samples | "
            f"max|force|={np.max(np.abs(force)):.2f} N | "
            f"max coupling lag={np.max(lag) * 1000:.1f} mm"
        )

    run_dir = None
    if not args.no_save:
        run_dir = save_bench_run(cfg, samples)
        print(f"[bench] saved to {run_dir}")

    if args.plot and run_dir is not None:
        plot_run(run_dir)
    elif args.plot:
        print("[bench] --plot requires saving; re-run without --no-save")

    print("[bench] Haptic stability bench completed")


if __name__ == "__main__":
    main()
