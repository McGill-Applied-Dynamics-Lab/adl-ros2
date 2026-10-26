r"""B3 — the feedback rate f_m (robot → haptic), from a pilot run with the model logged at 1 kHz.

EXP_PLAN's rule: f_m is shared by all methods (M1's state and M5's model travel on the same link);
choose the lowest f_m at which Λ changes by only a few percent per update and holding it adds little
to e_F. For each candidate f_m this holds the pilot's 1 kHz model at that rate (sample and hold,
what a slower link delivers) and reports:

- **Λ change per update**: ``‖Λ(t_{j+1}) − Λ(t_j)‖ / ‖Λ(t_j)‖`` (Frobenius) over the update instants;
- **force error from holding** (M5): ``(Λ_held − Λ) a_p + (z_held − z)``, with ``a_p`` the proxy's
  acceleration — what the proxy gets wrong between updates;
- **M1's cost** (the same link carries its robot state): the force ripple of a held robot position,
  ``K |v| / f_m`` (``K`` the coupling), and the mean hold delay ``1 / (2 f_m)``;

next to the run's own e_F, and recommends the lowest f_m within ``--max-change``, ``--max-force`` and,
for M1, ``--max-ripple``.

The pilot: proxy-rim, model at 1 kHz, a fast scripted motion (E2's circle, in the air):

    pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml --method proxy-rim \
        --interface-axis xyz --contact-surface 0.083 --home --start-position 0.4 0 0.2 \
        --device scripted --script circle --script-surface-z 0.17 --script-depth 0.01 \
        --script-radius 0.1 --script-period-s 1.5 --script-laps 4 --model-update-hz 1000 \
        --feedback-delay-ms 0 --command-delay-ms 0 --seconds 25 \
        --save --output-dir ~/data/franka/rim-delays/robot/pilots --run-name fm_circle
    pixi run -e humble python experiments/rim-delays/tools/pilot_fm.py ~/data/franka/rim-delays/robot/pilots/fm_circle
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import metrics as M  # noqa: E402
from runs import find, load_run  # noqa: E402

CANDIDATES = (1000, 500, 250, 200, 100, 50, 25, 10)


def hold(t: np.ndarray, values: np.ndarray, f_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """``values`` sampled at ``t`` as a link at ``f_hz`` delivers them: each one held until the next
    update instant. Returns the held values at ``t`` and the indices of the update samples."""
    grid = np.floor((t - t[0]) * f_hz + 1e-9)
    updates = np.flatnonzero(np.r_[True, np.diff(grid) > 0])
    idx = updates[np.searchsorted(updates, np.arange(len(t)), side="right") - 1]
    return values[idx], updates


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("run", type=Path)
    p.add_argument("--max-change", type=float, default=0.03, help="Λ change per update, p95 [fraction]")
    p.add_argument("--max-force", type=float, default=0.5, help="force error from holding, RMS [N]")
    p.add_argument("--max-ripple", type=float, default=1.0, help="M1 force ripple K|v|/f_m at p95 speed [N]")
    p.add_argument("--k", type=float, default=None, help="M1 coupling stiffness [N/m]; default the run's --kv")
    p.add_argument("--plot", action="store_true", help="save a figure in <run>/analysis/")
    args = p.parse_args(argv)

    run = load_run(args.run.expanduser())
    if run is None or run.get("model") is None:
        raise SystemExit(f"{args.run}: no model stream (run proxy-rim with --save)")
    m, h = run["model"], run["haptic"]
    t = m.ts.to_numpy()
    rate = (len(t) - 1) / (t[-1] - t[0])
    if rate < 900:
        print(f"[pilot_fm] WARNING: the model was logged at {rate:.0f} Hz; rates above it cannot be told apart")
    lam = m[[f"M_eff_{k}" for k in range(9)]].to_numpy().reshape(-1, 3, 3)
    z = m[[f"z_i_{k}" for k in range(3)]].to_numpy()

    # the proxy's acceleration at the model times
    pv = h[[f"proxy_v_{k}" for k in range(3)]].to_numpy()
    ht = h.ts.to_numpy()
    k_ma = 11  # ~10 ms moving average before differentiating
    pv_s = np.stack([np.convolve(pv[:, i], np.ones(k_ma) / k_ma, mode="same") for i in range(3)], 1)
    pa = np.stack([np.gradient(pv_s[:, i], ht) for i in range(3)], 1)
    a = np.stack([np.interp(t, ht, pa[:, i]) for i in range(3)], 1)
    true_force = np.einsum("nij,nj->ni", lam, a) + z

    # M1's state: the robot speed, and its coupling stiffness
    speed = np.linalg.norm(M.robot_at(run, t)["v"], axis=1)  # NaN where t is outside the robot log
    v95 = float(np.nanpercentile(speed, 95))
    K = args.k if args.k is not None else float(find(run["meta"], "kv", 1000.0))
    e_f = M.rms(np.linalg.norm(M.unfelt_force(run)["e_F"], axis=1))

    eig = np.linalg.eigvalsh(lam)
    print(f"[pilot_fm] {args.run}: model {len(t)} updates at {rate:.0f} Hz over {t[-1] - t[0]:.1f} s; "
          f"Λ eigenvalues {eig.min():.2f} … {eig.max():.2f} kg; proxy |a| p95 {np.percentile(np.linalg.norm(a, axis=1), 95):.2f} m/s², "
          f"robot |v| p95 {v95:.2f} m/s")
    print(f"  the run's e_F RMS (|e_F|): {e_f:.2f} N   M1 coupling K = {K:g} N/m\n")
    print("   f_m [Hz] | Λ change/update p95 / max | force error from holding RMS / p99 [N] | M1 ripple K|v|/f p95 [N] | hold delay [ms]")
    rows = []
    for f in CANDIDATES:
        if f > rate * 1.05:
            continue
        lam_h, upd = hold(t, lam, f)
        z_h, _ = hold(t, z, f)
        norm = np.linalg.norm(lam[upd], axis=(1, 2))
        change = np.linalg.norm(np.diff(lam[upd], axis=0), axis=(1, 2)) / norm[:-1]
        err = np.linalg.norm(np.einsum("nij,nj->ni", lam_h, a) + z_h - true_force, axis=1)
        ripple = K * v95 / f
        row = (f, np.percentile(change, 95), change.max(), M.rms(err), np.percentile(err, 99), ripple, 500.0 / f)
        rows.append(row)
        print(f"   {f:8.0f} | {100 * row[1]:8.2f} % / {100 * row[2]:6.2f} %      | {row[3]:6.3f} / {row[4]:6.3f}"
              f"                         | {ripple:8.2f}                 | {row[6]:6.1f}")
    print()
    for name, ok in (
        (f"M5 (Λ change ≤ {100 * args.max_change:g} %, force error ≤ {args.max_force:g} N)",
         [r for r in rows if r[1] <= args.max_change and r[3] <= args.max_force]),
        (f"M5 and M1 (also ripple ≤ {args.max_ripple:g} N)",
         [r for r in rows if r[1] <= args.max_change and r[3] <= args.max_force and r[5] <= args.max_ripple]),
    ):
        if ok:
            best = min(ok, key=lambda r: r[0])
            print(f"[pilot_fm] lowest f_m for {name}: {best[0]:g} Hz "
                  f"(M1 ripple {best[5]:.2f} N, hold delay {best[6]:.1f} ms)")
        else:
            print(f"[pilot_fm] no candidate for {name}")

    if args.plot:
        import matplotlib.pyplot as plt
        from runs import INK_2, save

        f = np.array([r[0] for r in rows])
        fig, ax = plt.subplots(1, 3, figsize=(12, 3.4))
        ax[0].plot(f, [100 * r[1] for r in rows], "o-", color="#2a78d6")
        ax[0].axhline(100 * args.max_change, color=INK_2, lw=1, ls="--")
        ax[0].set_ylabel("Λ change per update, p95 [%]")
        ax[1].plot(f, [r[3] for r in rows], "o-", color="#eb6834")
        ax[1].axhline(args.max_force, color=INK_2, lw=1, ls="--")
        ax[1].set_ylabel("force error from holding, RMS [N]")
        ax[2].plot(f, [r[5] for r in rows], "o-", color="#1baf7a")
        ax[2].set_ylabel("M1 ripple K|v|/f_m, p95 [N]")
        for a_ in ax:
            a_.set_xscale("log")
            a_.set_xlabel("f_m [Hz]")
        fig.suptitle(f"Feedback rate f_m — {args.run.name}", x=0.01, ha="left")
        fig.tight_layout()
        out = args.run.expanduser() / "analysis"
        for path in save(fig, out, "pilot_fm"):
            print(f"[pilot_fm] saved {path}")


if __name__ == "__main__":
    main()
