r"""B2 — OSC gains K_r, D_r against the arm's task inertia over the workspace (EXP_PLAN, rules).

``osc_controller`` runs with ``inertia_decoupling`` off: per base axis ``F = K_r e + D_r ė``, so the
arm's translational closed loop is ``Λ ë + D_r ė + K_r e = 0``. Λ couples the axes (x-z at the
start), so its modes are the generalized eigenvectors ``K_r v = ω² Λ v``; with v Λ-normalized,
mode i has natural frequency ω_i and damping ratio ``ζ_i = vᵢᵀ D_r vᵢ / (2 ω_i)``.

For each position of a workspace grid (Λ from RIM's Pinocchio model at the joints IK reaches from
home, orientation kept), this reports the modes for:

- a fixed ``--d`` per axis (what M6 can model; constant), and
- the controller's auto damping (``d = -1``: ``2 ζ₀ √(K_ii Λ_ii)``, recomputed live from the diagonal),

recommends the smallest fixed D_r (``--d`` scaled uniformly) whose least-damped mode over the grid
has ζ ≥ ``--zeta``, shows how overdamped the light modes then are, and checks the K_r δ rule
(an error δ in the table position is an unrendered force K_r δ).

    pixi run -e humble python experiments/rim-delays/tools/osc_gains.py --k 1500 1500 3500
    pixi run -e humble python experiments/rim-delays/tools/osc_gains.py --k 1500 1500 3500 --k 3000 3000 3000   # compare
    pixi run -e humble python experiments/rim-delays/tools/osc_gains.py --k 3000 3000 5000 --d 150 150 300 --plot osc_gains
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")  # the IK is small; leave the GPU alone

import numpy as np
from scipy.linalg import eigh

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

AXES = "xyz"


def grid(xs, ys, zs) -> np.ndarray:
    """Grid positions as a serpentine path (neighbours stay close, so sequential IK stays continuous)."""
    pts = []
    for i, z in enumerate(zs):
        for j, y in enumerate(ys if i % 2 == 0 else ys[::-1]):
            xs_row = xs if (i * len(ys) + j) % 2 == 0 else xs[::-1]
            pts += [(x, y, z) for x in xs_row]
    return np.array(pts, dtype=float)


def joints_along(inertia, points: np.ndarray, per_segment: int = 12) -> np.ndarray:
    """Joints at each point, by one sequential IK pass from home through all of them (one JAX
    compile), the start orientation kept."""
    from arm_client.planning.ik_pyroki import plan_fr3_joint_trajectory
    from arm_client.planning.types import CartesianWaypoint
    from scipy.spatial.transform import Rotation

    home = np.asarray(inertia.fr3.home_config, dtype=float)
    start = inertia.at(home)
    rot = Rotation.from_matrix(start["tcp_rotation"])
    path = np.vstack([start["tcp_position"], points])
    n = len(path)
    waypoints = [CartesianWaypoint(p, rot, k / (n - 1)) for k, p in enumerate(path)]
    traj = plan_fr3_joint_trajectory(waypoints, 1.0, inertia.fr3.joint_names, inertia.fr3.target_frame,
                                     (n - 1) * per_segment + 1, home, show_progress=False)
    return traj.joint_positions[per_segment::per_segment]  # one row per point (waypoint k at k * per_segment)


def modes(lam: np.ndarray, K: np.ndarray, D: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``(f_n [Hz], ζ, m_modal [kg])`` of ``Λ ë + D ė + K e = 0``, K and D diagonal; m_modal is
    the mode's effective mass along its own direction (``vᵀΛv`` for unit ``v``)."""
    w2, V = eigh(np.diag(K), lam)  # Vᵀ Λ V = I
    w = np.sqrt(w2)
    zeta = np.einsum("ij,ik,kj->j", V, np.diag(D), V) / (2 * w)
    unit = V / np.linalg.norm(V, axis=0)
    m = np.einsum("ij,ik,kj->j", unit, lam, unit)
    return w / (2 * np.pi), zeta, m


def auto_damping(lam: np.ndarray, K: np.ndarray, zeta0: float) -> np.ndarray:
    """The controller's ``d = -1``: ``2 ζ₀ √(K_ii Λ_ii)``, per configuration."""
    return 2 * zeta0 * np.sqrt(K * np.diag(lam))


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--k", type=float, nargs=3, action="append", default=None,
                   help="K_r per axis x y z [N/m]; repeat to compare (default 1500 1500 3500)")
    p.add_argument("--d", type=float, nargs=3, default=None,
                   help="fixed D_r per axis [N·s/m] (one --k only); default 2 ζ √(K max Λ_ii), scaled to --zeta")
    p.add_argument("--zeta", type=float, default=0.7, help="least damping ratio wanted over the grid")
    p.add_argument("--zeta0", type=float, default=0.7, help="the controller's damping_ratio_pos (auto damping)")
    p.add_argument("--x", type=float, nargs="+", default=[0.3, 0.4, 0.5, 0.6])
    p.add_argument("--y", type=float, nargs="+", default=[-0.15, 0.0, 0.15])
    p.add_argument("--z", type=float, nargs="+", default=[0.09, 0.2, 0.3], help="0.09: just above the table")
    p.add_argument("--delta", type=float, default=0.002, help="table position error δ [m]")
    p.add_argument("--max-force", type=float, default=10.0, help="accepted unrendered force K_r δ [N]")
    p.add_argument("--tool-tip-offset", type=float, nargs=3, default=None)
    p.add_argument("--plot", default=None, help="save a figure under this name, next to this script")
    args = p.parse_args(argv)

    from lambda_map import Inertia

    k_sets = args.k or [[1500.0, 1500.0, 3500.0]]
    if args.d is not None and len(k_sets) > 1:
        raise SystemExit("--d goes with a single --k")
    inertia = Inertia(args.tool_tip_offset)
    pts = grid(args.x, args.y, args.z)
    print(f"[osc_gains] IK from home through {len(pts)} grid points ...")
    qs = joints_along(inertia, pts)
    ats = [inertia.at(q) for q in qs]
    reached = np.array([a["tcp_position"] for a in ats])
    miss = np.linalg.norm(reached - pts, axis=1)
    lams = np.array([a["Lambda"] for a in ats])
    if miss.max() > 0.005:
        print(f"[osc_gains] WARNING: IK missed {np.sum(miss > 0.005)} points by > 5 mm (max {1e3 * miss.max():.0f} mm)")

    eig = np.linalg.eigvalsh(lams)
    diag = np.diagonal(lams, axis1=1, axis2=2)
    print(f"[osc_gains] Λ over the grid: eigenvalues {eig.min():.2f} … {eig.max():.2f} kg; "
          f"diagonal x {diag[:, 0].min():.2f}…{diag[:, 0].max():.2f}, y {diag[:, 1].min():.2f}…{diag[:, 1].max():.2f}, "
          f"z {diag[:, 2].min():.2f}…{diag[:, 2].max():.2f} kg")

    for k in k_sets:
        report(np.asarray(k, dtype=float), lams, pts, args)


def report(K: np.ndarray, lams: np.ndarray, pts: np.ndarray, args) -> None:
    """The modes, damping and force cap for one K_r."""
    diag = np.diagonal(lams, axis1=1, axis2=2)

    # fixed D: given, or 2 ζ √(K max Λ_ii) scaled so the least-damped mode over the grid has ζ = --zeta
    def zeta_min(D):
        return min(modes(lam, K, D)[1].min() for lam in lams)

    if args.d is not None:
        D = np.asarray(args.d, dtype=float)
    else:
        D = 2 * args.zeta * np.sqrt(K * diag.max(axis=0))
        lo, hi = 0.1, 10.0
        for _ in range(60):  # bisection on a uniform scale
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if zeta_min(mid * D) < args.zeta else (lo, mid)
        D = np.round(hi * D, 1)

    rows = []
    for pt, lam in zip(pts, lams):
        f_fix, z_fix, m = modes(lam, K, D)
        f_auto, z_auto, _ = modes(lam, K, auto_damping(lam, K, args.zeta0))
        rows.append((pt, f_fix, z_fix, z_auto, m))
    z_fix = np.array([r[2] for r in rows])
    z_auto = np.array([r[3] for r in rows])
    f_all = np.array([r[1] for r in rows])

    np.set_printoptions(precision=2, suppress=True)
    print(f"\n[osc_gains] K_r = {K} N/m (x, y, z)")
    print(f"  fixed D_r = {D} N·s/m{'' if args.d is not None else f'  (smallest uniform scale with ζ_min ≥ {args.zeta})'}")
    print(f"    ζ over all modes and points: min {z_fix.min():.2f}, max {z_fix.max():.2f}")
    print(f"  auto damping (d = -1, ζ₀ = {args.zeta0}): D_r = 2 ζ₀ √(K Λ_ii) per point,"
          f" x {auto_damping(lams[0], K, args.zeta0)[0]:.0f}… on the first point")
    print(f"    ζ over all modes and points: min {z_auto.min():.2f}, max {z_auto.max():.2f}")
    print(f"  natural frequencies: {f_all.min():.1f} … {f_all.max():.1f} Hz")
    print(f"  K_r δ at δ = {1e3 * args.delta:g} mm: {K * args.delta} N per axis "
          f"({'OK' if np.all(K * args.delta <= args.max_force) else 'ABOVE'} the {args.max_force:g} N accepted)")

    print("\n  point [m]            | modes f_n [Hz]     | ζ fixed           | ζ auto            | modal mass [kg]")
    worst = np.argsort(z_fix.min(axis=1))[:6]
    for k in sorted(set(worst) | {0, len(rows) - 1}):
        pt, f, zf, za, m = rows[k]
        print(f"  {np.round(pt, 2)!s:20} | {f!s:18} | {zf!s:17} | {za!s:17} | {m}")
    print("  (the least-damped points, plus the first and last)")

    if args.plot:
        import matplotlib.pyplot as plt
        from runs import save

        modal_m = np.array([r[4] for r in rows])
        fig, ax = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
        for a, z, title in ((ax[0], z_fix, f"fixed D_r = {D}"), (ax[1], z_auto, f"auto (ζ₀ = {args.zeta0})")):
            a.scatter(modal_m.ravel(), z.ravel(), s=14, color="#2a78d6")
            a.axhline(args.zeta, color="#8a8984", lw=1, ls="--")
            a.set_xscale("log")
            a.set_xlabel("modal mass [kg]")
            a.set_title(title, loc="left", fontsize=10)
        ax[0].set_ylabel("modal damping ratio ζ")
        fig.suptitle(f"OSC modes over the workspace, K_r = {K} N/m", x=0.01, ha="left")
        fig.tight_layout()
        name = args.plot + ("_" + "_".join(f"{v:g}" for v in K) if len(args.k or []) > 1 else "")
        for path in save(fig, HERE, name):
            print(f"[osc_gains] saved {path}")


if __name__ == "__main__":
    main()
