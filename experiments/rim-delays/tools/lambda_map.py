r"""B1 — the FR3's translational task-space inertia Λ at a configuration, and along a line.

Λ = (J M⁻¹ Jᵀ)⁻¹ for the translational Jacobian of the model interface point (``fr3_hand_tcp`` plus
``--tool-tip-offset``), base frame — the same Pinocchio model and reduction RIM uses. Prints Λ, its
eigenvalues and eigenvectors (E1's heaviest and lightest directions), and nᵀΛn along each
``--direction`` (M4's best-case mass). With ``--line``, Λ's eigenvalues and σ_min of the
translational Jacobian (distance to the singularity) along a straight path from the configuration,
orientation kept, by sequential IK (E4's path).

The configuration, one of:

    --live                 the robot's current joints (ROS, the robot running)
    --run <run dir>        a saved run's start joints (``q0`` in its metadata)
    --q q1 ... q7          given joints [rad]
    --start x y z          offline: from home, IK to this end-effector position, orientation kept
                           (what ``fr3_teleop --home --start-position`` does)

    pixi run -e humble python experiments/rim-delays/tools/lambda_map.py --start 0.4 0 0.2 --direction 1 0 0 --direction 0 0 1
    pixi run -e humble python experiments/rim-delays/tools/lambda_map.py --run <run dir> --line 0.7 0 0.2 --plot lambda_line
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


class _Logger:
    def info(self, msg):
        pass

    def warn(self, msg):
        print(f"[lambda_map] {msg}")


class Inertia:
    """Λ at a joint configuration, through `RobotModelAdapter` on a stand-in node and robot (it only
    needs their logger and joint names), so the model is exactly RIM's."""

    def __init__(self, tool_tip_offset=None) -> None:
        from arm_client.robot_config import FR3Config
        from fr3_haptic.adapters import RobotModelAdapter
        from fr3_haptic.config import ModelConfig
        from pyrim import InterfaceFrame, RIMCalculator

        cfg = ModelConfig()
        if tool_tip_offset is not None:
            cfg.tool_tip_offset = list(tool_tip_offset)
        self.fr3 = FR3Config()
        robot = SimpleNamespace(config=self.fr3)
        node = SimpleNamespace(get_logger=_Logger)
        self.adapter = RobotModelAdapter(node=node, robot=robot, model_cfg=cfg, frame=InterfaceFrame(np.eye(3)))
        self._calc = RIMCalculator()
        self._tcp = self.adapter._model.getFrameId(self.fr3.target_frame)

    def at(self, q) -> dict:
        """``Lambda`` (3, 3), interface point ``x``, ``sigma_min`` of the translational Jacobian, and
        the ``fr3_hand_tcp`` pose (``tcp_position``, ``tcp_rotation``) at joints ``q``."""
        q = np.asarray(q, dtype=float)
        zeros = np.zeros_like(q)
        model = self.adapter.compute_at(q, zeros, zeros)
        tcp = self.adapter._data.oMf[self._tcp]
        return {
            "Lambda": self._calc.compute(model).M_eff,
            "x": model.x_i,
            "sigma_min": float(np.linalg.svd(model.J_i, compute_uv=False)[-1]),
            "tcp_position": tcp.translation.copy(),
            "tcp_rotation": tcp.rotation.copy(),
        }


def line_joints(inertia: Inertia, q0, end, n_points: int) -> np.ndarray:
    """``(n_points, 7)`` joints along a straight ``fr3_hand_tcp`` line from ``q0`` to ``end``,
    orientation kept, by sequential warm-started IK (arm_client's planner)."""
    from arm_client.planning.ik_pyroki import plan_fr3_joint_trajectory
    from arm_client.planning.types import CartesianWaypoint
    from scipy.spatial.transform import Rotation

    start = inertia.at(q0)
    rot = Rotation.from_matrix(start["tcp_rotation"])
    waypoints = [CartesianWaypoint(start["tcp_position"], rot, 0.0), CartesianWaypoint(np.asarray(end, float), rot, 1.0)]
    traj = plan_fr3_joint_trajectory(waypoints, 1.0, inertia.fr3.joint_names, inertia.fr3.target_frame, n_points,
                                     np.asarray(q0, float), show_progress=False)
    return traj.joint_positions


def start_joints(args, inertia: Inertia) -> np.ndarray:
    if args.q is not None:
        return np.asarray(args.q, dtype=float)
    if args.run is not None:
        meta = json.loads((Path(args.run) / "metadata.json").read_text())
        meta = meta.get("metadata", meta)
        if meta.get("q0") is None:
            raise SystemExit(f"{args.run} has no q0 in its metadata (run recorded before it was logged)")
        return np.asarray(meta["q0"], dtype=float)
    if args.start is not None:
        home = np.asarray(inertia.fr3.home_config, dtype=float)
        return line_joints(inertia, home, args.start, 50)[-1]
    import rclpy
    from arm_client.robot import Robot

    rclpy.init()
    try:
        robot = Robot(namespace=args.namespace)
        robot.wait_until_ready()
        return robot.q.copy()
    finally:
        robot.shutdown()


def describe(name: str, at: dict, directions) -> None:
    lam = at["Lambda"]
    w, v = np.linalg.eigh(lam)
    np.set_printoptions(precision=3, suppress=True)
    print(f"[lambda_map] {name}: interface point {at['x']} m, σ_min(J) {at['sigma_min']:.3f} m/rad")
    print(f"  Λ [kg] =\n{lam}")
    print(f"  eigenvalues {w} kg  (ratio max/min {w[-1] / w[0]:.1f})")
    print(f"  heaviest direction {v[:, -1]}  ({w[-1]:.2f} kg)")
    print(f"  lightest direction {v[:, 0]}  ({w[0]:.2f} kg)")
    for n in directions:
        n = np.asarray(n, dtype=float) / np.linalg.norm(n)
        print(f"  nᵀΛn along {n}: {n @ lam @ n:.3f} kg")


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true")
    src.add_argument("--run", type=Path)
    src.add_argument("--q", type=float, nargs=7)
    src.add_argument("--start", type=float, nargs=3)
    p.add_argument("--direction", type=float, nargs=3, action="append", default=[], help="nᵀΛn along n (repeatable)")
    p.add_argument("--line", type=float, nargs=3, default=None, help="end of a straight path [m, fr3_hand_tcp]")
    p.add_argument("--points", type=int, default=40)
    p.add_argument("--tool-tip-offset", type=float, nargs=3, default=None)
    p.add_argument("--namespace", default="fr3")
    p.add_argument("--plot", default=None, help="--line: save the plot under this name, next to this script")
    args = p.parse_args(argv)

    inertia = Inertia(args.tool_tip_offset)
    q0 = start_joints(args, inertia)
    print(f"[lambda_map] q0 = {np.round(q0, 4).tolist()} rad")
    describe("start", inertia.at(q0), args.direction)
    if args.line is None:
        return

    qs = line_joints(inertia, q0, args.line, args.points)
    ats = [inertia.at(q) for q in qs]
    x = np.array([a["x"] for a in ats])
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(x, axis=0), axis=1))]
    eig = np.array([np.linalg.eigvalsh(a["Lambda"]) for a in ats])
    sigma = np.array([a["sigma_min"] for a in ats])
    print("[lambda_map] along the line: s [m] | Λ eigenvalues [kg] | σ_min [m/rad]")
    for k in np.linspace(0, len(s) - 1, min(len(s), 9)).astype(int):
        print(f"  {s[k]:.3f} | {eig[k]} | {sigma[k]:.3f}")
    print(f"[lambda_map] end reached: {np.round(x[-1], 4)} m (asked {np.round(args.line, 4)}, tcp)")
    if args.plot:
        import matplotlib.pyplot as plt
        from runs import INK_2, save

        fig, (a1, a2) = plt.subplots(2, 1, figsize=(6, 5), sharex=True)
        for k, label in enumerate(("lightest", "middle", "heaviest")):
            a1.plot(s, eig[:, k], label=label)
        a1.set_ylabel("Λ eigenvalue [kg]")
        a1.legend()
        a2.plot(s, sigma, color=INK_2)
        a2.set_ylabel("σ_min(J) [m/rad]")
        a2.set_xlabel("distance along the line [m]")
        for path in save(fig, HERE, args.plot):
            print(f"[lambda_map] saved {path}")


if __name__ == "__main__":
    main()
