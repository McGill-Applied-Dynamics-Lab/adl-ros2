"""PyRoki-based inverse kinematics planning helpers (FR3-focused)."""

import logging
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from arm_client.planning.types import CartesianWaypoint, PlannedJointTrajectory

import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls
import pyroki as pk
import yourdfpy
from ament_index_python.packages import get_package_share_directory


def _configure_quiet_ik_logging() -> None:
    """Reduce noisy solver logs from JAX/JAXLS when planning."""
    os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")

    # Standard Python logging path.
    for logger_name in ("jax", "jaxlib", "jaxls"):
        logging.getLogger(logger_name).setLevel(logging.ERROR)

    # jaxls may use loguru for INFO traces.
    try:
        from loguru import logger as loguru_logger

        loguru_logger.disable("jaxls")
    except Exception:
        pass


def _print_progress(current: int, total: int) -> None:
    """Print a compact one-line progress bar."""
    width = 30
    ratio = 0.0 if total <= 0 else current / total
    filled = int(width * ratio)
    bar = "#" * filled + "-" * (width - filled)
    pct = int(ratio * 100)
    sys.stdout.write(f"\rIK solve progress: [{bar}] {pct:3d}% ({current}/{total})")
    sys.stdout.flush()
    if current >= total:
        sys.stdout.write("\n")


def load_fr3_urdf() -> Any:
    """Load FR3 URDF from franka_rim package."""
    share = get_package_share_directory("franka_rim")
    models_dir = Path(share) / "models"
    urdf_path = models_dir / "fr3_franka_hand.urdf"
    meshes_dir = models_dir / "meshes"

    if not urdf_path.exists():
        raise FileNotFoundError(
            f"URDF not found at {urdf_path}. "
            "Generate it with:\n"
            "  xacro $(ros2 pkg prefix franka_description)/share/franka_description/"
            "robots/fr3/fr3.urdf.xacro hand:=true > fr3_franka_hand.urdf"
        )

    return yourdfpy.URDF.load(str(urdf_path), mesh_dir=str(meshes_dir))


def plan_fr3_joint_trajectory(
    waypoints: list[CartesianWaypoint],
    duration: float,
    joint_names: list[str],
    target_link_name: str,
    n_points: int,
    current_joint_config: np.ndarray,
    pos_weight: float = 100.0,
    ori_weight: float = 50.0,
    similarity_weight: float = 0.001,
    show_progress: bool = True,
) -> PlannedJointTrajectory:
    """Plan a dense joint-space trajectory by sequential warm-started IK."""
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required")
    if n_points < 2:
        raise ValueError("n_points must be >= 2")
    if duration <= 0.0:
        raise ValueError("duration must be positive")

    _configure_quiet_ik_logging()

    urdf = load_fr3_urdf()
    robot = pk.Robot.from_urdf(urdf)
    target_link_index = robot.links.names.index(target_link_name)

    ref_s = np.array([w.s for w in waypoints], dtype=float)
    ref_pos = np.array([w.position for w in waypoints], dtype=float)
    ref_quat_xyzw = np.array([w.orientation.as_quat() for w in waypoints], dtype=float)

    s_values = np.linspace(0.0, 1.0, n_points)
    interp_pos = np.column_stack([np.interp(s_values, ref_s, ref_pos[:, i]) for i in range(3)])

    slerp = Slerp(ref_s, Rotation.from_quat(ref_quat_xyzw))
    interp_quat_wxyz = slerp(s_values).as_quat()[:, [3, 0, 1, 2]]

    default_q = np.array(robot.joint_var_cls(0).default_factory())
    q_current = default_q.copy()
    q_seed = np.array(current_joint_config, dtype=float)
    q_current[: min(len(q_current), len(q_seed))] = q_seed[: min(len(q_current), len(q_seed))]

    @jdc.jit
    def solve_ik_single(
        target_wxyz,
        target_pos,
        q_init,
        q_prev,
    ):
        joint_var = robot.joint_var_cls(0)

        costs = [
            pk.costs.pose_cost_analytic_jac(
                robot,
                joint_var,
                jaxlie.SE3.from_rotation_and_translation(
                    jaxlie.SO3(target_wxyz),
                    target_pos,
                ),
                jnp.array(target_link_index),
                pos_weight=pos_weight,
                ori_weight=ori_weight,
            ),
            pk.costs.rest_cost(
                joint_var,
                q_prev,
                jnp.array(similarity_weight),
            ),
            pk.costs.limit_constraint(robot, joint_var),
        ]

        sol = (
            jaxls.LeastSquaresProblem(costs=costs, variables=[joint_var])
            .analyze()
            .solve(
                verbose=False,
                linear_solver="dense_cholesky",
                trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
                initial_vals=jaxls.VarValues.make([joint_var.with_value(q_init)]),
            )
        )
        return sol[joint_var]

    all_joint_configs = np.zeros((n_points, len(q_current)))
    if show_progress:
        _print_progress(0, n_points)

    for i in range(n_points):
        q_jax = jnp.array(q_current)
        q_sol = solve_ik_single(
            jnp.array(interp_quat_wxyz[i]),
            jnp.array(interp_pos[i]),
            q_jax,
            q_jax,
        )
        q_sol.block_until_ready()
        q_current = np.array(q_sol)
        all_joint_configs[i] = q_current

        if show_progress:
            _print_progress(i + 1, n_points)

    # Fix endpoint glitchiness: Force the final trajectory point to exactly match
    # the last waypoint by solving IK for the exact final Cartesian target.
    # This avoids interpolation artifacts and ensures smooth endpoint behavior.
    final_waypoint_quat_wxyz = waypoints[-1].orientation.as_quat()[[3, 0, 1, 2]]
    final_waypoint_pos = waypoints[-1].position
    
    q_final = solve_ik_single(
        jnp.array(final_waypoint_quat_wxyz),
        jnp.array(final_waypoint_pos),
        jnp.array(all_joint_configs[-1]),  # Use current last point as seed
        jnp.array(all_joint_configs[-1]),
    )
    q_final.block_until_ready()
    all_joint_configs[-1] = np.array(q_final)

    arm_joint_count = len(joint_names)
    arm_joint_positions = all_joint_configs[:, :arm_joint_count]
    time_from_start = (s_values * duration).tolist()

    return PlannedJointTrajectory(
        joint_names=joint_names,
        time_from_start=time_from_start,
        joint_positions=arm_joint_positions,
    )
