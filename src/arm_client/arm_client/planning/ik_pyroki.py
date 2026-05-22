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


# ---------------------------------------------------------------------------
# Module-level singletons: robot model and JIT-compiled IK solvers.
# Avoids re-loading the URDF and re-tracing JAX functions on every planning
# call.  _fr3_ik_solvers is keyed by (target_link_index, pos_w, ori_w, sim_w)
# so different weight combinations each get their own compiled function while
# still reusing the robot model.
# ---------------------------------------------------------------------------
_fr3_robot: Any | None = None
_fr3_ik_solvers: dict = {}


def _get_fr3_robot() -> Any:
    global _fr3_robot
    if _fr3_robot is None:
        _fr3_robot = pk.Robot.from_urdf(load_fr3_urdf())
    return _fr3_robot


def _get_ik_solver(robot: Any, target_link_index: int, pos_weight: float, ori_weight: float, similarity_weight: float):
    key = (target_link_index, pos_weight, ori_weight, similarity_weight)
    if key not in _fr3_ik_solvers:
        @jdc.jit
        def solve_ik_single(target_wxyz, target_pos, q_init, q_prev):
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
                pk.costs.rest_cost(joint_var, q_prev, jnp.array(similarity_weight)),
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

        _fr3_ik_solvers[key] = solve_ik_single
    return _fr3_ik_solvers[key]


def compute_fk_trajectory(
    joint_positions: np.ndarray,
    target_link_name: str,
) -> np.ndarray:
    """Compute Cartesian positions for a sequence of joint configs via FK.

    Reuses the cached robot model — no URDF reload after the first call.

    Args:
        joint_positions: (N, n_arm_joints) array of joint configurations, as
            stored in PlannedJointTrajectory (arm joints only, already sliced).
            Will be padded to the full pyroki actuated-joint count using the
            robot's default values for the remaining joints (e.g. hand/fingers).
        target_link_name: Name of the link to evaluate FK at.

    Returns:
        positions: (N, 3) Cartesian positions of the target link in world frame.
    """
    robot = _get_fr3_robot()
    target_link_index = robot.links.names.index(target_link_name)

    # PlannedJointTrajectory stores only arm joints; pad to full pyroki size.
    default_q = np.array(robot.joint_var_cls(0).default_factory())
    n_full = len(default_q)
    n_arm = joint_positions.shape[1]
    full_positions = np.tile(default_q, (len(joint_positions), 1))
    full_positions[:, :n_arm] = joint_positions

    # Batched FK: (N, n_full) -> (N, link_count, 7) in wxyz_xyz format
    Ts = robot.forward_kinematics(jnp.array(full_positions, dtype=float))
    return np.array(Ts[:, target_link_index, 4:])  # xyz is the last 3 of 7


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
    pin_start: bool = False,
) -> PlannedJointTrajectory:
    """Plan a dense joint-space trajectory by sequential warm-started IK."""
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required")
    if n_points < 2:
        raise ValueError("n_points must be >= 2")
    if duration <= 0.0:
        raise ValueError("duration must be positive")

    _configure_quiet_ik_logging()

    robot = _get_fr3_robot()
    target_link_index = robot.links.names.index(target_link_name)
    solve_ik_single = _get_ik_solver(robot, target_link_index, pos_weight, ori_weight, similarity_weight)

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

    all_joint_configs = np.zeros((n_points, len(q_current)))
    if show_progress:
        _print_progress(0, n_points)

    for i in range(n_points):
        # pin_start: skip IK for the first point and use q_seed directly so that
        # joint_trajectory_controller gets a trajectory whose first point exactly
        # matches the measured robot state, preventing a large instantaneous jump.
        if pin_start and i == 0:
            all_joint_configs[0] = q_current
            if show_progress:
                _print_progress(1, n_points)
            continue
        q_jax = jnp.array(q_current)
        q_sol = solve_ik_single(
            jnp.array(interp_quat_wxyz[i]),
            jnp.array(interp_pos[i]),
            q_jax,
            q_jax,
        )  # reuses cached JIT-compiled solver — no retrace after first call
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

    # Set boundary conditions
    q_dot_start = np.zeros(len(joint_names))
    q_dot_end = np.zeros(len(joint_names))

    q_ddot_start = np.zeros(len(joint_names))
    q_ddot_end = np.zeros(len(joint_names))

    arm_joint_velocities = np.full((len(s_values), len(joint_names)), np.nan)
    arm_joint_accels = np.full((len(s_values), len(joint_names)), np.nan)

    arm_joint_velocities[0, :] = q_dot_start
    arm_joint_velocities[-1, :] = q_dot_end

    arm_joint_accels[0, :] = q_ddot_start
    arm_joint_accels[-1, :] = q_ddot_end

    arm_joint_count = len(joint_names)
    arm_joint_positions = all_joint_configs[:, :arm_joint_count]
    time_from_start = (s_values * duration).tolist()

    return PlannedJointTrajectory(
        joint_names=joint_names,
        time_from_start=time_from_start,
        joint_positions=arm_joint_positions,
        joint_velocities=arm_joint_velocities,
        joint_accelerations=arm_joint_accels,
    )


def solve_online_planning(
    robot: pk.Robot,
    target_link_name: str,
    target_position: np.ndarray,
    target_wxyz: np.ndarray,
    timesteps: int,
    dt: float,
    start_cfg: np.ndarray,
    prev_sols: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Solve online planning in free space."""

    assert target_position.shape == (3,) and target_wxyz.shape == (4,)
    target_link_indices = [robot.links.names.index(target_link_name)]

    target_poses = jaxlie.SE3(jnp.concatenate([jnp.array(target_wxyz), jnp.array(target_position)], axis=-1))
    target_links = jnp.array(target_link_indices)

    # Pad joint configs to match the URDF (e.g. including finger joints)
    default_q = np.array(robot.joint_var_cls(0).default_factory())
    expected_joints = len(default_q)

    padded_start_cfg = default_q.copy()
    padded_start_cfg[: min(expected_joints, len(start_cfg))] = start_cfg[: min(expected_joints, len(start_cfg))]

    padded_prev_sols = np.tile(default_q, (len(prev_sols), 1))
    padded_prev_sols[:, : min(expected_joints, prev_sols.shape[1])] = prev_sols[
        :, : min(expected_joints, prev_sols.shape[1])
    ]

    # Warm start: use previous solution shifted by one step.
    timesteps = timesteps + 1  # for start pose cost.

    sol_traj, sol_pos, sol_wxyz = _solve_online_planning_jax(
        robot,
        target_poses,
        target_links,
        timesteps,
        dt,
        jnp.array(padded_start_cfg),
        jnp.concatenate([padded_prev_sols, padded_prev_sols[-1:]], axis=0),
    )
    sol_traj = sol_traj[1:]
    sol_pos = sol_pos[1:]
    sol_wxyz = sol_wxyz[1:]

    return np.array(sol_traj), np.array(sol_pos), np.array(sol_wxyz)


@jdc.jit
def _solve_online_planning_jax(
    robot: pk.Robot,
    target_poses: jaxlie.SE3,
    target_links: jnp.ndarray,
    timesteps: jdc.Static[int],
    dt: float,
    start_cfg: jnp.ndarray,
    prev_sols: jnp.ndarray,
) -> tuple[jnp.ndarray, jnp.ndarray, jnp.ndarray]:
    num_targets = len(target_links)

    def batched_rplus(
        pose: jaxlie.SE3,
        delta: jax.Array,
    ) -> jaxlie.SE3:
        return jax.vmap(jaxlie.manifold.rplus)(pose, delta.reshape(num_targets, -1))

    # Custom SE3 variable to batch across multiple joint targets.
    class BatchedSE3Var(  # pylint: disable=missing-class-docstring
        jaxls.Var[jaxlie.SE3],
        default_factory=lambda: jaxlie.SE3.identity((num_targets,)),
        retract_fn=batched_rplus,
        tangent_dim=jaxlie.SE3.tangent_dim * num_targets,
    ): ...

    # --- Define Variables ---
    traj_var = robot.joint_var_cls(jnp.arange(0, timesteps))
    traj_var_prev = robot.joint_var_cls(jnp.arange(0, timesteps - 1))
    traj_var_next = robot.joint_var_cls(jnp.arange(1, timesteps))
    pose_var = BatchedSE3Var(jnp.arange(0, timesteps))
    pose_var_prev = BatchedSE3Var(jnp.arange(0, timesteps - 1))
    pose_var_next = BatchedSE3Var(jnp.arange(1, timesteps))

    init_pose_vals = jaxlie.SE3(robot.forward_kinematics(prev_sols)[..., target_links, :])

    # --- Define Costs ---
    factors: list[jaxls.Cost] = []

    @jaxls.Cost.factory(name="SE3PoseMatchJointCost")
    def match_joint_to_pose_cost(
        vals: jaxls.VarValues,
        joint_var: jaxls.Var[jnp.ndarray],
        pose_var: BatchedSE3Var,
    ):
        joint_cfg = vals[joint_var]
        target_pose = vals[pose_var]
        Ts_joint_world = robot.forward_kinematics(joint_cfg)
        residual = ((jaxlie.SE3(Ts_joint_world[..., target_links, :])).inverse() @ (target_pose)).log()
        return residual.flatten() * 100.0

    @jaxls.Cost.factory(name="SE3SmoothnessCost")
    def pose_smoothness_cost(
        vals: jaxls.VarValues,
        pose_var: BatchedSE3Var,
        pose_var_prev: BatchedSE3Var,
    ):
        return (vals[pose_var].inverse() @ vals[pose_var_prev]).log().flatten() * 1.0

    @jaxls.Cost.factory(name="SE3PoseMatchCost")
    def pose_match_cost(
        vals: jaxls.VarValues,
        pose_var: BatchedSE3Var,
    ):
        return ((vals[pose_var].inverse() @ target_poses).log() * jnp.array([50.0] * 3 + [20.0] * 3)).flatten()

    @jaxls.Cost.factory(name="MatchStartPoseCost")
    def match_start_pose_cost(
        vals: jaxls.VarValues,
        joint_var: jaxls.Var[jnp.ndarray],
    ):
        return (vals[joint_var] - start_cfg).flatten() * 100.0

    # Add pose costs.
    factors.extend(
        [
            pose_match_cost(
                BatchedSE3Var(timesteps - 1),
            ),
            pose_smoothness_cost(
                pose_var_next,
                pose_var_prev,
            ),
        ]
    )

    # Need to constrain the start joint cfg.
    factors.append(match_start_pose_cost(robot.joint_var_cls(0)))

    factors.extend(
        [
            match_joint_to_pose_cost(
                traj_var,
                pose_var,
            ),
            pk.costs.smoothness_cost(
                traj_var_prev,
                traj_var_next,
                weight=10.0,
            ),
            pk.costs.limit_velocity_cost(
                jax.tree.map(lambda x: x[None], robot),
                traj_var_prev,
                traj_var_next,
                weight=1.0,
                dt=dt,
            ),
            pk.costs.limit_cost(
                jax.tree.map(lambda x: x[None], robot),
                traj_var,
                weight=100.0,
            ),
            pk.costs.rest_cost(
                traj_var,
                jnp.array(traj_var.default_factory())[None],
                weight=0.01,
            ),
            pk.costs.manipulability_cost(
                jax.tree.map(lambda x: x[None], robot),
                traj_var,
                weight=0.01,
                target_link_indices=target_links,
            ),
        ]
    )

    solution = (
        jaxls.LeastSquaresProblem(factors, [traj_var, pose_var])
        .analyze()
        .solve(
            verbose=False,
            initial_vals=jaxls.VarValues.make((traj_var.with_value(prev_sols), pose_var.with_value(init_pose_vals))),
            termination=jaxls.TerminationConfig(max_iterations=20),
        )
    )
    pose_traj = solution[pose_var]
    return (
        solution[traj_var],
        pose_traj.translation(),
        pose_traj.rotation().wxyz,
    )
