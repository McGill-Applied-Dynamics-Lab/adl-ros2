#!/usr/bin/env python3
"""
Trajectory validation script for FR3 robot.

This script validates pose tracking by:
1. Getting current robot state
2. Computing waypoints for a trajectory (straight line in Cartesian space)
3. Using PyRoki to plan joint trajectories
4. Visualizing the planned trajectories until user approval
5. Sending the trajectory to the arm
6. Recording the states during execution
7. Plotting the results (tracking error for position, orientation, joints)
"""

import time
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import rclpy
import rclpy.signals
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import pyroki as pk
import jax
import jax.numpy as jnp
import jaxlie
import jaxls
import jax_dataclasses as jdc

import numpy as onp
import yourdfpy

import matplotlib.pyplot as plt
import viser
from viser.extras import ViserUrdf

REAL_ROBOT = True

JOINT_STATE_TOPIC = "/fr3/franka/joint_states"

POSE_STATE_TOPIC = "/fr3/current_pose"

# Topic the joint_trajectory_controller listens on.
# Verify with: ros2 topic list | grep joint_trajectory
TRAJECTORY_TOPIC = "/fr3/joint_trajectory_controller/joint_trajectory"

N_WP = 2
N_POINTS = 10
DT = 0.02

JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

TRAJ_TYPE = "spherical"  # "straight" or "spherical"
SPHERICAL_RADIUS = 0.2  # meters
SPHERICAL_THETA = 90.0  # degrees (arc length)
SPHERICAL_PHI = 0.0  # degrees (azimuthal angle / direction of travel)

TRAJ_DURATION = 3.0
N_WP = 10
N_POINTS = 50
TRAJ_START_TIME = 2.0
SETTLE_TIME = 1.0


class TrajectoryRecorder:
    """Records joint and pose states during trajectory execution."""

    def __init__(self, node: Node):
        self.node = node
        # self.config = config
        self.joint_states = []
        self.pose_states = []
        self.joint_timestamps = []
        self.pose_timestamps = []
        self.start_time = None
        self._is_recording = False

        self.joint_positions: list = None
        self.position: np.ndarray = None
        self.orientation: Rotation = None

        # Subscribe to joint states
        self.joint_sub = self.node.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._joint_callback,
            qos_profile_sensor_data,
        )

        # Subscribe to current pose
        self.pose_sub = self.node.create_subscription(
            PoseStamped,
            POSE_STATE_TOPIC,
            self._pose_callback,
            qos_profile_sensor_data,
        )

        self.traj_pub = self.node.create_publisher(
            JointTrajectory,
            TRAJECTORY_TOPIC,
            10,
        )

    def _joint_callback(self, msg: JointState):
        """Record joint state."""
        # Extract joint positions for the actuated joints
        joint_positions = []
        for joint_name in JOINT_NAMES:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                joint_positions.append(msg.position[idx])

        self.joint_positions = joint_positions

        if self._is_recording:
            if self.start_time is None:
                self.start_time = time.time()

            elapsed = time.time() - self.start_time
            self.joint_timestamps.append(elapsed)

            self.joint_states.append(np.array(joint_positions))

    def _pose_callback(self, msg: PoseStamped):
        """Record pose state."""
        self.position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.orientation = Rotation.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )

        if self._is_recording:
            if self.start_time is None:
                self.start_time = time.time()

            elapsed = time.time() - self.start_time
            self.pose_timestamps.append(elapsed)
            self.pose_states.append({"position": self.position, "orientation": self.orientation})

    def get_data(self):
        """Return recorded data."""
        return {
            "joint_timestamps": np.array(self.joint_timestamps),
            "pose_timestamps": np.array(self.pose_timestamps),
            "joint_states": np.array(self.joint_states),
            "pose_states": self.pose_states,
        }

    def start_recording(self):
        self._is_recording = True

    def send_traj_msg(self, traj_msg):
        traj_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.traj_pub.publish(traj_msg)


def get_fr3_urdf() -> yourdfpy.URDF:
    """Load FR3 URDF from franka_rim package."""
    try:
        share = get_package_share_directory("franka_rim")
    except Exception:
        # Fallback for development
        share = Path(__file__).parent.parent / "src" / "robot_arm" / "franka_rim"

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

    urdf = yourdfpy.URDF.load(str(urdf_path), mesh_dir=str(meshes_dir))
    return urdf


def solve_ik_for_pose(
    robot: pk.Robot,
    target_link_name: str,
    target_position: onp.ndarray,
    target_wxyz: onp.ndarray,
    initial_config: Optional[onp.ndarray] = None,
) -> onp.ndarray:
    """
    Solve IK for a single pose using PyRoki.

    Args:
        robot: PyRoki Robot instance
        target_link_name: Name of the end-effector link
        target_position: (3,) array of target position
        target_wxyz: (4,) array of target orientation (wxyz)
        initial_config: (n_joints,) initial joint configuration (ignored for now)

    Returns:
        Joint configuration array of shape (n_joints,)
    """
    assert target_position.shape == (3,) and target_wxyz.shape == (4,)
    target_link_index = robot.links.names.index(target_link_name)

    sol = _solve_ik_jax(
        robot,
        jnp.array(target_link_index),
        jnp.array(target_wxyz),
        jnp.array(target_position),
    )
    return onp.array(sol)


def _solve_ik_jax(
    robot: pk.Robot,
    target_link_index: jnp.ndarray,
    target_wxyz: jnp.ndarray,
    target_position: jnp.ndarray,
) -> jnp.ndarray:
    """IK solver using JAX."""
    # Start from zero configuration
    joint_var = robot.joint_var_cls(0)

    variables = [joint_var]
    costs = [
        pk.costs.pose_cost_analytic_jac(
            robot,
            joint_var,
            jaxlie.SE3.from_rotation_and_translation(
                jaxlie.SO3(target_wxyz),
                target_position,
            ),
            target_link_index,
            pos_weight=50.0,
            ori_weight=10.0,
        ),
        pk.costs.limit_constraint(robot, joint_var),
    ]

    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=variables)
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
        )
    )
    return sol[joint_var]


@jdc.jit
def solve_start_end_ik(
    robot: pk.Robot,
    target_link_index: int,
    target_position_0: jax.Array,
    target_wxyz_0: jax.Array,
    target_position_1: jax.Array,
    target_wxyz_1: jax.Array,
    initial_joint_cfg: Optional[jax.Array] = None,
) -> tuple[jax.Array, jax.Array]:
    """Solves the basic IK problem for a start and end pose.
    Returns joint configuration.
    """
    joint_var_0 = robot.joint_var_cls(0)
    joint_var_1 = robot.joint_var_cls(1)
    joint_vars = robot.joint_var_cls(jnp.arange(2))
    variables = [joint_vars]

    # --- Define costs
    # Soft costs: pose matching, regularization, self-collision
    costs: list[jaxls.Cost] = []

    start_pose_cost = pk.costs.pose_cost(
        robot,
        joint_var_0,
        jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz_0), target_position_0),
        jnp.array(target_link_index),
        jnp.array([100.0] * 3),
        jnp.array([100.0] * 3),
    )
    costs.append(start_pose_cost)

    end_pose_cost = pk.costs.pose_cost(
        robot,
        joint_var_1,
        jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz_1), target_position_1),
        jnp.array(target_link_index),
        jnp.array([100.0] * 3),
        jnp.array([100.0] * 3),
    )
    costs.append(end_pose_cost)

    rest_cost = pk.costs.rest_cost(
        joint_vars,
        jnp.array(joint_vars.default_factory()[None]),
        jnp.array(0.001),
    )
    costs.append(rest_cost)

    # self_collision_cost = pk.costs.self_collision_cost(
    #     jax.tree.map(lambda x: x[None], robot),
    #     jax.tree.map(lambda x: x[None], coll),
    #     joint_vars,
    #     0.02,
    #     5.0,
    # )
    # costs.append(self_collision_cost)

    # Small cost to encourage the start + end configs to be close to each other.
    @jaxls.Cost.factory(name="JointSimilarityCost")
    def joint_similarity_cost(vals, var_0, var_1):
        return (vals[var_0] - vals[var_1]).flatten()

    costs.append(joint_similarity_cost(joint_var_0, joint_var_1))

    # Constraint: joint limits
    joint_limits_cost = pk.costs.limit_constraint(
        jax.tree.map(lambda x: x[None], robot),
        joint_vars,
    )
    costs.append(joint_limits_cost)

    # Solving
    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=variables)
        .analyze()
        .solve(
            verbose=False,
            # initial_vals=jaxls.VarValues.make([joint_vars.with_value(initial_joint_cfg)]),
            # initial_vals=jaxls.VarValues.make([joint_var.with_value(prev_cfg)]),
            initial_vals=jaxls.VarValues.make([joint_var_0.with_value(initial_joint_cfg), joint_var_1]),
        )
    )
    return sol[joint_var_0], sol[joint_var_1]


@jdc.jit
def solve_end_ik(
    robot: pk.Robot,
    target_link_index: int,
    target_position_end: jax.Array,
    target_wxyz_end: jax.Array,
    initial_joint_cfg: Optional[jax.Array] = None,
) -> jax.Array:
    """Solves the basic IK problem for a start and end pose.
    Returns joint configuration.
    """
    joint_var = robot.joint_var_cls(0)
    variables = [joint_var]

    # --- Define costs
    # Soft costs: pose matching, regularization, self-collision
    costs: list[jaxls.Cost] = []

    end_pose_cost = pk.costs.pose_cost(
        robot,
        joint_var,
        jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz_end), target_position_end),
        jnp.array(target_link_index),
        100.0,
        50.0,
    )
    costs.append(end_pose_cost)

    rest_cost = pk.costs.rest_cost(
        joint_var,
        jnp.array(joint_var.default_factory()),
        jnp.array(0.001),
    )
    costs.append(rest_cost)

    # self_collision_cost = pk.costs.self_collision_cost(
    #     robot,
    #     coll,
    #     joint_var,
    #     0.02,
    #     5.0,
    # )
    # costs.append(self_collision_cost)

    # Small cost to encourage the start + end configs to be close to each other.
    @jaxls.Cost.factory(name="JointSimilarityCost")
    def joint_similarity_cost(vals, var_end):
        return (vals[var_end] - initial_joint_cfg).flatten()

    costs.append(joint_similarity_cost(joint_var))

    # Constraint: joint limits
    joint_limits_cost = pk.costs.limit_constraint(
        robot,
        joint_var,
    )
    costs.append(joint_limits_cost)

    # Solving
    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=variables)
        .analyze()
        .solve(
            verbose=False,
            # initial_vals=jaxls.VarValues.make([joint_vars.with_value(initial_joint_cfg)]),
            # initial_vals=jaxls.VarValues.make([joint_var.with_value(prev_cfg)]),
            initial_vals=jaxls.VarValues.make([joint_var.with_value(initial_joint_cfg)]),
        )
    )
    return sol[joint_var]


def generate_cartesian_waypoints(
    start_pose: dict,
    end_pose: dict,
    num_waypoints: int = 10,
) -> list[dict]:
    """
    Generate cartesian waypoints for a straight line trajectory.

    Args:
        start_pose: Dict with 'position' (3,) and 'orientation' (Rotation)
        end_pose: Dict with 'position' (3,) and 'orientation' (Rotation)
        num_waypoints: Number of waypoints to generate

    Returns:
        List of dicts with 'position', 'orientation', 's' for each waypoint
    """
    waypoints = []

    for i in range(num_waypoints):
        s = i / (num_waypoints - 1)  # Normalized path parameter [0, 1]

        # Linear interpolation for position
        position = start_pose["position"] * (1 - s) + end_pose["position"] * s

        # Spherical linear interpolation for orientation
        slerp = Slerp(
            [0, 1],
            Rotation.concatenate(
                [
                    start_pose["orientation"],
                    end_pose["orientation"],
                ]
            ),
        )
        orientation = slerp(s)

        waypoints.append(
            {
                "position": position,
                "orientation": orientation,
                "s": s,
            }
        )

    return waypoints


def generate_spherical_waypoints(
    start_pose: dict,
    radius: float,
    theta_deg: float,
    phi_deg: float,
    num_waypoints: int = 10,
) -> list[dict]:
    """
    Generate spherical waypoints around a center point, always pointing the Z-axis to the center.

    Args:
        start_pose: Dict with 'position' (3,) and 'orientation' (Rotation) pointing at the central object.
        radius: Distance to the central point from the end-effector.
        theta_deg: The arc length in degrees to sweep.
        phi_deg: The azimuthal direction of the sweep.
        num_waypoints: Number of waypoints to generate.

    Returns:
        List of dicts with 'position', 'orientation', 's' for each waypoint.
    """
    import numpy as np

    waypoints = []

    start_pos = start_pose["position"]
    start_ori = start_pose["orientation"]

    # 1. Compute the center of the sphere
    # Using the local Z-axis (forward) of the starting orientation.
    local_z = start_ori.apply(np.array([0.0, 0.0, 1.0]))
    center = start_pos + local_z * radius

    # 2. Determine rotation axis based on phi
    # The trajectory will arc around an axis perpendicular to the local Z-axis.
    # Phi=0 means we rotate around local X (thus sweeping along local Y).
    local_x = start_ori.apply(np.array([1.0, 0.0, 0.0]))
    local_y = start_ori.apply(np.array([0.0, 1.0, 0.0]))

    phi_rad = np.deg2rad(phi_deg)
    rot_axis = local_x * np.cos(phi_rad) + local_y * np.sin(phi_rad)
    # Normalize just to be perfectly safe
    rot_axis = rot_axis / np.linalg.norm(rot_axis)

    theta_rad = np.deg2rad(theta_deg)

    for i in range(num_waypoints):
        s = i / (num_waypoints - 1)  # Normalized path parameter [0, 1]

        current_angle = s * theta_rad
        r_step = Rotation.from_rotvec(current_angle * rot_axis)

        # 3. Rotate position vector about the center
        vec_center_to_start = start_pos - center
        new_vec = r_step.apply(vec_center_to_start)
        position = center + new_vec

        # 4. Rotate orientation equally to maintain the "look at center" constraint
        orientation = r_step * start_ori

        waypoints.append(
            {
                "position": position,
                "orientation": orientation,
                "s": s,
            }
        )

    return waypoints


def plan_joint_trajectory(
    robot: pk.Robot,
    target_link_name: str,
    waypoints: list[dict],
    joint_names: list[str],
    n_points: int,
    current_joint_config: Optional[onp.ndarray] = None,
) -> tuple[list[float], list[onp.ndarray]]:
    """
    Plan joint trajectory using trajectory optimization.

    Solves IK for start and end poses, then optimizes the trajectory to:
    - Pass through all intermediate waypoints (hard constraints)
    - Minimize joint effort (rest cost)
    - Maximize smoothness (smoothness and acceleration costs)
    - Respect joint limits
    - Avoid self-collisions

    Args:
        robot: PyRoki Robot instance
        target_link_name: Name of end-effector link
        waypoints: List of waypoint dicts with position, orientation, s
        joint_names: List of joint names (only arm joints, e.g., 7 for FR3)
        timesteps: Number of points for the trajectory
        current_joint_config: Current joint configuration for warm start

    Returns:
        Tuple of (s_values, joint_configs) where joint_configs are full robot configs (all joints)
    """
    target_link_index = robot.links.names.index(target_link_name)

    # s parameter from 0 to 1
    s_values = np.linspace(0.0, 1.0, n_points)
    dummy_dt = 1.0 / (n_points - 1)  # Internal spacing for numeric differentials

    print(f"  Trajectory optimization: {n_points} points via spatial path parameter 's'")

    # Solve IK for start and end poses with similarity cost and self-collision
    print(f"  Solving IK for start pose...")
    target_link_index = robot.links.names.index(target_link_name)
    start_position = waypoints[0]["position"]
    start_wxyz = waypoints[0]["orientation"].as_quat()[[3, 0, 1, 2]]
    end_position = waypoints[-1]["position"]
    end_wxyz = waypoints[-1]["orientation"].as_quat()[[3, 0, 1, 2]]

    # start_joint_cfg, end_joint_cfg = solve_start_end_ik(
    #     robot=robot,
    #     target_link_index=target_link_index,
    #     target_position_0=jnp.array(start_position),
    #     target_wxyz_0=jnp.array(start_wxyz),
    #     target_position_1=jnp.array(end_position),
    #     target_wxyz_1=jnp.array(end_wxyz),
    #     initial_joint_cfg=jnp.array(current_joint_config),
    # )
    start_joint_cfg = jnp.array(current_joint_config)

    end_joint_cfg = solve_end_ik(
        robot=robot,
        target_link_index=target_link_index,
        target_position_end=jnp.array(end_position),
        target_wxyz_end=jnp.array(end_wxyz),
        initial_joint_cfg=jnp.array(current_joint_config),
    )

    # TODO: IK for intermediate points
    # # Solve intermediate waypoint IKs for constraint targets
    # intermediate_cfgs = [start_cfg]
    # for i in range(1, len(waypoints) - 1):
    #     print(f"\n  Solving IK for intermediate waypoint {i}/{len(waypoints) - 2}...")
    #     cfg = _solve_ik_with_similarity(
    #         robot,
    #         target_link_index,
    #         waypoints[i]["position"],
    #         waypoints[i]["orientation"].as_quat()[[3, 0, 1, 2]],
    #     )
    #     intermediate_cfgs.append(cfg)
    # intermediate_cfgs.append(end_cfg)
    # intermediate_cfgs = jnp.array(intermediate_cfgs)

    # Set up trajectory optimization problem
    print(f"\n  Setting up optimization problem...")

    # Initialize initial traj
    # TODO: Take into account intermediate points
    init_traj = jnp.linspace(start_joint_cfg, end_joint_cfg, n_points)

    # Create trajectory variables for all timesteps
    traj_vars = robot.joint_var_cls(jnp.arange(n_points))

    # Add batch dimension to robot and collision model
    robot_batched = jax.tree.map(lambda x: x[None], robot)

    # Define costs
    costs: list[jaxls.Cost] = []

    # Rest cost - minimize deviation from zero
    rest_cost = pk.costs.rest_cost(
        traj_vars,
        traj_vars.default_factory()[None],
        jnp.array([0.01]),  # Weight
    )
    costs.append(rest_cost)

    # Smoothness cost - minimize velocity
    smoothness_cost = pk.costs.smoothness_cost(
        robot.joint_var_cls(jnp.arange(1, n_points)),
        robot.joint_var_cls(jnp.arange(0, n_points - 1)),
        jnp.array([1.0]),  # Weight
    )
    costs.append(smoothness_cost)

    # Acceleration cost - minimize acceleration (5-point stencil)
    accel_cost = pk.costs.five_point_acceleration_cost(
        robot.joint_var_cls(jnp.arange(2, n_points - 2)),
        robot.joint_var_cls(jnp.arange(4, n_points)),
        robot.joint_var_cls(jnp.arange(3, n_points - 1)),
        robot.joint_var_cls(jnp.arange(1, n_points - 3)),
        robot.joint_var_cls(jnp.arange(0, n_points - 4)),
        dummy_dt,
        jnp.array([0.05]),  # Weight
    )
    costs.append(accel_cost)

    # # Self-collision avoidance
    # costs.append(
    #     pk.costs.self_collision_cost(
    #         robot_batched,
    #         robot_batched,  # Use robot's self-collision model
    #         traj_vars,
    #         0.02,
    #         5.0,
    #     )
    # )

    # Joint limits
    costs.append(pk.costs.limit_constraint(robot_batched, traj_vars))

    # # Velocity limits
    # vel_limits_cost = pk.costs.limit_velocity_constraint(
    #     robot_batched,
    #     robot.joint_var_cls(jnp.arange(1, timesteps)),
    #     robot.joint_var_cls(jnp.arange(0, timesteps - 1)),
    #     dt,
    # )
    # costs.append(vel_limits_cost)

    # Start / end pose constraints.
    @jaxls.Cost.factory(kind="constraint_eq_zero", name="start_pose_constraint")
    def start_pose_constraint(vals: jaxls.VarValues, var) -> jax.Array:
        return (vals[var] - start_joint_cfg).flatten()

    @jaxls.Cost.factory(kind="constraint_eq_zero", name="end_pose_constraint")
    def end_pose_constraint(vals: jaxls.VarValues, var) -> jax.Array:
        return (vals[var] - end_joint_cfg).flatten()

    costs.append(start_pose_constraint(robot.joint_var_cls(jnp.arange(0, 2))))
    costs.append(end_pose_constraint(robot.joint_var_cls(jnp.arange(n_points - 2, n_points))))

    # Intermediate waypoint Cartesian tracking (Soft constraints pulling joints to desired path)
    for i in range(1, n_points - 1):
        s_target = s_values[i]

        # Linearly interpolate between the provided discrete waypoints
        wp_idx = s_target * (len(waypoints) - 1)
        lower_idx = int(np.floor(wp_idx))
        upper_idx = int(np.ceil(wp_idx))

        if lower_idx == upper_idx:
            t_pos = waypoints[lower_idx]["position"]
            t_wxyz = waypoints[lower_idx]["orientation"].as_quat()[[3, 0, 1, 2]]
        else:
            w_upper = wp_idx - lower_idx
            w_lower = 1.0 - w_upper
            t_pos = w_lower * waypoints[lower_idx]["position"] + w_upper * waypoints[upper_idx]["position"]

            slerp_int = Slerp(
                [0, 1], Rotation.concatenate([waypoints[lower_idx]["orientation"], waypoints[upper_idx]["orientation"]])
            )
            t_wxyz = slerp_int(w_upper).as_quat()[[3, 0, 1, 2]]

        target_se3 = jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(jnp.array(t_wxyz)), jnp.array(t_pos))

        pose_cost = pk.costs.pose_cost_analytic_jac(
            robot,
            robot.joint_var_cls(jnp.array(i)),
            target_se3,
            jnp.array(target_link_index),
            pos_weight=500.0,
            ori_weight=200.0,
        )
        costs.append(pose_cost)

    # Solve optimization problem
    print(f"  Solving optimization problem...")
    solution = (
        jaxls.LeastSquaresProblem(
            costs=costs,
            variables=[traj_vars],
        )
        .analyze()
        .solve(
            verbose=False,
            # linear_solver="dense_cholesky",
            initial_vals=jaxls.VarValues.make((traj_vars.with_value(init_traj),)),
        )
    )

    optimized_traj = np.array(solution[traj_vars])

    return s_values.tolist(), optimized_traj


def _solve_ik_with_similarity(
    robot: pk.Robot,
    target_link_index: int,
    target_position: onp.ndarray,
    target_wxyz: onp.ndarray,
    initial_joint_config: Optional[onp.ndarray] = None,
) -> jnp.ndarray:
    """
    Solve IK with similarity cost and self-collision avoidance.

    Used for intermediate waypoint solving during trajectory optimization.
    Adds a regularization that encourages intermediate poses to be similar
    to the zero configuration.

    Args:
        robot: PyRoki Robot instance
        target_link_index: Index of target link
        target_position: Target position (3,)
        target_wxyz: Target orientation in wxyz format (4,)
        initial_joint_config: Initial joint configuration (n_joints,)

    Returns:
        Joint configuration (n_joints,)
    """
    joint_var = robot.joint_var_cls(0)
    variables = [joint_var]

    costs = [
        pk.costs.pose_cost_analytic_jac(
            robot,
            joint_var,
            jaxlie.SE3.from_rotation_and_translation(
                jaxlie.SO3(jnp.array(target_wxyz)),
                jnp.array(target_position),
            ),
            jnp.array(target_link_index),
            pos_weight=50.0,
            ori_weight=10.0,
        ),
        pk.costs.limit_constraint(robot, joint_var),
        # # Self-collision avoidance
        # pk.costs.self_collision_cost(
        #     jax.tree.map(lambda x: x[None], robot),
        #     jax.tree.map(lambda x: x[None], robot),
        #     joint_var,
        #     0.02,
        #     5.0,
        # ),
    ]

    # Similarity cost - encourage solution to be close to zero config
    @jaxls.Cost.factory(name="SimilarityCost")
    def similarity_cost(vals: jaxls.VarValues, var) -> jax.Array:
        return vals[var].flatten() * 0.1  # Soft regularization

    costs.append(similarity_cost(joint_var))

    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=variables)
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
        )
    )
    return sol[joint_var]


# Mark: create_joint_trajectory_msg
def create_joint_trajectory_msg(
    joint_names: list[str],
    s_values: list[float],
    joint_configs: list[onp.ndarray],
    start_joint_config: onp.ndarray,
    traj_duration: float,
    start_time: float,
    settle_time: float,
) -> JointTrajectory:
    """Create a JointTrajectory message from planned waypoints."""
    traj = JointTrajectory()
    traj.joint_names = joint_names

    import numpy as np

    # Compile the trajectory into single arrays for differentiation
    # The first point is the current state at t=0
    # Following points start after `start_time` buffer
    # Settle at last point for `settle_time`
    all_configs = np.vstack([start_joint_config] + [q[:-1] for q in joint_configs] + [joint_configs[-1][:-1]])
    all_times = np.array(
        [0.0] + [s * traj_duration + start_time for s in s_values] + [start_time + traj_duration + settle_time]
    )

    n_points = len(all_times)
    # velocities = np.zeros_like(all_configs)
    # accelerations = np.zeros_like(all_configs)

    # if n_points > 1:
    #     # Compute numerical derivatives using the actual execution times
    #     # to ensure controller tracks smoothly over the given durations.
    #     velocities = np.gradient(all_configs, all_times, axis=0)
    #     accelerations = np.gradient(velocities, all_times, axis=0)

    #     # Force zero velocity and acceleration at start and stop to ensure
    #     # a smooth transition from and to rest.
    #     velocities[0] = 0.0
    #     velocities[-1] = 0.0
    #     accelerations[0] = 0.0
    #     accelerations[-1] = 0.0

    #     # Enforce conservative FR3 limits
    #     max_vel = np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5])
    #     max_acc = np.array([8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0])

    #     if np.any(np.abs(velocities) > max_vel):
    #         print("  ⚠ WARNING: Generated trajectory exceeds max joint velocities. Clamping to limits.")
    #         velocities = np.clip(velocities, -max_vel, max_vel)

    #     if np.any(np.abs(accelerations) > max_acc):
    #         print("  ⚠ WARNING: Generated trajectory exceeds max joint accelerations. Clamping to limits.")
    #         accelerations = np.clip(accelerations, -max_acc, max_acc)

    # Populate the JointTrajectory message
    for i in range(n_points):
        point = JointTrajectoryPoint()
        point.positions = [float(q) for q in all_configs[i]]

        # Leaving velocities and accelerations empty to test the controller's interpolation
        point.velocities = []
        point.accelerations = []

        t = all_times[i]
        point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        traj.points.append(point)

    return traj


def visualize_trajectory(
    urdf: yourdfpy.URDF,
    robot: pk.Robot,
    times: list[float],
    joint_trajectory: list[onp.ndarray],
    joint_names: list[str],
) -> bool:
    """
    Visualize the planned trajectory using viser.

    Args:
        urdf: Robot URDF
        robot: PyRoki robot instance
        times: List of times for each waypoint
        joint_trajectory: List of joint configurations (7 arm joints)
        joint_names: List of joint names

    Returns:
        True if user approves, False otherwise
    """
    print("   Opening viser visualization server at http://localhost:8080")
    print("   (Viser server will stay open - check your browser)")

    # Create viser server
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)

    # Add robot URDF visualization
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    status_label = server.gui.add_text("Status", "Initializing...")

    # # Map joint names to robot indices to reconstruct full config
    # full_joint_indices = []
    # for name in robot.joints.names:
    #     if name in joint_names:
    #         full_joint_indices.append(joint_names.index(name))
    #     else:
    #         full_joint_indices.append(-1)  # Placeholder for non-arm joints
    timesteps = len(joint_trajectory)
    slider = server.gui.add_slider("Timestep", min=0, max=timesteps - 1, step=1, initial_value=0)
    playing = server.gui.add_checkbox("Playing", initial_value=True)
    finish_btn = server.gui.add_button("Finish Visualization")

    finished = False

    @finish_btn.on_click
    def _(_) -> None:
        nonlocal finished
        finished = True

    # Animation loop - run a few times
    print("   Animating trajectory...")
    print("   Click 'Finish Visualization' in the Viser GUI or press Ctrl+C to continue...")
    try:
        while not finished:
            if playing.value:
                slider.value = (slider.value + 1) % timesteps

            urdf_vis.update_cfg(joint_trajectory[slider.value])

            status_label.value = f"Waypoint {slider.value + 1}/{len(joint_trajectory)}"

            time.sleep(1.0 / 10.0)

            # for i, joint_values in enumerate(joint_trajectory):
            #     # full_cfg = np.hstack([joint_values, 0.04])

            #     # Update visualization
            #     urdf_vis.update_cfg(joint_values)
            #     # status_label.value = f"Waypoint {i + 1}/{len(joint_trajectory)} (t={times[i]:.2f}s)"
            #     status_label.value = f"Waypoint {i + 1}/{len(joint_trajectory)}"

            #     # Small delay between waypoints
            #     time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    # Ask user for approval
    print("\n   Trajectory visualization complete.")
    print("   You can still see the robot in the viser viewer.")

    # Keep server running briefly for user to inspect
    time.sleep(1)

    # Ask for approval
    response = input("\n   Approve trajectory? (y/n): ").strip().lower()

    return response == "y"


def plot_results(recorded_data: dict, reference_trajectory: dict, traj_msg, config):
    """
    Plot tracking results.

    Args:
        recorded_data: Dict from TrajectoryRecorder.get_data()
        reference_trajectory: Dict with reference positions, orientations, times
        traj_msg: JointTrajectory message with joint references
        config: Robot configuration
    """
    joint_timestamps = recorded_data["joint_timestamps"]
    pose_timestamps = recorded_data["pose_timestamps"]
    joint_states = recorded_data["joint_states"]
    pose_states = recorded_data["pose_states"]

    if len(pose_states) == 0:
        print("No pose data recorded!")
        return

    # Extract reference trajectory
    ref_times = np.array(reference_trajectory["times"]) + TRAJ_START_TIME
    ref_positions = np.array(reference_trajectory["positions"])
    # ref quats are in wxyz format, scipy needs xyzw
    ref_quats_wxyz = np.array(reference_trajectory["quats"])
    ref_quats_xyzw = np.column_stack([ref_quats_wxyz[:, 1:], ref_quats_wxyz[:, 0]])
    ref_rpy = Rotation.from_quat(ref_quats_xyzw).as_euler("xyz")

    # Extract actual trajectory
    actual_positions = np.array([p["position"] for p in pose_states])
    actual_quats_xyzw = np.array([p["orientation"].as_quat() for p in pose_states])  # assuming these are xyzw
    actual_rpy = Rotation.from_quat(actual_quats_xyzw).as_euler("xyz")

    # Interpolate reference to actual timestamps
    ref_pos_interp = np.column_stack(
        [
            np.interp(
                pose_timestamps,
                ref_times,
                ref_positions[:, i],
                left=ref_positions[0, i],
                right=ref_positions[-1, i],
            )
            for i in range(3)
        ]
    )

    ref_rpy_interp = np.column_stack(
        [
            np.interp(
                pose_timestamps,
                ref_times,
                ref_rpy[:, i],
                left=ref_rpy[0, i],
                right=ref_rpy[-1, i],
            )
            for i in range(3)
        ]
    )

    # Compute position and orientation error
    pos_error = actual_positions - ref_pos_interp
    pos_error_norm = np.linalg.norm(pos_error, axis=1)

    # Simple angular error (difference in euler angles, mapped to [-pi, pi])
    ori_error = (actual_rpy - ref_rpy_interp + np.pi) % (2 * np.pi) - np.pi
    ori_error_norm = np.linalg.norm(ori_error, axis=1)

    # Figure 1: Cartesian Tracking
    fig_cart = plt.figure(figsize=(16, 12))
    fig_cart.suptitle("Cartesian Tracking", fontsize=16)

    # Position tracking
    ax1 = fig_cart.add_subplot(4, 3, 1)
    ax1.plot(ref_times, ref_positions[:, 0], "b--", label="Reference")
    ax1.plot(pose_timestamps, actual_positions[:, 0], "r-", label="Actual")
    ax1.set_ylabel("X Position (m)")
    ax1.grid(True)

    ax2 = fig_cart.add_subplot(4, 3, 2)
    ax2.plot(ref_times, ref_positions[:, 1], "b--", label="Reference")
    ax2.plot(pose_timestamps, actual_positions[:, 1], "r-", label="Actual")
    ax2.set_ylabel("Y Position (m)")
    ax2.grid(True)

    ax3 = fig_cart.add_subplot(4, 3, 3)
    ax3.plot(ref_times, ref_positions[:, 2], "b--", label="Reference")
    ax3.plot(pose_timestamps, actual_positions[:, 2], "r-", label="Actual")
    ax3.set_ylabel("Z Position (m)")
    ax3.grid(True)

    # Orientation tracking (RPY)
    ax_rot1 = fig_cart.add_subplot(4, 3, 4)
    ax_rot1.plot(ref_times, ref_rpy[:, 0], "b--", label="Reference")
    ax_rot1.plot(pose_timestamps, actual_rpy[:, 0], "r-", label="Actual")
    ax_rot1.set_ylabel("Roll (rad)")
    ax_rot1.grid(True)

    ax_rot2 = fig_cart.add_subplot(4, 3, 5)
    ax_rot2.plot(ref_times, ref_rpy[:, 1], "b--", label="Reference")
    ax_rot2.plot(pose_timestamps, actual_rpy[:, 1], "r-", label="Actual")
    ax_rot2.set_ylabel("Pitch (rad)")
    ax_rot2.grid(True)

    ax_rot3 = fig_cart.add_subplot(4, 3, 6)
    ax_rot3.plot(ref_times, ref_rpy[:, 2], "b--", label="Reference")
    ax_rot3.plot(pose_timestamps, actual_rpy[:, 2], "r-", label="Actual")
    ax_rot3.set_ylabel("Yaw (rad)")
    ax_rot3.grid(True)

    # Position error
    ax4 = fig_cart.add_subplot(4, 3, 7)
    ax4.plot(pose_timestamps, pos_error[:, 0], "r-", label="X Error")
    ax4.plot(pose_timestamps, pos_error[:, 1], "g-", label="Y Error")
    ax4.plot(pose_timestamps, pos_error[:, 2], "b-", label="Z Error")
    ax4.set_ylabel("Position Error (m)")
    ax4.grid(True)

    ax5 = fig_cart.add_subplot(4, 3, 8)
    ax5.plot(pose_timestamps, pos_error_norm * 1000, "k-")
    ax5.set_ylabel("Pos Error Norm (mm)")
    ax5.grid(True)

    # Orientation error
    ax6 = fig_cart.add_subplot(4, 3, 10)
    ax6.plot(pose_timestamps, ori_error[:, 0], "r-", label="R Error")
    ax6.plot(pose_timestamps, ori_error[:, 1], "g-", label="P Error")
    ax6.plot(pose_timestamps, ori_error[:, 2], "b-", label="Y Error")
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Orientation Error (rad)")
    ax6.legend()
    ax6.grid(True)

    ax7 = fig_cart.add_subplot(4, 3, 11)
    ax7.plot(pose_timestamps, ori_error_norm, "k-")
    ax7.set_xlabel("Time (s)")
    ax7.set_ylabel("Ori Error Norm (rad)")
    ax7.grid(True)

    fig_cart.tight_layout()
    save_dir = Path(__file__).parent / "results"
    fig_cart.savefig(save_dir / "trajectory_cartesian.png", dpi=150)

    # Extract joint reference trajectory from traj_msg
    ref_joint_times = []
    ref_joint_positions = []
    for pt in traj_msg.points:
        t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        # Assuming plotting timeline is aligned to 0 roughly or relative to start
        ref_joint_times.append(t_sec)
        ref_joint_positions.append(pt.positions)

    ref_joint_times = np.array(ref_joint_times)
    ref_joint_positions = np.array(ref_joint_positions)

    # Figure 2: Joint Tracking
    fig_joints = plt.figure(figsize=(16, 12))
    fig_joints.suptitle("Joint Tracking", fontsize=16)

    for i in range(min(7, len(config.joint_names))):
        ax = fig_joints.add_subplot(3, 3, 1 + i)
        if len(ref_joint_positions) > 0 and i < ref_joint_positions.shape[1]:
            ax.plot(ref_joint_times, ref_joint_positions[:, i], "b--", label="Reference")
        if i < len(joint_states[0]):
            ax.plot(joint_timestamps, joint_states[:, i], "r-", label="Actual")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Joint {i + 1} (rad)")
        ax.set_title(f"{config.joint_names[i]}")
        ax.grid(True)
        ax.legend()

    fig_joints.tight_layout()
    fig_joints.savefig(save_dir / "trajectory_joints.png", dpi=150)

    print("\nPlots saved to /tmp/trajectory_cartesian.png and /tmp/trajectory_joints.png")
    plt.show()


def main():
    """Main validation function."""
    # Initialize ROS2
    # disable default signal handler so Ctrl+C doesn't shutdown rclpy and invalidate publishers
    # rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    rclpy.init()
    node = rclpy.create_node("trajectory_validator")
    recorder = TrajectoryRecorder(node)

    try:
        from arm_client.robot_config import FR3Config

        config = FR3Config()
    except ImportError:
        print("arm_client not found, using default FR3 config")

        # Create minimal config
        class MinimalFR3Config:
            joint_names = [
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ]
            current_pose_topic = "fr3/current_pose"
            current_joint_topic = "fr3/joint_states"

        config = MinimalFR3Config()

    print("=" * 80)
    print("FR3 TRAJECTORY VALIDATION")
    print("=" * 80)

    # Load URDF and create PyRoki robot
    print("\n1. Loading FR3 URDF and PyRoki robot...")
    urdf = get_fr3_urdf()
    robot = pk.Robot.from_urdf(urdf)
    target_link_name = "fr3_hand_tcp"

    # Get current state
    # MARK: 2 - Get state
    print("\n2. Getting current robot state...")
    print("   Waiting for first joint state message...")

    current_joint_state = None
    current_position = None
    current_orientation = None

    # Initialize with default values (for testing without robot)
    default_joint_state = [0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]  # Home config

    default_position = np.array([0.307, 0.0, 0.486])
    default_orientation = Rotation.from_quat(np.array([1.0, 0.0, 0.0, 0.0]))

    # Wait for first messages (with timeout - will use defaults if robot not connected)
    print("   Waiting for robot state messages (3s timeout)...")
    wait_time = 0.1
    timeout = time.time() + wait_time
    received_state = False
    while time.time() < timeout:
        if recorder.position is not None:
            received_state = True
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    if received_state:
        current_joint_state = recorder.joint_positions
        current_position = recorder.position
        current_orientation = recorder.orientation
        print("   ✓ Received live robot state")
    else:
        print("   ⚠ Using default state (robot may be disconnected)")
        current_joint_state = default_joint_state
        current_position = default_position
        current_orientation = default_orientation

        if REAL_ROBOT:
            raise RuntimeError("Robot state not received")

    print(f"   Current joint config: {current_joint_state}")

    # Extract current pose
    current_pose = {
        "position": current_position,
        "orientation": current_orientation,
    }
    print(f"   Current pose position: {current_pose['position']}")

    # Define trajectory parameters
    print("\n3. Defining trajectory parameters...")
    trajectory_duration = TRAJ_DURATION

    # Generate waypoints depending on the chosen trajectory type
    print(f"   Generating {TRAJ_TYPE} waypoints (start -> end in {trajectory_duration}s)...")
    if TRAJ_TYPE == "spherical":
        waypoints = generate_spherical_waypoints(
            start_pose=current_pose,
            radius=SPHERICAL_RADIUS,
            theta_deg=SPHERICAL_THETA,
            phi_deg=SPHERICAL_PHI,
            num_waypoints=N_WP,
        )
    else:  # straight
        end_position = current_pose["position"].copy()
        end_position[2] -= 0.30  # delta in z
        end_pose = {
            "position": end_position,
            "orientation": current_pose["orientation"],
        }
        waypoints = generate_cartesian_waypoints(
            current_pose,
            end_pose,
            num_waypoints=N_WP,
        )
    print(f"   Generated {len(waypoints)} waypoints")

    # ---------------------------
    # MARK: 4 - Plan trajectory
    # ---------------------------
    print("\n4. Planning joint trajectory using PyRoki...")
    s_values, joint_trajectory = plan_joint_trajectory(
        robot,
        target_link_name,
        waypoints,
        config.joint_names,
        n_points=N_POINTS,
        current_joint_config=np.hstack([current_joint_state, 0.04]),
    )
    print(f"   Planned trajectory with {len(joint_trajectory)} configurations")

    # Store reference trajectory for later plotting
    reference_cart_trajectory = {
        "times": np.array([w["s"] * trajectory_duration for w in waypoints]),
        "positions": np.array([w["position"] for w in waypoints]),
        "quats": np.array([w["orientation"].as_quat()[[3, 0, 1, 2]] for w in waypoints]),
    }

    # ---------------------------
    # MARK: 5 - Visualize
    # ---------------------------
    print("\n5. Visualizing planned trajectory...")
    approved = visualize_trajectory(
        urdf, robot, [s * trajectory_duration for s in s_values], joint_trajectory, config.joint_names
    )
    if not approved:
        print("\n   Trajectory rejected by user")
        node.destroy_node()
        # rclpy.shutdown()
        return

    # Create trajectory message
    print("\n6. Creating trajectory message...")
    traj_start_time = TRAJ_START_TIME
    traj_msg = create_joint_trajectory_msg(
        config.joint_names,
        s_values,
        joint_trajectory,
        current_joint_state,
        trajectory_duration,
        traj_start_time,
        settle_time=SETTLE_TIME,
    )

    # Start recording
    print("\n7. Starting state recording...")
    recorder.start_recording()

    # ---------------------------
    # MARK: 8 - Send trajectory
    # ---------------------------
    #
    print("\n8. Sending trajectory to robot...")
    recorder.send_traj_msg(traj_msg)

    print(f"   Trajectory sent (duration: {trajectory_duration}s)")

    # Wait for trajectory to complete + buffer
    wait_time = trajectory_duration + 1.0 + traj_start_time
    print(f"   Waiting {wait_time}s for execution...")
    start_wait = time.time()
    while (time.time() - start_wait) < wait_time:
        rclpy.spin_once(node, timeout_sec=0.01)

    # Get recorded data
    print("\n9. Saving recorded data...")
    recorded_data = recorder.get_data()
    print(
        f"   Recorded {len(recorded_data['joint_timestamps'])} joint samples and {len(recorded_data['pose_timestamps'])} pose samples"
    )
    if len(recorded_data["pose_states"]) > 0:
        print(
            f"   Position range: {recorded_data['pose_states'][0]['position']} -> "
            f"{recorded_data['pose_states'][-1]['position']}"
        )

    # Plot results
    print("\n10. Plotting results...")
    plot_results(recorded_data, reference_cart_trajectory, traj_msg, config)

    print("\n" + "=" * 80)
    print("VALIDATION COMPLETE")
    print("=" * 80)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
