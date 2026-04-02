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

REAL_ROBOT = False

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

TRAJ_DURATION = 5.0


class TrajectoryRecorder:
    """Records joint and pose states during trajectory execution."""

    def __init__(self, node: Node):
        self.node = node
        # self.config = config
        self.joint_states = []
        self.pose_states = []
        self.timestamps = []
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
            self.timestamps.append(elapsed)

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
            self.pose_states.append({"position": self.position, "orientation": self.orientation})

    def get_data(self):
        """Return recorded data."""
        return {
            "timestamps": np.array(self.timestamps),
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
def solve_iks(
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
        jnp.array([10.0] * 3),
        jnp.array([1.0] * 3),
    )
    costs.append(start_pose_cost)

    end_pose_cost = pk.costs.pose_cost(
        robot,
        joint_var_1,
        jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz_1), target_position_1),
        jnp.array(target_link_index),
        jnp.array([10.0] * 3),
        jnp.array([1.0] * 3),
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


def generate_cartesian_waypoints(
    start_pose: dict,
    end_pose: dict,
    num_waypoints: int = 10,
    duration: float = 5.0,
) -> list[dict]:
    """
    Generate cartesian waypoints for a straight line trajectory.

    Args:
        start_pose: Dict with 'position' (3,) and 'orientation' (Rotation)
        end_pose: Dict with 'position' (3,) and 'orientation' (Rotation)
        num_waypoints: Number of waypoints to generate
        duration: Total time for trajectory

    Returns:
        List of dicts with 'position', 'orientation', 'time' for each waypoint
    """
    waypoints = []

    for i in range(num_waypoints):
        t = i / (num_waypoints - 1)  # Normalized time [0, 1]

        # Linear interpolation for position
        position = start_pose["position"] * (1 - t) + end_pose["position"] * t

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
        orientation = slerp(t)

        waypoints.append(
            {
                "position": position,
                "orientation": orientation,
                "time": duration * t,
            }
        )

    return waypoints


def plan_joint_trajectory(
    robot: pk.Robot,
    target_link_name: str,
    waypoints: list[dict],
    joint_names: list[str],
    timesteps: int,
    dt: float,
    traj_duration: float,
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
        waypoints: List of waypoint dicts with position, orientation, time
        joint_names: List of joint names (only arm joints, e.g., 7 for FR3)
        timesteps: Number of points for the trajectory
        dt: Time step size
        current_joint_config: Current joint configuration for warm start

    Returns:
        Tuple of (times, joint_configs) where joint_configs are full robot configs (all joints)
    """
    target_link_index = robot.links.names.index(target_link_name)

    # # Map joint names to indices in the robot's joint list
    # joint_indices = []
    # for name in joint_names:
    #     try:
    #         idx = robot.joints.names.index(name)
    #         joint_indices.append(idx)
    #     except ValueError:
    #         print(f"WARNING: Joint '{name}' not found in robot model")

    # Extract times from waypoints
    # times = [w["time"] for w in waypoints]
    # timesteps = len(waypoints)
    # dt = times[1] - times[0] if len(times) > 1 else 0.1

    times = np.linspace(0, traj_duration, timesteps)

    print(f"  Trajectory optimization: {timesteps} timesteps, dt={dt:.3f}s")

    # Solve IK for start and end poses with similarity cost and self-collision
    print(f"  Solving IK for start pose...")
    target_link_index = robot.links.names.index(target_link_name)
    start_position = waypoints[0]["position"]
    start_wxyz = waypoints[0]["orientation"].as_quat()[[3, 0, 1, 2]]
    end_position = waypoints[-1]["position"]
    end_wxyz = waypoints[-1]["orientation"].as_quat()[[3, 0, 1, 2]]

    start_joint_cfg, end_joint_cfg = solve_iks(
        robot=robot,
        target_link_index=target_link_index,
        target_position_0=jnp.array(start_position),
        target_wxyz_0=jnp.array(start_wxyz),
        target_position_1=jnp.array(end_position),
        target_wxyz_1=jnp.array(end_wxyz),
        initial_joint_cfg=jnp.array(current_joint_config),
    )
    # start_joint_cfg = _solve_ik_with_similarity(
    #     robot,
    #     target_link_index,
    #     waypoints[0]["position"],
    #     waypoints[0]["orientation"].as_quat()[[3, 0, 1, 2]],
    #     initial_joint_config=current_joint_config,
    # )

    # print(f"\n  Solving IK for end pose...")
    # end_joint_cfg = _solve_ik_with_similarity(
    #     robot,
    #     target_link_index,
    #     waypoints[-1]["position"],
    #     waypoints[-1]["orientation"].as_quat()[[3, 0, 1, 2]],
    # )

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
    init_traj = jnp.linspace(start_joint_cfg, end_joint_cfg, timesteps)

    # Create trajectory variables for all timesteps
    traj_vars = robot.joint_var_cls(jnp.arange(timesteps))

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
        robot.joint_var_cls(jnp.arange(1, timesteps)),
        robot.joint_var_cls(jnp.arange(0, timesteps - 1)),
        jnp.array([1.0]),  # Weight
    )
    costs.append(smoothness_cost)

    # Acceleration cost - minimize acceleration (5-point stencil)
    accel_cost = pk.costs.five_point_acceleration_cost(
        robot.joint_var_cls(jnp.arange(2, timesteps - 2)),
        robot.joint_var_cls(jnp.arange(4, timesteps)),
        robot.joint_var_cls(jnp.arange(3, timesteps - 1)),
        robot.joint_var_cls(jnp.arange(1, timesteps - 3)),
        robot.joint_var_cls(jnp.arange(0, timesteps - 4)),
        dt,
        jnp.array([0.1]),  # Weight
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

    # Velocity limits
    vel_limits_cost = pk.costs.limit_velocity_constraint(
        robot_batched,
        robot.joint_var_cls(jnp.arange(1, timesteps)),
        robot.joint_var_cls(jnp.arange(0, timesteps - 1)),
        dt,
    )
    costs.append(vel_limits_cost)

    # Start / end pose constraints.
    @jaxls.Cost.factory(kind="constraint_eq_zero", name="start_pose_constraint")
    def start_pose_constraint(vals: jaxls.VarValues, var) -> jax.Array:
        return (vals[var] - start_joint_cfg).flatten()

    @jaxls.Cost.factory(kind="constraint_eq_zero", name="end_pose_constraint")
    def end_pose_constraint(vals: jaxls.VarValues, var) -> jax.Array:
        return (vals[var] - end_joint_cfg).flatten()

    costs.append(start_pose_constraint(robot.joint_var_cls(jnp.arange(0, 2))))
    costs.append(end_pose_constraint(robot.joint_var_cls(jnp.arange(timesteps - 2, timesteps))))

    # # Intermediate waypoint constraints (hard constraints)
    # TODO: Add intermediate waypoints
    # for i in range(timesteps):

    #     @jaxls.Cost.factory(kind="constraint_eq_zero", name=f"waypoint_{i}")
    #     def waypoint_constraint(vals: jaxls.VarValues, var, target_cfg=intermediate_cfgs[i]) -> jax.Array:
    #         return (vals[var] - target_cfg).flatten()

    #     costs.append(waypoint_constraint(robot.joint_var_cls(jnp.array(i))))

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

    return times, optimized_traj


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


def create_joint_trajectory_msg(
    joint_names: list[str],
    times: list[float],
    joint_configs: list[onp.ndarray],
    start_joint_config: onp.ndarray,
    start_time: float,
) -> JointTrajectory:
    """Create a JointTrajectory message from planned waypoints."""
    traj = JointTrajectory()
    traj.joint_names = joint_names

    # Add initial position
    point = JointTrajectoryPoint()
    joint_values = start_joint_config
    point.positions = [float(q) for q in joint_values]
    point.velocities = [0.0] * len(joint_values)
    point.accelerations = [0.0] * len(joint_values)
    point.time_from_start = Duration(sec=int(0.0), nanosec=int((0.0 % 1) * 1e9))
    traj.points.append(point)

    # point = JointTrajectoryPoint()
    # joint_values = start_joint_config
    # point.positions = [float(q) for q in joint_values]
    # point.velocities = [0.0] * len(joint_values)
    # point.accelerations = [0.0] * len(joint_values)
    # point.time_from_start = Duration(sec=int(start_time), nanosec=int((start_time % 1) * 1e9))
    # traj.points.append(point)

    for t, joint_config in zip(times, joint_configs):
        traj_time = t + start_time
        point = JointTrajectoryPoint()
        joint_values = joint_config[:-1]
        point.positions = [float(q) for q in joint_values]
        point.velocities = [0.0] * len(joint_values)
        point.accelerations = [0.0] * len(joint_values)
        point.time_from_start = Duration(sec=int(traj_time), nanosec=int((traj_time % 1) * 1e9))
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


def plot_results(recorded_data: dict, reference_trajectory: dict, config):
    """
    Plot tracking results.

    Args:
        recorded_data: Dict from TrajectoryRecorder.get_data()
        reference_trajectory: Dict with reference positions, orientations, times
        config: Robot configuration
    """
    timestamps = recorded_data["timestamps"]
    joint_states = recorded_data["joint_states"]
    pose_states = recorded_data["pose_states"]

    if len(pose_states) == 0:
        print("No pose data recorded!")
        return

    # Extract reference trajectory
    ref_times = np.array(reference_trajectory["times"])
    ref_positions = np.array(reference_trajectory["positions"])

    # Extract actual trajectory
    actual_positions = np.array([p["position"] for p in pose_states])

    # Interpolate reference to actual timestamps
    ref_pos_interp = np.column_stack(
        [
            np.interp(
                timestamps,
                ref_times,
                ref_positions[:, i],
                left=ref_positions[0, i],
                right=ref_positions[-1, i],
            )
            for i in range(3)
        ]
    )

    # Compute position error
    pos_error = actual_positions - ref_pos_interp
    pos_error_norm = np.linalg.norm(pos_error, axis=1)

    # Create plots
    fig = plt.figure(figsize=(16, 12))

    # Position tracking
    ax1 = fig.add_subplot(3, 3, 1)
    ax1.plot(ref_times, ref_positions[:, 0], "b--", label="Reference")
    ax1.plot(timestamps, actual_positions[:, 0], "r-", label="Actual")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("X Position (m)")
    ax1.set_title("X Position Tracking")
    ax1.legend()
    ax1.grid(True)

    ax2 = fig.add_subplot(3, 3, 2)
    ax2.plot(ref_times, ref_positions[:, 1], "b--", label="Reference")
    ax2.plot(timestamps, actual_positions[:, 1], "r-", label="Actual")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Y Position (m)")
    ax2.set_title("Y Position Tracking")
    ax2.legend()
    ax2.grid(True)

    ax3 = fig.add_subplot(3, 3, 3)
    ax3.plot(ref_times, ref_positions[:, 2], "b--", label="Reference")
    ax3.plot(timestamps, actual_positions[:, 2], "r-", label="Actual")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Z Position (m)")
    ax3.set_title("Z Position Tracking")
    ax3.legend()
    ax3.grid(True)

    # Position error
    ax4 = fig.add_subplot(3, 3, 4)
    ax4.plot(timestamps, pos_error[:, 0], "r-", label="X Error")
    ax4.plot(timestamps, pos_error[:, 1], "g-", label="Y Error")
    ax4.plot(timestamps, pos_error[:, 2], "b-", label="Z Error")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Position Error (m)")
    ax4.set_title("Position Error Components")
    ax4.legend()
    ax4.grid(True)

    ax5 = fig.add_subplot(3, 3, 5)
    ax5.plot(timestamps, pos_error_norm * 1000, "k-")  # Convert to mm
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Position Error Norm (mm)")
    ax5.set_title("Total Position Error")
    ax5.grid(True)

    # Joint tracking
    for i in range(min(7, len(config.joint_names))):
        ax = fig.add_subplot(3, 3, 6 + i)
        if i < len(joint_states[0]):
            ax.plot(timestamps, joint_states[:, i], "b-", label="Actual")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(f"Joint {i + 1} (rad)")
            ax.set_title(f"{config.joint_names[i]}")
            ax.grid(True)

    plt.tight_layout()
    plt.savefig("/tmp/trajectory_tracking_results.png", dpi=150)
    print("\nPlot saved to /tmp/trajectory_tracking_results.png")
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

    # Initialize with default values (for testing without robot)
    current_joint_state = [0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]  # Home config

    current_position = np.array([0.307, 0.0, 0.486])
    current_orientation = Rotation.from_quat(np.array([0.0, 1.0, 0.0, 0.0]))

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
        print("   ✓ Received live robot state")
    else:
        print("   ⚠ Using default state (robot may be disconnected)")
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
    # Straight line: 5cm offset in x direction
    end_position = current_pose["position"].copy()
    end_position[0] += 0.10  # 5cm in x
    end_pose = {
        "position": end_position,
        "orientation": current_pose["orientation"],
    }
    trajectory_duration = TRAJ_DURATION

    # Generate waypoints
    print(f"   Generating waypoints (start -> end in {trajectory_duration}s)...")
    waypoints = generate_cartesian_waypoints(
        current_pose,
        end_pose,
        num_waypoints=N_WP,
        duration=trajectory_duration,
    )
    print(f"   Generated {len(waypoints)} waypoints")

    # ---------------------------
    # MARK: 4 - Plan trajectory
    # ---------------------------
    print("\n4. Planning joint trajectory using PyRoki...")
    dt = 0.02
    times, joint_trajectory = plan_joint_trajectory(
        robot,
        target_link_name,
        waypoints,
        config.joint_names,
        timesteps=N_POINTS,
        dt=dt,
        traj_duration=trajectory_duration,
        current_joint_config=np.hstack([current_joint_state, 0.04]),
    )
    print(f"   Planned trajectory with {len(joint_trajectory)} configurations")

    # Store reference trajectory for later plotting
    reference_cart_trajectory = {
        "times": np.array([w["time"] for w in waypoints]),
        "positions": np.array([w["position"] for w in waypoints]),
        "quats": np.array([w["orientation"].as_quat()[[3, 0, 1, 2]] for w in waypoints]),
    }

    # ---------------------------
    # MARK: 5 - Visualize
    # ---------------------------
    print("\n5. Visualizing planned trajectory...")
    approved = visualize_trajectory(urdf, robot, times, joint_trajectory, JOINT_NAMES)
    if not approved:
        print("\n   Trajectory rejected by user")
        node.destroy_node()
        # rclpy.shutdown()
        return

    # Create trajectory message
    print("\n6. Creating trajectory message...")
    traj_msg = create_joint_trajectory_msg(config.joint_names, times, joint_trajectory, current_joint_state, 1.0)

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
    wait_time = trajectory_duration + 1.0
    print(f"   Waiting {wait_time}s for execution...")
    start_wait = time.time()
    while (time.time() - start_wait) < wait_time:
        rclpy.spin_once(node, timeout_sec=0.01)

    # Get recorded data
    print("\n9. Saving recorded data...")
    recorded_data = recorder.get_data()
    print(f"   Recorded {len(recorded_data['timestamps'])} samples")
    if len(recorded_data["pose_states"]) > 0:
        print(
            f"   Position range: {recorded_data['pose_states'][0]['position']} -> "
            f"{recorded_data['pose_states'][-1]['position']}"
        )

    # Plot results
    print("\n10. Plotting results...")
    plot_results(recorded_data, reference_cart_trajectory, config)

    print("\n" + "=" * 80)
    print("VALIDATION COMPLETE")
    print("=" * 80)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
