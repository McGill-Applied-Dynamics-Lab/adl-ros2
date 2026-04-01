import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

from yourdfpy import URDF

from ament_index_python.packages import get_package_share_directory

import viser
from viser.extras import ViserUrdf

from pathlib import Path
import pyroki as pk
import numpy as np
import time

import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls

JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

# Topic to read current joint positions from.
# Check yours with: ros2 topic list | grep joint_state
JOINT_STATE_TOPIC = "/fr3/franka/joint_states"

# Topic the joint_trajectory_controller listens on.
# Verify with: ros2 topic list | grep joint_trajectory
TRAJECTORY_TOPIC = "/fr3/joint_trajectory_controller/joint_trajectory"


def get_fr3_urdf() -> URDF:
    """
    Resolve the FR3 URDF path from the franka_description package.
    You need to pre-generate the URDF from xacro once:

        xacro $(ros2 pkg prefix franka_description)/share/franka_description/ \\
              robots/fr3/fr3.urdf.xacro hand:=true > /tmp/fr3.urdf

    Or point this at wherever you keep your generated URDF.
    """
    share = get_package_share_directory("franka_rim")
    # If you have a pre-generated URDF:
    models_dir = Path(share) / "models"
    urdf_path = models_dir / "fr3_franka_hand.urdf"
    meshes_dir = models_dir / "meshes"

    if not urdf_path.exists():
        raise FileNotFoundError(
            f"URDF not found at {urdf_path}. "
            "Generate it first:\n"
            "  xacro franka_description/robots/fr3/fr3.urdf.xacro "
            "hand:=true > fr3.urdf"
        )

    urdf = URDF.load(
        urdf_path,
        mesh_dir=meshes_dir,
    )

    return urdf


def solve_ik(
    robot: pk.Robot,
    target_link_name: str,
    target_wxyz: np.ndarray,
    target_position: np.ndarray,
) -> np.ndarray:
    """
    Solves the basic IK problem for a robot.

    Args:
        robot: PyRoKi Robot.
        target_link_name: String name of the link to be controlled.
        target_wxyz: np.ndarray. Target orientation.
        target_position: np.ndarray. Target position.

    Returns:
        cfg: np.ndarray. Shape: (robot.joint.actuated_count,).
    """
    assert target_position.shape == (3,) and target_wxyz.shape == (4,)
    target_link_index = robot.links.names.index(target_link_name)
    cfg = _solve_ik_jax(
        robot,
        jnp.array(target_link_index),
        jnp.array(target_wxyz),
        jnp.array(target_position),
    )
    assert cfg.shape == (robot.joints.num_actuated_joints,)
    return np.array(cfg)


@jdc.jit
def _solve_ik_jax(
    robot: pk.Robot,
    target_link_index: jax.Array,
    target_wxyz: jax.Array,
    target_position: jax.Array,
) -> jax.Array:
    joint_var = robot.joint_var_cls(0)
    variables = [joint_var]
    costs = [
        pk.costs.pose_cost_analytic_jac(
            robot,
            joint_var,
            jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz), target_position),
            target_link_index,
            pos_weight=50.0,
            ori_weight=10.0,
        ),
        pk.costs.limit_constraint(
            robot,
            joint_var,
        ),
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


class FrankaPyRokiPlanner(Node):
    def __init__(self, urdf: URDF):
        super().__init__("franka_pyroki_planner")

        self._current_joints: np.ndarray | None = None

        # 1. Initialize PyRoki Robot & Visualizer
        self.robot = pk.Robot.from_urdf(urdf)
        self._target_link_name = "fr3_hand_tcp"

        self._server = viser.ViserServer()
        self._server.scene.add_grid("/ground", width=2, height=2)
        self._urdf_vis = ViserUrdf(self._server, urdf, root_node_name="/base")

        # 2. ROS 2 Publisher for the FR3 Controller
        self._js_sub = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._joint_state_callback,
            10,
        )

        self._traj_pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)

        # Franka Joint Names
        self.joint_names = JOINT_NAMES

    def _joint_state_callback(self, msg: JointState):
        """Cache current joint positions, ordered to match JOINT_NAMES."""
        joint_map = dict(zip(msg.name, msg.position))
        try:
            self._current_joints = np.array([joint_map[n] for n in JOINT_NAMES])
        except KeyError:
            pass  # Message may not contain all joints yet

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> np.ndarray:
        """Block until at least one joint state message has arrived."""
        self.get_logger().info("Waiting for joint state...")
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self._current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                raise TimeoutError(
                    f"No joint state received on '{JOINT_STATE_TOPIC}' within {timeout_sec}s. Check the topic name."
                )
        self.get_logger().info(f"Got joint state: {np.round(self._current_joints, 4).tolist()}")
        return self._current_joints.copy()

    def plan_and_execute(self, n_waypoints=10, down_dist=0.002):
        # Start Configuration (Neutral/Home)
        q_start_np = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02])
        q_start = jnp.array(q_start_np)

        # 1. Get Current Pose (Forward Kinematics)
        # Assuming "fr3_link8" is your end-effector frame
        # ee_frame_id = "fr3_hand_tcp"
        # start_pose = self.robot.forward_kinematics(q_start, frame_name=ee_frame_id)
        start_pose = np.array([0.30753114904954515, 0.029937700083222843, 0.48166136638697377])
        self.get_logger().info(f"Start Pose Z: {start_pose[2]:.4f}")

        # 2. Generate Cartesian Waypoints (Moving down)
        target_poses = []
        for i in range(1, n_waypoints + 1):
            offset = (down_dist / n_waypoints) * i
            wp_pose = jnp.copy(start_pose)
            wp_pose = wp_pose.at[2].add(-offset)  # Subtract from Z
            target_poses.append(wp_pose)

        # 3. Compute Joint Trajectory using PyRoki Solver
        current_q = q_start_np
        self._urdf_vis.update_cfg(current_q)
        self.get_logger().info("Computing IK for waypoints...")
        # time.sleep(1.0)
        joint_trajectory = [q_start]

        for i, target in enumerate(target_poses):
            # Setup IK Costs: Match Pose + Smoothness (stay close to previous q)
            # costs = [
            #     pk.costs.PoseCost(target, frame_name=ee_frame_id, weight=1.0),
            #     pk.costs.JointLimitCost(weight=0.1),
            #     pk.costs.RegularizationCost(current_q, weight=0.01),
            # ]

            # optimizer = pk.LevenbergMarquardt(self.robot, costs)
            # sol = optimizer.solve(current_q)
            # joint_trajectory.append(sol)
            # current_q = sol

            sol = solve_ik(
                robot=self.robot,
                target_link_name=self._target_link_name,
                target_position=np.array(target),
                target_wxyz=np.array([0, 0, 1, 0]),
            )
            joint_trajectory.append(sol)
            current_q = sol

            # Visualize step-by-step
            self._urdf_vis.update_cfg(sol)

            time.sleep(0.1)

        # 4. User Confirmation
        print("\nTrajectory visualized in the browser.")
        confirm = input("Execute on real robot? (y/n): ")

        if confirm.lower() == "y":
            self.send_to_controller(joint_trajectory)
        else:
            self.get_logger().info("Execution cancelled.")

    def send_to_controller(self, q_list):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        for i, q in enumerate(q_list):
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in q]
            # 0.5s interval between waypoints for safety
            point.time_from_start = Duration(sec=0, nanosec=int((i + 1) * 5e8))
            msg.points.append(point)

        self._traj_pub.publish(msg)
        self.get_logger().info("Trajectory sent to controller!")

    def _send_via_action(self, joint_traj_msg: JointTrajectory) -> bool:
        """Send trajectory via FollowJointTrajectory action and wait for result."""
        self.get_logger().info("Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj_msg

        self.get_logger().info("Sending trajectory goal...")
        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was REJECTED by controller.")
            return False

        self.get_logger().info("Goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory completed successfully ✓")
            return True
        else:
            self.get_logger().error(
                f"Trajectory failed. Error code: {result.error_code}, message: {result.error_string}"
            )
            return False


def main():
    rclpy.init()
    # Update this with your actual URDF path
    urdf = get_fr3_urdf()
    planner = FrankaPyRokiPlanner(urdf)

    try:
        planner.plan_and_execute(n_waypoints=10, down_dist=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
