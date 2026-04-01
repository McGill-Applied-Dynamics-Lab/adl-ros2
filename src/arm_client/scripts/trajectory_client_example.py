"""
trajectory_client_example.py
------------------------------
Example showing how to update your existing Python client to use
joint_trajectory_controller instead of fr3_pose_controller.

Replace your existing CartesianTrajectory publisher with this pattern.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np
import pinocchio as pin
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory

from cartesian_to_joint_trajectory import (
    CartesianToJointTrajectory,
    CartesianWaypoint,
    pose_from_quaternion_xyz,
)


def get_urdf_path() -> str:
    """
    Resolve the FR3 URDF path from the franka_description package.
    You need to pre-generate the URDF from xacro once:

        xacro $(ros2 pkg prefix franka_description)/share/franka_description/ \\
              robots/fr3/fr3.urdf.xacro hand:=true > /tmp/fr3.urdf

    Or point this at wherever you keep your generated URDF.
    """
    share = get_package_share_directory("franka_description")
    # If you have a pre-generated URDF:
    urdf = Path(share) / "robots" / "fr3" / "fr3.urdf"
    if not urdf.exists():
        raise FileNotFoundError(
            f"URDF not found at {urdf}. "
            "Generate it first:\n"
            "  xacro franka_description/robots/fr3/fr3.urdf.xacro "
            "hand:=true > fr3.urdf"
        )
    return str(urdf)


class TrajectoryClient(Node):
    def __init__(self):
        super().__init__("trajectory_client")

        # --- Subscribe to joint states to get current configuration ---
        # This is used as the IK seed for the first waypoint, ensuring the
        # trajectory starts exactly where the robot currently is.
        self._current_joints = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/fr3/franka/joint_states",  # adjust to your actual joint states topic
            self._joint_state_callback,
            10,
        )

        # --- Publisher (simple topic mode) ---
        # Use this if you don't need feedback/result from the controller.
        # Topic: /fr3/joint_trajectory_controller/joint_trajectory
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            "/fr3/joint_trajectory_controller/joint_trajectory",
            10,
        )

        # --- Action client (recommended for experiments) ---
        # Use this if you want to wait for completion and check success/failure.
        # Action: /fr3/joint_trajectory_controller/follow_joint_trajectory
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fr3/joint_trajectory_controller/follow_joint_trajectory",
        )

        # --- IK converter ---
        self._converter = CartesianToJointTrajectory(
            urdf_path=get_urdf_path(),
            end_effector_frame="fr3_hand_tcp",
        )

        self.get_logger().info("TrajectoryClient ready.")

    def _joint_state_callback(self, msg: JointState):
        """Cache the latest joint state (used as IK seed)."""
        # JointState may not be in the same order as FR3_JOINT_NAMES.
        # Build a dict and extract in the right order.
        joint_map = dict(zip(msg.name, msg.position))
        joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]
        try:
            self._current_joints = np.array([joint_map[n] for n in joint_names])
        except KeyError:
            pass  # Message might not contain all joints yet

    def get_current_joints(self, timeout_sec: float = 5.0) -> np.ndarray:
        """Block until we have a joint state reading."""
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self._current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                raise TimeoutError("No joint state received within timeout.")
        return self._current_joints.copy()

    # ------------------------------------------------------------------
    # Main public method: run a Cartesian trajectory
    # ------------------------------------------------------------------

    def run_cartesian_trajectory(
        self,
        cartesian_waypoints: list[CartesianWaypoint],
        use_action: bool = True,
    ) -> bool:
        """
        Convert a Cartesian trajectory to joint space and execute it.

        Args:
            cartesian_waypoints: List of CartesianWaypoint (pose + timestamp).
            use_action: If True, use the action interface and wait for result.
                        If False, publish on topic and return immediately.

        Returns:
            True if trajectory completed successfully (action mode only).
        """
        # Get current joint state as IK seed for the first waypoint.
        # This ensures the first IK solution is close to the current config,
        # keeping the joint path smooth at the start.
        q_current = self.get_current_joints()
        self.get_logger().info(f"Current joints: {np.round(q_current, 3).tolist()}")

        # Run IK + limit validation (raises on failure — safe by default)
        try:
            joint_traj_msg = self._converter.convert(
                cartesian_waypoints,
                q_seed=q_current,
            )
        except ValueError as e:
            self.get_logger().error(f"Trajectory conversion failed:\n{e}")
            return False

        if use_action:
            return self._send_via_action(joint_traj_msg)
        else:
            self._traj_pub.publish(joint_traj_msg)
            self.get_logger().info("Trajectory published (topic mode, not waiting for completion).")
            return True

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


# ---------------------------------------------------------------------------
# Example usage — replace with your actual trajectory
# ---------------------------------------------------------------------------


def main():
    rclpy.init()
    client = TrajectoryClient()

    # --- Define your Cartesian trajectory here ---
    # Replace this with your actual experiment path.
    # Each waypoint is a pose (SE3) + time from start (seconds).
    #
    # Example: a simple linear motion in Z over 3 seconds with 30 waypoints.
    # In your real code, this comes from your existing trajectory generation logic.

    num_waypoints = 30
    total_duration = 3.0  # seconds
    times = np.linspace(0.0, total_duration, num_waypoints)

    waypoints = []
    for i, t in enumerate(times):
        # Example: move from z=0.5 to z=0.6 in a straight line
        z = 0.5 + 0.1 * (t / total_duration)
        pose = pose_from_quaternion_xyz(
            qx=1.0,
            qy=0.0,
            qz=0.0,
            qw=0.0,  # pointing down
            x=0.4,
            y=0.0,
            z=z,
        )
        waypoints.append(CartesianWaypoint(pose=pose, time_from_start=t))

    success = client.run_cartesian_trajectory(waypoints, use_action=True)
    if not success:
        print("Trajectory execution failed.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
