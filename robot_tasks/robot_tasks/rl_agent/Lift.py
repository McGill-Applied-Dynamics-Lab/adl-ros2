import rclpy
from ament_index_python.packages import get_package_share_directory

# RL imports
import pinocchio as pin
import numpy as np
import torch
from pathlib import Path
import yaml


# ROS2 Communications
import rclpy.time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist  # Example message types
from franka_msgs.msg import FrankaRobotState  # TODO: Update when processed in the interface

from robot_arm_interface.utils import motion2rostwist, rostwist2motion, rospose2se3, se32rospose
from robot_arm_interface.fr3_interface import GoalSource, ControlMode

# TF2
from tf2_ros import TransformListener, Buffer

# Action and services
from arm_interfaces.action import RlAgent
from rclpy.executors import MultiThreadedExecutor

from robot_tasks.rl_agent.Base import RlAgentNode

import os
import csv

MAX_TIME = 100


class Lift_AgentNode(RlAgentNode):
    """
    Implementation of the Lift agent.

    Actions (7,):
    - Delta translation [dx, dy, dz] (3,)
    - Delta rotation, angle-axis representation (3,)
    - Gripper opening/closing (1,)

    Observations (19,):
    - object: pose of the object (10,)
        - object position [x, y, z]. World frame (3,)
        - object orientation. quaternions representation. World frame (4,)
        - object relative position to gripper [x, y, z]. Gripper frame (3,)
    - robot0_eef_pos: end-effector position (3,)
        - [x, y, x]. World frame
    - robot0_eef_quat: end-effector orientation (4,)
        - quaternions representation. World frame
    - robot0_gripper_qpos: finger position (2,)
        - Distance of each finger to the gripper center.

    Control rate: 20 Hz
    """

    def __init__(self):
        # pkg_dir = Path(__file__).parent.parent
        pkg_dir = Path(get_package_share_directory("robot_tasks"))
        models_dir = pkg_dir / "agents" / "lift"

        default_agent = "lift_agent"

        # Initialize base class
        super().__init__(
            node_name="lift_agent_node",
            action_type=RlAgent,
            action_server_name="/lift_action",
            agent_lib="robomimic",
            default_models_dir=models_dir,
            default_agent_dir=default_agent,
            observation_dim=19,
            action_dim=7,
            default_ctrl_frequency=2.0,
        )

        #! Task parameters
        self.action_scale = 0.1

        self._ctrl_mode = ControlMode.CART_POSE
        self._goal_src = GoalSource.TOPIC

        # Initialize state storage specific to insertion task
        self.default_joint_poses = [
            0.0,
            0.53249995,
            0.0,
            -2.02528006,
            0.0,
            2.55778002,
            0.78539816,
        ]  # Start position of the joints [rad]

        self._joint_state = None

        self.task_start_time = None
        self.max_task_duration = rclpy.time.Duration(seconds=MAX_TIME)

        # Poses and twists
        self.X_GP = pin.SE3(
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), np.array([0, 0, 0.04])
        )  # Gripper to peg transform
        # self.V_G = None  # Gripper velocity
        # self.V_G_des = None  # Desired gripper velocity
        self.X_G_des = None

        self.X_C = pin.SE3(pin.rpy.rpyToMatrix(np.array([0, 0, 0])), np.array([0.6, 0.0, 0.0]))  # Cube pose
        self.X_G = None  # Gripper pose
        self.X_GC = None  # Gripper to cube transform

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world_frame = "world"
        self.cube_frame = "cube_frame"

        csv_path = os.path.join(str(Path(__file__).parent.parent), "demo_68_actions.csv")
        self.get_logger().info(f"Loading actions from CSV: {csv_path}")
        self._csv_actions = []
        with open(csv_path, "r") as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                # Only take the first 7 columns (actions)
                self._csv_actions.append([float(x) for x in row[:7]])
        self._csv_action_idx = 0

    def _init_subscribers(self):
        """
        Initialize subscribers for robot and env states.
        """
        self.get_logger().info("Initializing subscribers...")

        # --- robot ---
        robot_topic = "/franka_robot_state_broadcaster/robot_state"
        self._robot_sub = self.create_subscription(FrankaRobotState, robot_topic, self._robot_state_callback, 10)

        # # --- object ---
        # object_topic = "/object_pose"
        # self._object_sub = self.create_subscription(PoseStamped, object_topic, self._object_state_callback, 10)

    def _init_publishers(self):
        """
        Initialize publishers for robot commands.
        """
        self.get_logger().info("Initializing publishers...")

        robot_cmd_topic = "/robot_arm/gripper_pose_cmd"
        self._robot_cmd_pub = self.create_publisher(
            PoseStamped,
            robot_cmd_topic,
            10,
        )

    def _create_result(self, success=False, message=""):
        """Create a Lift result message."""
        result: RlAgent.Result = self._action_type.Result()
        result.success = success
        result.message = message
        return result

    def _robot_state_callback(self, robot_state_msg: FrankaRobotState):
        """
        Callback for the '/franka_robot_state_broadcaster/robot_state' topic.
        """
        self._joint_state = robot_state_msg.measured_joint_state
        self.X_G = rospose2se3(robot_state_msg.o_t_ee.pose)
        self.V_G = rostwist2motion(robot_state_msg.o_dp_ee_d.twist)

    def _get_observation(self) -> torch.Tensor:
        """
        Get the latest observation from the robot's sensors.
        """
        # Return if one observation component is not available
        tf_cube_pose = self._get_cube_pose_from_tf()

        if self._joint_state is None or self.X_GP is None or tf_cube_pose is None:
            self.get_logger().warn("Missing some observations...", throttle_duration_sec=5)
            return None

        #! Process observations
        p_G = self.X_G.translation
        quat_G = pin.Quaternion(self.X_G.rotation).coeffs()  # [x, y, z, w]
        gripper_state = np.array([0, 0])  # TODO: Update with actual gripper state

        # Try to get cube position from TF2, fall back to stored value if not available
        self.X_C = tf_cube_pose
        p_C = self.X_C.translation
        q_C = pin.Quaternion(self.X_C.rotation).coeffs()  # [x, y, z, w]

        p_GC = self.X_C.translation - self.X_G.translation
        p_CG = self.X_G.translation - self.X_C.translation

        R_WG = self.X_G.rotation
        p_GC_G = R_WG.T @ p_GC

        object_array = [
            p_C,
            q_C,
            p_GC_G,
        ]
        object_array = np.concatenate(object_array).astype(np.float32)

        observation_dict = {
            "object": torch.tensor(object_array, dtype=torch.float32).to(self.device),
            "robot0_eef_pos": torch.tensor(p_G, dtype=torch.float32).to(self.device),
            "robot0_eef_quat": torch.tensor(quat_G, dtype=torch.float32).to(self.device),
            "robot0_gripper_qpos": torch.tensor(gripper_state, dtype=torch.float32).to(self.device),
        }

        return observation_dict

    def _get_cube_pose_from_tf(self) -> pin.SE3:
        """
        Get the cube pose from the TF tree.

        Returns:
            pin.SE3: The cube pose in the world frame
        """
        # TODO: Return the the transform X_GC instead of X_WC
        try:
            # Look up the transform from world to cube
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.cube_frame, time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Extract position and orientation from the transform
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert to pin.SE3
            position = np.array([translation.x, translation.y, translation.z])
            quaternion = np.array([rotation.x, rotation.y, rotation.z, rotation.w])

            # Convert quaternion to rotation matrix
            rot_matrix = pin.Quaternion(quaternion).toRotationMatrix()

            # Create the SE3 transform
            return pin.SE3(rot_matrix, position)

        except Exception as e:
            self.get_logger().warn(f"Failed to get cube pose from TF: {str(e)}", throttle_duration_sec=0.01)
            return None

    def init_task_execution(self):
        """Initialize task-specific parameters."""
        self.task_start_time = self.get_clock().now()

        if self.X_G is not None:
            self.X_G_des = self.X_G.copy()

    def check_task_status(self):
        """
        Check if insertion is successful or failed.

        Returns:
            Tuple[bool, bool]: (is_success, is_failure)
        """
        success = False
        failure = False

        # if self.task_start_time is None:
        #     self.get_logger().warn("Task start time is not set.")
        #     return success, failure

        current_time = self.get_clock().now()

        if current_time - self.task_start_time > self.max_task_duration:
            failure = True
            self.get_logger().info("Task timed out.")

        return success, failure

    def process_action(self, action):
        """Process the raw action from the agent or from CSV for debugging."""

        # # On first call, load actions from CSV
        # if self._csv_actions is None:
        #     return

        # # Use next action from CSV
        # if self._csv_action_idx < len(self._csv_actions):
        #     action = np.array(self._csv_actions[self._csv_action_idx], dtype=np.float32)
        #     self._csv_action_idx += 1
        #     self.get_logger().info(f"Applying CSV action {self._csv_action_idx}/{len(self._csv_actions)}: {action}")

        # else:
        #     self.get_logger().warn("No more actions in CSV. Skipping action application.")
        #     return

        # Scale the action
        processed_action = action[:6] * self.action_scale

        R_WG = self.X_G.rotation

        # rot_vec = action[3:6]
        rot_vec = R_WG @ action[3:6]  # TODO TRANSPOSE ROT MATRIX??
        delta_rotation_matrix = pin.exp3(rot_vec)

        delta_trans = R_WG @ processed_action[:3]

        delta_pose_SE3 = pin.SE3(delta_rotation_matrix, delta_trans)

        self.X_G_des = self.X_G * delta_pose_SE3

        if action[6] > 0.5:
            self.gripper_state = "open"
        else:
            self.gripper_state = "close"

    def send_robot_command(self):
        """Send the command to the robot."""
        msg = PoseStamped()

        gripper_pose = se32rospose(self.X_G_des)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = gripper_pose

        self._robot_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Lift_AgentNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt detected.")
            # Properly handle any active action goals when interrupted
            with node._lock:
                if node._goal_handle is not None and node._goal_handle.is_active:
                    node.get_logger().info("Cancelling active goal due to keyboard interrupt")
                    node._task_finished = True
                    try:
                        node._goal_handle.abort()
                    except Exception as e:
                        node.get_logger().warn(f"Error aborting goal during shutdown: {e}")
        finally:
            node.get_logger().info("Shutting down executor...")
            executor.shutdown()
            if node is not None:
                # Make sure we're not in the middle of an action execution when shutting down
                node._task_finished = True
                node._cleanup_goal()
                node.destroy_node()

    except Exception as e:
        # Log any exceptions raised during node initialization
        print(f"Error during node initialization or shutdown: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
            print("RCLPY shutdown.")


if __name__ == "__main__":
    main()
