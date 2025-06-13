import rclpy
from ament_index_python.packages import get_package_share_directory

# RL imports
import pinocchio as pin
import numpy as np
import torch
from pathlib import Path
import yaml


# ROS2 Communications
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist  # Example message types
from franka_msgs.msg import FrankaRobotState  # TODO: Update when processed in the interface

from robot_arm_interface.utils import motion2rostwist, rostwist2motion, rospose2se3, se32rospose

# Action and services
from arm_interfaces.action import PegInHole
from rclpy.executors import MultiThreadedExecutor

from robot_tasks.rl_agent.Base import RlAgentNode


class Insert_AugRlAgentNode(RlAgentNode):
    """
    Implementation of the AugRlAgent for insertion tasks.
    """

    def __init__(self):
        # Constants
        self.SOCKET_POSE = np.array([0.6, 0.0, 0.0])
        self.Z_STOP = 0.015  # 1cm

        pkg_dir = Path(get_package_share_directory("robot_tasks"))
        models_dir = pkg_dir / "agents" / "insert"

        default_agent = "insert_agent"

        # Initialize base class
        super().__init__(
            node_name="rl_agent_node",
            action_type=PegInHole,
            action_server_name="/insert_action",
            agent_lib="skrl",
            default_models_dir=models_dir,
            default_agent_dir=default_agent,
            observation_dim=24,
            action_dim=3,
        )

        #! Set up and load environment config
        # pkg_dir = Path(__file__).parent.parent

        # Set up and load environment config
        env_cfg_path = self._agent_dir / "params" / "env.yaml"

        print(f"[INFO]: Parsing configuration from: {env_cfg_path}")
        with open(env_cfg_path, encoding="utf-8") as f:
            self.env_cfg = yaml.full_load(f)

        #! Initialize state storage specific to insertion task
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

        # Poses and twists
        self.X_GP = pin.SE3(
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), np.array([0, 0, 0.04])
        )  # Gripper to peg transform
        self.V_G = None  # Gripper velocity
        self.V_G_des = None  # Desired gripper velocity

        self.X_S = pin.SE3(pin.rpy.rpyToMatrix(np.array([0, 0, 0])), self.SOCKET_POSE)  # Socket position
        self.X_P = None  # Peg position
        self.X_G = None  # Gripper pose

    def _init_subscribers(self):
        """
        Initialize subscribers for robot and env states.
        """
        self.get_logger().info("Initializing subscribers...")

        # --- robot ---
        robot_topic = "/franka_robot_state_broadcaster/robot_state"
        self._robot_sub = self.create_subscription(FrankaRobotState, robot_topic, self._robot_state_callback, 10)

    def _init_publishers(self):
        """
        Initialize publishers for robot commands.
        """
        self.get_logger().info("Initializing publishers...")

        robot_cmd_topic = "/robot_arm/gripper_vel_command"
        self._robot_cmd_pub = self.create_publisher(
            TwistStamped,
            robot_cmd_topic,
            10,
        )

    def _create_result(self, success=False, message=""):
        """Create a PegInHole result message."""
        result = PegInHole.Result()
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
        if self._joint_state is None or self.X_GP is None or self.X_S is None:
            self.get_logger().warn("Missing some observations...", throttle_duration_sec=5)
            return None

        #! Process observations
        joint_pos_rel = np.array(self._joint_state.position) - self.default_joint_poses
        joint_pos_rel = np.concatenate([joint_pos_rel, np.array([0.0, 0.0])])  # Add fingers

        joint_vel_rel = np.array(self._joint_state.velocity)
        joint_vel_rel = np.concatenate([joint_vel_rel, np.array([0.0, 0.0])])  # Add fingers

        self.X_P = self.X_G * self.X_GP
        self.peg_error = self.X_P.translation - self.X_S.translation
        peg_pos_rel = self.peg_error

        last_action = self._last_action

        obs_list = [joint_pos_rel, joint_vel_rel, peg_pos_rel, last_action]

        try:
            observation = np.concatenate(obs_list).astype(np.float32)
            if observation.shape != (self._observation_dim,):
                self.get_logger().error(
                    f"Observation shape mismatch: expected ({self._observation_dim},), got {observation.shape}"
                )
                return None
        except ValueError as e:
            self.get_logger().error(f"Error concatenating observation: {e}")
            return None

        obs_tensor = torch.tensor(observation, dtype=torch.float32).to(self.device)
        return obs_tensor

    def init_task_execution(self):
        """Initialize task-specific parameters."""
        # Set motion parameters
        self.des_ee_speed = self.env_cfg["actions"]["arm_action"]["des_ee_speed"] / 10
        self.V_WG_ctrl = pin.Motion(np.array([0, 0, self.des_ee_speed]), np.zeros(3))  # Gripper velocity in world frame

    def check_task_status(self):
        """
        Check if insertion is successful or failed.

        Returns:
            Tuple[bool, bool]: (is_success, is_failure)
        """
        success = False
        failure = False
        z_thres = self.Z_STOP  # 1cm
        xy_thres = 0.03  # 3cm

        # Peg z reached the bottom
        if self.X_P.translation[2] < z_thres:
            # Check if x in range
            xy_err = np.linalg.norm(self.peg_error[:2])
            if xy_err < xy_thres:
                success = True
                failure = False
            else:
                success = False
                failure = True

        return success, failure

    def get_task_result(self):
        """Get the result of the insertion task with custom messages."""
        if self._is_task_success:
            return True, "Insertion succeeded."
        elif self._is_task_failure:
            return False, "Insertion failed."
        return False, "Insertion incomplete."

    def process_action(self, action):
        """Process the raw action from the agent."""
        # Scale the action
        processed_action = action * self.action_scale

        # Create motion from action
        self.V_G_agt = pin.Motion(
            np.array([processed_action[0], 0.0, processed_action[1]]),
            np.array([0.0, processed_action[2], 0.0]),
        )

        # Combine with control action
        self.V_G_des = self.V_WG_ctrl + self.V_G_agt

    def send_robot_command(self):
        """Send the command to the robot."""
        msg = TwistStamped()
        gripper_twist = motion2rostwist(self.V_G_des)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = gripper_twist

        self._robot_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Insert_AugRlAgentNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()

        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt detected.")

        finally:
            node.get_logger().info("Shutting down executor...")
            executor.shutdown()
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
