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
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist  # Example message types
from franka_msgs.msg import FrankaRobotState  # TODO: Update when processed in the interface

from robot_arm_interface.utils import motion2rostwist, rostwist2motion, rospose2se3, se32rospose
from robot_arm_interface.fr3_interface import GoalSource, ControlMode
from arm_interfaces.action import GripperClose, GripperOpen  # , GripperHoming

# TF2
from tf2_ros import TransformListener, Buffer

# Action and services
from arm_interfaces.action import RlAgent
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter


from robot_tasks.rl_agent.Base import RlAgentNode

import os
import csv

MAX_TIME = 20
KP = 7.0
KD = 0.6

Z_SUCCESS_THRESHOLD = 0.075  # 7.5 cm z threshold for success
GRIPPER_SUCCESS_THRESHOLD = 0.055  # 5.5 cm gripper distance threshold for success


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

        default_agent = "qpos_2025-05-27"

        # Slow down factor
        self.slow_down_factor = 0.5  # 0.2 = 5x slower
        base_freq = 20.0
        self.ctrl_rate = base_freq * self.slow_down_factor
        self.command_rate = 20.0

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
            default_ctrl_frequency=self.ctrl_rate,
            default_command_frequency=self.command_rate,
        )

        #! Task parameters
        self.action_scale = 0.05

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
        self._gripper_finger_distance = np.array([0.0, 0.0], dtype=np.float32)  # Store finger positions

        self.task_start_time = None
        self.max_task_duration = rclpy.time.Duration(seconds=MAX_TIME)

        # Poses and twists
        self.processed_action = None

        self.X_WG = None  # Gripper pose in world frame
        self.X_BG = None  # Gripper pose in base frame

        self.X_BG_des = None
        self.X_WG_des = None

        # self.X_C = pin.SE3(pin.rpy.rpyToMatrix(np.array([0, 0, 0])), np.array([0.6, 0.0, 0.0]))  # Cube pose
        self.X_C = None  # Cube pose in world frame
        self.X_GC = None  # Gripper to cube transform

        # World frame
        p_WBsim = np.array([-0.56, 0.0, 0.912])  # World frame position
        p_BsimB = np.array([0.0, 0.0, -0.1095])  # World frame position

        rpy_WB = np.array([0.0, 0.0, 0.0])  # World frame to base frame rpy angles

        self.X_WBsim = pin.SE3(pin.rpy.rpyToMatrix(rpy_WB), p_WBsim)  # World to base transform
        self.X_BsimB = pin.SE3(pin.rpy.rpyToMatrix(rpy_WB), p_BsimB)  # Base to base simulation transform
        self.X_WB = self.X_WBsim * self.X_BsimB  # World to base transform in simulation

        self.gripper_offset = 0.0

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = "base"
        self.cube_frame = "cube_frame"

        # Set gains
        # self._set_control_gains(KP, KD)

        # ? Temp, load from csv for testing
        file_name = "actions_20250521_144331.csv"
        # file_name = "demo_68_actions.csv"

        csv_path = os.path.join(str(Path(__file__).parent.parent), file_name)
        self.get_logger().info(f"Loading actions from CSV: {csv_path}")

        self._csv_actions = []
        with open(csv_path, "r") as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                # Only take the first 7 columns (actions)
                self._csv_actions.append([float(x) for x in row[:7]])

        # Load observations
        obs_mapping = {
            "object": ("obs_object_20250521_144331.csv", 10),
            "robot0_eef_pos": ("obs_robot0_eef_pos_20250521_144331.csv", 3),
            "robot0_eef_quat": ("obs_robot0_eef_quat_20250521_144331.csv", 4),
            "robot0_gripper_qpos": ("obs_robot0_gripper_qpos_20250521_144331.csv", 2),
        }

        self.obs_csv_data = {
            "object": [],
            "robot0_eef_pos": [],
            "robot0_eef_quat": [],
            "robot0_gripper_qpos": [],
        }

        for each_obs, (file_name, n_rows) in obs_mapping.items():
            obs_path = os.path.join(str(Path(__file__).parent.parent), "obs_test", file_name)
            self.get_logger().info(f"Loading {each_obs} from CSV: {obs_path}")

            with open(obs_path, "r") as f:
                reader = csv.reader(f)
                next(reader)

                for row in reader:
                    # Only take the first 7 columns (actions)
                    self.obs_csv_data[each_obs].append([float(x) for x in row[:n_rows]])

        self._csv_action_idx = 0

        self.get_logger().info("Lift agent node initialized.")
        self.get_logger().info(f"Control rate: {self.ctrl_rate} Hz")
        self.get_logger().info(f"Command rate: {self.command_rate} Hz")
        self.get_logger().info(f"Action scale: {self.action_scale}")

        # Gripper action clients
        self.gripper_state = "open"  # Initial gripper state
        self._last_gripper_command = "open"  # Track last command sent: 'open', 'close', or None
        self._gripper_goal_in_progress = False  # Prevent overlapping goals
        self._gripper_open_client = ActionClient(self, GripperOpen, "/gripper_open")
        self._gripper_close_client = ActionClient(self, GripperClose, "/gripper_close")

        while not self._gripper_open_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for gripper open action server...")

        while not self._gripper_close_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for gripper close action server...")

        # Add gripper joint state subscriber
        gripper_joint_topic = "/fr3_gripper/joint_states"
        self._gripper_joint_sub = self.create_subscription(
            JointState,
            gripper_joint_topic,
            self._gripper_joint_state_callback,
            10,
        )

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

    #! Observation and action methods
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
        self.X_BG = rospose2se3(robot_state_msg.o_t_ee.pose)
        self.V_G = rostwist2motion(robot_state_msg.o_dp_ee_d.twist)

    def _gripper_joint_state_callback(self, msg: JointState):
        """
        Callback for the '/fr3_gripper/joint_states' topic.
        Updates the gripper finger distances.
        """
        # Expecting two positions: [finger1, finger2]
        if len(msg.position) == 2:
            self._gripper_finger_distance = np.array(msg.position[:2], dtype=np.float32)
            self._gripper_finger_distance[1] = -self._gripper_finger_distance[1]  # Invert second finger position
        else:
            self.get_logger().warn("Received gripper joint state with insufficient positions.")

    def _get_observation(self) -> torch.Tensor:
        """
        Get the latest observation from the robot's sensors.
        """
        # Return if one observation component is not available
        tf_cube_pose = self._get_cube_pose_from_tf()
        # self.X_BG = self._get_tool_pose_from_tf()

        if self.X_BG is None or tf_cube_pose is None:
            self.get_logger().warn("Missing some observations...", throttle_duration_sec=5)
            return None

        #! Process observations
        # * Gripper state *
        gripper_state = self._gripper_finger_distance.copy()  # Use actual finger positions

        # * Gripper pose *
        self.X_WG = self.X_WB * self.X_BG

        p_WG = self.X_WG.translation
        quat_WG = pin.Quaternion(self.X_WG.rotation).coeffs()  # [x, y, z, w]

        # Add offset
        p_WG[2] -= self.gripper_offset  # Adjust z position by gripper offset

        # * Cube pose *
        self.X_BC = tf_cube_pose
        self.X_WC = self.X_WB * self.X_BC

        p_WC = self.X_WC.translation
        q_WC = pin.Quaternion(self.X_WC.rotation).coeffs()  # [x, y, z, w]

        # * Cube pose in gripper frame *
        p_BC_B = self.X_BC.translation
        p_BG_B = self.X_BG.translation

        p_GC_B = p_BC_B - p_BG_B
        p_CG_B = -p_GC_B
        # p_CG_W = self.X_WB.rotation @ p_CG_B

        R_BG = self.X_BG.rotation

        # p_GC_G = R_BG.T * p_GC_B
        p_GC_W = self.X_WB.rotation @ p_GC_B
        p_CG_W = -p_GC_W

        # offset
        p_CG_W[2] -= self.gripper_offset  # Adjust z position by gripper offset

        # * Observation vector *
        object_array = [
            p_WC,
            q_WC,
            p_CG_W,
        ]
        object_array = np.concatenate(object_array).astype(np.float32)

        # #! Overwrite with CSV data
        # if self.obs_csv_data is not None:
        #     object_array = np.array(self.obs_csv_data["object"][self._csv_action_idx], dtype=np.float32)
        #     p_WG = np.array(self.obs_csv_data["robot0_eef_pos"][self._csv_action_idx], dtype=np.float32)
        #     quat_WG = np.array(self.obs_csv_data["robot0_eef_quat"][self._csv_action_idx], dtype=np.float32)
        #     gripper_state = np.array(self.obs_csv_data["robot0_gripper_qpos"][self._csv_action_idx], dtype=np.float32)

        #     p_WC = object_array[:3]
        #     q_WC = object_array[3:7]
        #     p_CG_W = object_array[7:10]

        #     # p_WG = gripper_state[:3]

        #     self._csv_action_idx += 1
        #     if self._csv_action_idx >= len(self._csv_actions):
        #         self._csv_action_idx = 0

        observation_dict = {
            "object": torch.tensor(object_array, dtype=torch.float32).to(self.device),
            "robot0_eef_pos": torch.tensor(p_WG, dtype=torch.float32).to(self.device),
            "robot0_eef_quat": torch.tensor(quat_WG, dtype=torch.float32).to(self.device),
            "robot0_gripper_qpos": torch.tensor(gripper_state, dtype=torch.float32).to(self.device),
        }

        print(
            f"p_WC: {p_WC[0]:>7.4f} {p_WC[1]:>7.4f} {p_WC[2]:>7.4f} | "
            f"p_CG_W: {p_CG_W[0]:>7.4f} {p_CG_W[1]:>7.4f} {p_CG_W[2]:>7.4f} | "
            f"q_WC: {q_WC[0]:>7.4f} {q_WC[1]:>7.4f} {q_WC[2]:>7.4f} {q_WC[3]:>7.4f} | "
            # f"p_WG: {p_WG[0]:>7.4f} {p_WG[1]:>7.4f} {p_WG[2]:>7.4f} | "
            # f"fingers: {gripper_state[0]:>7.4f} {gripper_state[1]:>7.4f} | ",
            f"q_WG: {quat_WG[0]:>7.4f} {quat_WG[1]:>7.4f} {quat_WG[2]:>7.4f} {quat_WG[3]:>7.4f}",
        )

        return observation_dict

    def _get_cube_pose_from_tf(self) -> pin.SE3:
        """
        Get the cube pose from the TF tree.

        Returns:
            pin.SE3: The cube pose in the **base** frame
        """
        try:
            # Look up the transform from world to cube
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.cube_frame, time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Extract position and orientation from the transform
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert to pin.SE3
            position = np.array([translation.x, translation.y, translation.z])
            quaternion = np.array([rotation.x, rotation.y, rotation.z, rotation.w])

            # Convert quaternion to rotation matrix
            rot_matrix = pin.Quaternion(quaternion).toRotationMatrix()

            X_BC = pin.SE3(rot_matrix, position)  # Cube pose in base frame

            # Create the SE3 transform
            return X_BC

        except Exception as e:
            self.get_logger().warn(f"Failed to get cube pose from TF: {str(e)}", throttle_duration_sec=0.01)
            return None

    def _get_tool_pose_from_tf(self) -> pin.SE3:
        """
        Get the tool pose from the TF tree.

        Returns:
            pin.SE3: The tool pose in the **base** frame
        """
        try:
            # Look up the transform from world to gripper
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, "fr3_tool", time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Extract position and orientation from the transform
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert to pin.SE3
            position = np.array([translation.x, translation.y, translation.z])
            quaternion = np.array([rotation.x, rotation.y, rotation.z, rotation.w])

            # Convert quaternion to rotation matrix
            rot_matrix = pin.Quaternion(quaternion).toRotationMatrix()

            X_BG = pin.SE3(rot_matrix, position)  # Gripper pose in base frame

            return X_BG

        except Exception as e:
            self.get_logger().warn(f"Failed to get tool pose from TF: {str(e)}", throttle_duration_sec=0.01)
            return None

    async def init_task_execution(self):
        """Initialize task-specific parameters."""
        self.task_start_time = self.get_clock().now()

        self._last_action = None
        self.gripper_state = "open"  # Initial gripper state
        self._last_gripper_command = "open"  # Track last command sent: 'open', 'close', or None

        if self.X_BG is not None:
            self.X_BG_des = self.X_BG.copy()

        Kp_future, Kd_future = self._set_control_gains(KP, KD)

        # Wait for the service calls to complete
        await Kp_future
        await Kd_future

        if Kp_future.result() is None or Kd_future.result() is None:
            self.get_logger().error("Failed to set control gains.")
            return False

        # TODO: Remove
        self._csv_action_idx = 0

        #! Find gripper offset
        try:
            transform = self.tf_buffer.lookup_transform(
                "fr3_hand_tcp", "fr3_tool", time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"Failed to get gripper offset from TF: {str(e)}")
            return False

        if transform is not None:
            self.get_logger().info("Found gripper offset from TF.")
            # Extract position and orientation from the transform
            self.gripper_offset = transform.transform.translation.z
        else:
            raise RuntimeError("Failed to get gripper offset from TF. Ensure the transform is published.")

        return True

    def check_task_status(self):
        """
        Check if insertion is successful or failed.

        Returns:
            Tuple[bool, bool]: (is_success, is_failure)
        """
        success = False
        failure = False

        # Success if z > threshold and gripper dist < threshold
        if self.X_BG is not None:
            p_BG = self.X_BG.translation - np.array([0.0, 0.0, self.gripper_offset])  # Adjust for gripper offset

            gripper_distance = np.linalg.norm(self._gripper_finger_distance)

            # Check if gripper is above the cube
            if p_BG[2] > Z_SUCCESS_THRESHOLD and gripper_distance < GRIPPER_SUCCESS_THRESHOLD:
                success = True
                self.get_logger().info("Lift successful!")

        current_time = self.get_clock().now()

        if current_time - self.task_start_time > self.max_task_duration:
            failure = True
            self.get_logger().info("Task timed out.")

        return success, failure

    def _get_action(self):
        """
        Get the action from the agent.

        Called in the main loop (`control_loop_callback`).

        Runs at the control rate.
        """
        #! Update desired gripper pose to current gripper pose
        # if self.X_BG is not None:
        #     self.X_BG_des = self.X_BG.copy()

        #! From agent
        action = super()._get_action()
        # print(f"Gripper:\t {action[-1]:>7.4f} | ")
        # print(
        #     f"rot action: {action[3]:>7.4f} {action[4]:>7.4f} {action[5]:>7.4f} | "
        #     # f"Gripper: {{action[6]:>7.4f}} | "
        # )
        return action

        # #! Fixed action
        # current_time = self.get_clock().now()

        # # if current_time - self.task_start_time > rclpy.time.Duration(seconds=4):
        # #     action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0], dtype=np.float32)

        # # elif current_time - self.task_start_time > rclpy.time.Duration(seconds=2):
        # #     action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)

        # # else:
        # #     action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0], dtype=np.float32)

        # action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, -0.25, -1.0], dtype=np.float32)

        # return action

        # #! CSV Action
        # if self._csv_actions is None:
        #     return

        # # Use next action from CSV
        # self.get_logger().info("Applying CSV actions...", throttle_duration_sec=2.0)
        # if self._csv_action_idx < len(self._csv_actions):
        #     action = np.array(self._csv_actions[self._csv_action_idx], dtype=np.float32)
        #     self._csv_action_idx += 1
        #     # self.get_logger().info(f"Applying CSV action {self._csv_action_idx}/{len(self._csv_actions)}: {action}")

        # else:
        #     self.get_logger().warn("No more actions in CSV.", throttle_duration_sec=2.0)
        #     action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        #     return

        # return action

    def process_action(self):
        """Process the raw action from the agent to convert it to desired gripper pose and gripper state.

        Sends the computed commands to the robot.

        Called from the command timer callback.

        Runs at the command rate.
        """
        if not self._control_loop_active or self._last_action is None:
            return

        action = self._last_action

        # * Process action
        # Scale the action.
        self.processed_action = action[:6].copy()  # Copy the first 6 elements
        self.processed_action[:3] = action[:3] * self.action_scale * (self.ctrl_rate / self.command_rate)
        self.processed_action[3:6] = action[3:6] * (self.ctrl_rate / self.command_rate)

        #! ACTION WRT CURRENT GRIPPER POSE
        p_GGd_W = self.processed_action[:3]  # Gripper translation
        rot_vec = self.processed_action[3:6]

        # * Rotations
        # ? Option 1 - Assume rotation vector is in world frame *
        rvec_W = rot_vec

        R_GW = self.X_WG.rotation.T
        rvec_G = R_GW @ rvec_W  # Rotation vector in gripper frame

        R_GGd_G = pin.exp3(rvec_G)  # Rotation matrix from gripper to desired gripper pose

        # # ? Option 2 - Assume rotation vector is in gripper frame *
        # rvec_G = rot_vec
        # R_GGd_G = pin.exp3(rvec_G)  # Rotation matrix from gripper to desired gripper pose

        # * Translation
        p_GGd_G = R_GW @ p_GGd_W  # Translation vector in gripper frame

        # * Compute transform *
        X_GGd = pin.SE3(R_GGd_G, p_GGd_G)  # Desired gripper pose in world frame

        X_BGd_B = self.X_BG * X_GGd
        self.X_BG_des = X_BGd_B  # Desired gripper pose in base frame

        # #! ACTION WRT GOAL POSE
        # p_GdGd2_W = self.processed_action[:3]  # Gripper desired to new gripper desired
        # rot_vec = self.processed_action[3:6]

        # # * Rotations
        # # ? Option 1 - Assume rotation vector is in world frame *
        # rvec_W = rot_vec

        # R_WGd = self.X_WB.rotation @ self.X_BG_des.rotation
        # R_GdW = R_WGd.T  # Desired gripper pose in world frame
        # rvec_G = R_GdW @ rvec_W  # Rotation vector in gripper frame

        # R_GdGd2_G = pin.exp3(rvec_G)  # Rotation matrix from gripper des to new desired gripper pose

        # # # ? Option 2 - Assume rotation vector is in gripper frame *
        # # rvec_G = rot_vec
        # # R_GGd_G = pin.exp3(rvec_G)  # Rotation matrix from gripper to desired gripper pose

        # # * Translation
        # p_GdGd2_G = R_GdW @ p_GdGd2_W  # Translation vector in gripper frame

        # # * Compute transform *
        # X_GdGd2 = pin.SE3(R_GdGd2_G, p_GdGd2_G)  # Desired gripper pose in world frame

        # X_BGd2_B = self.X_BG_des * X_GdGd2
        # self.X_BG_des = X_BGd2_B  # Desired gripper pose in base frame

        #! Gripper state
        self._process_gripper_actions(action)

        #! Send command
        self.send_robot_command()

    def send_robot_command(self):
        """Send the command to the robot."""
        msg = PoseStamped()

        gripper_pose = se32rospose(self.X_BG_des)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = gripper_pose

        self._robot_cmd_pub.publish(msg)

    def _process_gripper_actions(self, action: np.ndarray):
        desired_gripper_state = None

        if action[6] > 0.8:
            desired_gripper_state = "close"
        elif action[6] < -0.8:
            desired_gripper_state = "open"
        # else:
        #     desired_gripper_state = None
        self.gripper_state = desired_gripper_state

        # Non-blocking gripper action logic
        if (
            self.gripper_state is not None
            and self.gripper_state != self._last_gripper_command
            and not self._gripper_goal_in_progress
        ):
            # if self.gripper_state == "open":
            #     print("Sending gripper open command")
            #     self._send_gripper_goal(self._gripper_open_client, GripperOpen)

            #     self._last_gripper_command = self.gripper_state

            if self.gripper_state == "close":
                print("Sending gripper close command")
                self._send_gripper_goal(self._gripper_close_client, GripperClose)

                self._last_gripper_command = self.gripper_state

    def _send_gripper_goal(self, client: ActionClient, action_type):
        """Send a gripper goal asynchronously and set in-progress flag."""
        self._gripper_goal_in_progress = True
        goal_msg = action_type.Goal()
        send_goal_future = client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._gripper_goal_response_callback)

    def _gripper_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Gripper action goal rejected.")
                self._gripper_goal_in_progress = False
                return
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self._gripper_result_callback)
        except Exception as e:
            self.get_logger().warn(f"Exception in gripper goal response: {e}")
            self._gripper_goal_in_progress = False

    def _gripper_result_callback(self, future):
        try:
            result = future.result().result
            # Optionally log result
        except Exception as e:
            self.get_logger().warn(f"Exception in gripper result callback: {e}")
        self._gripper_goal_in_progress = False

    #! Utility methods
    def _set_control_gains(self, Kp, Kd):
        target_node_name = "/fr3_interface"
        kp_param_name = "Kp_gripper_trans"
        kd_param_name = "Kd_gripper_trans"

        client = self.create_client(SetParameters, f"{target_node_name}/set_parameters")

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {target_node_name}/set_parameters not available.")
            return False

        param_msg = Parameter(kp_param_name, Parameter.Type.DOUBLE, float(Kp)).to_parameter_msg()
        req = SetParameters.Request()
        req.parameters = [param_msg]

        Kp_future = client.call_async(req)

        #! Kd
        param_msg = Parameter(kd_param_name, Parameter.Type.DOUBLE, float(Kd)).to_parameter_msg()
        req = SetParameters.Request()
        req.parameters = [param_msg]

        Kd_future = client.call_async(req)

        return Kp_future, Kd_future


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
