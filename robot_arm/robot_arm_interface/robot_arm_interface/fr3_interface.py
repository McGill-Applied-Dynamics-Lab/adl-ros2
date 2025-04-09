# Suppress the specific UserWarning about NumPy version
import warnings

import rclpy.task

warnings.filterwarnings("ignore", category=UserWarning, message="A NumPy version >=")

from typing import Any, List, Literal, Tuple
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# import asyncio
# import threading

# from rclpy.executors import Future
# from rclpy.task import TaskFactory

# Interfaces
import rclpy.time
from sensor_msgs.msg import JointState

# from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Twist, TwistStamped, AccelStamped

# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_msgs.msg import FrankaRobotState
from franka_msgs.action import Grasp, Homing, Move

# from control_msgs.action import FollowJointTrajectory
from arm_interfaces.action import (
    GotoJoints,
    GotoPose,
    GotoJointVelocities,
    GotoEEVelocity,
    GripperHoming,
    GripperToggle,
    GripperClose,
    GripperOpen,
)
from arm_interfaces.msg import Teleop
from arm_interfaces.srv import SetControlMode, GetControlMode, SetGoalSource, GetGoalSource

# from tf2_ros import TransformListener, Buffer

# from builtin_interfaces.msg import Duration
# from controller_manager_msgs.srv import (
#     ConfigureController,
#     ListControllers,
#     SwitchController,
# )
# from isaac_ros2_messages.srv import GetPrimAttribute, SetPrimAttribute

# from controller_manager_msgs.msg import ControllerState

# import roboticstoolbox as rtb
# from robotic_arm_controller.RobotArm import RobotArm

# from spatialmath.base.quaternions import q2r
# from spatialmath.pose3d import SO3, SE3
import pinocchio as pin

# from robot_arm_interface.utils import PoseStamped2SE3, SE32PoseStamped
from robot_arm_interface.utils import motion2rostwist, rostwist2motion, rospose2se3, se32rospose

# Python packages
import numpy as np
import time

from enum import Enum, auto

# from robot_arm_interface.FeedbackControllerManager import FeedbackControllerManager
# import debugpy  # Import the debugpy module


class GripperState(Enum):
    OPEN = auto()
    CLOSED = auto()


class GripperActionType(Enum):
    HOMING = auto()
    MOVE = auto()
    GRASP = auto()


# The command source type
class ControlMode(Enum):
    PAUSE = SetControlMode.Request.CONTROL_MODE_PAUSE  # Pause the robot
    CART_POSE = SetControlMode.Request.CONTROL_MODE_CART_POSE  # Cartesian pose control
    CART_VEL = SetControlMode.Request.CONTROL_MODE_CART_VEL  # Cartesian velocity control
    JOINT_POSE = SetControlMode.Request.CONTROL_MODE_JOINT_POSE  # Joint pose control
    JOINT_VEL = SetControlMode.Request.CONTROL_MODE_JOINT_VEL  # Joint velocity control


# Where the goal attribute is set from
class GoalSource(Enum):
    TELEOP = auto()
    TOPIC = auto()  # Get the goal from a topic. Topic name depends on `ControlMode`
    ACTION = auto()  # Get the goal from an action server


# MARK: FR3 Interface
class FR3Interface(Node):
    def __init__(self, hw_type: Literal["isaac", "fake", "real"]):
        super().__init__("fr3_interface")

        # Initialize robot arm
        # self._robot_arm = RobotArm("fr3")
        self._robot_arm_model = ...  # TODO: Pinocchio model
        self.hw_type = hw_type  # real, fake, isaac

        #! Attributes
        self._is_state_initialized = False  # Flag to indicate if the robot state has been initialized
        self._home_position = np.deg2rad([0, -45, 0, -135, 0, 90, 45])  # todo: to config

        self._control_mode = None
        self._goal_source = None
        self.control_mode = ControlMode.CART_VEL
        self.goal_source = GoalSource.TELEOP

        # self._controller_switch_in_progress = False
        # self._current_controller = "cartesian_vel_controller"

        self._gripper_in_action = False
        self._gripper_state: GripperState = GripperState.OPEN

        #! Frames
        self.X_WG = pin.SE3()  # World to gripper transform
        self.X_WG_goal = pin.SE3()  # World to gripper transform, goal
        self.X_WG_start = pin.SE3()  # Gripper transform, start

        self.V_WG = pin.Motion().Zero()  # Gripper velocity (world frame)
        self.V_WG_goal = pin.Motion().Zero()  # Gripper goal velocity (gripper frame)

        # self._start_pose = PoseStamped()
        # self._goal_reached = False

        # Goto goal
        # self._goal_pose = PoseStamped()

        #! Action parameters
        self._start_time = self.get_clock().now()  # Start time of the node
        self._goto_goal_epsilon = 0.001  # Error tolerance for goal reaching [m] todo: move to action interface
        self._goto_goal_gain = 1

        # # joint velocities
        # self._joint_vels_desired: None | np.ndarray = None

        # # ee velocity
        # self._ee_vel_desired: None | np.ndarray = None

        # teleop
        self._teleop_msg = Teleop()

        # Gains
        self._kP = 2 * np.ones(3)
        self._kD = 0.1 * np.ones(3)

        # * Async parameters
        self._as_cb_group = MutuallyExclusiveCallbackGroup()
        self._ac_cb_group = MutuallyExclusiveCallbackGroup()
        self._sub_cb_group = MutuallyExclusiveCallbackGroup()
        # self._feedback_controller_manager_cb_group = MutuallyExclusiveCallbackGroup()

        # # Create a separate event loop for async operations
        # self._async_loop = asyncio.new_event_loop()
        # self._async_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        # self._async_thread.start()

        # #! Feedback Controller Manager
        # self._feedback_controller_manager = FeedbackControllerManager(
        #     node=self, cb_group=self._feedback_controller_manager_cb_group
        # )
        # self.get_logger().info(f"{self._feedback_controller_manager._controllers}")
        # self._feedback_controller_manager.switch_controller("joint_trajectory_controller")
        # self._feedback_controller_manager.switch_controller("my_vel_controller")

        #! Init ros interfaces
        self._init_publishers()

        self._init_action_servers()

        self._init_action_clients()

        self._init_services()

        self._init_subscribers()

        #! Init Gripper
        ...

        #! Set control mode
        self.control_mode = ControlMode.PAUSE
        self.goal_source = GoalSource.TELEOP

    def _init_publishers(self):
        self.get_logger().info("Initializing publishers...")

        # Pose Publisher
        self._pose_pub = self.create_publisher(PoseStamped, "/fr3_pose", 10)
        self._pose_msg = PoseStamped()
        self._pose_pub_timer = self.create_timer(0.1, self._publish_ee_pose)

        # Command Publisher
        # - self._joint_vels_cmd_pub (my_vel_controller/commands)
        # - self._cartesian_pose_cmd_pub (cartesian_pose_controller/commands)
        # - self._cartesian_vel_cmd_msg (cartesian_vel_controller/commands)

        self._ctrl_cmd_freq = 100  # Hz
        self._ctrl_cmd_pub_timer = self.create_timer(1 / self._ctrl_cmd_freq, self._compute_ctrl_cmd)

        # # --- velocity_controller ---
        # joint_vels_controller: str = "my_vel_controller"
        # joint_vels_controller_topic = f"/{joint_vels_controller}/commands"
        # self._joint_vels_cmd_pub = self.create_publisher(Float64MultiArray, joint_vels_controller_topic, 10)

        # self._joint_vels_cmd_msg = Float64MultiArray()
        # self._joint_vels_cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # # self._joint_vels_cmd_freq = 100  # Hz
        # # self._joint_vels_cmd_pub_timer = self.create_timer(1 / self._joint_vels_cmd_freq, self._pub_joint_vels_cmd)

        # # --- cartesian_pose_controller ---
        # cartesian_pose_controller: str = "cartesian_pose_controller"
        # cartesian_pose_controller_topic = f"/{cartesian_pose_controller}/commands"
        # self._cartesian_pose_pub_active = False
        # self._cartesian_pose_cmd_pub = self.create_publisher(PoseStamped, cartesian_pose_controller_topic, 10)

        # self._cartesian_pose_cmd_msg = PoseStamped()
        # self._cartesian_pose_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        # self._cartesian_pose_cmd_msg.pose.position.x = 0.0
        # self._cartesian_pose_cmd_msg.pose.position.y = 0.0
        # self._cartesian_pose_cmd_msg.pose.position.z = 0.0
        # self._cartesian_pose_cmd_msg.pose.orientation.x = 0.0
        # self._cartesian_pose_cmd_msg.pose.orientation.y = 0.0
        # self._cartesian_pose_cmd_msg.pose.orientation.z = 0.0
        # self._cartesian_pose_cmd_msg.pose.orientation.w = 0.0

        # # self._cartesian_cmd_freq = 100  # Hz
        # # self._cartesian_cmd_pub_timer = self.create_timer(1 / self._cartesian_cmd_freq, self._pub_cartesian_pose_cmd)

        # --- cartesian_vel_controller ---
        controller_name: str = "cartesian_vel_controller"
        topic = f"/{controller_name}/commands"
        self._cartesian_vel_pub_active = True
        self._start_vel = PoseStamped()

        self._cartesian_vel_pub = self.create_publisher(TwistStamped, topic, 10)

        self._cartesian_vel_cmd_msg = TwistStamped()
        self._cartesian_vel_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        self._cartesian_vel_cmd_msg.twist.linear.x = 0.0
        self._cartesian_vel_cmd_msg.twist.linear.y = 0.0
        self._cartesian_vel_cmd_msg.twist.linear.z = 0.0
        self._cartesian_vel_cmd_msg.twist.angular.x = 0.0
        self._cartesian_vel_cmd_msg.twist.angular.y = 0.0
        self._cartesian_vel_cmd_msg.twist.angular.z = 0.0

        # self._cartesian_vel_cmd_freq = 100  # Hz
        # self._cartesian_vel_cmd_pub_timer = self.create_timer(
        #     1 / self._cartesian_vel_cmd_freq, self._pub_cartesian_vel_cmd
        # )

    def _init_subscribers(self):
        self.get_logger().info("Initializing subscribers...")

        # # --- joint states ---
        # joint_states_topic = "/joint_states"
        # self._joint_state_sub = self.create_subscription(JointState, joint_states_topic, self._joint_state_callback, 10)

        # # --- current_pose --- [geometry_msgs/msg/PoseStamped]
        # current_pose_topic = "/franka_robot_state_broadcaster/current_pose"
        # self._current_pose_sub = self.create_subscription(..., current_pose_topic, ...)

        # # --- desired_end_effector_twist --- [geometry_msgs/msg/TwistStamped]
        # desired_end_effector_twist_topic = "/franka_robot_state_broadcaster/desired_end_effector_twist"
        # self._desired_end_effector_twist_sub = self.create_subscription(..., desired_end_effector_twist_topic, ...)

        # # --- desired_joint_states --- [sensor_msgs/msg/JointState]
        # desired_joint_states_topic = "/franka_robot_state_broadcaster/desired_joint_states"
        # self._desired_joint_states_sub = self.create_subscription(..., desired_joint_states_topic, ...)

        # # --- external_joint_torques --- [sensor_msgs/msg/JointState]
        # # external_joint_torques_topic = "/franka_robot_state_broadcaster/external_joint_torques"
        # # self._external_joint_torques__sub = self.create_subscription(..., external_joint_torques_topic, ...)

        # # --- external_wrench_in_base_frame --- [geometry_msgs/msg/WrenchStamped]
        # external_wrench_in_base_frame_topic = "/franka_robot_state_broadcaster/external_wrench_in_base_frame"
        # self._external_wrench_in_base_frame_sub= self.create_subscription(..., desired_end_effector_twist_topic, ...)

        # # --- external_wrench_in_end_effector_frame --- [geometry_msgs/msg/WrenchStamped]
        # # external_wrench_in_stiffness_frame_topic = "/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame"
        # # self._external_wrench_in_stiffness_frame__sub = self.create_subscription(..., external_wrench_in_stiffness_frame_topic, ...)

        # # --- last_desired_pose --- [geometry_msgs/msg/PoseStamped]
        # last_desired_pose_topic = "/franka_robot_state_broadcaster/last_desired_pose"
        # self._last_desired_pose__sub = self.create_subscription(..., last_desired_pose_topic, ...)

        # # --- measured_joint_states --- [sensor_msgs/msg/JointState]
        # measured_joint_states_topic = "/franka_robot_state_broadcaster/measured_joint_states"
        # self._measured_joint_states__sub = self.create_subscription(..., measured_joint_states_topic, ...)

        # --- robot_state --- [franka_msgs/msg/FrankaRobotState]
        robot_state_topic = "/franka_robot_state_broadcaster/robot_state"
        self._robot_state__sub = self.create_subscription(
            FrankaRobotState, robot_state_topic, self._robot_state_callback, 10
        )

        # --- teleop ---
        teleop_topic = "/teleop/ee_cmd"
        self._teleop_sub = self.create_subscription(Teleop, teleop_topic, self._teleop_callback, 10)

        # --- desired gripper vel ---
        desired_vel_topic = "/robot_arm/gripper_vel_command"
        self._desired_gripper_vel_sub = self.create_subscription(
            TwistStamped, desired_vel_topic, self._desired_gripper_vel_callback, 10
        )

        self.get_logger().info("Subscribers initialized")

    def _init_action_servers(self):
        self.get_logger().info("Initializing action servers...")
        # _as_freq = 100  # Asction
        # self._as_loop_rate = self.create_rate(_as_freq, self.get_clock())  # 100 Hz rate

        # # * goto_joints Action Server
        # self._joint_traj_msg = JointTrajectory()
        # self._goto_joint_as = ActionServer(
        #     self,
        #     GotoJoints,
        #     "goto_joints",
        #     execute_callback=self._goto_joints_action,
        #     callback_group=self._as_cb_group,
        # )

        # * goto_pose Action Server
        self._goto_pose_as = ActionServer(self, GotoPose, "goto_pose", self._goto_pose_action)

        # # * joint_trajectory_controller action client (from ros_control)
        # self.get_logger().info("Subscribing to controller's action server...")
        # self._follow_joint_trajectory_ac = ActionClient(
        #     self,
        #     FollowJointTrajectory,
        #     "/joint_trajectory_controller/follow_joint_trajectory",
        #     callback_group=self._ac_cb_group,
        # )
        # self._follow_joint_trajectory_ac.wait_for_server()
        # self.get_logger().info("Controller's action server is up!")

        # * goto_joint_vels Action Server
        self._goto_joint_vels_as = ActionServer(
            self, GotoJointVelocities, "goto_joint_vels", self._goto_joint_vels_action
        )

        # * goto_ee_vels Action Server
        self._goto_ee_vels_as = ActionServer(
            self,
            GotoEEVelocity,
            "goto_ee_vels",
            execute_callback=self._goto_ee_vel_action,
            callback_group=self._as_cb_group,
        )

        # * Gripper homing
        self._as_gripper_homing = ActionServer(
            self,
            GripperHoming,
            "gripper_homing",
            execute_callback=self._action_gripper_homing,
            callback_group=self._as_cb_group,
        )

        # * Gripper toggle
        self._as_gripper_toggle = ActionServer(
            self,
            GripperToggle,
            "gripper_toggle",
            execute_callback=self._action_gripper_toggle,
            callback_group=self._as_cb_group,
        )

        # * Gripper open
        self._as_gripper_open = ActionServer(
            self,
            GripperOpen,
            "gripper_open",
            execute_callback=self._action_gripper_open,
            callback_group=self._as_cb_group,
        )

        # * Gripper toggle
        self._as_gripper_close = ActionServer(
            self,
            GripperClose,
            "gripper_close",
            execute_callback=self._action_gripper_close,
            callback_group=self._as_cb_group,
        )

    def _init_action_clients(self):
        self.get_logger().info("Initializing action servers...")

        #! Action clients
        self._gripper_actions = {
            GripperActionType.HOMING: {"type": Homing, "server": "/fr3_gripper/homing", "client": None},
            GripperActionType.MOVE: {"type": Move, "server": "/fr3_gripper/move", "client": None},
            GripperActionType.GRASP: {"type": Grasp, "server": "/fr3_gripper/grasp", "client": None},
        }

        for action_type, spec in self._gripper_actions.items():
            spec["client"] = ActionClient(self, spec["type"], spec["server"], callback_group=self._ac_cb_group)

        # action_client_list = []

        # # goto_joints
        # self._gripper_grasp_ac = ActionClient(self, Grasp, "/fr3_gripper/grasp", callback_group=self._ac_cb_group)
        # action_client_list.append(self._gripper_grasp_ac)

        # # goto_pose
        # self._gripper_move_ac = ActionClient(self, Move, "/fr3_gripper/move", callback_group=self._ac_cb_group)
        # action_client_list.append(self._gripper_move_ac)

        # # goto_joint_vels
        # self._gripper_homing_ac = ActionClient(self, Homing, "/fr3_gripper/homing", callback_group=self._ac_cb_group)
        # action_client_list.append(self._gripper_homing_ac)

        #! Wait for action servers
        self.get_logger().info("Waiting for action servers...")
        # for ac in action_client_list:
        #     ac.wait_for_server()

        for spec in self._gripper_actions.values():
            spec["client"].wait_for_server()

        self.get_logger().info("Action servers are up!")

    def _init_services(self):
        self.get_logger().info("Initializing services...")

        # Control mode services
        # TODO: Change /fr3_interface to read node namespace
        self._get_control_mode_srv = self.create_service(
            GetControlMode, "/fr3_interface/get_control_mode", self._get_control_mode_srv_cb
        )

        self._set_control_mode_srv = self.create_service(
            SetControlMode, "/fr3_interface/set_control_mode", self._set_control_mode_srv_cb
        )

        # Goal source services
        self._get_goal_source_srv = self.create_service(
            GetGoalSource, "/fr3_interface/get_goal_source", self._get_goal_source_srv_cb
        )

        self._set_goal_source_srv = self.create_service(
            SetGoalSource, "/fr3_interface/set_goal_source", self._set_goal_source_srv_cb
        )

        self.get_logger().info("Services initialized")

    def destroy(self):
        self._goto_joint_as.destroy()
        self._goto_pose_as.destroy()
        self._goto_joint_vels_as.destroy()

        super().destroy_node()

    #! Callbacks
    # MARK: Subscribers
    def _robot_state_callback(self, robot_state_msg: FrankaRobotState):
        """
        Update the robot state with the robot state message from '/franka_robot_state_broadcaster/robot_state' topic
        """
        self.measured_joint_state: JointState = robot_state_msg.measured_joint_state
        self.desired_joint_state: JointState = robot_state_msg.desired_joint_state

        # The desired joint acceleration
        self.desired_joint_accel: list = robot_state_msg.ddq_d

        # The poses describing the transformations between different frames of the arm. Measured end-effector pose in base frame
        o_t_ee: PoseStamped = robot_state_msg.o_t_ee
        self.X_WG = rospose2se3(o_t_ee.pose)

        # Last desired end-effector pose of motion generation in base frame
        o_t_ee_d: PoseStamped = robot_state_msg.o_t_ee_d

        # Last commanded end-effector pose of motion generation in base frame
        o_t_ee_c: PoseStamped = robot_state_msg.o_t_ee_c

        # Desired end effector twist in base frame
        o_dp_ee_d: TwistStamped = robot_state_msg.o_dp_ee_d
        self.V_WG = rostwist2motion(o_dp_ee_d.twist)

        # Last commanded end effector twist in base frame
        o_dp_ee_c: TwistStamped = robot_state_msg.o_dp_ee_c

        # Last commanded end effector acceleration in base frame
        o_ddp_ee_c: AccelStamped = robot_state_msg.o_ddp_ee_c

        # Other infos...
        # dtau_j # The derivative of the measured torque signal

        # tau_ext_hat_filtered # Filtered external torque. The JointState consists out of effort (tau_ext_hat_filtered)

        # The state of the elbow
        # elbow

        # The active wrenches acting on the stiffness frame expressed relative to stiffness frame
        # k_f_ext_hat_k

        # inertias...

        # errors...

        # #! Print
        # print(f"EE Pose: {self.o_t_ee.pose.position.x:<6.4f}, {self.o_t_ee.pose.position.y:<6.4f}, {self.o_t_ee.pose.position.z:<6.4f}")

        #! Commands
        # commanded_position = self.o_t_ee_c.pose.position
        # _curr_ts = rclpy.time.Time.from_msg(self.o_dp_ee_c.header.stamp)
        # commanded_ee_vel = self.o_dp_ee_c.twist
        # commanded_ee_accel = self.o_ddp_ee_c.accel

        # commanded_ee_vel_x = commanded_ee_vel.linear.x
        # commanded_ee_accel_x = commanded_ee_accel.linear.x

        #! Initialize
        if not self._is_state_initialized:
            self.X_WG_start: pin.SE3 = self.X_WG
            # self._cartesian_pose_cmd_msg.pose = self._start_pose.pose
            print(f"Start position (x, y, z): {self.X_WG_start.translation}")
            print("Start orientation:")
            q_ros = np.array(
                [
                    o_t_ee.pose.orientation.x,
                    o_t_ee.pose.orientation.y,
                    o_t_ee.pose.orientation.z,
                    o_t_ee.pose.orientation.w,
                ]
            )
            print(f"\t-Quaternions (ROS): \t\t{q_ros}")
            print(f"\t-Quaternions: \t{pin.Quaternion(self.X_WG_start.rotation)}")
            print(f"\t-RPY: {pin.rpy.matrixToRpy(self.X_WG_start.rotation)}")
            print(f"\t-R:\n{self.X_WG_start.rotation}")

            self.X_WG_goal = self.X_WG_start
            self.V_WG_goal = pin.Motion.Zero()

            self._is_state_initialized = True

    def _joint_state_callback(self, joint_msg):
        """
        Update the robot state with the joint state message from '\joint_states' topic
        """
        # q = np.array(joint_msg.position)
        # q = q[:7]  # Remove the fingers
        # self._robot_arm.state.q = q
        return

    # Goal topics callbacks
    def _teleop_callback(self, teleop_msg: Teleop):
        """Read the teleop message and update the goal pose (X_WG_goal) or velocity (V_WG_goal)

        Args:
            teleop_msg (PoseStamped): The teleop message
        """
        self._teleop_msg: Teleop = teleop_msg

        if self.goal_source != GoalSource.TELEOP:
            return

        # Set the goal
        if self._teleop_msg.control_mode == Teleop.CONTROL_MODE_POSITION:
            self.control_mode = ControlMode.CART_POSE
            self.X_WG_goal = rospose2se3(self._teleop_msg.ee_des)

            # print(f"x_ee_des: {x_ee_des[0]:<6.4f} x_ee: {x_ee[0]:<6.4f} error: {error[0]:<6.4f}")
            # print(f"x_ee_des: {x_ee_des[0]:<6.4f} x_ee: {x_ee[0]:<6.4f} error: {error[0]:<6.4f} desired_vel: {desired_vel[0]:<6.4f} dx_ee: {dx_ee[0]:<6.4f} ")

        elif self._teleop_msg.control_mode == Teleop.CONTROL_MODE_VEL:
            self.control_mode = ControlMode.CART_VEL
            self.V_WG_goal = rostwist2motion(self._teleop_msg.ee_vel_des)

            # goal_twist: Twist = self._teleop_msg.ee_vel_des
            # lin_vel_des, ang_vel_des = self.twist_msg_to_array(goal_twist)
            # desired_vel = lin_vel_des.astype(float)

        else:
            raise ValueError(f"Invalid control mode received from teleop: {self._teleop_goal_msg.control_mode}")

    def _desired_gripper_vel_callback(self, twist_msg: TwistStamped):
        # Only update goal if the goal source is topic and the control mode is cartesian velocity
        if self.goal_source != GoalSource.TOPIC or self.control_mode != ControlMode.CART_VEL:
            return
        self.V_WG_goal = rostwist2motion(twist_msg.twist)

    # --- MARK: Publishers
    def _publish_ee_pose(self):
        if not self._is_state_initialized:
            return

        # ee_position = self._robot_arm.state.ee_position
        # ee_quaternion = self._robot_arm.state.ee_quaternion

        self._pose_msg.header.stamp = self.get_clock().now().to_msg()
        self._pose_msg.pose = se32rospose(self.X_WG)  # self.o_t_ee.pose

        # self.get_logger().info(f"Publishing ee pose: {ee_position}")
        self._pose_pub.publish(self._pose_msg)

    def _pub_joint_vels_cmd(self):
        """
        Publish joint velocities command at 100 Hz.
        """
        raise NotImplementedError("Method not implemented")
        if self._joint_vels_desired is None:
            return

        if self._joint_vels_desired.shape != (7,):
            self.get_logger().warn("Invalid joint velocities shape!")
            return

        if isinstance(self._joint_vels_desired, np.ndarray):
            self._joint_vels_cmd_msg.data = self._joint_vels_desired.tolist()
        elif isinstance(self._joint_vels_desired, list):
            self._joint_vels_cmd_msg.data = self._joint_vels_desired
        else:
            self.get_logger().warn("Invalid type for joint velocities command!")
            return

        self._joint_vels_cmd_pub.publish(self._joint_vels_cmd_msg)

    # ---- MARK: Actions
    async def _goto_joints_action(self, goal_handle):
        """Go to joints"""
        # self.get_logger().info("Received goal joint position goal")

        # self._current_goal_handle = goal_handle

        # # Switch controller
        # await self._feedback_controller_manager.switch_controller("joint_trajectory_controller")

        # start_q = np.array(self._robot_arm.state.q)  # [rad]
        # goal = np.array(goal_handle.request.joints_goal)  # [rad]
        # duration: rclpy.time.Duration = rclpy.time.Duration.from_msg(goal_handle.request.duration)
        # duration_secs = duration.nanoseconds / 1e9

        # self.get_logger().info(f"Current joint positions: {np.rad2deg(self._robot_arm.state.q)}")
        # self.get_logger().info(f"Goal joint positions: {np.rad2deg(goal)}")
        # self.get_logger().info(f"Duration [s]: {duration_secs}")

        # traj_time_step = 0.001  # time between the trajectory points [s]

        # # Compute traj
        # n_points = int(duration_secs / traj_time_step)
        # t = np.linspace(0, duration_secs, n_points)

        # joint_traj = rtb.jtraj(start_q, goal, t=t)
        # # self.get_logger().info(f"\n\nTrajectory shape: {joint_traj.q.shape}")
        # jx = 2
        # max_vel_jx = np.abs(joint_traj.qd[:, jx - 1]).max()
        # max_accel_jx = np.abs(joint_traj.qdd[:, jx - 1]).max()
        # self.get_logger().info(f"Max vel joint {jx}: {max_vel_jx}")
        # self.get_logger().info(f"Max accel joint {jx}: {max_accel_jx}")

        # start_time = self.get_clock().now()

        # # Build the JointTrajectory message
        # self._joint_traj_msg.header.stamp = self.get_clock().now().to_msg()
        # self._joint_traj_msg.joint_names = [
        #     "fr3_joint1",
        #     "fr3_joint2",
        #     "fr3_joint3",
        #     "fr3_joint4",
        #     "fr3_joint5",
        #     "fr3_joint6",
        #     "fr3_joint7",
        # ]

        # self._joint_traj_msg.points = joint_traj_to_msg(joint_traj, traj_time_step)

        # #! Send the trajectory to the controller
        # # ? Via the action server
        # traj_goal = FollowJointTrajectory.Goal()
        # traj_goal.trajectory = self._joint_traj_msg

        # # Send goal asynchronously
        # traj_goal = FollowJointTrajectory.Goal()
        # traj_goal.trajectory = self._joint_traj_msg

        # send_goal_future = self._follow_joint_trajectory_ac.send_goal_async(traj_goal)
        # goal_handle_future = await send_goal_future

        # if not goal_handle_future.accepted:
        #     self.get_logger().error("Goal rejected by follow_traj action server")
        #     goal_handle.abort()
        #     return GotoJoints.Result(success=False)

        # self.get_logger().info("Goal accepted, waiting for result...")

        # # Wait for the result asynchronously
        # result_future = goal_handle_future.get_result_async()
        # follow_traj_result = await result_future

        # if follow_traj_result.result:
        #     self.get_logger().info("Trajectory execution completed successfully")
        #     # goal_handle.succeed()
        #     # return GotoJoints.Result(success=True)
        # else:
        #     self.get_logger().error("Trajectory execution failed")
        #     goal_handle.abort()
        #     return GotoJoints.Result(success=False)

        # # # Via the publisher
        # # self._commmand_pub.publish(self._joint_traj_msg)

        # #! Send result
        # end_time = self.get_clock().now()
        # duration = (end_time - start_time).nanoseconds / 1e9
        # self.get_logger().info(f"Trajectory execution completed in {duration:.2f} seconds.")

        # goal_handle.succeed()
        # result = GotoJoints.Result()
        # result.success = True
        # self.get_logger().info("Goal succeeded.")

        # return result
        ...

    async def _goto_pose_action(self, goal_handle):
        """To go to a pose using position based servoing

        Args:
            goal_handle (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.get_logger().info("Received cartesian position goal")
        self.goal_source = GoalSource.ACTION
        self.control_mode = ControlMode.CART_POSE

        # goal_pose = goal_handle.request.pose_goal

        # start_cartesian_pose, _ = self.pose_msg_to_array(self.o_t_ee.pose)
        # start_cartesian_pose = self.X_WG.translation
        self._goal_reached = False

        goal_msg: PoseStamped = goal_handle.request.pose_goal
        self.X_WG_goal = rospose2se3(goal_msg.pose)

        duration: rclpy.time.Duration = rclpy.time.Duration.from_msg(goal_handle.request.duration)
        duration_secs = duration.nanoseconds / 1e9

        self.get_logger().info(f"Current cartesian pose: {self.X_WG.translation}")
        self.get_logger().info(f"Goal cartesian pose: {self.X_WG_goal.translation}")
        self.get_logger().info(f"Duration [s]: {duration_secs}")

        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time) < duration:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled by client.")
                self.control_mode = ControlMode.PAUSE

                goal_handle.canceled()

                return GotoPose.Result()

            error = self.X_WG_goal.translation - self.X_WG.translation
            error_norm = np.linalg.norm(error)

            if error_norm < self._goto_goal_epsilon:
                self._goal_reached = True
                self.get_logger().info("Goal reached!")
                break

            else:
                time.sleep(0.01)

        #! Return results
        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e9
        self.get_logger().info(f"Goal reached in {duration:.2f} seconds.")
        self.get_logger().info(f"Final cartesian pose: {self.X_WG.translation}")

        goal_handle.succeed()
        result = GotoPose.Result()
        result.success = self._goal_reached

        # end_pose = self.X_WG
        # result.final_pose = se32rospose(end_pose)  #TODO: Implement
        self.get_logger().info(f"Goal reached: {self._goal_reached}")

        self.control_mode = ControlMode.PAUSE

        return result

    async def _goto_joint_vels_action(self, goal_handle):
        self.get_logger().info("Received goal joint velocities goal")

        # Switch controller
        await self._feedback_controller_manager.switch_controller("my_vel_controller")

        self._joint_vels_desired = np.array(goal_handle.request.joint_velocities)
        duration: rclpy.time.Duration = rclpy.time.Duration.from_msg(goal_handle.request.duration)

        self.get_logger().info(f"Target joint velocities: {self._joint_vels_desired}")
        self.get_logger().info(f"Duration [s]: {duration.nanoseconds / 1e9}")

        #! Setup joint velocities
        start_time = self.get_clock().now()

        # self._joint_vels_cmd_msg.data = joint_vels.tolist()
        self._joint_vels_cmd_pub_timer.reset()

        while (self.get_clock().now() - start_time) < duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled by client.")
                return GotoJointVelocities.Result()

            else:
                time.sleep(0.01)

        #! Return results
        self._joint_vels_desired = np.zeros(7)

        goal_handle.succeed()
        result = GotoJointVelocities.Result()
        result.success = True
        self.get_logger().info("Goal succeeded.")
        return result

    async def _goto_ee_vel_action(self, goal_handle):
        self.get_logger().info("Received ee velocity goal")

        # Switch controller
        await self._feedback_controller_manager.switch_controller("my_vel_controller")

        self._ee_vel_desired = np.array(goal_handle.request.ee_velocity)
        duration: rclpy.time.Duration = rclpy.time.Duration.from_msg(goal_handle.request.duration)

        self.get_logger().info(f"Target ee velocities: {self._ee_vel_desired}")
        self.get_logger().info(f"Duration [s]: {duration.nanoseconds / 1e9}")

        #! Setup joint velocities
        start_time = self.get_clock().now()

        self._joint_vels_desired = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._joint_vels_cmd_pub_timer.reset()

        dik_timer = self.create_timer(0.1, self._compute_dik, callback_group=ReentrantCallbackGroup())

        while (self.get_clock().now() - start_time) < duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled by client.")
                return GotoEEVelocity.Result()

            else:
                time.sleep(0.01)

        #! Cleanup
        dik_timer.cancel()
        time.sleep(0.01)
        self._ee_vel_desired = np.zeros(6)
        self._joint_vels_desired = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        #! Return results
        goal_handle.succeed()
        result = GotoEEVelocity.Result()
        result.success = True
        self.get_logger().info("Goal succeeded.")
        return result

    def _action_gripper_homing(self, goal_handle):
        self.get_logger().info("(Action: Gripper homing) Received goal")

        start_time = self.get_clock().now()
        max_duration = Duration(seconds=10)

        self.gripper_homing()

        while self._gripper_in_action:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("(Action: Gripper homing) Goal canceled by client.")
                return GripperHoming.Result()

            if (self.get_clock().now() - start_time) > max_duration:
                self.get_logger().info("(Action: Gripper homing) Gripper homing timed out.")
                goal_handle.abort()
                return GripperHoming.Result()

            else:
                time.sleep(0.01)

        #! Return results
        goal_handle.succeed()
        result = GripperHoming.Result()
        result.success = True
        self.get_logger().info("(Action: Gripper homing) Goal succeeded.")
        return result

    def _action_gripper_toggle(self, goal_handle):
        self.get_logger().info("(Action: Gripper toggle) Received goal")

        start_time = self.get_clock().now()
        max_duration = Duration(seconds=10)

        if self._gripper_state == GripperState.OPEN:
            self.gripper_close()
        else:
            self.gripper_open()

        while self._gripper_in_action:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("(Action: Gripper toggle) Goal canceled by client.")
                return GripperToggle.Result()

            if (self.get_clock().now() - start_time) > max_duration:
                self.get_logger().info("(Action: Gripper toggle) Gripper toggle timed out.")
                goal_handle.abort()
                return GripperToggle.Result()

            else:
                time.sleep(0.01)

        #! Return results
        goal_handle.succeed()
        result = GripperToggle.Result()
        result.success = True
        self.get_logger().info("(Action: Gripper toggle) Goal succeeded.")
        return result

    def _action_gripper_open(self, goal_handle):
        self.get_logger().info("(Action: Gripper open) Received goal")

        start_time = self.get_clock().now()
        max_duration = Duration(seconds=10)

        self.gripper_open()

        while self._gripper_in_action:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("(Action: Gripper open) Goal canceled by client.")
                return GripperOpen.Result()

            if (self.get_clock().now() - start_time) > max_duration:
                self.get_logger().info("(Action: Gripper open) Gripper open timed out.")
                goal_handle.abort()
                return GripperOpen.Result()

            else:
                time.sleep(0.01)

        #! Return results
        goal_handle.succeed()
        result = GripperOpen.Result()
        result.success = True
        self.get_logger().info("(Action: Gripper open) Goal succeeded.")
        return result

    def _action_gripper_close(self, goal_handle):
        self.get_logger().info("(Action: Gripper close) Received goal")

        start_time = self.get_clock().now()
        max_duration = Duration(seconds=10)

        self.gripper_close()

        while self._gripper_in_action:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("(Action: Gripper close) Goal canceled by client.")
                return GripperClose.Result()

            if (self.get_clock().now() - start_time) > max_duration:
                self.get_logger().info("(Action: Gripper close) Gripper open timed out.")
                goal_handle.abort()
                return GripperClose.Result()

            else:
                time.sleep(0.01)

        #! Return results
        goal_handle.succeed()
        result = GripperClose.Result()
        result.success = True
        self.get_logger().info("(Action: Gripper close) Goal succeeded.")
        return result

    # --- MARK: Services, clients
    def _get_control_mode_srv_cb(self, request, response: GetControlMode.Response):
        self.get_logger().info("Service request to get control mode")

        response.control_mode = self._control_mode.value
        return response

    def _set_control_mode_srv_cb(self, request: SetControlMode.Request, response: SetControlMode.Response):
        self.get_logger().info(f"Service request to set control mode to: {ControlMode(request.control_mode)}")
        try:
            self.control_mode = ControlMode(request.control_mode)
            self.get_logger().info(f"Control mode set to: {self.control_mode}")
            response.success = True
            response.message = f"Control mode set to {self.control_mode}"

        except (ValueError, NotImplementedError) as e:
            response.success = False
            response.message = str(e)

        return response

    def _get_goal_source_srv_cb(self, request: GetGoalSource.Request, response: GetGoalSource.Response):
        self.get_logger().info("Service request to get goal source")

        response.goal_source = self._goal_source.value
        return response

    def _set_goal_source_srv_cb(self, request: SetGoalSource.Request, response: SetGoalSource.Response):
        self.get_logger().info(f"Service request to set goal source to: {GoalSource(request.goal_source)}")

        try:
            self.goal_source = GoalSource(request.goal_source)
            self.get_logger().info(f"Goal source set to: {self.goal_source}")

            response.success = True
            response.message = f"Goal source set to {self.goal_source}"

        except (ValueError, NotImplementedError) as e:
            self.get_logger().error(f"Error setting goal source: {e}")
            response.success = False
            response.message = str(e)

        return response

    # ---  Functions
    #! MARK: Arm
    def _compute_ctrl_cmd(self):
        """Compute the control command based on the current control mode and publish it"""
        if not self._is_state_initialized:
            return

        if self.control_mode == ControlMode.PAUSE:
            self.V_WG_goal = pin.Motion.Zero()
            self._compute_cart_vel_cmd()

        elif self.control_mode == ControlMode.CART_VEL:
            self._compute_cart_vel_cmd()

        elif self.control_mode == ControlMode.CART_POSE:
            self._compute_cart_pose_cmd()

        else:
            raise ValueError(f"Invalid control mode: {self.control_mode}")

    def _compute_cart_vel_cmd(self):
        """Publishes self.V_WG_goal to `/cartesian_vel_controller/commands` topic"""
        # self.get_logger().info("Compute cartesian velocity command...")

        # TODO: Add limiters
        ...

        # Convert Motion to Twist
        desired_gripper_twist: Twist = motion2rostwist(self.V_WG_goal)

        command_msg = TwistStamped()
        command_msg.header.stamp = self.get_clock().now().to_msg()
        command_msg.twist = desired_gripper_twist

        self._cartesian_vel_pub.publish(command_msg)

    def _compute_cart_pose_cmd(self):
        """
        Compute gripper velocity to reach the target goal pose using position based servoing
        """
        # self.get_logger().info("Compute cartesian pose command...")

        # goal_pose: Pose = self._teleop_goal_msg.ee_des
        goal_t = self.X_WG_goal.translation
        current_t = self.X_WG.translation

        # Linear
        K = 0.5
        gains = np.array([K, K, K])

        error = goal_t - current_t

        desired_lin_vel = np.diag(gains) @ error

        # Angular
        # print(
        #     f"RPY: {pin.rpy.matrixToRpy(self.X_WG.rotation)} \t\tRPY_g: {pin.rpy.matrixToRpy(self.X_WG_goal.rotation)}"
        # )

        K_ome = 0.5
        R_error = self.X_WG_goal.rotation.dot(self.X_WG.rotation.T)
        S = 1 / 2 * (R_error - R_error.T)
        omega = np.array([S[2, 1], S[0, 2], S[1, 0]])
        omega_des = K_ome * omega

        desired_motion = pin.Motion(desired_lin_vel, omega_des)

        # # PD Control
        # x_ee_des, _ = self.pose_msg_to_array(goal_pose)
        # x_ee, _ = self.pose_msg_to_array(self.o_t_ee.pose)
        # dx_ee, _ = self.twist_msg_to_array(self.o_dp_ee_d.twist)

        # error = x_ee_des - x_ee

        # desired_vel = self._kP * error - self._kD * dx_ee

        #! Publish
        command_msg = TwistStamped()
        command_msg.header.stamp = self.get_clock().now().to_msg()
        command_msg.twist = motion2rostwist(desired_motion)

        self._cartesian_vel_pub.publish(command_msg)

    #! --- MARK: Gripper
    def gripper_open(self):
        self.get_logger().info("Opening gripper...")
        self._gripper_state = GripperState.OPEN
        self.gripper_move(0.08, 0.1)

    def gripper_close(self):
        self.get_logger().info("Closing gripper...")
        self._gripper_state = GripperState.CLOSED

        self.gripper_grasp(0.0, 0.1, 50)

    def gripper_homing(self):
        self.get_logger().info("Homming gripper...")
        self._gripper_in_action = True
        self._gripper_state: GripperState = GripperState.OPEN

        goal_msg = Homing.Goal()
        self._send_gripper_action_goal(GripperActionType.HOMING, goal_msg)

    def gripper_move(self, width: float, speed: float):
        """Moves to a target width at a defined speed

        Args:
            width (float): Target width
            speed (float): Move speed

        Returns:
            bool: Success
        """
        self.get_logger().info("Moving gripper")
        self._gripper_in_action = True

        goal_msg = Move.Goal()

        if width > 0.08:
            self.get_logger().warn("Width is greater than max widht (0.08m)")

        goal_msg.width = width
        goal_msg.speed = speed

        self._send_gripper_action_goal(GripperActionType.MOVE, goal_msg)

    def gripper_grasp(self, width: float, speed: float, force: float):
        """Tries to grasp at the desired width with the desired force while closing with the given speed. The operation is successful if the distance `d` between the gripper fingers is:
                `width - epsilon.inner < d < width + epsilon.outer

        Args:
            width (float): Target width (m)
            speed (float): Target speed (m/s)
            force (float): Target force (N)

        Returns:
            bool: Success
        """
        self.get_logger().info(f"Grasping (width, speed, force): {width}, {speed}, {force}")
        self._gripper_in_action = True

        goal_msg = Grasp.Goal()

        if width > 0.08:
            self.get_logger().warn("Width is greater than max width (0.08m)")

        goal_msg.width = float(width)
        goal_msg.speed = float(speed)
        goal_msg.force = float(force)

        grasp_epsilon = 0.1
        goal_msg.epsilon.inner = grasp_epsilon
        goal_msg.epsilon.outer = grasp_epsilon

        self._send_gripper_action_goal(GripperActionType.GRASP, goal_msg)

    def _send_gripper_action_goal(self, gripper_action_type: GripperActionType, goal_msg: Any):
        """Generic method to send any action goal with proper callbacks"""

        client: ActionClient = self._gripper_actions[gripper_action_type]["client"]

        # Send goal asynchronously
        self.get_logger().info(f"Sending {gripper_action_type.name} goal")
        send_goal_future = client.send_goal_async(goal_msg)

        # Add callback with action type context
        send_goal_future.add_done_callback(
            lambda future: self._gripper_goal_response_callback(future, gripper_action_type)
        )

    def _gripper_goal_response_callback(self, future, gripper_action_type: GripperActionType):
        """Unified callback for all action goal responses"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"{gripper_action_type.name} goal rejected")
            self._gripper_in_action = False
            return

        self.get_logger().info(f"{gripper_action_type.name} goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self._gripper_get_result_callback(future, gripper_action_type))

    def _gripper_get_result_callback(self, future, gripper_action_type: GripperActionType):
        """Unified callback for all action results"""
        result = future.result().result
        self.get_logger().info(f"{gripper_action_type.name} action completed")

        # Process specific result data if needed based on action_type
        if gripper_action_type == GripperActionType.HOMING:
            self.get_logger().info("Homing completed successfully")
        elif gripper_action_type == GripperActionType.MOVE:
            self.get_logger().info(f"Move completed with success: {getattr(result, 'success', True)}")
        elif gripper_action_type == GripperActionType.GRASP:
            self.get_logger().info(f"Grasp completed with success: {getattr(result, 'success', True)}")

        # Mark action as complete
        self._gripper_in_action = False

    # --- MARK: Utilities
    def _compute_dik(self):
        """
        Compute differential inverse kinematics; the joint velocities to achieve the desired ee velocity.
        """
        if self._ee_vel_desired is None:
            self.get_logger().warn("Desired ee velocity is not set!")
            return

        self._joint_vels_desired = self._robot_arm.diff_ikine(self._ee_vel_desired)

    def stop_robot(self):
        self._cartesian_vel_pub_active = False
        self._cartesian_vel_cmd_msg.twist.linear.x = 0.0
        self._cartesian_vel_cmd_msg.twist.linear.y = 0.0
        self._cartesian_vel_cmd_msg.twist.linear.z = 0.0
        self._cartesian_vel_pub.publish(self._cartesian_vel_cmd_msg)

    @staticmethod
    def pose_msg_to_array(pose: Pose) -> Tuple[np.ndarray, np.ndarray]:
        """
        TODO: Move to utils
        Convert a Pose message to a numpy array.

        Returns:
            - position: np.ndarray
            - orientation: np.ndarray
        """
        position = pose.position
        orientation = pose.orientation

        position_array = np.array([position.x, position.y, position.z])
        orientation_array = np.array([orientation.x, orientation.y, orientation.z, orientation.w])

        return position_array, orientation_array

    @staticmethod
    def twist_msg_to_array(twist: Twist) -> Tuple[np.ndarray, np.ndarray]:
        """
        TODO: Move to utils
        Convert a Twist message to a numpy array.

        Returns:
            - linear: np.ndarray
            - angular: np.ndarray
        """
        linear = twist.linear
        angular = twist.angular

        linear_array = np.array([linear.x, linear.y, linear.z])
        angular_array = np.array([angular.x, angular.y, angular.z])

        return linear_array, angular_array

    def destroy_node(self):
        """Clean up resources when the node is destroyed"""
        # # Stop the async loop
        # if self._async_loop.is_running():
        #     self._async_loop.call_soon_threadsafe(self._async_loop.stop)

        # # Wait for the thread to finish
        # if self._async_thread.is_alive():
        #     self._async_thread.join(timeout=1.0)

        super().destroy_node()

    # --- MARK: Setter/getter
    @property
    def control_mode(self) -> ControlMode:
        return self._control_mode

    @control_mode.setter
    def control_mode(self, value: ControlMode):
        if self._control_mode == value:
            return

        if value not in [ControlMode.CART_VEL, ControlMode.CART_POSE, ControlMode.PAUSE]:
            raise NotImplementedError("Only CART_VEL control mode is supported")

        self.get_logger().info(f"Setting control mode to: {value}")
        self._control_mode = ControlMode(value)

        # if value == ControlMode.PAUSE:
        #     self.V_WG_goal = pin.Motion()
        #     # self._compute_cart_vel_cmd()

    @property
    def goal_source(self) -> GoalSource:
        return self._goal_source

    @goal_source.setter
    def goal_source(self, value: GoalSource):
        """Possible values

        - TELEOP: Reads goal_source from teleop topic
        - ACTION: goal_source set by the action callback
        - TOPIC: Reads goal_source from a topic (not implemented yet)

        Args:
            value (GoalSource): _description_

        Raises:
            NotImplementedError: _description_
        """
        if self._goal_source == value:
            return

        if value == GoalSource.TELEOP:
            self.control_mode = ControlMode.CART_VEL

        if value == GoalSource.TOPIC:
            # self.go
            ...

        self.get_logger().info(f"Setting goal source to: {value}")
        self._goal_source = GoalSource(value)

    #! Old Async functions
    # def _compute_goto_cmd(self):
    #     """Probably not needed"""
    #     goal_t, _ = self.pose_msg_to_array(self._goal_pose.pose)
    #     # current_t, _ = self.pose_msg_to_array(self.o_t_ee.pose)
    #     current_t = self.X_WG.translation
    #     # self.get_logger().info(f"Goal pose: {goal_t}")

    #     K = 1
    #     gains = np.array([K, K, K])

    #     error = goal_t - current_t

    #     desired_vel = np.diag(gains) @ error

    #     if np.linalg.norm(error) < self._goto_goal_epsilon:
    #         self._goal_reached = True

    #     return desired_vel

    # def _pub_cartesian_vel_cmd(self):
    #     """Publishes self._V_G to `/cartesian_vel_controller/commands` topic

    #     Raises:
    #         ValueError: _description_
    #     """

    #     if not self._is_state_initialized or not self._cartesian_vel_pub_active:
    #         return

    #     # elapsed_time_ = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    #     if self.goal_source == GoalSource.TELEOP:
    #         desired_vel = self._compute_teleop_cmd()
    #     elif self.goal_source == GoalSource.GO_TO:
    #         desired_vel = self._compute_goto_cmd()
    #     else:
    #         raise ValueError(f"Invalid control mode: {self.control_mode}")

    #     # Send the desired ee vel
    #     self._cartesian_vel_cmd_msg.twist.linear.x = desired_vel[0]
    #     self._cartesian_vel_cmd_msg.twist.linear.y = desired_vel[1]
    #     self._cartesian_vel_cmd_msg.twist.linear.z = desired_vel[2]

    #     self._cartesian_vel_pub.publish(self._cartesian_vel_cmd_msg)

    # def _pub_cartesian_pose_cmd(self):
    #     if not self._is_state_initialized or not self._cartesian_pose_pub_active:
    #         return

    #     elapsed_time_ = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    #     radius = 0.01
    #     angle = np.pi / 4 * (1 - np.cos(np.pi / 5.0 * elapsed_time_ / 10))
    #     radius = 0.01
    #     angle = np.pi / 4 * (1 - np.cos(np.pi / 5.0 * elapsed_time_ / 10))
    #     # angle = np.pi/20*elapsed_time_

    #     delta_x = radius * np.sin(angle)
    #     delta_z = radius * (np.cos(angle) - 1)
    #     delta_x = 0.0001

    #     cmd_x = self._start_pose.pose.position.x + delta_x

    #     # commanded_position = self.o_t_ee_c.pose.position
    #     # commanded_ee_vel = self.o_dp_ee_c.twist
    #     # commanded_ee_accel = self.o_ddp_ee_c.accel

    #     # commanded_ee_vel_x = commanded_ee_vel.linear.x
    #     # commanded_ee_accel_x = commanded_ee_accel.linear.x

    #     # cmd_x = commanded_position.x + delta_x

    #     self._cartesian_pose_cmd_msg.pose.position.x = cmd_x
    #     # self._cartesian_pose_cmd_msg.pose.position.z = self._start_pose.pose.position.z - delta_z
    #     # print(f'dx: {delta_x}')

    #     self._cartesian_pose_cmd_pub.publish(self._cartesian_pose_cmd_msg)

    # def _run_async_loop(self):
    #     """Run the async loop in a separate thread"""
    #     asyncio.set_event_loop(self._async_loop)
    #     self._async_loop.run_forever()

    # def go_home(self):
    #     self.get_logger().info("Going home")
    #     self._current_controller = "move_to_start_controller"

    #     # Switch controller
    #     if not self._controller_switch_in_progress:
    #         self._controller_switch_in_progress = True

    #         # Submit the async coroutine to our dedicated async loop
    #         asyncio.run_coroutine_threadsafe(self._async_go_home(), self._async_loop)

    # async def _async_go_home(self):
    #     """Async implementation of go_home function"""
    #     try:
    #         # Call the async switch_controller method
    #         await self._feedback_controller_manager.switch_controller("move_to_start_controller")
    #         self.get_logger().info("Controller switched successfully")

    #         # Add an artificial delay to ensure debouncing works even for very fast switches
    #         await asyncio.sleep(0.5)

    #     except Exception as e:
    #         self.get_logger().error(f"Error in go_home: {str(e)}")

    #     finally:
    #         # Reset the flag whether successful or not
    #         self._controller_switch_in_progress = False

    # def switch_to_cartesian_vel_controller(self):
    #     self.get_logger().info("Switching to cartesian vel controller")
    #     self._current_controller = "cartesian_vel_controller"
    #     self._is_state_initialized = False

    #     # Switch controller
    #     if not self._controller_switch_in_progress:
    #         self._controller_switch_in_progress = True

    #         # Submit the async coroutine to our dedicated async loop
    #         asyncio.run_coroutine_threadsafe(self._async_switch_to_cartesian_vel_controller(), self._async_loop)

    # async def _async_switch_to_cartesian_vel_controller(self):
    #     """Async implementation of switch_to_cartesian_vel_controller function"""
    #     try:
    #         # Call the async switch_controller method
    #         await self._feedback_controller_manager.switch_controller("cartesian_vel_controller")
    #         self.get_logger().info("Controller switched successfully")

    #         # Add an artificial delay to ensure debouncing works even for very fast switches
    #         await asyncio.sleep(0.5)

    #     except Exception as e:
    #         self.get_logger().error(f"Error in switch_to_cartesian_vel_controller: {str(e)}")

    #     finally:
    #         # Reset the flag whether successful or not
    #         self._controller_switch_in_progress = False

    # def _handle_transform(self, trans: TransformStamped):
    #     # Process tf2 transform
    #     translation = trans.transform.translation
    #     ee_position_rviz = np.array([translation.x, translation.y, translation.z])
    #     # self.get_logger().info(f"Received transform: {ee_position_rviz}")

    #     orientation = trans.transform.rotation
    #     R_rviz = SO3(
    #         q2r(
    #             [orientation.w, orientation.x, orientation.y, orientation.z],
    #             order="sxyz",
    #         )
    #     )
    #     rpy_rviz = R_rviz.rpy(unit="deg", order="zyx")

    #     # self.get_logger().info(f"Received transform: \n{R_rviz}")

    #     # Position from python library
    #     ee_position_py = self._robot_arm.state.ee_position
    #     # self.get_logger().info(f"Received transform: {ee_position_py}")

    #     ee_quaternion_py = self._robot_arm.state.ee_quaternion
    #     R_py = SO3(q2r(ee_quaternion_py, order="sxyz"))
    #     rpy_py = R_py.rpy(unit="deg", order="zyx")

    #     # self.get_logger().info(f"Received transform: \n{R_py}")
    #     # self.get_logger().info(f"Received transform: {rpy_py}")

    #     # ERRORS
    #     # self.get_logger().info(
    #     #     f"EE Position error: {ee_position_rviz - ee_position_py}"
    #     # )
    #     # self.get_logger().info(f"rviz: {rpy_rviz}\tpy: {rpy_py}")
    #     # self.get_logger().info(f"EE Orientation error: {rpy_rviz - rpy_py}")


"""
Utilities
"""


# def joint_traj_to_msg(
#     joint_traj: rtb.tools.trajectory.Trajectory,
#     traj_time_step: float,
# ) -> List[JointTrajectoryPoint]:
#     """
#     Convert a joint trajectory to a list of JointTrajectoryPoint messages.
#     """
#     joint_traj_points = []

#     for i, (q, qd, qdd) in enumerate(zip(joint_traj.q, joint_traj.qd, joint_traj.qdd)):
#         point = JointTrajectoryPoint()
#         seconds = i * traj_time_step
#         time_from_start = Duration(seconds=seconds)
#         # time_from_start.sec = int(seconds)
#         # time_from_start.nanosec = int((seconds - time_from_start.sec) * 1e9)

#         point.positions = q.tolist()
#         point.velocities = qd.tolist()
#         point.accelerations = qdd.tolist()

#         point.time_from_start = time_from_start.to_msg()
#         joint_traj_points.append(point)

#     return joint_traj_points


def main(args=None):
    # # Start the debugger and wait for the VS Code debugger to attach
    # debugpy.listen(("0.0.0.0", 5678))  # Listen on all interfaces, port 5678
    # debugpy.wait_for_client()  # Pause execution until the debugger attaches
    # print("Debugger attached!")
    hw_type: Literal["isaac", "fake", "real"] = "real"

    rclpy.init(args=args)
    node = FR3Interface(hw_type=hw_type)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("fr3_interface launched, end with CTRL-C")
        executor.spin()

    except KeyboardInterrupt:
        # node.stop_robot()
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    finally:
        node.destroy_node()
        # rclpy.shutdown()

    # try:
    #     rclpy.spin(node, executor=executor)
    # except KeyboardInterrupt:
    #     pass

    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

    # rclpy.shutdown()


if __name__ == "__main__":
    main()
