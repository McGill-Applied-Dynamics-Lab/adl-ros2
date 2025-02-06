from typing import List, Literal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# Interfaces
import rclpy.time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from control_msgs.action import FollowJointTrajectory

from arm_interfaces.action import Empty, GotoJoints, GotoPose, GotoJointVelocities, GotoEEVelocity

# from arm_interfaces.msg import Joints
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    SwitchController,
)
from isaac_ros2_messages.srv import GetPrimAttribute, SetPrimAttribute

from controller_manager_msgs.msg import ControllerState
import roboticstoolbox as rtb

from robotic_arm_controller.RobotArm import RobotArm

from spatialmath.base.quaternions import q2r
from spatialmath.pose3d import SO3, SE3

from robot_arm_interface.utils import PoseStamped2SE3, SE32PoseStamped

# Python packages
import numpy as np
import time

from enum import Enum

import debugpy  # Import the debugpy module


class ControllerStateEnum(Enum):
    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    FINALIZED = "finalized"


class FeedbackController:
    def __init__(self, name: str, state: ControllerStateEnum, type: str):
        self.name = name
        self.state = state
        self.type = type

    @classmethod
    def from_controller_state_msg(cls, controller_state: ControllerState):
        ctrl_name = controller_state.name
        ctrl_state = ControllerStateEnum(controller_state.state)
        type = controller_state.type

        return cls(ctrl_name, ctrl_state, type)


class FeedbackControllerManager:
    def __init__(self, node: Node, cb_group: MutuallyExclusiveCallbackGroup):
        self._node = node
        self._node.get_logger().info("Initializing Feedback Controller Manager...")

        self._controllers: dict | None = None
        self._activated_controller = None
        self._cb_group = cb_group

        #! Services
        self._list_controllers_srv = None
        self._switch_controller_srv = None
        self._init_services()

        # await self._get_cm_controllers()

    def _init_services(self):
        self._service_list = []

        # Service - List controllers
        self._list_controllers_srv = self._node.create_client(
            ListControllers, "/controller_manager/list_controllers", callback_group=self._cb_group
        )
        self._service_list.append(self._list_controllers_srv)

        # Service - Switch controller
        self._switch_controller_srv = self._node.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=self._cb_group
        )
        self._service_list.append(self._switch_controller_srv)

        # IsaacSim services if hw_type is isaac
        if self._node.hw_type == "isaac":
            self._get_prim_attribute_srv = self._node.create_client(
                GetPrimAttribute, "/get_prim_attribute", callback_group=self._cb_group
            )
            self._service_list.append(self._get_prim_attribute_srv)

            self._set_prim_attribute_srv = self._node.create_client(
                SetPrimAttribute, "/set_prim_attribute", callback_group=self._cb_group
            )

        # Wait for services
        self._node.get_logger().info("Waiting cm services...")
        for srv in self._service_list:
            while not srv.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().info(f"service {srv.srv_name} not available, waiting again...")

        self._node.get_logger().info("CM services are up!")

    async def _get_cm_controllers(self):
        """
        Get a list of controllers from the controller manager
        """
        self._node.get_logger().info("Getting list of controllers from cm...")

        self._controllers = {}

        req = ListControllers.Request()
        future = self._list_controllers_srv.call_async(req)
        # rclpy.spin_until_future_complete(self._node, future)

        result = await future

        if result is not None:
            controllers = result.controller

            for controller_msg in controllers:
                controller = FeedbackController.from_controller_state_msg(controller_msg)

                self._controllers[controller.name] = controller

        else:
            self._node.get_logger().error("Service call failed!")

    async def switch_controller(self, controller_name: str):
        """
        To switch to a controller
        """
        self._node.get_logger().info(f"Switching to controller: {controller_name}")

        await self._get_cm_controllers()

        if controller_name not in self._controllers:
            self._node.get_logger().error(f"Controller: {controller_name} is not available!")

        if self._controllers[controller_name].state == ControllerStateEnum.ACTIVE:
            self._node.get_logger().info(f"Controller: {controller_name} is already active!")
            return

        #! Switch controller
        if self._node.hw_type == "isaac":
            self._node.get_logger().info("Setting Isaac Sim drive gains...")
            if controller_name == "joint_trajectory_controller":
                await self._set_isaac_drive_gains("position")
            elif controller_name == "joint_velocity_controller":
                await self._set_isaac_drive_gains("velocity")

        req = SwitchController.Request()

        # Set message
        req.activate_controllers = [controller_name]

        if self._activated_controller is not None:
            req.deactivate_controllers = [self._activated_controller]

        req.strictness = 1  # BEST_EFFORT=1, STRICT=2
        # req.activate_asap
        req.timeout = Duration(seconds=1).to_msg()

        # Call service
        future = self._switch_controller_srv.call_async(req)
        result = await future

        if result is not None:
            switch_ok = result.ok

            if switch_ok:
                self._node.get_logger().info(f"Switched to controller: {controller_name}")
                self._activated_controller = controller_name

            else:
                self._node.get_logger().error(f"Failed to switch to controller: {controller_name}")

        else:
            self._node.get_logger().error("Service call failed!")

    async def _set_isaac_drive_gains(self, ctrl_type: Literal["position", "velocity"]):
        self._node.get_logger().info(f"Setting Isaac Sim drive gains for {ctrl_type}")

        base_prim_path = "/World/franka_alt_fingers"
        damping_attr = "drive:angular:physics:damping"
        stiffness_attr = "drive:angular:physics:stiffness"

        GAINS = {
            # '<prim>': ([position_damping, position_stiffness], [velocity_damping, velocity_stiffness])
            "/panda_link0/panda_joint1": ([52, 1050], [500, 0]),
            "/panda_link1/panda_joint2": ([52, 1050], [500, 0]),
            "/panda_link2/panda_joint3": ([52, 1050], [500, 0]),
            "/panda_link3/panda_joint4": ([52, 436], [500, 0]),
            "/panda_link4/panda_joint5": ([52, 436], [500, 0]),
            "/panda_link5/panda_joint6": ([52, 261], [500, 0]),
            "/panda_link6/panda_joint7": ([52, 87], [500, 0]),
        }

        for each_joint, gains in GAINS.items():
            srv_req = SetPrimAttribute.Request()
            srv_req.path = base_prim_path + each_joint

            # Damping
            srv_req.attribute = damping_attr
            val = gains[0][0] if ctrl_type == "position" else gains[1][0]
            srv_req.value = str(val)

            future = self._set_prim_attribute_srv.call_async(srv_req)
            result = await future

            if result is not None:
                if result.success:
                    self._node.get_logger().info(f"Set {ctrl_type} damping for {each_joint} to {val}")

                else:
                    self._node.get_logger().error(f"Failed to set {ctrl_type} damping for {each_joint}")
                    return

            else:
                self._node.get_logger().error("Service call failed!")
                return

            # Stiffness
            srv_req.attribute = stiffness_attr
            val = gains[0][1] if ctrl_type == "position" else gains[1][1]
            srv_req.value = str(val)

            future = self._set_prim_attribute_srv.call_async(srv_req)
            result = await future

            if result is not None:
                if result.success:
                    self._node.get_logger().info(f"Set {ctrl_type} stiffness for {each_joint} to {val}")

                else:
                    self._node.get_logger().error(f"Failed to set {ctrl_type} stiffness for {each_joint}")
                    return

            else:
                self._node.get_logger().error("Service call failed!")
                return


class FR3Interface(Node):
    def __init__(self, hw_type: Literal["isaac", "fake", "real"]):
        super().__init__("fr3_interface")

        # Initialize robot arm
        self._robot_arm = RobotArm("fr3")
        self.hw_type = hw_type

        #! Attributes
        self._is_state_initialialized = False
        self._home_position = np.deg2rad([0, -45, 0, -135, 0, 90, 45])

        self._as_cb_group = MutuallyExclusiveCallbackGroup()
        self._ac_cb_group = MutuallyExclusiveCallbackGroup()
        self._feedback_controller_manager_cb_group = MutuallyExclusiveCallbackGroup()

        # TODO: remove
        self.trajectory = None

        self._joint_vels_desired: None | np.ndarray = None
        self._ee_vel_desired: None | np.ndarray = None

        #! Subscribers
        joint_states_topic = "/joint_states"
        self._joint_state_sub = self.create_subscription(JointState, joint_states_topic, self._joint_state_callback, 10)

        self._init_publishers()

        self._init_actions()

        self._init_services()

        # Feedback Controller Manager
        self._feedback_controller_manager = FeedbackControllerManager(
            node=self, cb_group=self._feedback_controller_manager_cb_group
        )
        # self.get_logger().info(f"{self._feedback_controller_manager._controllers}")

        # self._feedback_controller_manager.switch_controller("joint_trajectory_controller")
        # self._feedback_controller_manager.switch_controller("joint_velocity_controller")

    def _init_publishers(self):
        self.get_logger().info("Initializing publishers...")

        # Pose Publisher
        self._pose_pub = self.create_publisher(PoseStamped, "/fr3_pose", 10)
        self._pose_msg = PoseStamped()
        self._pose_pub_timer = self.create_timer(0.1, self._publish_ee_pose)

        # TF Publisher
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_tf = self.create_timer(0.25, self._tf_callback)  # Publish at 4 Hz

        # Command Publisher
        # --- velocity_controller ---
        joint_vels_controller: str = "joint_velocity_controller"
        joint_vels_controller_topic = f"/{joint_vels_controller}/commands"
        self._joint_vels_cmd_pub = self.create_publisher(Float64MultiArray, joint_vels_controller_topic, 10)

        self._joint_vels_cmd_msg = Float64MultiArray()
        self._joint_vels_cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._joint_vels_cmd_freq = 100  # Hz
        self._joint_vels_cmd_pub_timer = self.create_timer(1 / self._joint_vels_cmd_freq, self._pub_joint_vels_cmd)

    def _init_actions(self):
        self.get_logger().info("Initializing action servers...")
        # _as_freq = 100  # Asction
        # self._as_loop_rate = self.create_rate(_as_freq, self.get_clock())  # 100 Hz rate

        self._joint_traj_msg = JointTrajectory()

        # goto_joints Action Server
        self._goto_joint_as = ActionServer(
            self,
            GotoJoints,
            "goto_joints",
            execute_callback=self._goto_joints_action,
            callback_group=self._as_cb_group,
        )

        # goto_pose Action Server
        self._goto_pose_as = ActionServer(self, GotoPose, "goto_pose", self._goto_pose_action)

        # joint_trajectory_controller action client (from ros_control)
        self.get_logger().info("Subscribing to controller's action server...")
        self._follow_joint_trajectory_ac = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
            callback_group=self._ac_cb_group,
        )
        self._follow_joint_trajectory_ac.wait_for_server()
        self.get_logger().info("Controller's action server is up!")

        # goto_joint_vels Action Server
        self._goto_joint_vels_as = ActionServer(
            self, GotoJointVelocities, "goto_joint_vels", self._goto_joint_vels_action
        )

        # goto_ee_vels Action Server
        self._goto_ee_vels_as = ActionServer(
            self,
            GotoEEVelocity,
            "goto_ee_vels",
            execute_callback=self._goto_ee_vel_action,
            callback_group=self._as_cb_group,
        )

    def _init_services(self):
        self.get_logger().info("Initializing services...")

        ...

        self.get_logger().info("Services initialized")

    def destroy(self):
        self._goto_joint_as.destroy()
        self._goto_pose_as.destroy()
        self._goto_joint_vels_as.destroy()
        super().destroy_node()

    # Callbacks
    def _joint_state_callback(self, joint_msg):
        """
        Update the robot state with the joint state message from '\joint_states' topic
        """
        q = np.array(joint_msg.position)
        q = q[:7]  # Remove the fingers
        self._robot_arm.state.q = q

        self._is_state_initialialized = True

    def _publish_ee_pose(self):
        ee_position = self._robot_arm.state.ee_position
        ee_quaternion = self._robot_arm.state.ee_quaternion

        self._pose_msg.pose.position.x = ee_position[0]
        self._pose_msg.pose.position.y = ee_position[1]
        self._pose_msg.pose.position.z = ee_position[2]

        self._pose_msg.pose.orientation.x = ee_quaternion[0]
        self._pose_msg.pose.orientation.y = ee_quaternion[1]
        self._pose_msg.pose.orientation.z = ee_quaternion[2]
        self._pose_msg.pose.orientation.w = ee_quaternion[3]

        # self.get_logger().info(f"Publishing ee pose: {ee_position}")
        self._pose_pub.publish(self._pose_msg)

    def _tf_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("base", "fr3_hand", rclpy.time.Time())
            self._handle_transform(trans)

        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")

    def _handle_transform(self, trans: TransformStamped):
        # Process tf2 transform
        translation = trans.transform.translation
        ee_position_rviz = np.array([translation.x, translation.y, translation.z])
        # self.get_logger().info(f"Received transform: {ee_position_rviz}")

        orientation = trans.transform.rotation
        R_rviz = SO3(
            q2r(
                [orientation.w, orientation.x, orientation.y, orientation.z],
                order="sxyz",
            )
        )
        rpy_rviz = R_rviz.rpy(unit="deg", order="zyx")

        # self.get_logger().info(f"Received transform: \n{R_rviz}")

        # Position from python library
        ee_position_py = self._robot_arm.state.ee_position
        # self.get_logger().info(f"Received transform: {ee_position_py}")

        ee_quaternion_py = self._robot_arm.state.ee_quaternion
        R_py = SO3(q2r(ee_quaternion_py, order="sxyz"))
        rpy_py = R_py.rpy(unit="deg", order="zyx")

        # self.get_logger().info(f"Received transform: \n{R_py}")
        # self.get_logger().info(f"Received transform: {rpy_py}")

        # ERRORS
        # self.get_logger().info(
        #     f"EE Position error: {ee_position_rviz - ee_position_py}"
        # )
        # self.get_logger().info(f"rviz: {rpy_rviz}\tpy: {rpy_py}")
        # self.get_logger().info(f"EE Orientation error: {rpy_rviz - rpy_py}")

    def _pub_joint_vels_cmd(self):
        """
        Publish joint velocities command at 100 Hz.
        """
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

    # ---- Actions
    async def _goto_joints_action(self, goal_handle):
        self.get_logger().info("Received goal joint position goal")

        self._current_goal_handle = goal_handle

        # Switch controller
        await self._feedback_controller_manager.switch_controller("joint_trajectory_controller")

        start_q = np.array(self._robot_arm.state.q)  # [rad]
        goal = np.array(goal_handle.request.joints_goal)  # [rad]
        duration = goal_handle.request.duration  # [s]

        self.get_logger().info(f"Current joint positions: {np.rad2deg(self._robot_arm.state.q)}")
        self.get_logger().info(f"Goal joint positions: {np.rad2deg(goal)}")
        self.get_logger().info(f"Duration [s]: {duration}")

        traj_time_step = 0.01  # time between the trajectory points [s]

        # Compute traj
        n_points = int(duration / traj_time_step)
        joint_traj = rtb.jtraj(start_q, goal, n_points)
        # self.get_logger().info(f"\n\nTrajectory shape: {joint_traj.q.shape}")

        start_time = self.get_clock().now()

        # Build the JointTrajectory message
        self._joint_traj_msg.header.stamp = self.get_clock().now().to_msg()
        self._joint_traj_msg.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        self._joint_traj_msg.points = joint_traj_to_msg(joint_traj, traj_time_step)

        #! Send the trajectory to the controller
        # ? Via the action server
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory = self._joint_traj_msg

        # Send goal asynchronously
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory = self._joint_traj_msg

        send_goal_future = self._follow_joint_trajectory_ac.send_goal_async(traj_goal)
        goal_handle_future = await send_goal_future

        if not goal_handle_future.accepted:
            self.get_logger().error("Goal rejected by follow_traj action server")
            goal_handle.abort()
            return GotoJoints.Result(success=False)

        self.get_logger().info("Goal accepted, waiting for result...")

        # Wait for the result asynchronously
        result_future = goal_handle_future.get_result_async()
        follow_traj_result = await result_future

        if follow_traj_result.result:
            self.get_logger().info("Trajectory execution completed successfully")
            # goal_handle.succeed()
            # return GotoJoints.Result(success=True)
        else:
            self.get_logger().error("Trajectory execution failed")
            goal_handle.abort()
            return GotoJoints.Result(success=False)

        # # Via the publisher
        # self._commmand_pub.publish(self._joint_traj_msg)

        #! Send result
        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e9
        self.get_logger().info(f"Trajectory execution completed in {duration:.2f} seconds.")

        goal_handle.succeed()
        result = GotoJoints.Result()
        result.success = True
        self.get_logger().info("Goal succeeded.")

        return result

    async def _goto_pose_action(self, goal_handle):
        self.get_logger().info("Received cartesian position goal")

        # Switch controller
        await self._feedback_controller_manager.switch_controller("joint_trajectory_controller")

        start_cartesian_pose: SE3 = self._robot_arm.state.ee_pose
        start_q = np.array(self._robot_arm.state.q)  # [rad]

        goal_msg = goal_handle.request.pose_goal
        goal_pose = PoseStamped2SE3(goal_msg)
        duration = goal_handle.request.duration  # [s]

        self.get_logger().info(f"Current cartesian pose:\n {start_cartesian_pose}")
        self.get_logger().info(f"Goal cartesian pose:\n {goal_pose}")
        self.get_logger().info(f"Duration [s]: {duration}")

        traj_time_step = 0.01  # time between the trajectory points [s]

        #! Ikine to find goal joint positions
        goal_q = self._robot_arm.ikine(goal_pose)
        self.get_logger().info(f"Goal joint positions (from ikine): {np.rad2deg(goal_q)}")

        #! Compute traj
        n_points = int(duration / traj_time_step)
        joint_traj = rtb.jtraj(start_q, goal_q, n_points)
        print(joint_traj.q.shape)
        self.get_logger().info(f"\n\nTrajectory shape: {joint_traj.q.shape}")

        start_time = self.get_clock().now()

        #! Build the JointTrajectory message
        self._joint_traj_msg.header.stamp = self.get_clock().now().to_msg()
        self._joint_traj_msg.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        self._joint_traj_msg.points = joint_traj_to_msg(joint_traj, traj_time_step)

        #! Send the trajectory to the controller
        # Via the action server
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory = self._joint_traj_msg

        # self._follow_joint_trajectory_ac.send_goal(traj_goal)
        send_goal_future = self._follow_joint_trajectory_ac.send_goal_async(traj_goal)
        goal_handle_future = await send_goal_future

        if not goal_handle_future.accepted:
            self.get_logger().error("Goal rejected by follow_traj action server")
            goal_handle.abort()
            return GotoJoints.Result(success=False)

        self.get_logger().info("Goal accepted, waiting for result...")

        # Wait for the result asynchronously
        result_future = goal_handle_future.get_result_async()
        follow_traj_result = await result_future

        if follow_traj_result.result:
            self.get_logger().info("Trajectory execution completed successfully")
            # goal_handle.succeed()
            # return GotoJoints.Result(success=True)
        else:
            self.get_logger().error("Trajectory execution failed")
            goal_handle.abort()
            return GotoJoints.Result(success=False)

        #! Return results
        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e9
        self.get_logger().info(f"Trajectory execution completed in {duration:.2f} seconds.")

        end_pose = self._robot_arm.state.ee_pose

        goal_handle.succeed()
        result = GotoPose.Result()
        result.success = True
        result.final_pose = SE32PoseStamped(end_pose)
        self.get_logger().info("Goal succeeded.")
        return result

    async def _goto_joint_vels_action(self, goal_handle):
        self.get_logger().info("Received goal joint velocities goal")

        # Switch controller
        await self._feedback_controller_manager.switch_controller("joint_velocity_controller")

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
        await self._feedback_controller_manager.switch_controller("joint_velocity_controller")

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

    # --- Services, clients
    ...

    # ---
    def _compute_dik(self):
        """
        Compute differential inverse kinematics; the joint velocities to achieve the desired ee velocity.
        """
        if self._ee_vel_desired is None:
            self.get_logger().warn("Desired ee velocity is not set!")
            return

        self._joint_vels_desired = self._robot_arm.diff_ikine(self._ee_vel_desired)


"""
Utilities
"""


def joint_traj_to_msg(
    joint_traj: rtb.tools.trajectory.Trajectory,
    traj_time_step: float,
) -> List[JointTrajectoryPoint]:
    """
    Convert a joint trajectory to a list of JointTrajectoryPoint messages.
    """
    joint_traj_points = []

    for i, (q, qd, qdd) in enumerate(zip(joint_traj.q, joint_traj.qd, joint_traj.qdd)):
        point = JointTrajectoryPoint()
        seconds = i * traj_time_step
        time_from_start = Duration(seconds=seconds)
        # time_from_start.sec = int(seconds)
        # time_from_start.nanosec = int((seconds - time_from_start.sec) * 1e9)

        point.positions = q.tolist()
        point.velocities = qd.tolist()
        point.accelerations = qdd.tolist()

        point.time_from_start = time_from_start.to_msg()
        joint_traj_points.append(point)

    return joint_traj_points


def main(args=None):
    # # Start the debugger and wait for the VS Code debugger to attach
    # debugpy.listen(("0.0.0.0", 5678))  # Listen on all interfaces, port 5678
    # debugpy.wait_for_client()  # Pause execution until the debugger attaches
    # print("Debugger attached!")
    hw_type: Literal["isaac", "fake", "real"] = "fake"

    rclpy.init(args=args)
    node = FR3Interface(hw_type=hw_type)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("fr3_interface launched, end with CTRL-C")
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    node.destroy_node()
    rclpy.shutdown()

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
