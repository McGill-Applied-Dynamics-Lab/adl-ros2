"""
Test that launch of the fr3_interface works as expected.

Main tests:
 -

"""

import os
import sys
import time
import unittest

import launch
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

import launch_testing.actions
# from turtlesim.msg import Pose

import rclpy
from rclpy.action import ActionClient

# Interfaces
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped
from franka_msgs.msg import FrankaRobotState
from arm_interfaces.srv import SetControlMode, GetControlMode, SetGoalSource, GetGoalSource
from arm_interfaces.action import (
    GotoJoints,
    GotoPose,
    GotoJointVelocities,
    GotoEEVelocity,
    GripperHoming,
    GripperToggle,
    GripperOpen,
    GripperClose,
)

from robot_arm_interface.fr3_interface import GoalSource, ControlMode
from robot_arm_interface.utils import motion2rostwist, rostwist2motion, rospose2se3, se32rospose


import pinocchio as pin
import numpy as np
from rclpy.duration import Duration


def generate_test_description():
    #! Launch arguments
    arm_id_parameter_name = "arm_id"
    robot_ip_parameter_name = "robot_ip"
    load_gripper_parameter_name = "load_gripper"
    hw_type_parameter_name = "hw_type"
    use_rviz_parameter_name = "use_rviz"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)

    robot_ip_launch_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value=EnvironmentVariable("FR3_IP", default_value="dont-care"),
        description="Hostname or IP address of the robot.",
    )

    # hw_type_launch_arg = DeclareLaunchArgument(
    #     hw_type_parameter_name,
    #     default_value="isaac",
    #     description="Which hardware to use: 'real', 'fake', or 'isaac'",
    #     choices=["real", "fake", "isaac", "gazebo"],
    # )

    #! Launch Descriptions
    fr3_interface_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("robot_arm_bringup"),
                        "launch",
                        "fr3_interface.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            arm_id_parameter_name: "fr3",
            robot_ip_parameter_name: robot_ip,
            load_gripper_parameter_name: "true",
            hw_type_parameter_name: "real",
            use_rviz_parameter_name: "false",
        }.items(),
    )

    launch_description = LaunchDescription(
        [
            robot_ip_launch_arg,
            fr3_interface_launch_description,
            # Launch tests 0.5 s later
            launch.actions.TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )

    return (launch_description, {"fr3_interface": fr3_interface_launch_description})


# Active tests
@unittest.skip("Skipping FR3 interface initialization tests")
class TestFR3InterfaceInit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_fr3_interface")

    def tearDown(self):
        self.node.destroy_node()

    #! Publishers
    def test_publishes_fr_pose(self, proc_output):
        """Check whether /fr3_pose messages published"""
        msgs_rx = []
        sub = self.node.create_subscription(PoseStamped, "/fr3_pose", lambda msg: msgs_rx.append(msg), 100)
        try:
            # Listen to the pose topic for 10 s
            listen_time = 5
            end_time = time.time() + listen_time
            msg_freq = 10
            exp_msgs = 10  # listen_time * msg_freq * 0.5  # 0.5 for margin

            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)

            # There should have been 100 messages received
            assert len(msgs_rx) > exp_msgs, f"Number of messages received {len(msgs_rx)} < expected {exp_msgs}"

        finally:
            self.node.destroy_subscription(sub)

    #! Set/get control mode
    def test_set_get_control_mode_srv_cart_vel(self, proc_output):
        """Check whether /fr3_interface/set_control_mode service works"""
        #! Set control mode to Cartesian velocity
        srv = self.node.create_client(SetControlMode, "/fr3_interface/set_control_mode")

        req = SetControlMode.Request()
        req.control_mode = ControlMode.CART_VEL.value
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.success, f"Failed to set control mode: {response.message}"

        #! Get control mode
        srv = self.node.create_client(GetControlMode, "/fr3_interface/get_control_mode")
        req = GetControlMode.Request()
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.control_mode == ControlMode.CART_VEL.value, "Control mode is not CART_VEL"

    def test_set_get_control_mode_srv_pause(self, proc_output):
        """Check whether /fr3_interface/set_control_mode service works"""
        #! Set control mode to Cartesian velocity
        srv = self.node.create_client(SetControlMode, "/fr3_interface/set_control_mode")

        req = SetControlMode.Request()
        req.control_mode = ControlMode.PAUSE.value
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.success, f"Failed to set control mode: {response.message}"

        #! Get control mode
        srv = self.node.create_client(GetControlMode, "/fr3_interface/get_control_mode")
        req = GetControlMode.Request()
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.control_mode == ControlMode.PAUSE.value, "Control mode is not PAUSE"

    @unittest.expectedFailure  # Not implemented yet
    def test_set_control_mode_srv_cart_pose(self, proc_output):
        """Check whether /fr3_interface/set_control_mode service works"""
        #! Set control mode to Cartesian velocity
        srv = self.node.create_client(SetControlMode, "/fr3_interface/set_control_mode")

        req = SetControlMode.Request()
        req.control_mode = ControlMode.CART_POSE.value
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.success, f"Failed to set control mode: {response.message}"

        #! Get control mode
        srv = self.node.create_client(GetControlMode, "/fr3_interface/get_control_mode")
        req = GetControlMode.Request()
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.control_mode == ControlMode.CART_POSE.value, "Control mode is not CART_POSE"

    #! Set/get goal source
    def test_set_get_goal_src_srv_teleop(self, proc_output):
        """Check whether `/fr3_interface/set_goal_source` and `/fr3_interface/get_goal_source` service works"""
        #! Set control mode to Cartesian velocity
        srv = self.node.create_client(SetGoalSource, "/fr3_interface/set_goal_source")

        req = SetGoalSource.Request()
        req.goal_source = GoalSource.TELEOP.value
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.success, f"Failed to set goal source to TELEOP: {response.message}"

        #! Get goal source
        srv = self.node.create_client(GetGoalSource, "/fr3_interface/get_goal_source")
        req = GetGoalSource.Request()
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response: GetGoalSource.Response = future.result()

        # Check if the control mode was set
        assert response.goal_source == GoalSource.TELEOP.value, "Goal source is not TELEOP"

    def test_set_get_goal_src_srv_topic(self, proc_output):
        """Check whether `/fr3_interface/set_goal_source` and `/fr3_interface/get_goal_source` service works"""

        #! Set goal source to TOPIC
        srv = self.node.create_client(SetGoalSource, "/fr3_interface/set_goal_source")

        req = SetGoalSource.Request()
        req.goal_source = GoalSource.TOPIC.value
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.success, f"Failed to set goal source to TOPIC: {response.message}"

        #! Get goal source
        srv = self.node.create_client(GetGoalSource, "/fr3_interface/get_goal_source")
        req = GetGoalSource.Request()
        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response: GetGoalSource.Response = future.result()

        # Check if the control mode was set
        assert response.goal_source == GoalSource.TOPIC.value, "Goal source is not TOPIC"


@unittest.skip("Skipping FR3 interface initialization tests")
class TestFR3InterfaceGripper(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_fr3_interface")

    def tearDown(self):
        self.node.destroy_node()

    #! Publishers
    def test_publishes_gripper_pose(self, proc_output):
        """Check whether /franka_gripper/joint_states messages published"""
        msgs_rx = []
        sub = self.node.create_subscription(
            JointState, "/fr3_gripper/joint_states", lambda msg: msgs_rx.append(msg), 100
        )
        # sub = self.node.create_subscription(PoseStamped, "/fr3_pose", lambda msg: msgs_rx.append(msg), 100)
        try:
            # Listen to the pose topic for 10 s
            listen_time = 5
            end_time = time.time() + listen_time
            msg_freq = 10
            exp_msgs = 10  # listen_time * msg_freq * 0.5  # 0.5 for margin

            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)

            # There should have been 100 messages received
            assert len(msgs_rx) > exp_msgs, f"Number of messages received {len(msgs_rx)} < expected {exp_msgs}"
            assert msgs_rx[0].name == ["fr3_finger_joint1", "fr3_finger_joint2"], (
                f"Gripper joint names are incorrect ({msgs_rx[0].name})"
            )

        finally:
            self.node.destroy_subscription(sub)

    #! Actions
    def test_gripper_homing(self, proc_output):
        """Check whether /fr3_interface/gripper_homing action works"""

        # Create the action client
        action_client = ActionClient(self.node, GripperHoming, "gripper_homing")
        action_client.wait_for_server()

        self.node.get_logger().info("Homing gripper")
        goal_msg = GripperHoming.Goal()
        future = action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, future)

        success = False
        if future.done():
            goal_handle = future.result()

            # Goal refused
            if not goal_handle.accepted:
                self.node.get_logger().error("Goal was rejected by the action server!")
                success = False

            # Goal accepted
            self.node.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)

            # Action completed
            if result_future.done():
                result = result_future.result()
                self.node.get_logger().info(f"Result received: {result.result.success}")
                success = result.result.success

            # Action failed
            else:
                self.node.get_logger().error("Failed to get result!")
                success = False

        # Goal not received
        else:
            self.node.get_logger().error("Failed to send goal!")
            success = False

        # Check if the control mode was set
        assert success, "Failed to home gripper"

    def test_gripper_toggle(self, proc_output):
        """Check whether /fr3_interface/toggle action works"""
        # Sub to gripper state
        self.gripper_state: JointState = None

        sub = self.node.create_subscription(
            JointState, "/fr3_gripper/joint_states", lambda msg: setattr(self, "gripper_state", msg), 100
        )

        # Create the action client
        action_client = ActionClient(self.node, GripperToggle, "gripper_toggle")
        action_client.wait_for_server()
        try:
            while self.gripper_state is None:
                print("Waiting for gripper state...")
                rclpy.spin_once(self.node, timeout_sec=1)

            def toggle_gripper(self):
                start_state = self.gripper_state.position

                self.node.get_logger().info("Toggling gripper")
                goal_msg = GripperToggle.Goal()
                future = action_client.send_goal_async(goal_msg)

                rclpy.spin_until_future_complete(self.node, future)

                success = False
                if future.done():
                    goal_handle = future.result()

                    # Goal refused
                    if not goal_handle.accepted:
                        self.node.get_logger().error("Goal was rejected by the action server!")
                        success = False

                    # Goal accepted
                    self.node.get_logger().info("Goal accepted, waiting for result...")
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, result_future)

                    # Action completed
                    if result_future.done():
                        result = result_future.result()
                        self.node.get_logger().info(f"Result received: {result.result.success}")
                        success = result.result.success

                    # Action failed
                    else:
                        self.node.get_logger().error("Failed to get result!")
                        success = False

                # Goal not received
                else:
                    self.node.get_logger().error("Failed to send goal!")
                    success = False

                # Check if the control mode was set
                end_state = self.gripper_state.position

                return success, start_state, end_state

            success_1, start_state_1, end_state_1 = toggle_gripper(self)
            start_dist_1 = abs(start_state_1[0] + start_state_1[1])
            end_dist_1 = abs(end_state_1[0] + end_state_1[1])

            success_2, start_state_2, end_state_2 = toggle_gripper(self)
            start_dist_2 = abs(start_state_2[0] + start_state_2[1])
            end_dist_2 = abs(end_state_2[0] + end_state_2[1])

            self.node.get_logger().info(f"Start 1: {start_state_1}")
            self.node.get_logger().info(f"End 1: {end_state_1}")
            self.node.get_logger().info(f"End 2: {end_state_2}")

            assert success_1, "Action execution failed"
            assert start_dist_1 != end_dist_1, f"Gripper did not toggle. Start: {start_dist_1}, End: {end_dist_1}"

            assert success_2, "Action execution failed"
            assert start_dist_2 != end_dist_2, f"Gripper did not toggle. Start: {start_dist_2}, End: {end_dist_2}"

            # assert start_dist_1 == end_dist_2, "Gripper did not toggle back to original state"

        finally:
            self.node.destroy_subscription(sub)

    def test_gripper_open_close(self, proc_output):
        """Check whether `gripper_open` and `gripper_close` actions works"""
        # Sub to gripper state
        self.gripper_state: JointState = None

        sub = self.node.create_subscription(
            JointState, "/fr3_gripper/joint_states", lambda msg: setattr(self, "gripper_state", msg), 100
        )

        # Create the action client
        action_client_open = ActionClient(self.node, GripperOpen, "gripper_open")
        action_client_open.wait_for_server()

        action_client_close = ActionClient(self.node, GripperClose, "gripper_close")
        action_client_close.wait_for_server()

        try:
            while self.gripper_state is None:
                print("Waiting for gripper state...")
                rclpy.spin_once(self.node, timeout_sec=1)

            start_state = self.gripper_state.position

            def is_gripper_closed(gripper_state: JointState):
                """Check if the gripper is closed"""
                return gripper_state.position[0] < 0.01 and gripper_state.position[1] < 0.01

            def open_gripper():
                #! Open
                self.node.get_logger().info("Opening gripper")
                goal_msg = GripperOpen.Goal()
                future = action_client_open.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self.node, future)

                success = False
                if future.done():
                    goal_handle = future.result()

                    # Goal refused
                    if not goal_handle.accepted:
                        self.node.get_logger().error("Goal was rejected by the action server!")
                        success = False

                    # Goal accepted
                    self.node.get_logger().info("Goal accepted, waiting for result...")
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, result_future)

                    # Action completed
                    if result_future.done():
                        result = result_future.result()
                        self.node.get_logger().info(f"Result received: {result.result.success}")
                        success = result.result.success

                    # Action failed
                    else:
                        self.node.get_logger().error("Failed to get result!")
                        success = False

                # Goal not received
                else:
                    self.node.get_logger().error("Failed to send goal!")
                    success = False

                return success

            def close_gripper():
                start_state = self.gripper_state.position

                self.node.get_logger().info("Closing gripper")
                goal_msg = GripperClose.Goal()
                future = action_client_close.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self.node, future)

                success = False
                if future.done():
                    goal_handle = future.result()

                    # Goal refused
                    if not goal_handle.accepted:
                        self.node.get_logger().error("Goal was rejected by the action server!")
                        success = False

                    # Goal accepted
                    self.node.get_logger().info("Goal accepted, waiting for result...")
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, result_future)

                    # Action completed
                    if result_future.done():
                        result = result_future.result()
                        self.node.get_logger().info(f"Result received: {result.result.success}")
                        success = result.result.success

                    # Action failed
                    else:
                        self.node.get_logger().error("Failed to get result!")
                        success = False

                # Goal not received
                else:
                    self.node.get_logger().error("Failed to send goal!")
                    success = False

                return success

            if is_gripper_closed(self.gripper_state):
                self.node.get_logger().info("Gripper is closed, opening...")
                success = open_gripper()
                assert success, "Failed to open gripper"
                time.sleep(1)

                self.node.get_logger().info("Gripper is opened, closing...")
                success = close_gripper()
                assert success, "Failed to close gripper"
                time.sleep(1)

            else:
                self.node.get_logger().info("Gripper is opened, closing...")
                success = close_gripper()
                assert success, "Failed to close gripper"
                time.sleep(1)

                self.node.get_logger().info("Gripper is closed, opening...")
                success = open_gripper()
                assert success, "Failed to open gripper"
                time.sleep(1)

        finally:
            self.node.destroy_subscription(sub)


@unittest.skip("Skipping FR3 interface initialization tests")
class TestFR3InterfaceActions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_fr3_interface")

    def tearDown(self):
        self.node.destroy_node()

    #! Actions
    def test_goto_pose(self, proc_output):
        """Check whether goto_pose action works"""

        # Create the action client
        action_client = ActionClient(self.node, GotoPose, "goto_pose")
        action_client.wait_for_server()

        self.node.get_logger().info("Going to pose")

        goal_position = np.array([0.30834807, 0.00050769, 0.48611357 - 0.2])
        goal_rpy = np.array([-np.pi, 0.0, 0.0])
        duration = Duration(seconds=5.0)

        X_WG_goal = pin.SE3(pin.rpy.rpyToMatrix(goal_rpy), goal_position)

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose_msg.pose = se32rospose(X_WG_goal)

        goal_msg = GotoPose.Goal()
        goal_msg.pose_goal = goal_pose_msg
        goal_msg.duration = duration.to_msg()

        future = action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, future)

        success = False
        if future.done():
            goal_handle = future.result()

            # Goal refused
            if not goal_handle.accepted:
                self.node.get_logger().error("Goal was rejected by the action server!")
                success = False

            # Goal accepted
            self.node.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)

            # Action completed
            if result_future.done():
                result = result_future.result()
                self.node.get_logger().info(f"Result received: {result.result.success}")
                success = result.result.success

            # Action failed
            else:
                self.node.get_logger().error("Failed to get result!")
                success = False

        # Goal not received
        else:
            self.node.get_logger().error("Failed to send goal!")
            success = False

        # Check if the control mode was set
        assert success, "Failed to reach pose in given duration"


# @unittest.skip("Skipping FR3 interface initialization tests")
class TestFR3InterfaceFollowGripperVel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_fr3_interface")

        #! Set goal source to TOPIC
        srv = self.node.create_client(SetGoalSource, "/fr3_interface/set_goal_source")

        req = SetGoalSource.Request()
        req.goal_source = GoalSource.TOPIC.value

        while not srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Service '/fr3_interface/set_goal_source' not available, waiting...")

        srv.wait_for_service(timeout_sec=1)
        future = srv.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.success, f"Failed to set goal source to TOPIC: {response.message}"

        #! Set control mode to Cartesian velocity
        srv = self.node.create_client(SetControlMode, "/fr3_interface/set_control_mode")

        req = SetControlMode.Request()
        req.control_mode = ControlMode.CART_VEL.value

        while not srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Service '/fr3_interface/set_control_mode' not available, waiting...")

        future = srv.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Check if the control mode was set
        assert response.success, f"Failed to set control mode: {response.message}"

        #! Create gripper command publisher
        self.V_G_cmd = pin.Motion()
        self.twist_cmd_msg = TwistStamped()
        self.gripper_cmd_pub = self.node.create_publisher(TwistStamped, "/robot_arm/gripper_vel_command", 10)

    def tearDown(self):
        self.node.destroy_node()

    def pub_vel(self, twist_msg: TwistStamped):
        # self.node.get_logger().info("!!!!!!! Publishing gripper command !!!!!!!!!")
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.gripper_cmd_pub.publish(twist_msg)

    #! Tests
    @unittest.skip("Skipping...")
    def test_vz_pos(self, proc_output):
        """Check whether goto_pose action works"""
        self.V_G_cmd = pin.Motion()
        self.V_G_cmd.linear = np.array([0.0, 0.0, 0.05])

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)

        start_time = time.time()

        timer = self.node.create_timer(0.1, lambda: self.pub_vel(twist_msg))

        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=1)

        timer.cancel()

        # Stop robot
        self.V_G_cmd = pin.Motion()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)
        self.pub_vel(twist_msg)

        assert True

    @unittest.skip("Skipping...")
    def test_vz_neg(self, proc_output):
        """Check whether goto_pose action works"""
        self.V_G_cmd = pin.Motion()
        self.V_G_cmd.linear = np.array([0.0, 0.0, -0.05])

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)

        start_time = time.time()

        timer = self.node.create_timer(0.1, lambda: self.pub_vel(twist_msg))

        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=1)

        timer.cancel()

        # Stop robot
        self.V_G_cmd = pin.Motion()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)
        self.pub_vel(twist_msg)

        assert True

    @unittest.skip("Skipping...")
    def test_vx(self, proc_output):
        """Check whether goto_pose action works"""
        self.V_G_cmd = pin.Motion()
        self.V_G_cmd.linear = np.array([0.05, 0.0, 0.0])

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)

        start_time = time.time()

        timer = self.node.create_timer(0.1, lambda: self.pub_vel(twist_msg))

        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=1)

        timer.cancel()

        # Stop robot
        self.V_G_cmd = pin.Motion()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)
        self.pub_vel(twist_msg)

        assert True

    @unittest.skip("Skipping...")
    def test_wx(self, proc_output):
        """Check whether goto_pose action works"""
        self.V_G_cmd = pin.Motion()
        self.V_G_cmd.angular = np.array([0.25, 0.0, 0.0])

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)

        start_time = time.time()

        timer = self.node.create_timer(0.1, lambda: self.pub_vel(twist_msg))

        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=1)

        timer.cancel()

        # Stop robot
        self.V_G_cmd = pin.Motion()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)
        self.pub_vel(twist_msg)

        assert True

    # @unittest.skip("Skipping...")
    def test_wy(self, proc_output):
        """Check whether goto_pose action works"""
        self.V_G_cmd = pin.Motion()
        self.V_G_cmd.angular = np.array([0.0, 0.25, 0.0])

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)

        start_time = time.time()

        timer = self.node.create_timer(0.1, lambda: self.pub_vel(twist_msg))

        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=1)

        timer.cancel()

        # Stop robot
        self.V_G_cmd = pin.Motion()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)
        self.pub_vel(twist_msg)

        assert True

    @unittest.skip("Skipping...")
    def test_wz(self, proc_output):
        """Check whether goto_pose action works"""
        self.V_G_cmd = pin.Motion()
        self.V_G_cmd.angular = np.array([0.0, 0.0, 0.25])

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)

        start_time = time.time()

        timer = self.node.create_timer(0.1, lambda: self.pub_vel(twist_msg))

        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=1)

        timer.cancel()

        # Stop robot
        self.V_G_cmd = pin.Motion()
        twist_msg.twist = motion2rostwist(self.V_G_cmd)
        self.pub_vel(twist_msg)

        assert True


# # Post-shutdown tests
# @launch_testing.post_shutdown_test()
# class TestTurtleSimShutdown(unittest.TestCase):
#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)
