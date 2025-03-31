"""
Test that launch of the franka robot arm bringup works as expected.

Main tests:
 -

 Checks the following topics:
    - /joint_states [sensor_msgs/msg/JointState]
    - /fr3_gripper/joint_states [sensor_msgs/msg/JointState]
    - /franka_robot_state_broadcaster/robot_state [franka_msgs/msg/FrankaRobotState]

    (not checked)
    - /franka_robot_state_broadcaster/current_pose [geometry_msgs/msg/PoseStamped]
    - /franka/joint_states [sensor_msgs/msg/JointState]
    - /franka_robot_state_broadcaster/desired_end_effector_twist [geometry_msgs/msg/TwistStamped]
    - /franka_robot_state_broadcaster/desired_joint_states [sensor_msgs/msg/JointState]
    - /franka_robot_state_broadcaster/external_joint_torques [sensor_msgs/msg/JointState]
    - /franka_robot_state_broadcaster/external_wrench_in_base_frame [geometry_msgs/msg/WrenchStamped]
    - /franka_robot_state_broadcaster/external_wrench_in_stiffness_frame [geometry_msgs/msg/WrenchStamped]
    - /franka_robot_state_broadcaster/last_desired_pose [geometry_msgs/msg/PoseStamped]
    - /franka_robot_state_broadcaster/measured_joint_states [sensor_msgs/msg/JointState]

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

# Interfaces
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaRobotState


def generate_test_description():
    #! Launch arguments
    arm_id_parameter_name = "arm_id"
    robot_ip_parameter_name = "robot_ip"
    load_gripper_parameter_name = "load_gripper"
    hw_type_parameter_name = "hw_type"
    use_rviz_parameter_name = "use_rviz"

    # arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    # hw_type = LaunchConfiguration(hw_type_parameter_name)  # fake, real, or isaac
    # use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    robot_ip_launch_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        # default_value="dont-care",
        default_value=EnvironmentVariable("FR3_IP", default_value="dont-care"),
        description="Hostname or IP address of the robot.",
    )

    # arm_id_launch_arg = DeclareLaunchArgument(
    #     arm_id_parameter_name,
    #     default_value="fr3",
    #     description="ID of the type of arm used. Supported values: fer, fr3, fp3",
    # )

    # use_rviz_launch_arg = DeclareLaunchArgument(
    #     use_rviz_parameter_name,
    #     default_value="true",
    #     description="Visualize the robot in Rviz",
    # )

    load_gripper_launch_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value="true",
        description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded without an end-effector.",
    )

    # hw_type_launch_arg = DeclareLaunchArgument(
    #     hw_type_parameter_name,
    #     default_value="isaac",
    #     description="Which hardware to use: 'real', 'fake', or 'isaac'",
    #     choices=["real", "fake", "isaac", "gazebo"],
    # )

    launch_args = [
        robot_ip_launch_arg,
        load_gripper_launch_arg,
    ]

    #! Launch Descriptions
    fr3_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("robot_arm_bringup"),
                        "launch",
                        "fr3.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            arm_id_parameter_name: "fr3",
            robot_ip_parameter_name: robot_ip,
            load_gripper_parameter_name: load_gripper,
            hw_type_parameter_name: "real",
            use_rviz_parameter_name: "false",
        }.items(),
    )

    launch_description = LaunchDescription(
        launch_args
        + [
            fr3_launch_description,
            # Launch tests 0.5 s later
            launch.actions.TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )

    return (launch_description, {"fr3": fr3_launch_description})


# Active tests
class TestFR3BringupReal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_fr3_bringup_real")

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_joint_state(self, proc_output):
        """Check whether /joint_states messages published"""
        msgs_rx = []
        sub = self.node.create_subscription(JointState, "/joint_states", lambda msg: msgs_rx.append(msg), 100)
        try:
            # Listen to the pose topic for 10 s
            end_time = time.time() + 5
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # There should have been 100 messages received
            assert len(msgs_rx) > 50
        finally:
            self.node.destroy_subscription(sub)

    def test_publishes_robot_state(self, proc_output):
        """Check whether /franka_robot_state_broadcaster/robot_state messages published"""
        msgs_rx = []
        sub = self.node.create_subscription(
            FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", lambda msg: msgs_rx.append(msg), 100
        )
        try:
            # Listen to the pose topic for 10 s
            end_time = time.time() + 5
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # There should have been 100 messages received
            assert len(msgs_rx) > 50
        finally:
            self.node.destroy_subscription(sub)

    # def test_logs_spawning(self, proc_output):
    #     """Check whether logging properly"""
    #     proc_output.assertWaitFor("Spawning turtle [turtle1] at x=", timeout=5, stream="stderr")

    # def test_fail(self, proc_output):
    #     """Check whether test fails"""
    #     self.fail("This test should fail")


# # Post-shutdown tests
# @launch_testing.post_shutdown_test()
# class TestTurtleSimShutdown(unittest.TestCase):
#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)
