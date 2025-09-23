import pytest
import time
import numpy as np
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from arm_interfaces.action import (
    GripperToggle,
    GripperHoming,
    GotoPose,
    GotoJoints,
    GotoJointVelocities,
    GotoEEVelocity,
)
from arm_interfaces.msg import Teleop
from builtin_interfaces.msg import Duration as DurationMsg


class TestNode(Node):
    def __init__(self):
        super().__init__("fr3_interface_test_client")

        # Create action clients
        self.goto_joints_client = ActionClient(self, GotoJoints, "goto_joints")
        self.goto_pose_client = ActionClient(self, GotoPose, "goto_pose")
        self.goto_joint_vels_client = ActionClient(self, GotoJointVelocities, "goto_joint_vels")
        self.goto_ee_vels_client = ActionClient(self, GotoEEVelocity, "goto_ee_vels")
        self.gripper_homing_client = ActionClient(self, GripperHoming, "gripper_homing")
        self.gripper_toggle_client = ActionClient(self, GripperToggle, "gripper_toggle")

        # Create teleop publisher
        self.teleop_pub = self.create_publisher(Teleop, "/teleop/ee_cmd", 10)


@pytest.fixture(scope="module")
def ros_context():
    """Initialize and shutdown ROS context"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def test_node_and_executor(ros_context):
    """Set up test node and executor"""
    test_node = TestNode()

    # Create an executor for the test node
    executor = MultiThreadedExecutor()
    executor.add_node(test_node)

    # Create a thread to spin the test node
    spin_thread = Thread(target=lambda: executor.spin())
    spin_thread.daemon = True
    spin_thread.start()

    # Wait for servers to be available
    test_node.get_logger().info("Waiting for goto_joints action server...")
    test_node.goto_joints_client.wait_for_server()

    test_node.get_logger().info("Waiting for goto_pose action server...")
    test_node.goto_pose_client.wait_for_server()

    test_node.get_logger().info("Waiting for goto_joint_vels action server...")
    test_node.goto_joint_vels_client.wait_for_server()

    test_node.get_logger().info("Waiting for goto_ee_vels action server...")
    test_node.goto_ee_vels_client.wait_for_server()

    test_node.get_logger().info("Waiting for gripper_homing action server...")
    test_node.gripper_homing_client.wait_for_server()

    test_node.get_logger().info("Waiting for gripper_toggle action server...")
    test_node.gripper_toggle_client.wait_for_server()

    time.sleep(2.0)  # Additional wait for everything to initialize

    # Return the test node for testing
    yield test_node

    # Clean up
    test_node.destroy_node()


async def send_teleop_command(test_node, position=None, velocity=None, mode=Teleop.CONTROL_MODE_POSITION):
    """Send a teleop command to the robot"""
    teleop_msg = Teleop()
    teleop_msg.control_mode = mode

    if position is not None:
        teleop_msg.ee_des.position.x = position[0]
        teleop_msg.ee_des.position.y = position[1]
        teleop_msg.ee_des.position.z = position[2]
        teleop_msg.ee_des.orientation.w = 1.0

    if velocity is not None:
        teleop_msg.ee_vel_des.linear.x = velocity[0]
        teleop_msg.ee_vel_des.linear.y = velocity[1]
        teleop_msg.ee_vel_des.linear.z = velocity[2]

    test_node.teleop_pub.publish(teleop_msg)
    await rclpy.task.Future()  # Small wait to ensure teleop is processed


@pytest.mark.integration
@pytest.mark.asyncio
async def test_a_gripper_homing(test_node_and_executor):
    """Test homing the gripper"""
    test_node = test_node_and_executor
    test_node.get_logger().info("Testing gripper homing...")

    # Create goal message
    goal_msg = GripperHoming.Goal()

    # Send goal
    send_goal_future = test_node.gripper_homing_client.send_goal_async(goal_msg)
    goal_handle = await send_goal_future

    assert goal_handle.accepted, "Gripper homing goal was not accepted"

    # Wait for result
    result_future = goal_handle.get_result_async()
    result = await result_future

    assert result.result.success, "Gripper homing failed"
    test_node.get_logger().info("Gripper homing test completed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_b_gripper_toggle(test_node_and_executor):
    """Test toggling the gripper"""
    test_node = test_node_and_executor
    test_node.get_logger().info("Testing gripper toggle...")

    # Create goal message
    goal_msg = GripperToggle.Goal()

    # Send goal
    send_goal_future = test_node.gripper_toggle_client.send_goal_async(goal_msg)
    goal_handle = await send_goal_future

    assert goal_handle.accepted, "Gripper toggle goal was not accepted"

    # Wait for result
    result_future = goal_handle.get_result_async()
    result = await result_future

    assert result.result.success, "Gripper toggle failed"
    test_node.get_logger().info("Gripper toggle test completed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_c_goto_joints(test_node_and_executor):
    """Test goto_joints action"""
    test_node = test_node_and_executor
    test_node.get_logger().info("Testing goto_joints...")

    # Create goal message
    goal_msg = GotoJoints.Goal()
    goal_msg.joints_goal = [0.0, -0.7, 0.0, -2.3, 0.0, 1.6, 0.8]  # Safe position

    # Set duration to 3 seconds
    duration = DurationMsg()
    duration.sec = 3
    duration.nanosec = 0
    goal_msg.duration = duration

    # Send goal
    send_goal_future = test_node.goto_joints_client.send_goal_async(goal_msg)
    goal_handle = await send_goal_future

    assert goal_handle.accepted, "Goto joints goal was not accepted"

    # Wait for result
    result_future = goal_handle.get_result_async()
    result = await result_future

    assert result.result.success, "Goto joints action failed"
    test_node.get_logger().info("Goto joints test completed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_d_goto_pose(test_node_and_executor):
    """Test goto_pose action"""
    test_node = test_node_and_executor
    test_node.get_logger().info("Testing goto_pose...")

    # Create goal message
    goal_msg = GotoPose.Goal()

    # Use current pose as starting position and move it a bit
    goal_msg.pose_goal = PoseStamped()
    goal_msg.pose_goal.header.frame_id = "fr3_link0"
    goal_msg.pose_goal.pose.position.x = 0.4  # Example position
    goal_msg.pose_goal.pose.position.y = 0.0
    goal_msg.pose_goal.pose.position.z = 0.4
    goal_msg.pose_goal.pose.orientation.w = 1.0

    # Set duration to 3 seconds
    duration = DurationMsg()
    duration.sec = 3
    duration.nanosec = 0
    goal_msg.duration = duration

    # Send goal
    send_goal_future = test_node.goto_pose_client.send_goal_async(goal_msg)
    goal_handle = await send_goal_future

    assert goal_handle.accepted, "Goto pose goal was not accepted"

    # Wait for result
    result_future = goal_handle.get_result_async()
    result = await result_future

    assert result.result.success, "Goto pose action failed"
    test_node.get_logger().info("Goto pose test completed")
