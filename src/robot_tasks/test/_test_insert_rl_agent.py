import os
import sys
import time
import pytest  # Use pytest directly
import uuid
import asyncio  # Import asyncio for async functions if needed

import launch
import launch_ros
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
import launch_pytest

import rclpy
import rclpy.action
import rclpy.node
from rclpy.task import Future

import std_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaRobotState  # Use actual import

# Use actual imports for your interfaces
from arm_interfaces.action import PegInHole
from arm_interfaces.srv import SetControlMode, SetGoalSource
from ament_index_python.packages import get_package_share_directory
from rclpy.action.client import ClientGoalHandle, GoalStatus

# Package name where RLAgent node and dummy files are located
PACKAGE_NAME = "robot_tasks"
ACTION_SERVER_NAME = "/insert_action"  # Match parameter default or override in launch
RL_AGENT_NODE_NAME = "rl_agent"  # Match node name in RLAgent class


# --- Test Fixture for Launch Description ---
# @pytest.fixture(scope="module")  # Make launch fixture module-scoped
@launch_pytest.fixture
def launch_description():
    """Sets up the launch description for the test environment."""
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    dummy_agent_path = os.path.join(pkg_share, "agents/insert/agent.pt")
    # TODO: Ensure dummy file exists or create it programmatically

    rl_agent_node = launch_ros.actions.Node(
        package=PACKAGE_NAME,
        executable="insert_rl_agent",
        name=RL_AGENT_NODE_NAME,
        parameters=[
            {"agent_checkpoint_path": dummy_agent_path},
        ],
        output="screen",
    )

    mock_ctrl_mode_service_node = launch_ros.actions.Node(
        package=PACKAGE_NAME,
        executable="mock_control_mode_service",
        output="screen",
    )
    mock_goal_src_service_node = launch_ros.actions.Node(
        package=PACKAGE_NAME,
        executable="mock_goal_source_service",
        output="screen",
    )

    return launch.LaunchDescription(
        [
            rl_agent_node,
            mock_ctrl_mode_service_node,
            mock_goal_src_service_node,
            launch_pytest.actions.ReadyToTest(),  # Signal readiness
        ]
    )


# --- Module-Scoped Fixture for Test Node and RCLPY Context ---
@pytest.fixture(scope="module")
def test_context():
    """Initializes RCLPY, creates a test node and resources, and cleans up."""
    rclpy.init()
    node = rclpy.node.Node(f"test_node_{uuid.uuid4().hex}")
    action_client = rclpy.action.ActionClient(node, PegInHole, ACTION_SERVER_NAME)
    cmd_subscriber_list = []  # Shared list to store messages
    cmd_sub = node.create_subscription(
        TwistStamped, "/robot_arm/gripper_vel_command", lambda msg: cmd_subscriber_list.append(msg), 10
    )
    robot_state_pub = node.create_publisher(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", 10)
    print("Test node setup complete (module scope).")
    # Yield resources needed by tests
    yield node, action_client, cmd_subscriber_list, robot_state_pub
    # Teardown
    print("Test node teardown starting (module scope)...")
    node.destroy_subscription(cmd_sub)
    node.destroy_publisher(robot_state_pub)
    # node.destroy_client(action_client.action_client)  # Destroy underlying client
    node.destroy_node()
    rclpy.shutdown()
    print("Test node teardown complete (module scope).")


# --- Helper Function ---
def send_dummy_robot_state(node: rclpy.node.Node, pub: rclpy.publisher.Publisher, z_pos=0.1, x_pos=0.6):
    """Sends a robot state message."""
    msg = FrankaRobotState()
    # Populate with plausible default data (same as before)
    default_joint_poses = [0.0, 0.53249995, 0.0, -2.02528006, 0.0, 2.55778002, 0.78539816]
    msg.measured_joint_state = JointState(
        header=std_msgs.msg.Header(stamp=node.get_clock().now().to_msg()),
        position=[float(p) for p in default_joint_poses],
        velocity=[0.0] * 7,
        effort=[0.0] * 7,
    )
    X_G_dummy = Pose()
    X_G_dummy.position = Point(x=x_pos, y=0.0, z=z_pos)
    X_G_dummy.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    msg.o_t_ee = PoseStamped(
        header=std_msgs.msg.Header(stamp=node.get_clock().now().to_msg()),
        pose=X_G_dummy,
    )
    msg.o_dp_ee_d = TwistStamped()
    pub.publish(msg)
    time.sleep(0.05)  # Allow time for message propagation


def spin_until_future_complete(node: rclpy.node.Node, future: Future, timeout_sec=5.0):
    """Helper function to spin node until a future is complete."""
    start_time = time.time()
    while rclpy.ok() and not future.done() and (time.time() - start_time < timeout_sec):
        rclpy.spin_once(node, timeout_sec=0.1)

    assert future.done(), f"Future did not complete within timeout ({timeout_sec}s)"


# --- Test Functions ---
# Apply the launch fixture marker to all test functions that need the nodes running
@pytest.mark.launch(fixture=launch_description)
def test_action_server_available(test_context):
    """Test if the action server becomes available."""
    node, action_client, _, _ = test_context
    node.get_logger().info("Checking action server availability...")
    assert action_client.wait_for_server(timeout_sec=10.0), f"Action server '{ACTION_SERVER_NAME}' not available."
    node.get_logger().info("Action server available.")


@pytest.mark.launch(fixture=launch_description)
def test_action_goal_acceptance(test_context):
    """Test if a goal is accepted when server is idle."""
    node, action_client, cmd_list, _ = test_context
    cmd_list.clear()  # Ensure list is clear for this test
    node.get_logger().info("Running test_action_goal_acceptance...")
    assert action_client.wait_for_server(timeout_sec=5.0)

    goal_msg = PegInHole.Goal()
    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future)

    goal_handle: ClientGoalHandle = send_goal_future.result()
    assert goal_handle is not None, "send_goal_async failed (returned None)"
    assert goal_handle.accepted, "Goal was rejected"
    node.get_logger().info("Goal accepted as expected.")

    # Cleanup: Cancel the goal
    cancel_future = goal_handle.cancel_goal_async()
    spin_until_future_complete(node, cancel_future)


@pytest.mark.launch(fixture=launch_description)
def test_command_publishing(test_context):
    """Test if commands are published after goal acceptance and state update."""
    node, action_client, cmd_list, robot_state_pub = test_context
    cmd_list.clear()
    node.get_logger().info("Running test_command_publishing...")
    assert action_client.wait_for_server(timeout_sec=5.0)

    goal_msg = PegInHole.Goal()
    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    # Wait briefly for mode setting calls (async in RLAgent)
    time.sleep(1.0)

    # Send initial robot state
    send_dummy_robot_state(node, robot_state_pub, z_pos=0.1)

    # Wait for a command
    start_time = time.time()
    cmd_received = False
    while time.time() - start_time < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cmd_list:
            cmd_received = True
            break

    assert cmd_received, "No command received on /robot_arm/gripper_vel_command"
    assert isinstance(cmd_list[0], TwistStamped)
    node.get_logger().info(f"Command received: {cmd_list[0].twist}")

    # Cleanup
    cancel_future = goal_handle.cancel_goal_async()
    spin_until_future_complete(node, cancel_future)


@pytest.mark.launch(fixture=launch_description)
def test_task_success(test_context):
    """Test if the action succeeds when insertion criteria are met."""
    node, action_client, cmd_list, robot_state_pub = test_context
    cmd_list.clear()
    node.get_logger().info("Running test_task_success...")
    assert action_client.wait_for_server(timeout_sec=5.0)

    #! Start the action
    node.get_logger().info("Sending goal to action server")
    goal_msg = PegInHole.Goal()
    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    get_result_future = goal_handle.get_result_async()

    # Wait briefly for mode setting
    time.sleep(1.0)

    #! Send robot state simulating successful insertion
    node.get_logger().info("Sending dummy states")
    send_dummy_robot_state(node, robot_state_pub, z_pos=0.01)  # z < 0.01 threshold

    #! Wait until the action is completed
    node.get_logger().info("Waiting for action to complete")
    spin_until_future_complete(node, get_result_future, timeout_sec=5.0)

    #! Check the result
    node.get_logger().info("Action completed, checking result")
    result_response = get_result_future.result()
    status = result_response.status
    result = result_response.result

    assert status == GoalStatus.STATUS_SUCCEEDED
    assert result.success
    node.get_logger().info(f"Goal succeeded as expected. Message: {result.message}")


@pytest.mark.launch(fixture=launch_description)
def test_task_failure(test_context):
    """Test if the action aborts when insertion fails."""
    node, action_client, cmd_list, robot_state_pub = test_context
    cmd_list.clear()
    node.get_logger().info("Running test_task_failure...")
    assert action_client.wait_for_server(timeout_sec=5.0)

    goal_msg = PegInHole.Goal()
    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()
    assert goal_handle.accepted, "Goal was rejected"

    get_result_future = goal_handle.get_result_async()

    # Wait briefly for mode setting
    time.sleep(1.0)

    # Send robot state simulating successful insertion
    send_dummy_robot_state(node, robot_state_pub, z_pos=0.01, x_pos=0.5)  # z < 0.01 threshold

    # Spin until the result is available
    spin_until_future_complete(node, get_result_future, timeout_sec=5.0)

    result_response = get_result_future.result()
    status = result_response.status
    result = result_response.result

    assert status == GoalStatus.STATUS_ABORTED, "Expected action to be aborted"
    assert not result.success, "Expected result to be failed"
    assert result.message == "Insertion failed.", "Failure message mismatch"
    node.get_logger().info(f"Goal succeeded as expected. Message: {result.message}")


@pytest.mark.launch(fixture=launch_description)
def test_task_canceled(test_context):
    """Test if the action succeeds when insertion criteria are met."""
    node, action_client, cmd_list, robot_state_pub = test_context
    cmd_list.clear()
    node.get_logger().info("Running test_task_success...")
    assert action_client.wait_for_server(timeout_sec=5.0)

    #! Start the action
    node.get_logger().info("Sending goal to action server")
    goal_msg = PegInHole.Goal()
    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    get_result_future = goal_handle.get_result_async()

    # Wait briefly for mode setting
    time.sleep(1.0)

    #! Send robot state simulating successful insertion
    node.get_logger().info("Sending dummy states")
    send_dummy_robot_state(node, robot_state_pub, z_pos=1.0)  # z < 0.01 threshold

    #! Cancel the action
    time.sleep(1.0)
    node.get_logger().info("Canceling the action")
    cancel_future = goal_handle.cancel_goal_async()
    spin_until_future_complete(node, cancel_future, timeout_sec=5.0)

    # Wait until the action is completed
    node.get_logger().info("Waiting for action to complete")
    spin_until_future_complete(node, get_result_future, timeout_sec=5.0)

    #! Check the result
    result_response = get_result_future.result()
    result_status = result_response.status
    result = result_response.result

    assert goal_handle.status == GoalStatus.STATUS_CANCELED
    assert result_status == GoalStatus.STATUS_CANCELED
    node.get_logger().info(f"Action cancelled as expected. Message: {result.message}")


@pytest.mark.launch(fixture=launch_description)
def test_cancel_repeat(test_context):
    """Test if the action can be canceled and then restarted for a success."""
    node, action_client, cmd_list, robot_state_pub = test_context
    cmd_list.clear()
    node.get_logger().info("Running test_task_success...")
    assert action_client.wait_for_server(timeout_sec=5.0)

    #! Start 1st action
    node.get_logger().info("Sending goal to action server")
    goal_msg = PegInHole.Goal()
    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    get_result_future = goal_handle.get_result_async()

    # Wait briefly for mode setting
    time.sleep(1.0)

    # * Send robot state simulating successful insertion
    node.get_logger().info("Sending dummy states")
    send_dummy_robot_state(node, robot_state_pub, z_pos=1.0)  # z < 0.01 threshold

    # * Cancel the action
    time.sleep(1.0)
    node.get_logger().info("Canceling the action")
    cancel_future = goal_handle.cancel_goal_async()
    spin_until_future_complete(node, cancel_future, timeout_sec=5.0)

    # Wait until the action is completed
    node.get_logger().info("Waiting for action to complete")
    spin_until_future_complete(node, get_result_future, timeout_sec=5.0)

    # * Check the result
    result_response = get_result_future.result()
    result_status = result_response.status
    result = result_response.result

    assert goal_handle.status == GoalStatus.STATUS_CANCELED
    assert result_status == GoalStatus.STATUS_CANCELED
    node.get_logger().info(f"Action cancelled as expected. Message: {result.message}")

    #! Start second action
    time.sleep(2.0)
    node.get_logger().info("Starting second action")

    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    get_result_future = goal_handle.get_result_async()

    # Wait briefly for mode setting
    time.sleep(1.0)

    # * Send robot state
    node.get_logger().info("Sending dummy states")
    send_dummy_robot_state(node, robot_state_pub, z_pos=0.01)  # z < 0.01 threshold

    # Wait until the action is completed
    node.get_logger().info("Waiting for action to complete")
    spin_until_future_complete(node, get_result_future, timeout_sec=5.0)

    # * Check the result
    node.get_logger().info("Action completed, checking result")
    result_response = get_result_future.result()
    status = result_response.status
    result = result_response.result

    assert status == GoalStatus.STATUS_SUCCEEDED
    assert result.success
    node.get_logger().info(f"Goal succeeded as expected. Message: {result.message}")
