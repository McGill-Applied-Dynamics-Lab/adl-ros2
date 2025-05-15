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
import rclpy.publisher
from rclpy.task import Future

import std_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaRobotState  # Use actual import

# Use actual imports for your interfaces
from arm_interfaces.action import RlAgent
from arm_interfaces.srv import SetControlMode, SetGoalSource
from ament_index_python.packages import get_package_share_directory
from rclpy.action.client import ClientGoalHandle, GoalStatus

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from typing import Generator, Tuple, List

# Package name where RLAgent node and dummy files are located
PACKAGE_NAME = "robot_tasks"
ACTION_SERVER_NAME = "/lift_action"  # Match parameter default or override in launch
ACTION_TYPE = RlAgent
RL_AGENT_NODE_NAME = "rl_agent"  # Match node name in RLAgent class

START_RL_NODE = False


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
        executable="lift_rl_agent",
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

    nodes_to_launch = [
        mock_ctrl_mode_service_node,
        mock_goal_src_service_node,
        launch_pytest.actions.ReadyToTest(),  # Signal readiness
    ]

    if START_RL_NODE:
        nodes_to_launch.insert(0, rl_agent_node)

    return launch.LaunchDescription(nodes_to_launch)


# --- Module-Scoped Fixture for Test Node and RCLPY Context ---
@pytest.fixture(scope="module")
def test_context() -> Generator[
    Tuple[rclpy.node.Node, rclpy.action.ActionClient, List[TwistStamped], rclpy.publisher.Publisher], None, None
]:
    """Initializes RCLPY, creates a test node and resources, and cleans up."""
    rclpy.init()
    node = rclpy.node.Node(f"test_node_{uuid.uuid4().hex}")
    action_client = rclpy.action.ActionClient(node, ACTION_TYPE, ACTION_SERVER_NAME)
    cmd_subscriber_list = []  # Shared list to store messages
    cmd_sub = node.create_subscription(
        PoseStamped, "/robot_arm/gripper_pose_des", lambda msg: cmd_subscriber_list.append(msg), 10
    )
    robot_state_pub = node.create_publisher(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", 10)
    print("Test node setup complete (module scope).")

    #! Yield resources needed by tests
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


def send_tf_transform(
    node: rclpy.node.Node,
    pub: rclpy.publisher.Publisher,
    parent_frame: str,
    child_frame: str,
    translation: list,
    rotation: list,
):
    """Sends a TF transform between two frames."""
    tf_broadcaster = StaticTransformBroadcaster(node)
    static_transform = TransformStamped()

    static_transform.header.stamp = node.get_clock().now().to_msg()
    static_transform.header.frame_id = parent_frame
    static_transform.child_frame_id = child_frame

    # Set translation
    static_transform.transform.translation.x = translation[0]
    static_transform.transform.translation.y = translation[1]
    static_transform.transform.translation.z = translation[2]

    # Set rotation (as quaternion)
    static_transform.transform.rotation.x = rotation[0]
    static_transform.transform.rotation.y = rotation[1]
    static_transform.transform.rotation.z = rotation[2]
    static_transform.transform.rotation.w = rotation[3]

    # Publish the transform
    tf_broadcaster.sendTransform(static_transform)
    time.sleep(0.5)  # Give time for the transform to be processed


# --- Test Functions ---
@pytest.mark.launch(fixture=launch_description)
def test_node_initialisation(test_context):
    time.sleep(3.0)  # Allow time for the node to initialize

    node, _, _, _ = test_context
    node.get_logger().info("Listing running nodes...")
    node_names_and_namespaces = node.get_node_names_and_namespaces()

    rl_agent_alive = False
    for name, namespace in node_names_and_namespaces:
        node.get_logger().info(f"Node: {name}, Namespace: {namespace}")

        if name == RL_AGENT_NODE_NAME:
            rl_agent_alive = True
            node.get_logger().info(f"Found RL agent node: {name}")

    assert rl_agent_alive, f"RL agent node '{RL_AGENT_NODE_NAME}' not found in running nodes."


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

    goal_msg = ACTION_TYPE.Goal()
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
def test_observations(
    test_context: Tuple[rclpy.node.Node, rclpy.action.ActionClient, List[TwistStamped], rclpy.publisher.Publisher],
):
    """Test if the agent correctly processes cube positions from TF2 transforms.

    Also checks that commands are published after goal acceptance and state update."""
    node, action_client, cmd_list, robot_state_pub = test_context
    cmd_list.clear()
    node.get_logger().info("Running test_observations...")

    # Start the agent by sending a goal
    assert action_client.wait_for_server(timeout_sec=5.0)
    goal_msg = ACTION_TYPE.Goal()
    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    # Wait for agent initialization
    time.sleep(1.0)

    # First send robot state (gripper pose)
    gripper_x = 0.5
    gripper_z = 0.3
    send_dummy_robot_state(node, robot_state_pub, z_pos=gripper_z, x_pos=gripper_x)

    # Send a TF transform for the cube - different position from the default in the agent
    cube_x = 0.8
    cube_y = 0.2
    cube_z = 0.05
    send_tf_transform(
        node=node,
        pub=None,  # Not needed for TF
        parent_frame="world",
        child_frame="cube_frame",
        translation=[cube_x, cube_y, cube_z],
        rotation=[0.0, 0.0, 0.0, 1.0],  # Identity quaternion
    )

    # Wait for commands - a good indicator that observations were processed
    start_time = time.time()
    while time.time() - start_time < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        # if cmd_list:
        #     cmd_received = True
        #     break

    assert len(cmd_list) > 50, "No command received, suggesting observations weren't processed"
    assert isinstance(cmd_list[0], PoseStamped), "Expected command to be of type PoseStamped"

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

    goal_msg = ACTION_TYPE.Goal()
    send_goal_future = action_client.send_goal_async(goal_msg)
    spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    get_result_future = goal_handle.get_result_async()

    # Wait briefly for mode setting
    time.sleep(1.0)

    # Send robot state simulating successful insertion
    send_dummy_robot_state(node, robot_state_pub, z_pos=0.01)  # z < 0.01 threshold

    # Spin until the result is available
    spin_until_future_complete(node, get_result_future, timeout_sec=10.0)

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

    goal_msg = ACTION_TYPE.Goal()
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
