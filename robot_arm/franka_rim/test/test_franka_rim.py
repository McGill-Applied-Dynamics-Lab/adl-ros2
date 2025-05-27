from contextlib import redirect_stdout
import io
import os
import time
import pytest  # Use pytest directly
import uuid
import threading

from rclpy.executors import SingleThreadedExecutor

import launch
import launch_ros
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
from ament_index_python.packages import get_package_share_directory
from rclpy.action.client import ClientGoalHandle, GoalStatus

# Package name where RLAgent node and dummy files are located
PACKAGE_NAME = "franka_rim"


# --- Test Fixture for Launch Description ---
@launch_pytest.fixture
def launch_description():
    """Sets up the launch description for the test environment."""
    # pkg_share = get_package_share_directory(PACKAGE_NAME)

    rl_agent_node = launch_ros.actions.Node(
        package=PACKAGE_NAME,
        executable="franka_rim_node",
        # name=RL_AGENT_NODE_NAME,
        # parameters=[
        #     {"agent_checkpoint_path": dummy_agent_path},
        # ],
        output="screen",
    )

    # mock_ctrl_mode_service_node = launch_ros.actions.Node(
    #     package=PACKAGE_NAME,
    #     executable="mock_control_mode_service",
    #     output="screen",
    # )
    # mock_goal_src_service_node = launch_ros.actions.Node(
    #     package=PACKAGE_NAME,
    #     executable="mock_goal_source_service",
    #     output="screen",
    # )

    return launch.LaunchDescription(
        [
            rl_agent_node,
            launch_pytest.actions.ReadyToTest(),  # Signal readiness
        ]
    )


# --- Module-Scoped Fixture for Test Node and RCLPY Context ---
@pytest.fixture(scope="module")
def test_context():
    """Initializes RCLPY, creates a test node and resources, and cleans up."""
    rclpy.init()
    node = rclpy.node.Node(f"test_node_{uuid.uuid4().hex}")
    cmd_subscriber_list = []  # Shared list to store messages
    # cmd_sub = node.create_subscription(
    #     TwistStamped, "/robot_arm/gripper_vel_command", lambda msg: cmd_subscriber_list.append(msg), 10
    # )

    # robot_state_pub = node.create_publisher(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", 10)
    print("Test node setup complete (module scope).")

    # Yield resources needed by tests
    yield node, cmd_subscriber_list

    # Teardown
    print("Test node teardown starting (module scope)...")
    # node.destroy_subscription(cmd_sub)
    # node.destroy_publisher(robot_state_pub)
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
def test_node_init(test_context):
    """Test if the node initializes correctly."""
    (
        node,
        _,
    ) = test_context

    time.sleep(1)
    node_names = node.get_node_names()
    assert any("franka_rim_node" in n for n in node_names), "franka_rim_node is not alive"
