from typing import Generator, Tuple
import numpy as np
import time
import uuid
import threading

# testing imports
import pytest
import launch
import launch_ros
import launch_pytest

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from arm_interfaces.msg import FrankaRIM, Teleop
from geometry_msgs.msg import WrenchStamped
import std_msgs.msg

# My pkg imports
from franka_rim.delay_rim_node import DelayRIMNode


PACKAGE_NAME = "franka_rim"


#! ROS 2 Launch Fixture
@pytest.fixture
def launch_description():
    delay_rim_node = launch_ros.actions.Node(
        package=PACKAGE_NAME,
        executable="delay_rim_node",
        output="screen",
        parameters=[
            {
                "control_period": 0.01,  # 100Hz for testing (slower than production)
                "delay_compensation_method": "DelayRIM",
                "interface_stiffness": 3000.0,
                "interface_damping": 2.0,
                "force_scaling": 0.02,
            }
        ],
    )
    return launch.LaunchDescription(
        [
            delay_rim_node,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.fixture(scope="module")
def test_context() -> Generator[Tuple[DelayRIMNode, Node, Publisher], None, None]:
    """
    Set up a test context with a ROS 2 node and publisher for FrankaRIM.

    Yields:
        tuple: Containing the DelayRIMNode, test Node, and publisher.
    """
    rclpy.init()
    test_node: Node = rclpy.node.Node(f"test_node_{uuid.uuid4().hex}")
    pub: Publisher = test_node.create_publisher(FrankaRIM, "/fr3_rim", 10)

    delay_rim_node: DelayRIMNode = DelayRIMNode()
    yield delay_rim_node, test_node, pub

    delay_rim_node.destroy_node()
    test_node.destroy_publisher(pub)
    test_node.destroy_node()
    rclpy.shutdown()


def send_dummy_rim_message(
    node: Node, pub: Publisher, interface_velocity=None, effective_mass=None, effective_force=None
):
    """Send a dummy FrankaRIM message for testing."""
    msg = FrankaRIM()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.interface_stiffness = 3000.0
    msg.interface_damping = 2.0

    if effective_mass is None:
        effective_mass = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # 3x3 identity
    if effective_force is None:
        effective_force = [0.1, 0.2, 0.3]
    if interface_velocity is None:
        interface_velocity = [0.01, 0.02, 0.03]

    msg.effective_mass = effective_mass
    msg.effective_force = effective_force
    msg.interface_velocity = interface_velocity

    pub.publish(msg)
    time.sleep(0.05)


#! Tests


def test_node_initialization(test_context):
    """Test that the DelayRIM node initializes successfully."""
    delay_rim_node, test_node, pub = test_context

    # If we reach this point, the node has started successfully
    # Spin briefly to ensure node is active
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while waited < timeout:
        rclpy.spin_once(delay_rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=poll_period)
        waited += poll_period

    # Verify node properties
    assert delay_rim_node._delay_method is not None
    assert delay_rim_node._interface_stiffness == 3000.0
    assert delay_rim_node._interface_damping == 2.0
    assert delay_rim_node._force_scaling == 0.02


def test_rim_subscription(test_context):
    """Test that the node can receive and process RIM messages."""
    delay_rim_node, test_node, pub = test_context

    # Publish test RIM message
    send_dummy_rim_message(test_node, pub)

    # Wait for processing
    timeout = 2.0
    poll_period = 0.1
    waited = 0.0
    while waited < timeout:
        rclpy.spin_once(delay_rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=poll_period)
        waited += poll_period

    # Check that RIM message was received
    assert delay_rim_node._last_rim_msg is not None
    assert delay_rim_node._last_rim_msg.interface_stiffness == 3000.0


def test_force_publication(test_context):
    """Test that the node publishes force messages."""
    delay_rim_node, test_node, pub = test_context
    received_msgs = []
    lock = threading.Lock()

    def force_callback(msg):
        with lock:
            received_msgs.append(msg)

    force_sub = test_node.create_subscription(WrenchStamped, "/haptic_force", force_callback, 10)

    # Spin for a short duration to collect force messages
    timeout = 1.5
    poll_period = 0.05
    waited = 0.0
    while waited < timeout:
        rclpy.spin_once(delay_rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=poll_period)
        waited += poll_period

    with lock:
        num_received = len(received_msgs)

    assert num_received > 0, f"No force messages published, got {num_received}"

    # Check message structure
    with lock:
        if len(received_msgs) > 0:
            force_msg = received_msgs[0]
            assert hasattr(force_msg, "wrench"), "Force message missing wrench field"
            assert hasattr(force_msg.wrench, "force"), "Wrench missing force field"

            # Force values should be finite numbers
            assert isinstance(force_msg.wrench.force.x, float)
            assert isinstance(force_msg.wrench.force.y, float)
            assert isinstance(force_msg.wrench.force.z, float)

    test_node.destroy_subscription(force_sub)


def test_teleop_publication(test_context):
    """Test that the node publishes teleop messages."""
    delay_rim_node, test_node, pub = test_context
    received_msgs = []
    lock = threading.Lock()

    def teleop_callback(msg):
        with lock:
            received_msgs.append(msg)

    teleop_sub = test_node.create_subscription(Teleop, "/fr3_teleop_cmd", teleop_callback, 10)

    # Spin for a short duration to collect teleop messages
    timeout = 1.5
    poll_period = 0.05
    waited = 0.0
    while waited < timeout:
        rclpy.spin_once(delay_rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=poll_period)
        waited += poll_period

    with lock:
        num_received = len(received_msgs)

    assert num_received > 0, f"No teleop messages published, got {num_received}"

    # Check message structure
    with lock:
        if len(received_msgs) > 0:
            teleop_msg = received_msgs[0]
            assert hasattr(teleop_msg, "control_mode"), "Teleop message missing control_mode field"
            assert teleop_msg.control_mode == Teleop.CONTROL_MODE_POSITION

    test_node.destroy_subscription(teleop_sub)


def test_force_computation_with_rim_input(test_context):
    """Test force computation when RIM data is available."""
    delay_rim_node, test_node, pub = test_context
    received_msgs = []
    lock = threading.Lock()

    def force_callback(msg):
        with lock:
            received_msgs.append(msg)

    force_sub = test_node.create_subscription(WrenchStamped, "/haptic_force", force_callback, 10)

    # Send RIM message with known values
    send_dummy_rim_message(
        test_node,
        pub,
        effective_mass=[2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0],
        effective_force=[1.0, 2.0, 3.0],
        interface_velocity=[0.1, 0.2, 0.3],
    )

    # Wait for processing and force publication
    timeout = 2.0
    poll_period = 0.05
    waited = 0.0
    while waited < timeout:
        rclpy.spin_once(delay_rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=poll_period)
        waited += poll_period

    with lock:
        num_received = len(received_msgs)

    assert num_received > 0, f"No force messages after RIM input, got {num_received}"

    # Forces should be computed (non-zero in general case)
    with lock:
        if len(received_msgs) > 0:
            force_msg = received_msgs[-1]  # Get latest message
            force_magnitude = (
                force_msg.wrench.force.x**2 + force_msg.wrench.force.y**2 + force_msg.wrench.force.z**2
            ) ** 0.5

            # Force magnitude should be reasonable (not infinite or NaN)
            assert force_magnitude >= 0.0 and force_magnitude < 1000.0, (
                f"Force magnitude {force_magnitude} is unreasonable"
            )
            assert np.isfinite(force_magnitude), "Force magnitude is not finite"

    test_node.destroy_subscription(force_sub)


@pytest.mark.skip(reason="Skipping delay compensation method tests for now")
@pytest.mark.parametrize("delay_method", ["ZOH", "ZOHPhi", "DelayRIM"])
def test_delay_compensation_methods(test_context, delay_method):
    """Test different delay compensation methods."""
    # delay_rim_node, test_node, pub = test_context

    # Create node with specific delay method
    test_node = rclpy.create_node(f"test_delay_rim_{delay_method.lower()}")
    delay_rim_node = DelayRIMNode()

    # Override the delay method parameter
    delay_rim_node._delay_method = delay_rim_node.DelayCompensationMethod(delay_method)

    # Test that the method is set correctly
    assert delay_rim_node._delay_method.value == delay_method

    # Test force computation doesn't crash
    force = delay_rim_node._compute_rendered_force()
    assert force.shape == (3, 1)
    assert np.isfinite(force).all()

    delay_rim_node.destroy_node()
    test_node.destroy_node()


def test_reduced_model_state_update(test_context):
    """Test that reduced model state is updated correctly."""
    delay_rim_node, test_node, pub = test_context

    # Send RIM message
    interface_velocity = [0.5, 0.3, 0.1]
    send_dummy_rim_message(test_node, pub, interface_velocity=interface_velocity)

    # Wait for processing
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while waited < timeout:
        rclpy.spin_once(delay_rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=poll_period)
        waited += poll_period

    # Check that reduced model velocity was updated
    expected_velocity = np.array(interface_velocity).reshape((3, 1))
    np.testing.assert_allclose(delay_rim_node._reduced_model.reduced_model_velocity, expected_velocity, atol=1e-6)
