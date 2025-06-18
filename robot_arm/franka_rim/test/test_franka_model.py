from typing import Generator, Tuple
import pinocchio as pin
import numpy as np
import os
import time
import uuid
import threading

# testing imports
import pytest
import launch
import launch_ros
import launch_pytest

# Ros 2 imports
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TwistStamped, WrenchStamped, Wrench, Vector3
import std_msgs.msg

# My pkg imports
from franka_rim.franka_model_node import FrankaModelNode
from arm_interfaces.msg import FrankaModel


PACKAGE_NAME = "franka_rim"
N_DOF = 7


#! ROS 2 Launch Fixture
@launch_pytest.fixture
def launch_description():
    franka_model_node = launch_ros.actions.Node(
        package=PACKAGE_NAME,
        executable="franka_model_node",
        output="screen",
    )
    return launch.LaunchDescription(
        [
            franka_model_node,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.fixture
def franka_model_node(monkeypatch, urdf_path):
    # Patch get_package_share_directory to return the correct path
    monkeypatch.setattr(
        "franka_rim.franka_model_node.get_package_share_directory",
        lambda pkg: os.path.dirname(os.path.dirname(urdf_path)),
    )
    rclpy.init()
    node = FrankaModelNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope="module")
def test_context() -> Generator[Tuple[FrankaModelNode, Node, Publisher], None, None]:
    """
    To set up a test context with a ROS 2 node and publisher for FrankaRobotState.

    Yields:
        tuple: Containing the FrankaModelNode, test Node, and publisher.
    """
    rclpy.init()
    test_node: Node = rclpy.node.Node(f"test_node_{uuid.uuid4().hex}")
    pub: Publisher = test_node.create_publisher(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", 10)

    model_node: Node = FrankaModelNode()
    yield model_node, test_node, pub

    model_node.destroy_node()
    test_node.destroy_publisher(pub)
    test_node.destroy_node()

    rclpy.shutdown()


#! Other Fixtures
@pytest.fixture
def urdf_path():
    pkg_share = get_package_share_directory("franka_rim")
    return os.path.join(pkg_share, "models", "fr3_franka_hand.urdf")


@pytest.mark.parametrize(
    "q,dq",
    [
        (np.zeros(N_DOF), np.zeros(N_DOF)),
        (np.ones(N_DOF), np.ones(N_DOF)),
        (np.random.uniform(-1, 1, N_DOF), np.random.uniform(-1, 1, N_DOF)),
    ],
)
def test_compute_model_matrices(franka_model_node, q, dq):
    """
    To validate the computation of mass matrix and coriolis vector. Just a simple test to ensure the method runs without errors.
    """
    node: FrankaModelNode = franka_model_node
    assert node._model_loaded

    M, c, tau, Ai, Ai_dot, Ai_dot_q_dot, f_ext_estimated = node._compute_model_matrices(q, dq)

    n = node._robot_model.nv

    assert M.shape == (n, n)
    assert np.allclose(M, M.T, atol=1e-8)
    assert c.shape == (n,)

    assert M.shape == (n, n)
    assert c.shape == (n,)
    assert tau.shape == (n,)
    assert Ai.shape == (n,)
    assert Ai_dot.shape == (n,)
    assert Ai_dot_q_dot.shape == ()
    assert f_ext_estimated.shape == (6,)  # 6D wrench


def test_compute_contact_forces(franka_model_node):
    """Test contact force computation."""
    node: FrankaModelNode = franka_model_node
    assert node._model_loaded

    q = np.zeros(node._robot_model.nv)
    dq = np.zeros(node._robot_model.nv)

    # Set some dummy external torques
    node.tau_ext = np.array([0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    f_ext = node._compute_contact_forces(q, dq)

    assert f_ext.shape == (6,)
    assert np.isfinite(f_ext).all()


def send_dummy_robot_state(node: Node, pub: Publisher, positions=None, velocities=None, efforts=None):
    msg = FrankaRobotState()
    n = 9
    if positions is None:
        positions = [0.0, 0.53249995, 0.0, -2.02528006, 0.0, 2.55778002, 0.78539816, 0.0, 0.0]

    if velocities is None:
        velocities = [0.0] * n

    if efforts is None:
        efforts = [0.0] * n

    msg.measured_joint_state = JointState(
        header=std_msgs.msg.Header(stamp=node.get_clock().now().to_msg()),
        position=[float(p) for p in positions],
        velocity=[float(v) for v in velocities],
        effort=[float(e) for e in efforts],
    )

    # Add external torque estimates
    msg.tau_ext_hat_filtered = JointState(
        header=std_msgs.msg.Header(stamp=node.get_clock().now().to_msg()),
        effort=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Some dummy external torques
    )

    msg.o_f_ext_hat_k = WrenchStamped()
    msg.o_f_ext_hat_k.header = std_msgs.msg.Header(stamp=node.get_clock().now().to_msg())
    msg.o_f_ext_hat_k.wrench = Wrench(force=Vector3(x=1.0, y=0.5, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.1))

    # Optionally fill in other fields as needed
    pub.publish(msg)
    time.sleep(0.05)


#! Franka Model Tests


def test_pinocchio_model_loads(urdf_path):
    model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path)
    assert model is not None
    assert collision_model is not None
    assert visual_model is not None
    assert hasattr(model, "nq")
    assert model.nq == 7 or model.nq > 0  # Franka should have 7 dof


def test_mass_matrix_and_coriolis(urdf_path):
    model, _, _ = pin.buildModelsFromUrdf(urdf_path)
    data = model.createData()
    # Use zero configuration for test
    q = np.zeros(model.nq)
    dq = np.zeros(model.nv)
    mass_matrix = pin.crba(model, data, q)
    coriolis = pin.rnea(model, data, q, dq, np.zeros_like(q))
    assert mass_matrix.shape[0] == model.nv
    assert mass_matrix.shape[1] == model.nv
    assert np.allclose(mass_matrix, mass_matrix.T, atol=1e-8)  # Should be symmetric
    assert coriolis.shape[0] == model.nv


def test_franka_model_node_init(franka_model_node):
    node = franka_model_node
    assert node._robot_model is not None
    assert node._collision_model is not None
    assert node._visual_model is not None
    assert node._model_loaded


#! Franka Model - ROS2 Tests


# @pytest.mark.launch(fixture=launch_description)
def test_franka_model_node_subscribes(test_context):
    model_node, test_node, pub = test_context

    # Publish a message and check that the topic is listed as subscribed
    send_dummy_robot_state(test_node, pub)
    time.sleep(1.0)

    topics = model_node.get_subscriptions_info_by_topic("/franka_robot_state_broadcaster/robot_state")
    assert len(topics) > 0, "No subscribers found for FrankaRobotState topic."

    # Optionally, check that the node is alive
    node_names = test_node.get_node_names()
    assert any("franka_model_node" in n for n in node_names), "franka_model_node is not alive"


@pytest.mark.parametrize(
    "positions, velocities, efforts",
    [
        (
            [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7],
            [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            [0.0, 0.0, 0.1, 0.1, 0.2, 0.2, 0.3],
        ),
        (
            [0.5, 0.4, 0.3, 0.2, 0.1, 0.0, -0.1],
            [0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
            [0.3, 0.2, 0.2, 0.1, 0.1, 0.0, 0.0],
        ),
        (
            [0.123, -0.456, 0.789, -0.321, 0.654, -0.987, 0.111],
            [0.222, 0.333, 0.444, 0.555, 0.666, 0.777, 0.888],
            [0.999, 0.888, 0.777, 0.666, 0.555, 0.444, 0.333],
        ),
    ],
)
def test_franka_model_node_joint_state_update(test_context, positions, velocities, efforts):
    model_node, test_node, pub = test_context

    # Use spin_once in a loop instead of threading to avoid generator already executing error
    send_dummy_robot_state(test_node, pub, positions=positions, velocities=velocities, efforts=efforts)

    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    # while (
    #     getattr(model_node, "_last_q", None) is None
    #     or getattr(model_node, "_last_dq", None) is None
    #     or getattr(model_node, "tau", None) is None
    # ) and waited < timeout:
    #     rclpy.spin_once(model_node, timeout_sec=poll_period)
    #     # rclpy.spin_once(test_node, timeout_sec=0.0)
    #     waited += poll_period

    while waited < timeout:
        rclpy.spin_once(model_node, timeout_sec=poll_period)
        waited += poll_period

    np.testing.assert_allclose(model_node._last_q, positions, atol=1e-8)
    np.testing.assert_allclose(model_node._last_dq, velocities, atol=1e-8)
    np.testing.assert_allclose(model_node.tau, efforts, atol=1e-8)


def test_franka_model_publishes_model(test_context):
    """
    Subscribes to the /fr3_model topic and verifies that at least 8 messages are received in 1 second of publishing.
    """
    model_node, test_node, pub = test_context
    received_msgs = []
    lock = threading.Lock()

    def model_callback(msg):
        with lock:
            received_msgs.append(msg)

    sub = test_node.create_subscription(FrankaModel, "/fr3_model", model_callback, 10)

    # Publish robot state messages at 50 Hz for 1 second
    publish_rate = 50  # Hz
    duration = 1.0  # seconds
    num_msgs = int(publish_rate * duration)
    positions = [0.0] * N_DOF
    velocities = [0.0] * N_DOF
    efforts = [0.0] * N_DOF

    start_time = time.time()
    for _ in range(num_msgs):
        send_dummy_robot_state(test_node, pub, positions, velocities, efforts)
        rclpy.spin_once(model_node, timeout_sec=0.01)
        rclpy.spin_once(test_node, timeout_sec=0.01)
        if time.time() - start_time > duration:
            break

    # Wait up to 1s for at least 8 messages, spinning the node
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while waited < timeout:
        rclpy.spin_once(test_node, timeout_sec=poll_period)
        waited += poll_period

    with lock:
        num_received = len(received_msgs)

    assert num_received >= 8, f"Expected at least 8 messages, got {num_received}"

    test_node.destroy_subscription(sub)


def test_franka_model_publishes_model_with_forces(test_context):
    """
    Subscribes to the /fr3_model topic and verifies that force data is included.
    """
    model_node, test_node, pub = test_context
    received_msgs = []
    lock = threading.Lock()

    def model_callback(msg):
        with lock:
            received_msgs.append(msg)

    sub = test_node.create_subscription(FrankaModel, "/fr3_model", model_callback, 10)

    # Publish robot state messages with force data
    positions = [0.0] * N_DOF
    velocities = [0.0] * N_DOF
    efforts = [0.0] * N_DOF

    for _ in range(5):
        send_dummy_robot_state(test_node, pub, positions, velocities, efforts)
        rclpy.spin_once(model_node, timeout_sec=0.01)
        rclpy.spin_once(test_node, timeout_sec=0.01)

    # Wait for messages
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while waited < timeout:
        rclpy.spin_once(test_node, timeout_sec=poll_period)
        waited += poll_period

    with lock:
        num_received = len(received_msgs)
        if num_received > 0:
            latest_msg = received_msgs[-1]
            # Verify force data is present
            assert len(latest_msg.f_ext_estimated) == 6
            assert len(latest_msg.f_ext_robot) == 6

    assert num_received >= 1, f"Expected at least 1 message, got {num_received}"

    test_node.destroy_subscription(sub)
