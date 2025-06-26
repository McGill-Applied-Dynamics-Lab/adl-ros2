import numpy as np
import pytest
import rclpy
import time
import threading
from rclpy.node import Node
from arm_interfaces.msg import FrankaModel
from franka_rim.franka_rim_node import FrankaRIMNode


@pytest.fixture(scope="module")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def rim_node(rclpy_init_shutdown):
    node = FrankaRIMNode()
    yield node
    node.destroy_node()


@pytest.fixture
def test_node(rclpy_init_shutdown):
    node = rclpy.create_node("test_rim_node")
    yield node
    node.destroy_node()


def make_franka_model_msg(n=7):
    msg = FrankaModel()
    # Fill with dummy data
    msg.n = n
    msg.q = [1.0 + i for i in range(n)]  # Joint positions
    msg.q_dot = [2.0 + i for i in range(n)]  #

    msg.mass_matrix = [1.0 if i == j else 0.0 for i in range(n) for j in range(n)]  # Identity matrix
    msg.coriolis = [0.1] * n
    msg.tau = [0.2] * n
    msg.ai = [0.3] * n
    msg.ai_dot_q_dot = [0.4]
    msg.fa = [0.5] * n  # Applied forces
    return msg


def test_rim_node_receives_and_stores_matrices(rim_node: FrankaRIMNode, test_node: Node):
    pub = test_node.create_publisher(FrankaModel, "/fr3_model", 10)
    msg = make_franka_model_msg()
    pub.publish(msg)

    # Spin both nodes to process the message
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while rim_node.M is None and waited < timeout:
        rclpy.spin_once(rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=0.0)
        waited += poll_period

    assert rim_node.q is not None
    assert rim_node.q_dot is not None
    assert rim_node.M is not None
    assert rim_node.c is not None
    assert rim_node.tau is not None
    assert rim_node.Ai is not None
    assert rim_node.Ai_dot_q_dot is not None
    assert rim_node.fa is not None

    # Check matrix shapes
    n = 7
    assert rim_node.q.shape == (n,)
    assert rim_node.q_dot.shape == (n,)
    assert rim_node.M.shape == (n, n)
    assert rim_node.c.shape == (n,)
    assert rim_node.tau.shape == (n,)
    assert rim_node.Ai.shape == (1, n)
    assert rim_node.Ai_dot_q_dot.shape == (1,)
    assert rim_node.fa.shape == (n,)


def test_rim_node_computes_rim(rim_node, test_node):
    pub = test_node.create_publisher(FrankaModel, "/fr3_model", 10)
    msg = make_franka_model_msg()
    pub.publish(msg)
    # Spin to process
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while rim_node.M is None and waited < timeout:
        rclpy.spin_once(rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=0.0)
        waited += poll_period

    # Call compute_rim directly
    rim_node._compute_rim()

    rim_msg = rim_node._rim_msg

    assert rim_msg is not None, "No RIM message returned"

    # Check size of the RIM message
    m = rim_msg.m

    M_eff = np.array(rim_msg.effective_mass).reshape((m, m))
    f_eff = np.array(rim_msg.effective_force)
    phi_dot = np.array(rim_msg.interface_velocity)

    interface_stiffness = rim_msg.interface_stiffness
    interface_damping = rim_msg.interface_damping

    # Interface stiffness and damping
    assert interface_stiffness != 0.0
    assert interface_damping != 0.0

    assert M_eff.shape == (m, m)

    # Effective force vector (3x1)
    assert f_eff.shape == (m,)

    # Interface velocity (3x1)
    assert phi_dot.shape == (m,)
