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


def make_franka_model_msg(n=9):
    msg = FrankaModel()
    # Fill with dummy data
    msg.mass_matrix = [1.0 if i == j else 0.0 for i in range(n) for j in range(n)]  # Identity matrix
    msg.coriolis = [0.1] * n
    msg.tau = [0.2] * n
    msg.ai = [0.3] * n
    msg.ai_dot_q_dot = [0.4]
    msg.n = n
    return msg


def test_rim_node_receives_and_stores_matrices(rim_node: FrankaRIMNode, test_node: Node):
    pub = test_node.create_publisher(FrankaModel, "/fr3_model", 10)
    msg = make_franka_model_msg()
    pub.publish(msg)
    # Spin both nodes to process the message
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while rim_node._M is None and waited < timeout:
        rclpy.spin_once(rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=0.0)
        waited += poll_period

    assert rim_node._M is not None
    assert rim_node._c is not None
    assert rim_node._tau is not None
    assert rim_node._Ai is not None
    assert rim_node._Ai_dot_q_dot is not None

    # Check matrix shapes
    n = 9
    assert rim_node._M.shape == (n, n)
    assert rim_node._c.shape == (n,)
    assert rim_node._tau.shape == (n,)
    assert rim_node._Ai.shape == (1, n)
    assert rim_node._Ai_dot_q_dot.shape == (1,)


@pytest.mark.xfail(reason="RIM computation not implemented yet")
def test_rim_node_computes_rim(rim_node, test_node):
    pub = test_node.create_publisher(FrankaModel, "/fr3_model", 10)
    msg = make_franka_model_msg()
    pub.publish(msg)
    # Spin to process
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while rim_node._M is None and waited < timeout:
        rclpy.spin_once(rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=0.0)
        waited += poll_period

    # Call compute_rim directly
    rim = rim_node._compute_rim()
    assert rim is not None
    assert rim.shape == (1, 1)
    # For identity M and Ai=[0.3,...], rim should be sum(ai^2)
    import numpy as np

    expected = np.sum(np.array([0.3] * 7) ** 2)
    assert abs(rim[0, 0] - expected) < 1e-6
