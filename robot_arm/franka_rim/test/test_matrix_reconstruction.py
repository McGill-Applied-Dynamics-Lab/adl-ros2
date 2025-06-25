import pytest
import rclpy
import time
import numpy as np
from rclpy.node import Node
from arm_interfaces.msg import FrankaModel
from franka_rim.franka_model_node import FrankaModelNode
from franka_rim.franka_rim_node import FrankaRIMNode

N_DOF = 7


@pytest.fixture(scope="module")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def model_node(rclpy_init_shutdown):
    node = FrankaModelNode()
    yield node
    node.destroy_node()


@pytest.fixture
def rim_node(rclpy_init_shutdown):
    node = FrankaRIMNode()
    yield node
    node.destroy_node()


@pytest.fixture
def test_node(rclpy_init_shutdown):
    node = rclpy.create_node("test_rim_matrix_reconstruction")
    yield node
    node.destroy_node()


def test_matrix_reconstruction_between_nodes(model_node: FrankaModelNode, rim_node: FrankaRIMNode, test_node):
    # Set known values in FrankaModelNode
    n = N_DOF
    M = np.arange(n * n).reshape((n, n)).astype(float)
    c = np.arange(n).astype(float) + 0.1
    tau = np.arange(n).astype(float) + 10
    Ai = (np.arange(n).astype(float) + 20).reshape(1, n)
    Ai_dot = (np.arange(n).astype(float) + 30).reshape(1, n)
    dq = np.arange(n).astype(float) + 40
    Ai_dot_q_dot = Ai_dot @ dq
    f_ext_estimated = np.zeros(6)

    msg = model_node._build_model_message(M, c, tau, Ai, Ai_dot_q_dot, f_ext_estimated)

    pub = test_node.create_publisher(FrankaModel, "/fr3_model", 10)
    pub.publish(msg)

    # Spin rim_node to receive the message
    timeout = 1.0
    poll_period = 0.05
    waited = 0.0
    while (
        rim_node._M is None
        or rim_node._c is None
        or rim_node._tau is None
        or rim_node._Ai is None
        or rim_node._Ai_dot_q_dot is None
    ) and waited < timeout:
        rclpy.spin_once(rim_node, timeout_sec=poll_period)
        rclpy.spin_once(test_node, timeout_sec=0.0)
        waited += poll_period

    # Check that the matrices are reconstructed correctly
    np.testing.assert_allclose(rim_node._M, M)
    np.testing.assert_allclose(rim_node._c, c)
    np.testing.assert_allclose(rim_node._tau, tau)
    np.testing.assert_allclose(rim_node._Ai, Ai)
    np.testing.assert_allclose(rim_node._Ai_dot_q_dot, (Ai_dot @ dq))
