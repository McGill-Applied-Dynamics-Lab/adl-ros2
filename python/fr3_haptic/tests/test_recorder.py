"""RobotStateRecorder, StreamLog and flatten_message, without ROS spinning."""

from __future__ import annotations

import numpy as np
import pytest
from fr3_haptic import recorder as rec
from fr3_haptic.recorder import RecordedTopic, RobotStateRecorder, StreamLog, flatten_message
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState


def robot_state(q0: float = 0.1, stamp_ns: int = 0) -> FrankaRobotState:
    m = FrankaRobotState()
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    m.header.frame_id = "fr3_link0"
    for js in (m.measured_joint_state, m.desired_joint_state, m.measured_joint_motor_state, m.tau_ext_hat_filtered):
        js.name = [f"fr3_joint{i}" for i in range(1, 8)]
        js.position = [q0 + i for i in range(7)]
        js.velocity = [0.2] * 7
        js.effort = [0.3] * 7
    m.o_f_ext_hat_k.wrench.force.z = -4.5
    m.robot_mode = FrankaRobotState.ROBOT_MODE_MOVE
    return m


def test_flatten_keeps_every_number_and_skips_the_rest():
    cols = dict(flatten_message(robot_state()))
    assert cols["measured_joint_state.position_3"] == pytest.approx(3.1)
    assert cols["o_f_ext_hat_k.wrench.force.z"] == -4.5
    assert cols["robot_mode"] == float(FrankaRobotState.ROBOT_MODE_MOVE)
    assert "dtau_j_6" in cols and "o_t_ee.pose.orientation.w" in cols
    assert "current_errors.joint_position_limits_violation" in cols  # booleans as 0/1
    assert not any("header" in c or "frame_id" in c or c.endswith(".name_0") for c in cols)
    assert len(cols) > 250


def test_flatten_skips_covariance():
    cols = dict(flatten_message(Odometry()))
    assert "pose.pose.position.x" in cols and "twist.twist.angular.z" in cols
    assert not any("covariance" in c for c in cols)


def test_stream_log_layout_capacity_and_name_matching():
    log = StreamLog(capacity=3)
    log.record(1.0, 10.0, [("a", 1.0), ("b", 2.0)])
    log.record(2.0, 11.0, [("b", 5.0)])  # different layout: matched by name, missing -> NaN
    log.record(3.0, 12.0, [("a", 7.0), ("b", 8.0)])
    log.record(4.0, 13.0, [("a", 0.0), ("b", 0.0)])  # past capacity
    assert log.columns == ("t_s", "rx_s", "a", "b")
    np.testing.assert_allclose(log.column("a"), [1.0, np.nan, 7.0])
    np.testing.assert_allclose(log.column("b"), [2.0, 5.0, 8.0])
    assert (log.rows, log.dropped) == (3, 1)


class FakeNode:
    def __init__(self) -> None:
        self.subs: dict[str, tuple] = {}
        self.destroyed: list = []

    def create_subscription(self, msg_type, topic, callback, qos, raw=False):
        assert raw, "the recorder must subscribe raw: decoding is what it avoids"
        self.subs[topic] = (msg_type, callback)
        return topic

    def destroy_subscription(self, sub):
        self.destroyed.append(sub)


class FakeLogger:
    def __init__(self) -> None:
        self.samples: list[tuple[str, dict, float]] = []

    def log_sample(self, stream, values, timestamp_s=None):
        self.samples.append((stream, values, timestamp_s))


def test_recorder_decimates_before_decoding(monkeypatch):
    decoded = []
    real = rec.deserialize_message

    def counting(raw, msg_type):
        decoded.append(msg_type)
        return real(raw, msg_type)

    monkeypatch.setattr(rec, "deserialize_message", counting)
    now = [0.0]
    node = FakeNode()
    topics = (
        RecordedTopic("/rs", FrankaRobotState, "robot_state"),
        RecordedTopic("/tw", WrenchStamped, "task_wrench"),
    )
    recorder = RobotStateRecorder(node, rate_hz=50.0, seconds=1.0, topics=topics, clock=lambda: now[0])
    _, cb = node.subs["/rs"]
    for k in range(100):  # 1 kHz for 0.1 s
        now[0] = 100.0 + k * 1e-3
        cb(serialize_message(robot_state(q0=float(k), stamp_ns=k * 1_000_000)))
    log = recorder.logs["robot_state"]
    assert log.rows == 5  # 50 Hz over 0.1 s
    assert len(decoded) == 5  # the other 95 were never decoded
    np.testing.assert_allclose(log.column("t_s"), [0.0, 0.02, 0.04, 0.06, 0.08])
    np.testing.assert_allclose(log.column("measured_joint_state.position_0"), [0, 20, 40, 60, 80])
    assert recorder.logs["task_wrench"].rows == 0

    logger = FakeLogger()
    recorder.to_logger(logger, t0=100.03)  # rows received before t0 (the setup) are left out
    assert [ts for s, _, ts in logger.samples if s == "robot_state"] == pytest.approx([0.01, 0.03, 0.05])
    logger = FakeLogger()
    recorder.to_logger(logger, t0=100.0)
    stream, values, ts = logger.samples[1]
    assert stream == "robot_state" and ts == pytest.approx(0.02)
    assert values["t_s"] == pytest.approx(0.02) and values["measured_joint_state.position_0"] == 20.0
    assert "rx_s" not in values

    recorder.close()
    assert set(node.destroyed) == {"/rs", "/tw"}


def test_recorder_validates_rate():
    with pytest.raises(ValueError):
        RobotStateRecorder(FakeNode(), rate_hz=0.0, seconds=1.0)


def test_default_topics_cover_robot_and_controller_state():
    streams = {t.stream: t for t in rec.DEFAULT_TOPICS}
    assert streams["robot_state"].msg_type is FrankaRobotState
    assert streams["joint_torques_cmd"].msg_type is JointState
    assert {"task_error", "ee_state", "task_wrench"} <= set(streams)
