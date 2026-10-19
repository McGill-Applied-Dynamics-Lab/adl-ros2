"""Robot target streams: reference (pose + feedforward) vs jog (no robot needed)."""

import threading
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from arm_client.robot import Pose, Robot, Twist
from arm_client.robot_config import FR3Config
from scipy.spatial.transform import Rotation

from arm_client import robot as robot_module

HOME = Pose(np.array([0.4, 0.0, 0.3]), Rotation.from_quat([1.0, 0.0, 0.0, 0.0]))


@pytest.fixture
def robot():
    rclpy.init()
    node = rclpy.create_node("test_target_streams")
    r = Robot.__new__(Robot)
    r.node = node
    r.config = FR3Config()
    r._streaming = False
    r._target_publish_timers = [MagicMock() for _ in range(4)]
    r._trajectory_mode_active = False
    r._target_pose = r._target_twist = r._target_accel = r._target_wrench = r._target_jog = None
    r._q_target = r._q_current = None
    r._jog_holds_pose = False
    r._jog_lock = threading.Lock()
    for name in ("pose", "twist", "accel", "wrench", "jog"):
        setattr(r, f"_target_{name}_publisher", MagicMock())
    r._current_pose = HOME.copy()
    r.controller_switcher_client = MagicMock()
    r.controller_switcher_client.get_active_controller.return_value = "fr3_pose_controller"
    yield r
    node.destroy_node()
    rclpy.shutdown()


def published(publisher):
    return [c.args[0] for c in publisher.publish.call_args_list]


def jog_velocities(r):
    return [np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]) for m in published(r._target_jog_publisher)]


# --- Reference: pose and feedforward published together ---


def test_reference_publishes_pose_and_feedforward_together(robot):
    robot.set_target(pose=HOME, twist=Twist(np.array([0.1, 0, 0]), np.zeros(3)), accel=Twist(np.array([0, 0, 1.0]), np.zeros(3)))
    robot._callback_publish_reference()

    assert len(published(robot._target_pose_publisher)) == 1
    (twist_msg,) = published(robot._target_twist_publisher)
    (accel_msg,) = published(robot._target_accel_publisher)
    assert twist_msg.twist.linear.x == pytest.approx(0.1)
    assert accel_msg.accel.linear.z == pytest.approx(1.0)
    assert accel_msg.header.frame_id == robot.config.base_frame


def test_set_target_without_feedforward_stops_publishing_it(robot):
    robot.set_target(pose=HOME, twist=Twist(np.array([0.1, 0, 0]), np.zeros(3)))
    robot.set_target(pose=HOME)
    robot._callback_publish_reference()

    assert len(published(robot._target_pose_publisher)) == 1
    robot._target_twist_publisher.publish.assert_not_called()
    robot._target_accel_publisher.publish.assert_not_called()


def test_measured_twist_does_not_seed_a_feedforward(robot):
    robot._callback_current_twist(robot._twist_to_twist_msg(Twist(np.array([0.2, 0, 0]), np.zeros(3))))
    assert robot._target_twist is None


def test_reseed_clears_feedforward_and_jog(robot):
    robot.set_target(pose=HOME, twist=Twist(np.ones(3), np.zeros(3)), accel=Twist(np.ones(3), np.zeros(3)))
    robot._reseed_targets()
    assert robot._target_twist is None and robot._target_accel is None

    robot.jog(linear=[0, 0, 0.01])
    robot._reseed_targets()
    assert not robot.jogging and not robot._jog_holds_pose


def test_publish_target_accel(robot):
    robot.publish_target(accel=Twist(np.array([0, 0.5, 0]), np.zeros(3)))
    (msg,) = published(robot._target_accel_publisher)
    assert msg.accel.linear.y == pytest.approx(0.5)


# --- Jog: exclusive with the pose reference ---


def test_jog_publishes_at_once_and_pauses_the_reference(robot):
    robot.set_target(pose=HOME)
    robot.jog(linear=[0, 0, 0.02])

    np.testing.assert_allclose(jog_velocities(robot), [[0, 0, 0.02]])
    robot._callback_publish_target_jog()
    robot._callback_publish_reference()
    assert len(jog_velocities(robot)) == 2
    robot._target_pose_publisher.publish.assert_not_called()
    assert robot.jogging


def test_stop_jog_sends_one_zero_and_keeps_holding(robot):
    robot.set_target(pose=HOME)
    robot.jog(linear=[0, 0, 0.02])
    robot.stop_jog()
    robot.stop_jog()  # no second zero
    robot._callback_publish_target_jog()
    robot._callback_publish_reference()

    velocities = jog_velocities(robot)
    assert len(velocities) == 2
    np.testing.assert_allclose(velocities[-1], 0.0)
    assert not robot.jogging
    # The held target would pull the robot back: still not republished after the stop
    robot._target_pose_publisher.publish.assert_not_called()


@pytest.mark.parametrize("take_over", ["set_target", "publish_target"])
def test_pose_target_takes_over_from_jog(robot, take_over):
    robot.jog(angular=[0, 0, 0.1])
    if take_over == "set_target":
        robot.set_target(pose=HOME)
    else:
        robot.publish_target(pose=HOME)
    robot._callback_publish_target_jog()
    robot._callback_publish_reference()

    assert not robot.jogging
    assert len(jog_velocities(robot)) == 1  # only the jog() itself, no stop command
    assert len(published(robot._target_pose_publisher)) >= 1


def test_jog_checks_the_controller_only_when_starting(robot):
    robot.jog(linear=[0.01, 0, 0])
    robot.jog(linear=[0.02, 0, 0])
    assert robot.controller_switcher_client.get_active_controller.call_count == 1


@pytest.mark.parametrize("active", ["osc_controller", None])
def test_jog_rejected_by_other_controllers(robot, active):
    robot.controller_switcher_client.get_active_controller.return_value = active
    with pytest.raises(RuntimeError, match="jog needs"):
        robot.jog(linear=[0, 0, 0.01])
    robot._target_jog_publisher.publish.assert_not_called()
    assert not robot._jog_holds_pose


def test_jog_rejects_bad_shapes(robot):
    with pytest.raises(ValueError):
        robot.jog(linear=[0, 0.01])


def test_reset_targets_stops_the_jog(robot):
    robot.jog(linear=[0, 0, 0.01])
    robot.reset_targets()
    assert not robot.jogging
    np.testing.assert_allclose(jog_velocities(robot)[-1], 0.0)


# --- move_to with fr3_pose_controller: one goal, wait for arrival ---


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now

    def sleep(self, dt):
        self.now += dt


@pytest.fixture
def clock(monkeypatch):
    fake = FakeClock()
    monkeypatch.setattr(robot_module.time, "monotonic", fake.monotonic)
    monkeypatch.setattr(robot_module.time, "sleep", fake.sleep)
    return fake


def test_move_to_fr3_pose_sends_the_goal_once_and_waits(robot, clock):
    goal = Pose(HOME.position + np.array([0, 0, 0.05]), HOME.orientation)
    arrive_at = clock.now + 0.8

    def sleep(dt):
        clock.now += dt
        if clock.now >= arrive_at:
            robot._current_pose = goal.copy()

    robot_module.time.sleep = sleep
    robot.move_to(pose=goal)

    assert clock.now == pytest.approx(arrive_at, abs=0.02)
    np.testing.assert_allclose(robot._target_pose.position, goal.position)
    robot._target_twist_publisher.publish.assert_not_called()


def test_move_to_fr3_pose_warns_when_not_reached(robot, clock):
    robot.node.get_logger = MagicMock()
    goal = Pose(HOME.position + np.array([0, 0, 0.05]), HOME.orientation)
    robot.move_to(pose=goal, speed=0.05)

    # time_to_move = 1 s -> waits 2 * 1 + 5 s
    assert clock.now - 10.0 == pytest.approx(7.0, abs=0.02)
    robot.node.get_logger.return_value.warn.assert_called_once()
