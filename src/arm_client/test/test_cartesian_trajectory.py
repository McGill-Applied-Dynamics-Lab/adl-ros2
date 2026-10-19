"""execute_cartesian_traj / wait_for_trajectory_completion: targets and timing (no robot needed)."""

from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from arm_client.robot import Pose, Robot, Twist
from arm_client.robot_config import FR3Config
from scipy.spatial.transform import Rotation

from arm_client import robot as robot_module


class FakeClock:
    def __init__(self):
        self.now = 100.0

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


@pytest.fixture
def bare_robot():
    rclpy.init()
    node = rclpy.create_node("test_cartesian_trajectory")
    robot = Robot.__new__(Robot)
    robot.node = node
    robot.config = FR3Config()
    robot._trajectory_mode_active = False
    robot._trajectory_start_time = None
    robot._target_pose = Pose(np.array([0.3, 0.0, 0.5]), Rotation.identity())  # the pose before the trajectory
    robot._target_trajectory_publisher = MagicMock()
    yield robot
    node.destroy_node()
    rclpy.shutdown()


def plunge(depth=0.1, n=5, duration=2.0):
    times = list(np.linspace(0.0, duration, n))
    waypoints = [(Pose(np.array([0.3, 0.0, 0.5 - depth * t / duration]), Rotation.identity()), Twist(np.zeros(3), np.zeros(3))) for t in times]
    return waypoints, times


def test_republishing_resumes_on_the_final_waypoint(bare_robot, clock):
    waypoints, times = plunge()
    bare_robot.execute_cartesian_traj(waypoints, times)

    bare_robot._target_trajectory_publisher.publish.assert_called_once()
    assert bare_robot._trajectory_mode_active
    np.testing.assert_allclose(bare_robot._target_pose.position, waypoints[-1][0].position)


def test_wait_counts_from_the_send_and_paces_the_loop(bare_robot, clock):
    waypoints, times = plunge(duration=2.0)
    bare_robot.execute_cartesian_traj(waypoints, times)

    calls = 0
    while bare_robot.wait_for_trajectory_completion(2.0, timeout_margin=0.5):
        calls += 1
    # 2.5 s at 100 Hz, measured from the send: not a hot loop, not cut short
    assert calls == pytest.approx(250, abs=2)
    assert not bare_robot._trajectory_mode_active
    np.testing.assert_allclose(bare_robot._target_pose.position, waypoints[-1][0].position)


def test_wait_without_a_trajectory_returns_at_once(bare_robot, clock):
    assert bare_robot.wait_for_trajectory_completion(2.0) is False


def test_an_abandoned_wait_does_not_end_the_next_trajectory(bare_robot, clock):
    waypoints, times = plunge(duration=2.0)
    bare_robot.execute_cartesian_traj(waypoints, times)
    assert bare_robot.wait_for_trajectory_completion(2.0)  # called once, then abandoned
    clock.now += 60.0

    bare_robot.execute_cartesian_traj(waypoints, times)
    assert bare_robot.wait_for_trajectory_completion(2.0)  # the new trajectory is running
