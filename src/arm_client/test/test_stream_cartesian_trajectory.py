"""Robot.stream_cartesian_traj: interpolation, timing and cleanup (no robot needed)."""

import gc
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from arm_client.robot import Pose, Robot, Twist
from arm_client.robot_config import FR3Config
from scipy.spatial.transform import Rotation

from arm_client import robot as robot_module

RATE = 500.0


class FakeClock:
    def __init__(self):
        self.now = 50.0

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
    node = rclpy.create_node("test_stream_cartesian_trajectory")
    robot = Robot.__new__(Robot)
    robot.node = node
    robot.config = FR3Config()
    robot._streaming = False
    robot._target_publish_timers = [MagicMock() for _ in range(4)]
    robot._trajectory_mode_active = False
    robot._target_pose = robot._target_twist = robot._target_wrench = None
    robot._target_pose_publisher = MagicMock()
    robot._target_twist_publisher = MagicMock()
    robot._target_wrench_publisher = MagicMock()
    robot._current_pose = Pose(np.array([0.4, 0.0, 0.5]), Rotation.identity())
    robot.controller_switcher_client = MagicMock()
    robot.controller_switcher_client.get_active_controller.return_value = "osc_controller"
    yield robot
    node.destroy_node()
    rclpy.shutdown()


def plunge(depth=0.1, duration=2.0):
    """Down by depth and back, with a yaw of 0.2 rad at the bottom."""
    times = [0.0, duration / 2, duration]
    z = [0.5, 0.5 - depth, 0.5]
    yaw = [0.0, 0.2, 0.0]
    return [(Pose(np.array([0.4, 0.0, zk]), Rotation.from_euler("z", yk)), Twist(np.zeros(3), np.zeros(3))) for zk, yk in zip(z, yaw)], times


def test_streams_through_the_waypoints_at_the_rate(bare_robot, clock):
    waypoints, times = plunge()
    log = []
    stats = bare_robot.stream_cartesian_traj(waypoints, times, rate_hz=RATE, on_tick=lambda t, p, v: log.append((t, p, v)))

    assert stats["ticks"] == pytest.approx(times[-1] * RATE + 1, abs=2)
    t = np.array([s[0] for s in log])
    z = np.array([s[1].position[2] for s in log])
    vz = np.array([s[2].linear[2] for s in log])
    # Through the waypoints, at rest at both ends, velocity consistent with the positions
    assert z[0] == pytest.approx(0.5) and z[-1] == pytest.approx(0.5)
    assert z[np.argmin(np.abs(t - 1.0))] == pytest.approx(0.4, abs=1e-4)
    assert vz[0] == pytest.approx(0.0) and vz[-1] == pytest.approx(0.0, abs=1e-6)
    np.testing.assert_allclose(np.gradient(z, t)[5:-5], vz[5:-5], atol=2e-3)
    # Orientation interpolated, with its angular velocity as feedforward
    yaw = np.array([s[1].orientation.as_euler("xyz")[2] for s in log])
    assert yaw[np.argmin(np.abs(t - 1.0))] == pytest.approx(0.2, abs=1e-3)
    assert log[10][2].angular[2] == pytest.approx(0.2 / 1.0)


def test_ends_holding_the_final_waypoint_at_rest(bare_robot, clock):
    waypoints, times = plunge()
    bare_robot.stream_cartesian_traj(waypoints, times, rate_hz=RATE)

    np.testing.assert_allclose(bare_robot._target_pose.position, waypoints[-1][0].position, atol=1e-9)
    np.testing.assert_allclose(bare_robot._target_twist.linear, 0.0)
    assert bare_robot._streaming is False  # republishing resumed (on the final waypoint)
    assert gc.isenabled()


def test_a_late_tick_does_not_delay_the_trajectory(bare_robot, clock):
    waypoints, times = plunge()
    log = []

    def slow_tick(t, pose, twist):
        log.append(t)
        if len(log) == 100:
            clock.now += 0.3  # a 300 ms stall in the caller

    bare_robot.stream_cartesian_traj(waypoints, times, rate_hz=RATE, on_tick=slow_tick)
    assert log[-1] == pytest.approx(times[-1], abs=1e-3)  # still ends on time
    # The tick after the stall goes out at once, at the current time (no burst of missed ticks)
    assert max(np.diff(log)) == pytest.approx(0.3, abs=1e-3)
    assert len(log) == pytest.approx((times[-1] - 0.3) * RATE + 1, abs=3)


def test_interrupted_stream_holds_the_last_target(bare_robot, clock):
    waypoints, times = plunge()

    def stop(t, pose, twist):
        if t > 0.5:
            raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        bare_robot.stream_cartesian_traj(waypoints, times, rate_hz=RATE, on_tick=stop)
    assert bare_robot._target_pose.position[2] < 0.5  # mid-plunge, not jumped to the end
    np.testing.assert_allclose(bare_robot._target_twist.linear, 0.0)
    assert bare_robot._streaming is False
    assert gc.isenabled()


def test_starts_from_the_current_pose_when_the_first_time_is_after_zero(bare_robot, clock):
    log = []
    target = Pose(np.array([0.4, 0.05, 0.5]), Rotation.identity())
    bare_robot.stream_cartesian_traj([target], [1.0], rate_hz=RATE, on_tick=lambda t, p, v: log.append(p.position.copy()))
    np.testing.assert_allclose(log[0], [0.4, 0.0, 0.5], atol=1e-9)
    np.testing.assert_allclose(log[-1], [0.4, 0.05, 0.5], atol=1e-9)


def test_refuses_a_joint_controller(bare_robot, clock):
    bare_robot.controller_switcher_client.get_active_controller.return_value = "joint_trajectory_controller"
    with pytest.raises(RuntimeError):
        bare_robot.stream_cartesian_traj(*plunge())
    assert bare_robot._streaming is False


def test_rejects_bad_times(bare_robot, clock):
    waypoints, _ = plunge()
    with pytest.raises(ValueError):
        bare_robot.stream_cartesian_traj(waypoints, [0.0, 1.0, 1.0])
