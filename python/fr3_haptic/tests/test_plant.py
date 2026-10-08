"""FR3Plant, FR3System and Robot streaming mode, without hardware."""

from __future__ import annotations

import numpy as np
import pytest
from arm_client.robot import Pose, Robot, Twist
from fr3_haptic.plant import FR3Plant, FR3System
from geometry_msgs.msg import WrenchStamped
from haptic_teleop import PlantState
from nav_msgs.msg import Odometry
from pyrim import InterfaceFrame
from scipy.spatial.transform import Rotation

EE_TOPIC = "/fr3/osc/ee_state"
WRENCH_TOPIC = "/fr3/osc/task_wrench"


class FakeNode:
    def __init__(self) -> None:
        self.callbacks: dict[str, object] = {}
        self.destroyed: list[object] = []

    def create_subscription(self, msg_type, topic, callback, qos):
        self.callbacks[topic] = callback
        return topic

    def destroy_subscription(self, sub) -> None:
        self.destroyed.append(sub)


class FakeRobot:
    def __init__(self) -> None:
        self.node = FakeNode()
        self.streaming = False
        self.published: list[dict] = []

    def set_target_streaming(self, enabled: bool) -> None:
        self.streaming = enabled

    def publish_target(self, pose=None, twist=None, force=None, torque=None) -> None:
        self.published.append({"pose": pose, "twist": twist, "force": force})


HOLD = Pose(np.array([0.4, 0.1, 0.3]), Rotation.from_euler("x", 180, degrees=True))


def make_plant(**kwargs) -> tuple[FR3Plant, FakeRobot]:
    robot = FakeRobot()
    plant = FR3Plant(robot, InterfaceFrame.from_direction([0.0, 0.0, 1.0]), HOLD, node=robot.node, **kwargs)
    return plant, robot


def ee_msg(stamp_ns: int, position=(0.4, 0.1, 0.3), velocity=(0.0, 0.0, 0.0)) -> Odometry:
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
    return msg


def wrench_msg(stamp_ns: int, force=(0.0, 0.0, 0.0)) -> WrenchStamped:
    msg = WrenchStamped()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
    return msg


def feed(robot: FakeRobot, ee: Odometry | None = None, wrench: WrenchStamped | None = None) -> None:
    if ee is not None:
        robot.node.callbacks[EE_TOPIC](ee)
    if wrench is not None:
        robot.node.callbacks[WRENCH_TOPIC](wrench)


# ------------------------------------------------------------------ state side


def test_enters_streaming_and_subscribes():
    plant, robot = make_plant()
    assert robot.streaming
    assert set(robot.node.callbacks) == {EE_TOPIC, WRENCH_TOPIC}
    assert plant.sample is None
    assert plant.sample_age_s() == float("inf")


def test_pairs_halves_by_stamp_in_either_order():
    plant, robot = make_plant()
    stamp = 5_000_000_123
    feed(robot, ee=ee_msg(stamp, position=(0.4, 0.1, 0.25), velocity=(0.0, 0.0, -0.1)))
    assert plant.sample is None  # waiting for the wrench of the same tick
    feed(robot, wrench=wrench_msg(stamp, force=(1.0, 2.0, 3.0)))

    updates, sample = plant.sample
    assert updates == 1
    assert isinstance(sample, PlantState)
    np.testing.assert_allclose(sample.x_i, [0.25])
    np.testing.assert_allclose(sample.v_i, [-0.1])
    np.testing.assert_allclose(sample.lam, [-3.0])  # operator-should-feel sign
    np.testing.assert_allclose(sample.task_force, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(sample.ee_pose.position, [0.4, 0.1, 0.25])
    assert sample.t_s == pytest.approx(5.000000123)

    feed(robot, wrench=wrench_msg(stamp + 1_000_000), ee=ee_msg(stamp + 1_000_000))
    assert plant.sample[0] == 2
    assert plant.history == []  # record is off by default


def test_samples_are_published_to_a_mailbox():
    from haptic_teleop import PlantMailbox

    box = PlantMailbox(1)
    try:
        plant, robot = make_plant(mailbox=box)
        feed(robot, ee=ee_msg(7, position=(0.4, 0.1, 0.25)), wrench=wrench_msg(7, force=(0.0, 0.0, 3.0)))
        updates, s = box.sample
        assert updates == 1 and s.t_s == pytest.approx(7e-9)
        np.testing.assert_allclose((s.x_i, s.lam), ([0.25], [-3.0]))
        assert s.rx_s == plant.sample[1].rx_s  # same receive time as in-process
    finally:
        box.close()
        box.unlink()


def test_record_keeps_every_sample():
    plant, robot = make_plant(record=True)
    for k in range(5):
        feed(robot, ee=ee_msg(k), wrench=wrench_msg(k))
    assert [s.t_s for s in plant.history] == pytest.approx([k * 1e-9 for k in range(5)])


def test_unmatched_and_stale_halves_are_not_paired():
    plant, robot = make_plant()
    feed(robot, ee=ee_msg(1_000_000))
    feed(robot, wrench=wrench_msg(2_000_000))
    assert plant.sample is None
    # Completing tick 3 drops the leftovers of ticks 1 and 2
    feed(robot, ee=ee_msg(3_000_000), wrench=wrench_msg(3_000_000))
    assert plant.sample[0] == 1
    feed(robot, wrench=wrench_msg(1_000_000))
    assert plant.sample[0] == 1


def test_pending_buffer_is_bounded():
    plant, robot = make_plant()
    for k in range(100):
        feed(robot, ee=ee_msg(k * 1_000_000))
    assert len(plant._pending_ee) <= FR3Plant._MAX_PENDING


def test_decimation_keeps_every_tick_at_the_controller_rate_despite_jitter():
    plant, robot = make_plant(sample_period_s=0.001)
    rng = np.random.default_rng(0)
    stamps = np.cumsum(1_000_000 + rng.integers(-50_000, 50_000, 500))  # 1 kHz +- 50 us
    for k in stamps:
        feed(robot, ee=ee_msg(int(k)), wrench=wrench_msg(int(k)))
    assert plant.sample[0] == 500  # the old since-last-kept test kept ~60 % of these


def test_decimation_restarts_after_a_gap():
    plant, robot = make_plant(sample_period_s=0.02)
    for k in list(range(0, 100)) + list(range(300, 400)):  # 1 kHz ticks with a 200 ms hole
        feed(robot, ee=ee_msg(k * 1_000_000), wrench=wrench_msg(k * 1_000_000))
    assert plant.sample[0] == 10


def test_sample_period_decimates_controller_ticks():
    plant, robot = make_plant(sample_period_s=0.02)  # 50 Hz plant from 1 kHz ticks
    for k in range(100):  # 0.1 s of ticks
        stamp = k * 1_000_000
        feed(robot, ee=ee_msg(stamp), wrench=wrench_msg(stamp))
    assert plant.sample[0] == 5


# ------------------------------------------------------------------ command side


def test_aim_holds_free_axes_and_orientation():
    plant, robot = make_plant()
    plant.aim(np.array([0.2]), np.array([0.05]))
    cmd = robot.published[-1]
    np.testing.assert_allclose(cmd["pose"].position, [0.4, 0.1, 0.2])
    np.testing.assert_allclose(cmd["pose"].orientation.as_quat(), HOLD.orientation.as_quat())
    np.testing.assert_allclose(cmd["twist"].linear, [0.0, 0.0, 0.05])
    np.testing.assert_allclose(cmd["twist"].angular, [0.0, 0.0, 0.0])
    assert cmd["force"] is None


def test_command_sends_feedforward_along_interface():
    plant, robot = make_plant()
    plant.command(np.array([0.2]), f_ff=np.array([-4.0]))
    cmd = robot.published[-1]
    np.testing.assert_allclose(cmd["twist"].linear, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(cmd["force"], [0.0, 0.0, -4.0])


def test_interface_limits_clip_position_and_outward_velocity():
    plant, robot = make_plant(interface_limits=(0.1, 0.5))
    plant.aim(np.array([0.05]), np.array([-0.2]))
    cmd = robot.published[-1]
    np.testing.assert_allclose(cmd["pose"].position[2], 0.1)
    np.testing.assert_allclose(cmd["twist"].linear[2], 0.0)
    plant.aim(np.array([0.05]), np.array([0.2]))  # moving back inside is allowed
    np.testing.assert_allclose(robot.published[-1]["twist"].linear[2], 0.2)


def test_interface_limits_validated():
    with pytest.raises(ValueError):
        make_plant(interface_limits=(0.5, 0.1))


def test_freeze_holds_last_measured_position():
    plant, robot = make_plant()
    plant.freeze()  # no sample yet: hold pose
    np.testing.assert_allclose(robot.published[-1]["pose"].position, HOLD.position)
    plant.release()
    feed(robot, ee=ee_msg(1, position=(0.4, 0.1, 0.22)), wrench=wrench_msg(1))
    plant.freeze()
    assert plant.frozen
    cmd = robot.published[-1]
    np.testing.assert_allclose(cmd["pose"].position[2], 0.22)
    np.testing.assert_allclose(cmd["twist"].linear, 0.0)
    np.testing.assert_allclose(cmd["force"], 0.0)
    # Latched: the robot sagging does not move the frozen target
    feed(robot, ee=ee_msg(2, position=(0.4, 0.1, 0.20)), wrench=wrench_msg(2))
    plant.freeze()
    np.testing.assert_allclose(robot.published[-1]["pose"].position[2], 0.22)
    plant.release()
    assert not plant.frozen


def test_hold_pose_from_first_sample():
    robot = FakeRobot()
    plant = FR3Plant(robot, InterfaceFrame.from_direction([0.0, 0.0, 1.0]), node=robot.node)
    with pytest.raises(RuntimeError):
        plant.aim(np.array([0.2]), np.array([0.0]))
    plant.freeze()  # nothing to hold yet: no-op, no target published
    assert robot.published == []
    with pytest.raises(TimeoutError):
        plant.wait_for_sample(timeout_s=0.01)
    feed(robot, ee=ee_msg(1, position=(0.5, -0.1, 0.4)), wrench=wrench_msg(1))
    plant.set_hold_pose(plant.wait_for_sample().ee_pose)
    plant.aim(np.array([0.3]), np.array([0.0]))
    np.testing.assert_allclose(robot.published[-1]["pose"].position, [0.5, -0.1, 0.3])


def test_close_unsubscribes_and_leaves_streaming():
    plant, robot = make_plant()
    plant.close()
    assert not robot.streaming
    assert set(robot.node.destroyed) == {EE_TOPIC, WRENCH_TOPIC}


# ------------------------------------------------------------------ FR3System


def test_fr3_system_keeps_last_model():
    class Adapter:
        def __init__(self) -> None:
            self.value = None

        def compute(self):
            return self.value

    adapter = Adapter()
    system = FR3System(adapter)
    system.update()
    assert not system.ready
    with pytest.raises(RuntimeError):
        _ = system.model
    adapter.value = "model"
    system.update()
    adapter.value = None
    system.update()
    assert system.model == "model"


# ------------------------------------------------------------------ Robot streaming mode


class FakeTimer:
    def __init__(self) -> None:
        self.active = True

    def cancel(self) -> None:
        self.active = False

    def reset(self) -> None:
        self.active = True


class FakePublisher:
    def __init__(self) -> None:
        self.msgs = []

    def publish(self, msg) -> None:
        self.msgs.append(msg)


@pytest.fixture
def bare_robot():
    """A Robot with only what the streaming methods touch (no controller services)."""
    import rclpy
    from arm_client.robot_config import FR3Config

    rclpy.init()
    node = rclpy.create_node("test_streaming")
    robot = Robot.__new__(Robot)
    robot.node = node
    robot.config = FR3Config()
    robot._streaming = False
    robot._target_publish_timers = [FakeTimer() for _ in range(4)]
    robot._trajectory_mode_active = True
    robot._target_pose = robot._target_twist = robot._target_wrench = None
    robot._target_pose_publisher = FakePublisher()
    robot._target_twist_publisher = FakePublisher()
    robot._target_wrench_publisher = FakePublisher()
    yield robot
    node.destroy_node()
    rclpy.shutdown()


def test_robot_streaming_toggles_timers(bare_robot):
    bare_robot.set_target_streaming(True)
    assert bare_robot.streaming
    assert not any(t.active for t in bare_robot._target_publish_timers)
    bare_robot.set_target_streaming(False)
    assert all(t.active for t in bare_robot._target_publish_timers)


def test_robot_publish_target_publishes_and_stores(bare_robot):
    pose = Pose(np.array([0.4, 0.0, 0.3]), Rotation.identity())
    bare_robot.publish_target(pose=pose, twist=Twist(np.array([0.0, 0.0, 0.1]), np.zeros(3)))
    assert len(bare_robot._target_pose_publisher.msgs) == 1
    assert len(bare_robot._target_twist_publisher.msgs) == 1
    assert bare_robot._target_wrench_publisher.msgs == []
    assert bare_robot._trajectory_mode_active is False
    np.testing.assert_allclose(bare_robot._target_pose.position, pose.position)
    assert bare_robot._target_pose is not pose  # stored a copy

    bare_robot.publish_target(force=[0.0, 0.0, -2.0])
    msg = bare_robot._target_wrench_publisher.msgs[-1]
    assert (msg.wrench.force.z, msg.wrench.torque.z) == (-2.0, 0.0)
    assert len(bare_robot._target_pose_publisher.msgs) == 1
