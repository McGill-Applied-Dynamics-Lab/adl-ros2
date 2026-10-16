"""Tests of the waveguide demos logic (no robot, no Teensy).

    pixi run -e wg pytest experiments/waveguide/tests
"""

import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from touch import HoleTouch, TerminalTouchSource, TouchFrame  # noqa: E402
from wg_body_demo import BodyDemoConfig, JogLogic  # noqa: E402

CFG = BodyDemoConfig(
    direction_holes={"+x": (1, 2), "-x": (3,), "+y": (4,), "-y": (5,), "+z": (6,), "-z": (7,)},
    acceleration=1.0,
    max_speed=0.5,
)
DT = 0.01


def hold(logic: JogLogic, holes: set[int], n: int) -> list[bool]:
    return [logic.update(DT, frozenset(holes)) for _ in range(n)]


def test_hold_accelerates_and_release_keeps_velocity():
    logic = JogLogic(CFG)
    hold(logic, {1}, 10)
    np.testing.assert_allclose(logic.velocity, [0.1, 0.0, 0.0])
    hold(logic, set(), 10)
    np.testing.assert_allclose(logic.velocity, [0.1, 0.0, 0.0])


def test_holes_of_one_direction_count_once():
    logic = JogLogic(CFG)
    hold(logic, {1, 2}, 10)
    np.testing.assert_allclose(logic.velocity, [0.1, 0.0, 0.0])


def test_speed_is_capped():
    logic = JogLogic(CFG)
    hold(logic, {1, 4}, 1000)
    assert np.linalg.norm(logic.velocity) == pytest.approx(CFG.max_speed)


def test_opposite_touch_stops_until_release():
    logic = JogLogic(CFG)
    hold(logic, {1}, 10)
    assert hold(logic, {3}, 1) == [True]
    np.testing.assert_allclose(logic.velocity, 0.0)
    # Still touched: stays stopped, even on another direction
    assert hold(logic, {3, 4}, 10) == [False] * 10
    np.testing.assert_allclose(logic.velocity, 0.0)
    hold(logic, set(), 1)
    hold(logic, {3}, 10)
    np.testing.assert_allclose(logic.velocity, [-0.1, 0.0, 0.0])


def test_perpendicular_touch_adds_velocity():
    logic = JogLogic(CFG)
    hold(logic, {1}, 10)
    assert hold(logic, {4}, 10) == [False] * 10
    np.testing.assert_allclose(logic.velocity, [0.1, 0.1, 0.0])


def test_unmapped_holes_are_ignored():
    logic = JogLogic(CFG)
    hold(logic, {99}, 10)
    np.testing.assert_allclose(logic.velocity, 0.0)


def test_hole_mapped_twice_is_rejected():
    with pytest.raises(ValueError):
        JogLogic(BodyDemoConfig(direction_holes={"+x": (1,), "-x": (1,)}))


def test_terminal_parse():
    source = TerminalTouchSource(aliases={"+x": (1, 2)})
    assert source.parse("3 +x") == (frozenset({1, 2, 3}), False)
    assert source.parse("t 4") == (frozenset({4}), True)
    assert source.parse("") == (frozenset(), False)
    assert source.parse("q") is None
    with pytest.raises(ValueError):
        source.parse("+w")


def test_terminal_held_frame_is_fresh():
    source = TerminalTouchSource()
    t0 = time.monotonic()
    assert source.latest().t >= t0


def test_frame_amplitude():
    frame = TouchFrame((HoleTouch(1, 25.0, 3.0), HoleTouch(2, 26.0, 5.0)))
    assert frame.covered == {1, 2}
    assert frame.amplitude == 5.0
    assert TouchFrame().amplitude == 0.0


# --- Gripper demo, with a fake gripper closing 1 mm per check
class FakeFuture:
    def __init__(self, gripper):
        self.gripper, self.cancelled = gripper, False

    def done(self):
        if self.cancelled:
            return True
        self.gripper.value = max(self.gripper.value - 1e-3, self.gripper.target)
        return self.gripper.value <= self.gripper.target

    def cancel(self):
        self.cancelled = True


class FakeGripper:
    def __init__(self, width):
        self.value, self.target = width, None

    def set_target(self, width, speed=None, block=True):
        self.target = width
        return FakeFuture(self)


class FakeTouch:
    def __init__(self, gripper, contact_width):
        self.gripper, self.contact_width = gripper, contact_width

    def latest(self):
        if self.gripper.value > self.contact_width:
            return TouchFrame()
        squeeze_mm = (self.contact_width - self.gripper.value) * 1e3
        return TouchFrame((HoleTouch(5, 25.0, 10.0 + 10.0 * squeeze_mm),))


@pytest.fixture
def egg_grasp():
    wg_gripper_demo = pytest.importorskip("wg_gripper_demo")  # Needs arm_client
    gripper = FakeGripper(0.055)
    cfg = wg_gripper_demo.GripperDemoConfig(poll_s=0.0, width_settle_s=0.0)
    return wg_gripper_demo.EggGrasp(None, gripper, FakeTouch(gripper, 0.045), cfg), gripper


def test_close_until_contact(egg_grasp):
    grasp, gripper = egg_grasp
    stopped, frame = grasp.close_until(0.01, 0.005, lambda f: bool(f.covered))
    assert stopped and frame.covered == {5}
    assert gripper.value == pytest.approx(0.045)


def test_close_until_amplitude_rise(egg_grasp):
    grasp, gripper = egg_grasp
    grasp.close_until(0.01, 0.005, lambda f: bool(f.covered))
    stopped, frame = grasp.close_until(0.040, 0.002, lambda f: f.amplitude - 10.0 >= 30.0)
    assert stopped
    assert gripper.value == pytest.approx(0.042)


def test_close_until_target_first(egg_grasp):
    grasp, gripper = egg_grasp
    stopped, _ = grasp.close_until(0.050, 0.005, lambda f: bool(f.covered))
    assert not stopped
    assert gripper.value == pytest.approx(0.050)
