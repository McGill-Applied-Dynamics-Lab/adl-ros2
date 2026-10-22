"""Tests of the waveguide demos logic (no robot, no Teensy).

    pixi run -e wg pytest experiments/waveguide/tests
"""

import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from touch import HoleTouch, TerminalTouchSource, TouchFrame  # noqa: E402
from wg_body_demo import BodyDemoConfig, DirectionCalibration, HoleMapping, JogLogic, hole_vectors, load_mapping, save_mapping  # noqa: E402

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


def frames(*holes: int, n: int = 5) -> list[TouchFrame]:
    return [TouchFrame(tuple(HoleTouch(h, 0.0, 1.0) for h in holes)) for _ in range(n)]


def test_calibration_assigns_each_hole_to_its_most_touched_direction():
    calibration = DirectionCalibration(min_frames=3)
    for frame in frames(1, 2) + frames(3, n=2):  # 3: too few frames, noise
        calibration.add("+x", frame)
    for frame in frames(4) + frames(2, n=10):  # 2: more often here than in +x
        calibration.add("-x", frame)
    calibration.add("+y", TouchFrame())
    assert calibration.direction_holes() == {"+x": (1,), "-x": (2, 4), "+y": ()}
    assert calibration.conflicts() == {2: ["+x", "-x"]}


def test_calibration_round_trip(tmp_path):
    path = tmp_path / "calibration" / "holes.json"
    assert load_mapping(path) is None
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    save_mapping(HoleMapping({"+x": (1, 2), "-x": ()}, rotation), path)
    mapping = load_mapping(path)
    assert mapping.direction_holes == {"+x": (1, 2), "-x": ()}
    np.testing.assert_allclose(mapping.link_rotation, rotation)
    JogLogic(BodyDemoConfig(direction_holes=mapping.direction_holes))


def test_load_first_format(tmp_path):
    path = tmp_path / "holes.json"
    path.write_text('{"+x": [1, 2]}')
    mapping = load_mapping(path)
    assert mapping.direction_holes == {"+x": (1, 2)} and mapping.link_rotation is None


def angle_deg(a, b):
    return np.rad2deg(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0)))


def test_interpolation_between_axes():
    vectors = hole_vectors({"+y": (10,), "+z": (14,)})
    for hole, expected in [(11, 22.5), (12, 45.0), (13, 67.5)]:
        assert np.linalg.norm(vectors[hole]) == pytest.approx(1.0)
        assert angle_deg(vectors[hole], [0.0, 1.0, 0.0]) == pytest.approx(expected)
        assert vectors[hole][0] == pytest.approx(0.0)  # Rotation about x
    assert 9 not in vectors and 15 not in vectors  # Not outside the mapped holes


def test_interpolation_skips_opposite_and_can_be_disabled():
    assert set(hole_vectors({"+y": (1,), "-y": (5,)})) == {1, 5}
    assert set(hole_vectors({"+y": (10,), "+z": (14,)}, interpolate=False)) == {10, 14}


def test_interpolated_hole_moves_diagonally():
    logic = JogLogic(BodyDemoConfig(direction_holes={"+y": (10,), "+z": (14,)}, acceleration=1.0))
    hold(logic, {12}, 10)
    np.testing.assert_allclose(logic.velocity, [0.0, 0.1 / np.sqrt(2), 0.1 / np.sqrt(2)])


def test_follow_link_rotation():
    logic = JogLogic(CFG)
    rz90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # Link turned 90 deg about z
    for _ in range(10):
        logic.update(DT, frozenset({1}), rz90)  # +x hole now faces +y
    np.testing.assert_allclose(logic.velocity, [0.0, 0.1, 0.0], atol=1e-12)
    # The -y hole (5) now faces +x: perpendicular, no stop; the -x hole (3) faces -y: opposite, stop
    assert not logic.update(DT, frozenset({5}), rz90)
    assert logic.update(DT, frozenset({3}), rz90)


def test_stop_angle():
    cfg = BodyDemoConfig(direction_holes={"+x": (1,), "+y": (5,)}, acceleration=1.0, stop_angle_deg=120.0)
    logic = JogLogic(cfg)
    hold(logic, {1}, 10)
    assert hold(logic, {5}, 1) == [False]  # 90 deg: adds velocity
    # Holes 6-8 between +y (5) and -x (9): 112.5, 135, 157.5 deg from +x
    logic = JogLogic(BodyDemoConfig(direction_holes={"+x": (1,), "+y": (5,), "-x": (9,)}, stop_angle_deg=120.0))
    hold(logic, {1}, 10)
    assert angle_deg(logic.vectors[6], [1.0, 0.0, 0.0]) == pytest.approx(112.5)
    assert hold(logic, {6}, 1) == [False]
    assert angle_deg(logic.vectors[7], [1.0, 0.0, 0.0]) == pytest.approx(135.0)
    assert hold(logic, {7}, 1) == [True]


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


# --- Teensy source, against a fake TEENSY_BLUETOOTH firmware on a pseudo-terminal (serial link)
def test_frame_from_result():
    from touch import frame_from_result

    result = dict(detected=True, hole=5, distance_cm=30.0, amplitude=120.0, hole2=7)
    frame = frame_from_result(result, t=1.0)
    assert frame.covered == {5, 7} and frame.amplitude == 120.0 and frame.t == 1.0
    assert frame.holes[0].distance_cm == 30.0 and np.isnan(frame.holes[1].distance_cm)
    assert frame_from_result(dict(result, detected=False)).covered == frozenset()
    assert frame_from_result(dict(result, hole2=None)).covered == {5}


class FakeTeensy:
    """Answers every command with an ACK, get_status with a loaded calibration, and streams results once asked."""

    def __init__(self, master_fd, wg_helpers, has_calibration=True):
        import struct
        import threading

        self.fd, self.h, self.struct = master_fd, wg_helpers, struct
        self.has_calibration = has_calibration
        self.commands, self.streaming, self.stopped = [], False, False
        self._lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._stream, daemon=True).start()

    def _send(self, packet_type, payload):
        import os

        with self._lock:
            os.write(self.fd, self.h.encode_packet(packet_type, payload))

    def _read(self):
        import os

        buffer = bytearray()
        names = {code: name for name, code in self.h.CMD.items()}
        while not self.stopped:
            try:
                buffer += os.read(self.fd, 4096)
            except OSError:
                return
            while (start := buffer.find(self.h.BT_SYNC)) >= 0 and len(buffer) >= start + 5:
                code, length = buffer[start + 2], int.from_bytes(buffer[start + 3 : start + 5], "little")
                if len(buffer) < start + 7 + length:
                    break
                payload = bytes(buffer[start + 5 : start + 5 + length])
                del buffer[: start + 7 + length]
                name = names[code]
                self.commands.append(name)
                if name == "get_status":
                    ok = int(self.has_calibration)
                    status = (ok, 1, ok, 4, 1, int(self.streaming), 0, 0, 0, 0, 0, 0, 0)
                    self._send(self.h.PKT_STATUS, self.struct.pack("<BBBBBBHHBHHIH", *status))
                    continue
                if name == "stream":
                    self.streaming = bool(payload[0])
                self._send(self.h.PKT_ACK, self.struct.pack("<BBI", code, 0, 0))

    def _stream(self):
        import time as _time

        sequence = 0
        while not self.stopped:
            _time.sleep(0.01)
            if self.streaming:
                sequence += 1
                result = (sequence, 0, 4, 1, 5, 30.0, float("nan"), 120.0, 7, 9.0)
                self._send(self.h.PKT_RESULT, self.struct.pack("<HIBBhfffhf", *result))


@pytest.fixture
def fake_teensy():
    import os
    import pty

    pytest.importorskip("serial")
    wg_helpers = pytest.importorskip("wg_helpers")
    master, slave = pty.openpty()
    teensy = FakeTeensy(master, wg_helpers)
    yield teensy, os.ttyname(slave)
    teensy.stopped = True
    os.close(slave)
    os.close(master)


def test_teensy_source_over_serial(fake_teensy):
    from touch import TeensyConfig, TeensyTouchSource

    teensy, port = fake_teensy
    source = TeensyTouchSource(TeensyConfig(link="serial", port=port, link_timeout_s=5.0))
    source.start()
    try:
        assert teensy.commands[:2] == ["stream", "get_status"]  # Calibration already there: no upload
        assert teensy.commands[2:] == ["record_background", "set_method", "set_average", "stream"]
        t_end = time.monotonic() + 2.0
        while (frame := source.latest()) is None and time.monotonic() < t_end:
            time.sleep(0.01)
        assert frame is not None and frame.covered == {5, 7} and frame.amplitude == 120.0
    finally:
        source.close()
    assert teensy.commands[-1] == "stream" and not teensy.streaming


def test_match_target_round_trip(tmp_path):
    sensor = pytest.importorskip("sensor")  # Needs pyserial and scipy (wg environment)
    trace = np.linspace(2000.0, 2600.0, 1500)
    path = sensor.save_target(trace, tmp_path)
    np.testing.assert_allclose(sensor.load_target(path), trace, atol=0.01)
    single = tmp_path / "single.csv"
    single.write_text("value\n1\n2\n3\n")
    np.testing.assert_allclose(sensor.load_target(single), [1, 2, 3])
    (tmp_path / "reference_a.csv").write_text("1\n")
    newest = tmp_path / "reference_b.csv"
    newest.write_text("2\n")
    os.utime(tmp_path / "reference_a.csv", (2e9, 2e9))
    os.utime(newest, (2e9 + 10, 2e9 + 10))
    assert sensor.latest_match_target(tmp_path) == newest
    with pytest.raises(FileNotFoundError):
        sensor.latest_match_target(tmp_path / "empty")
