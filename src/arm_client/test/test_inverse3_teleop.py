import numpy as np
import pytest

from arm_client.teleop import inverse3_teleop as i3mod


class _FakeI3Websocket:
    def __init__(self, uri: str = "ws://localhost:10001", start_ok: bool = True):
        self.uri = uri
        self._start_ok = start_ok
        self._connected = start_ok
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        self.last_force = np.zeros(3)
        self.forces = []
        self.stopped = False

    def start(self) -> bool:
        self._connected = self._start_ok
        return self._start_ok

    def stop(self):
        self.stopped = True
        self._connected = False

    def get_state(self):
        return self._position.copy(), self._velocity.copy()

    def apply_force(self, force: np.ndarray):
        self.last_force = np.array(force, dtype=float)
        self.forces.append(self.last_force.copy())

    def is_connected(self) -> bool:
        return self._connected


def _make_teleop(monkeypatch, config: i3mod.Inverse3Config, fake: _FakeI3Websocket):
    monkeypatch.setattr(i3mod, "Inverse3Websocket", lambda uri: fake)
    return i3mod.Inverse3Device(initial_robot_position=np.zeros(3), config=config, center_on_start=False)


def test_init_rejects_invalid_interface_axis():
    cfg = i3mod.Inverse3Config(interface_axis="bad")
    with pytest.raises(ValueError, match="Invalid interface_axis"):
        i3mod.Inverse3Device(initial_robot_position=np.zeros(3), config=cfg, center_on_start=False)


def test_init_rejects_bad_rpy_length():
    cfg = i3mod.Inverse3Config(haptic_to_robot_rpy=[0.0, 0.0])
    with pytest.raises(ValueError, match="Invalid haptic_to_robot_rpy length"):
        i3mod.Inverse3Device(initial_robot_position=np.zeros(3), config=cfg, center_on_start=False)


def test_get_interface_state_projects_selected_axis(monkeypatch):
    fake = _FakeI3Websocket()
    cfg = i3mod.Inverse3Config(interface_axis="z", interface_workspace_scale=2.0)
    teleop = _make_teleop(monkeypatch, cfg, fake)

    fake._position = np.array([1.0, 2.0, 3.0])
    fake._velocity = np.array([0.1, 0.2, 0.3])
    pos1, vel1 = teleop.get_interface_state()
    np.testing.assert_allclose(pos1, np.array([0.0]))
    np.testing.assert_allclose(vel1, np.array([0.6]))

    fake._position = np.array([1.0, 2.0, 3.5])
    fake._velocity = np.array([0.0, 0.0, 0.1])
    pos2, vel2 = teleop.get_interface_state()
    np.testing.assert_allclose(pos2, np.array([1.0]))
    np.testing.assert_allclose(vel2, np.array([0.2]))


def test_set_interface_force_clamps_and_maps_axis(monkeypatch):
    fake = _FakeI3Websocket()
    cfg = i3mod.Inverse3Config(
        interface_axis="y",
        interface_force_scale=2.0,
        force_cap=1.5,
        haptic_to_robot_rpy=[0.0, 0.0, 0.0],
    )
    teleop = _make_teleop(monkeypatch, cfg, fake)

    teleop.set_interface_force(np.array([1.0]))
    np.testing.assert_allclose(fake.last_force, np.array([0.0, 1.5, 0.0]))


def test_set_interface_force_zero_when_feedback_disabled(monkeypatch):
    fake = _FakeI3Websocket()
    cfg = i3mod.Inverse3Config(interface_axis="x", enable_force_feedback=False)
    teleop = _make_teleop(monkeypatch, cfg, fake)

    teleop.set_interface_force(np.array([10.0]))
    np.testing.assert_allclose(fake.last_force, np.zeros(3))


def test_start_raises_when_device_connection_fails(monkeypatch):
    fake = _FakeI3Websocket(start_ok=False)
    cfg = i3mod.Inverse3Config()
    teleop = _make_teleop(monkeypatch, cfg, fake)

    with pytest.raises(RuntimeError, match="Failed to connect"):
        teleop.start()


def test_stop_sends_zero_force_then_stops(monkeypatch):
    fake = _FakeI3Websocket(start_ok=True)
    fake._position = np.array([0.1, 0.2, 0.3])
    cfg = i3mod.Inverse3Config()
    teleop = _make_teleop(monkeypatch, cfg, fake)

    teleop.start()
    teleop.set_device_force(np.array([0.5, -0.1, 0.2]))
    teleop.stop()

    np.testing.assert_allclose(fake.last_force, np.zeros(3))
    assert fake.stopped


def test_position_origin_and_robot_properties(monkeypatch):
    fake = _FakeI3Websocket()
    fake._connected = True
    cfg = i3mod.Inverse3Config(
        i3_origin=[0.1, -0.2, 0.3],
        i3_origin_rpy=[0.0, 0.0, 0.0],
        haptic_to_robot_rpy=[0.0, 0.0, 0.0],
        haptic_to_robot_rpy_degrees=False,
    )
    teleop = i3mod.Inverse3Device(initial_robot_position=np.array([0.5, 0.0, 0.0]), config=cfg, center_on_start=False)
    teleop.i3_interface = fake

    fake._position = np.array([1.0, 2.0, 3.0])
    fake._velocity = np.array([0.1, 0.2, 0.3])

    np.testing.assert_allclose(teleop.position_origin, np.array([1.1, 1.8, 3.3]), atol=1e-7)
    np.testing.assert_allclose(teleop.position_robot, np.array([1.6, 1.8, 3.3]), atol=1e-7)
    np.testing.assert_allclose(teleop.velocity_origin, np.array([0.1, 0.2, 0.3]), atol=1e-7)
    np.testing.assert_allclose(teleop.velocity_robot, np.array([0.1, 0.2, 0.3]), atol=1e-7)
