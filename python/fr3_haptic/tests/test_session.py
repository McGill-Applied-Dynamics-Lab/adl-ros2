"""TeleopSession loop logic and the fr3_teleop CLI, without hardware."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from fr3_haptic.plant import StalenessWatchdog
from fr3_haptic.session import SessionConfig, TeleopSession, haptic_columns
from fr3_haptic.teleop import build_parser, controller_gains, interface_limits, parse_axes
from haptic_teleop import PassivityObserver, TickLog
from haptic_teleop.config import RenderingConfigs, SafetyConfig
from pyrim import InterfaceFrame
from utilities import apply_yaml_config

ORIGIN = np.array([0.4, 0.0, 0.3])
HZ = 1000.0
DT = 1.0 / HZ
K, D = 500.0, 30.0


class FakeDevice:
    """World-frame device with identity axes, unit scale and unit force gain."""

    def __init__(self) -> None:
        self.x = ORIGIN.copy()
        self.v = np.zeros(3)
        self.written = np.zeros(3)

    def read(self):
        return self.x.copy(), self.v.copy()

    def write(self, f):
        self.written = np.asarray(f, dtype=float).copy()

    @property
    def last_command_dev(self):
        return self.written.copy()

    def sample_age_s(self):
        return 0.0


class FakePlant:
    def __init__(self) -> None:
        self.sample = None
        self.age = 0.001
        self.calls: list[tuple] = []
        self._updates = 0

    def set_sample(self, x_i, v_i=0.0, lam=0.0):
        self._updates += 1
        self.sample = (
            self._updates,
            SimpleNamespace(x_i=np.array([x_i]), v_i=np.array([v_i]), lam=np.array([lam]), t_s=self._updates * 0.02),
        )

    def sample_age_s(self):
        return self.age

    def aim(self, x, v):
        self.calls.append(("aim", np.asarray(x).copy(), np.asarray(v).copy()))

    def command(self, x, v=None, f_ff=None):
        self.calls.append(("command", np.asarray(x).copy(), v, None if f_ff is None else np.asarray(f_ff).copy()))

    def freeze(self):
        self.calls.append(("freeze",))


def make_session(method="zoh", n_ticks=5000, guard_cooldown_s=3.0, chatter_hz=0.0, **cfg_kwargs):
    frame = InterfaceFrame.from_direction([0.0, 0.0, 1.0])
    rendering = RenderingConfigs().build(
        method, 1, dt=DT, stiffness=K, damping=D, basis=frame.basis
    )
    cfg = SessionConfig(method=method, haptic_hz=HZ, max_force_world=100.0, guard_cooldown_s=guard_cooldown_s, **cfg_kwargs)
    device, plant = FakeDevice(), FakePlant()
    session = TeleopSession(
        cfg,
        device=device,
        plant=plant,
        method=rendering,
        frame=frame,
        leader_origin=ORIGIN,
        observer=PassivityObserver(),
        guard=SafetyConfig(chatter_hz=chatter_hz, guard_cooldown_s=guard_cooldown_s).build(max_i3_force=100.0),
        watchdog=StalenessWatchdog(max_age_s=0.05, ramp_s=0.1),
        device_axes=(0, 1, 2),
        device_signs=np.ones(3),
        device_scale=1.0,
        log=TickLog(n_ticks, haptic_columns(1)),
    )
    return session, device, plant


def run_ticks(session, n, start=0):
    for tick in range(start, start + n):
        session.haptic_step(tick, tick * DT)
    return start + n


def test_zoh_force_ramps_in_then_renders_the_coupling():
    session, device, plant = make_session(force_ramp_s=1.0)
    plant.set_sample(x_i=0.29)  # plant 1 cm below the handle
    session.haptic_step(0, 0.0)
    np.testing.assert_allclose(device.written, 0.0)  # t = 0: ramp and watchdog both at zero
    run_ticks(session, 1500, start=1)
    # Operator feels K (x_i - x_l) = 500 * (-0.01) along z, fully ramped in
    np.testing.assert_allclose(device.written, [0.0, 0.0, -5.0], atol=1e-9)
    session.log.finish(session.shared.ticks)
    assert session.log.column("watchdog_gain")[-1] == pytest.approx(1.0)
    np.testing.assert_allclose(session.log.column("f_0")[-1], -5.0)


def test_coupling_plant_step_aims_the_leader():
    session, device, plant = make_session()
    device.x = ORIGIN + [0.0, 0.0, 0.02]
    device.v = np.array([0.0, 0.0, 0.1])
    session.haptic_step(0, 0.0)
    session.plant_step(0, 0.0)
    kind, x, v = plant.calls[-1]
    assert kind == "aim"
    np.testing.assert_allclose(x, [0.32])
    np.testing.assert_allclose(v, [0.1])


def test_stale_plant_freezes_robot_and_ends_run():
    session, device, plant = make_session(force_ramp_s=0.0)
    plant.set_sample(x_i=0.29)
    t = run_ticks(session, 200)
    plant.age = 0.2  # samples stopped
    run_ticks(session, 200, start=t)
    np.testing.assert_allclose(device.written, 0.0)  # force ramped out
    session.plant_step(0, 0.0)
    assert plant.calls[-1] == ("freeze",)
    assert session.stopped and "stale" in session.shared.stop_reason


def test_free_axis_spring_pulls_hand_back_onto_the_axis():
    session, device, plant = make_session(free_axis_stiffness=100.0)
    device.x = ORIGIN + [0.01, -0.02, 0.0]
    session.haptic_step(0, 0.0)
    np.testing.assert_allclose(device.written, [-1.0, 2.0, 0.0])


def test_guard_trip_zeroes_force_for_the_cooldown():
    session, device, plant = make_session(force_ramp_s=0.0, guard_cooldown_s=0.5)
    plant.set_sample(x_i=0.29)
    t = run_ticks(session, 300)
    session.guard._trip("test")
    t = run_ticks(session, 2, start=t)
    np.testing.assert_allclose(device.written, 0.0)
    assert session.shared.guard_trips == 1
    run_ticks(session, 600, start=t)  # past the cooldown: force back
    assert np.linalg.norm(device.written) > 0.0
    assert not session.stopped


def test_guard_trip_without_cooldown_ends_the_run():
    session, device, plant = make_session(force_ramp_s=0.0, guard_cooldown_s=0.0)
    plant.set_sample(x_i=0.29)
    t = run_ticks(session, 10)
    session.guard._trip("test")
    run_ticks(session, 1, start=t)
    assert session.stopped and "guard" in session.shared.stop_reason


def test_fixed_mass_proxy_commands_proxy_with_feedforward():
    correction = np.array([0.05])
    session, device, plant = make_session("proxy-fixed-mass", tool_correction=correction, feedforward_cap=2.0)
    session.plant_step(0, 0.0)
    assert plant.calls == []  # proxy not seeded yet: nothing commanded
    device.x = ORIGIN + [0.0, 0.0, -0.05]  # leader pulls the proxy down
    run_ticks(session, 20)
    session.plant_step(1, 0.02)
    kind, x, v, f_ff = plant.calls[-1]
    assert kind == "command"
    x_proxy, _ = session.method.rim_state
    np.testing.assert_allclose(x, x_proxy + correction)
    # Feedforward on the robot = minus the operator's force, clamped to the cap
    f_op = session.method.haptic_force()
    assert f_ff[0] == pytest.approx(-np.sign(f_op[0]) * min(abs(f_op[0]), 2.0))


def test_proxy_rim_requires_a_system():
    with pytest.raises(ValueError):
        make_session("proxy-rim")


# ------------------------------------------------------------------ CLI


def test_parse_axes():
    assert parse_axes("y,-x,z") == ((1, 0, 2), (1.0, -1.0, 1.0))


def test_default_yaml_keys_are_all_flags():
    conf = Path(__file__).resolve().parents[1] / "configs" / "teleop.yaml"
    args = apply_yaml_config(build_parser(), ["--conf", str(conf)])
    assert args.axes == ((1, 0, 2), (1.0, -1.0, 1.0))
    assert args.method == "zoh" and args.plant_hz == 50.0
    assert not args.force  # forces stay a CLI decision


def test_interface_limits_combine_absolute_and_range():
    assert interface_limits(None, 0.3, 0.1) == pytest.approx((0.2, 0.4))
    assert interface_limits((0.25, np.inf), 0.3, 0.1) == pytest.approx((0.25, 0.4))
    assert interface_limits(None, 0.3, 0.0) is None
    assert interface_limits((0.1, 0.5), 0.3, 0.0) == (0.1, 0.5)
    with pytest.raises(ValueError):
        interface_limits((0.4, 0.5), 0.3, 0.1)


def test_controller_gains_follow_the_method():
    args = build_parser().parse_args(["--method", "linear", "--kv", "300", "--dv", "20", "--interface-axis", "x"])
    gains = dict(controller_gains(args))
    assert gains["gains.k_pos_x"] == 300.0 and gains["gains.d_pos_x"] == 20.0
    assert gains["control.inertia_decoupling"] is False and gains["feedforward.wrench"] is False
    proxy = dict(controller_gains(build_parser().parse_args(["--method", "proxy-rim"])))
    assert proxy["feedforward.wrench"] is True
    assert not any(name.startswith("gains.") for name in proxy)
