"""PlantLoop (the plant side of a run) and the fr3_teleop CLI, without hardware.

The haptic step itself is tested in adl-python (haptic_teleop/tests/test_haptic_process.py).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from fr3_haptic.session import PlantLoop, PlantLoopConfig
from fr3_haptic.teleop import build_parser, controller_gains, interface_limits, parse_axes
from haptic_teleop import HapticStateMailbox, ModelMailbox, StatusMailbox
from pyrim import DynModel
from utilities import apply_yaml_config


class FakePlant:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def aim(self, x, v):
        self.calls.append(("aim", np.asarray(x).copy(), np.asarray(v).copy()))

    def command(self, x, v=None, f_ff=None):
        self.calls.append(("command", np.asarray(x).copy(), np.asarray(v).copy(), np.asarray(f_ff).copy()))

    def freeze(self):
        self.calls.append(("freeze",))


@pytest.fixture
def boxes():
    made = []

    def make(box):
        made.append(box)
        return box

    yield make
    for box in made:
        box.close()
        box.unlink()


def make_loop(boxes, method="zoh", **cfg):
    state, status = boxes(HapticStateMailbox(1)), boxes(StatusMailbox())
    plant = FakePlant()
    loop = PlantLoop(PlantLoopConfig(method=method, **cfg), plant=plant, haptic_state=state, status=status, dim=1)
    return loop, plant, state, status


def ok_status(**over):
    values = dict(ticks=1, t_s=0.0, watchdog_gain=1.0, watchdog_tripped=0, watchdog_trips=0, guard_tripped=0, guard_trips=0)
    values.update(over)
    return values


def test_nothing_sent_before_the_haptic_loop_publishes(boxes):
    loop, plant, _, _ = make_loop(boxes)
    loop.step(0, 0.0)
    assert plant.calls == []


def test_coupling_methods_aim_the_leader(boxes):
    loop, plant, state, status = make_loop(boxes)
    status.write(**ok_status())
    state.write(10, 0.01, np.array([0.32]), np.array([0.1]), force=np.array([-1.0]))
    loop.step(0, 0.0)
    kind, x, v = plant.calls[-1]
    assert kind == "aim" and x[0] == pytest.approx(0.32) and v[0] == pytest.approx(0.1)
    assert len(loop.shared.commands) == 1


def test_proxy_methods_command_proxy_with_capped_feedforward(boxes):
    loop, plant, state, status = make_loop(
        boxes, method="proxy-fixed-mass", feedforward_cap=2.0, tool_correction=np.array([0.05])
    )
    status.write(**ok_status())
    state.write(1, 0.001, np.array([0.3]), np.array([0.0]))  # proxy not running yet (NaN)
    loop.step(0, 0.0)
    assert plant.calls == []
    state.write(2, 0.002, np.array([0.3]), np.array([0.0]), np.array([0.28]), np.array([0.01]), np.array([5.0]))
    loop.step(1, 0.02)
    kind, x, v, f_ff = plant.calls[-1]
    assert kind == "command" and x[0] == pytest.approx(0.33) and v[0] == pytest.approx(0.01)
    assert f_ff[0] == pytest.approx(-2.0)  # minus the operator's force, clamped to the cap


def test_watchdog_trip_freezes_and_ends_the_run(boxes):
    loop, plant, state, status = make_loop(boxes)
    state.write(1, 0.001, np.array([0.3]), np.array([0.0]))
    status.write(**ok_status(watchdog_tripped=1, watchdog_trips=1))
    loop.step(0, 0.0)
    assert plant.calls == [("freeze",)] and loop.stopped and "stale" in loop.shared.stop_reason


def test_proxy_rim_publishes_the_model(boxes):
    n = 7
    model = DynModel(n=n, m=1, q=np.zeros(n), q_dot=np.zeros(n), x_i=np.array([0.3]), v_i=np.zeros(1),
                     M=np.eye(n), c=np.zeros(n), J_i=np.ones((1, n)), b_i=np.zeros(1), stamp_s=1.0)  # fmt: skip

    class System:
        ready = True

        def __init__(self):
            self.model = model

        def update(self):
            pass

    state, status, out = boxes(HapticStateMailbox(1)), boxes(StatusMailbox()), boxes(ModelMailbox(n, 1))
    with pytest.raises(ValueError):
        PlantLoop(PlantLoopConfig(method="proxy-rim"), plant=FakePlant(), haptic_state=state, status=status, dim=1)
    loop = PlantLoop(
        PlantLoopConfig(method="proxy-rim"), plant=FakePlant(), haptic_state=state, status=status, dim=1,
        system=System(), model_out=out,
    )  # fmt: skip
    loop.step(0, 0.0)
    loop.step(1, 0.02)
    updates, got = out.model
    assert updates == 2 and got.x_i[0] == 0.3


# ------------------------------------------------------------------ CLI


def test_parse_axes():
    assert parse_axes("y,-x,z") == ((1, 0, 2), (1.0, -1.0, 1.0))


def test_default_yaml_keys_are_all_flags():
    conf = Path(__file__).resolve().parents[1] / "configs" / "teleop.yaml"
    args = apply_yaml_config(build_parser(), ["--conf", str(conf)])
    assert args.axes == ((1, 0, 2), (1.0, -1.0, 1.0))
    assert args.method in ("zoh", "linear", "tdpa-zoh", "tdpa-linear", "proxy-rim", "proxy-fixed-mass")
    assert args.plant_hz > 0
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
