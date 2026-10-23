"""The robot end of the two-port TDPA (RobotPort), in FR3Plant, the delayed link and the command loop."""

from __future__ import annotations

import numpy as np
import pytest
from fr3_haptic.links import DelayedPlant
from fr3_haptic.port import RobotPort
from haptic_teleop import DelayConfig, DelayLine, PlantMailbox
from test_plant import HOLD, FakeRobot, ee_msg, feed, wrench_msg


def tick(port, force, velocity, n=100, dt=1e-3, t0=0.0):
    for k in range(n):
        port.observe(np.atleast_1d(force), np.atleast_1d(velocity), t0 + k * dt)


def test_energy_is_booked_by_direction():
    port = RobotPort()
    tick(port, force=-2.0, velocity=0.1)  # the robot pushes against the controller: input
    assert port.energy_in == pytest.approx(2.0 * 0.1 * 0.099) and port.energy_out == 0.0
    tick(port, force=2.0, velocity=0.1, t0=0.1)  # the controller drives the robot: output
    assert port.energy_out == pytest.approx(2.0 * 0.1 * 0.1)  # includes the step from 0.099 to 0.1


def test_damping_only_on_a_deficit_and_capped():
    port = RobotPort(max_damping=50.0)
    tick(port, force=2.0, velocity=0.1)  # 0.0198 J out, nothing received
    w = port.wrench(now_s=0.0)  # first command: period taken as 1 ms
    assert port.damping == 50.0  # the deficit asks for far more than the cap
    np.testing.assert_allclose(w, [-5.0])
    port.receive(1.0)
    port.receive(0.5)  # late, older: ignored
    assert port.po_energy == pytest.approx(1.0 - port.energy_out)
    np.testing.assert_allclose(port.wrench(now_s=0.002), [0.0])
    assert port.damping == 0.0


def test_no_wrench_before_the_first_tick():
    assert RobotPort().wrench(now_s=0.0) is None


def test_plant_observes_every_tick_sends_its_energy_and_damps_through_the_wrench():
    from fr3_haptic.plant import FR3Plant
    from pyrim import InterfaceFrame

    robot = FakeRobot()
    box = PlantMailbox(1)
    port = RobotPort(max_damping=50.0)
    try:
        plant = FR3Plant(robot, InterfaceFrame.from_direction([0.0, 0.0, 1.0]), HOLD, node=robot.node, mailbox=box,
                         port=port, sample_period_s=0.01)
        for k in range(1, 21):  # controller drives the robot down: output energy
            t = k * 1_000_000
            feed(robot, ee=ee_msg(t, velocity=(0.0, 0.0, -0.1)), wrench=wrench_msg(t, force=(0.0, 0.0, -2.0)))
        assert port.energy_out == pytest.approx(19 * 2.0 * 0.1 * 1e-3)  # every tick, not only the kept ones
        assert box.sample[1].energy_in == port.energy_in == 0.0
        plant.aim(np.array([0.3]), np.array([0.0]), energy_in=0.0)
        f = robot.published[-1]["force"]
        assert f is not None and f[2] > 0  # damping opposes the downward velocity
        plant.aim(np.array([0.3]), np.array([0.0]), energy_in=1.0)  # enough received: no damping
        np.testing.assert_allclose(robot.published[-1]["force"], 0.0)
    finally:
        box.close()
        box.unlink()


def test_delayed_link_carries_the_energy_with_the_target():
    class Plant:
        calls = []

        def aim(self, x, v, energy_in=None, tick=-1):
            self.calls.append(energy_in)

        def freeze(self):
            pass

    plant = Plant()
    link = DelayedPlant(plant, DelayLine(DelayConfig(base_ms=10.0).build()))
    link.aim(np.zeros(1), np.zeros(1), energy_in=0.7)
    link.aim(np.zeros(1), np.zeros(1))
    for item in (link.line.poll(1e9),):
        link.deliver(item[0])
    assert plant.calls == [None]  # the newest (no energy) supersedes
    link.aim(np.zeros(1), np.zeros(1), energy_in=0.9)
    link.deliver(link.line.poll(2e9)[0])
    assert plant.calls[-1] == 0.9
