"""The programmed operator: script geometry, continuity and the device interface."""

from __future__ import annotations

import numpy as np
import pytest
from fr3_haptic.scripted import Script, ScriptConfig, ScriptedOperator

ORIGIN = np.array([0.4, 0.0, 0.2])


def sampled(script: Script, dt: float = 1e-3) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = np.arange(0.0, script.duration_s + 0.5, dt)
    xs, vs = zip(*(script.sample(ti) for ti in t))
    return t, np.array(xs), np.array(vs)


@pytest.mark.parametrize("profile", ["press", "circle", "sine", "approach", "line"])
def test_script_is_continuous_and_velocity_matches_position(profile):
    cfg = ScriptConfig(profile=profile, surface_z=0.1, depth=0.01, laps=1, period_s=4.0, repeats=1, cycles=2,
                       frequency_hz=1.5, direction=(1.0, 0.0, 0.5), approach_speed=0.1, line_end=(0.55, 0.05, 0.2),
                       dither_hz=2.0, speed=0.05)
    script = Script(cfg, ORIGIN)
    t, x, v = sampled(script)
    dt = t[1] - t[0]
    assert np.max(np.linalg.norm(np.diff(x, axis=0), axis=1)) < 1e-3  # no position jump at a segment boundary
    assert np.max(np.linalg.norm(np.diff(v, axis=0), axis=1)) < 5e-3  # nor a velocity jump
    np.testing.assert_allclose(np.gradient(x, dt, axis=0)[5:-5], v[5:-5], atol=2e-3)
    np.testing.assert_allclose(x[0], ORIGIN)
    np.testing.assert_allclose(x[-1], ORIGIN, atol=1e-9)
    np.testing.assert_allclose(v[-1], 0.0, atol=1e-9)


def test_press_reaches_the_depth_and_repeats():
    cfg = ScriptConfig(profile="press", surface_z=0.1, depth=0.015, repeats=3, hold_s=1.0)
    _, x, _ = sampled(Script(cfg, ORIGIN))
    z = x[:, 2]
    assert z.min() == pytest.approx(0.085)
    below = z < 0.1
    assert np.count_nonzero(np.diff(below.astype(int)) == 1) == 3  # three contacts
    np.testing.assert_allclose(x[:, :2], np.tile(ORIGIN[:2], (len(x), 1)))


def test_circle_closes_at_the_radius_on_the_table():
    cfg = ScriptConfig(profile="circle", surface_z=0.1, depth=0.01, radius=0.05, laps=2, period_s=5.0, ramp_s=1.0)
    script = Script(cfg, ORIGIN)
    circle = next(s for s in script.segments if type(s).__name__ == "_Circle")
    th_end, w_end = circle.angle(circle.t0 + circle.duration)
    assert th_end == pytest.approx(2 * 2 * np.pi)
    assert w_end == pytest.approx(0.0, abs=1e-9)
    assert circle.angle(circle.t0 + circle.duration / 2)[1] == pytest.approx(2 * np.pi / 5.0)
    t = np.linspace(circle.t0, circle.t0 + circle.duration, 500)
    xs = np.array([script.sample(ti)[0] for ti in t])
    np.testing.assert_allclose(np.linalg.norm(xs[:, :2] - ORIGIN[:2], axis=1), 0.05)
    np.testing.assert_allclose(xs[:, 2], 0.09)


def test_sine_has_its_amplitude_along_its_direction():
    cfg = ScriptConfig(profile="sine", direction=(1.0, 0.0, 1.0), amplitude=0.04, frequency_hz=2.0, cycles=3, ramp_cycles=1.0)
    script = Script(cfg, ORIGIN)
    _, x, _ = sampled(script)
    d = x - ORIGIN
    n = np.array([1.0, 0.0, 1.0]) / np.sqrt(2)
    np.testing.assert_allclose(d - np.outer(d @ n, n), 0.0, atol=1e-12)  # only along n
    assert np.max(np.abs(d @ n)) == pytest.approx(0.04, rel=1e-3)
    assert script.duration_s == pytest.approx(1.0 + 3 / 2.0 + 2 * 0.5 + 1.0)  # rests + cycles + ramps


def test_approach_crosses_the_surface_at_its_constant_speed():
    cfg = ScriptConfig(profile="approach", surface_z=0.1, depth=0.01, approach_speed=0.2, pre_height=0.03, repeats=2)
    t, x, v = sampled(Script(cfg, ORIGIN))
    z, vz = x[:, 2], v[:, 2]
    crossings = np.flatnonzero((z[:-1] >= 0.1) & (z[1:] < 0.1))
    assert len(crossings) == 2
    np.testing.assert_allclose(vz[crossings], -0.2, rtol=1e-6)  # full speed at the table
    assert z.min() == pytest.approx(0.09)
    np.testing.assert_allclose(x[:, :2], np.tile(ORIGIN[:2], (len(x), 1)))
    between = z[crossings[0] : crossings[1]]
    assert between.max() == pytest.approx(0.13)  # back to pre_height between repeats, not the origin


def test_line_reaches_its_end_with_a_dither():
    end = np.array([0.6, 0.0, 0.2])
    cfg = ScriptConfig(profile="line", line_end=tuple(end), speed=0.05, dither_amplitude=0.01, dither_hz=1.0)
    script = Script(cfg, ORIGIN)
    line = next(s for s in script.segments if type(s).__name__ == "_Oscillation")
    np.testing.assert_allclose(script.sample(line.t0 + line.duration)[0], end, atol=1e-9)
    mid = line.t0 + np.floor(line.duration / 2) + 0.25  # whole dither periods plus a quarter: sin = 1
    base = line.base.sample(mid)[0]
    assert np.linalg.norm(script.sample(mid)[0] - base) == pytest.approx(0.01, rel=1e-6)


def test_origin_must_be_above_the_bottom():
    with pytest.raises(ValueError):
        Script(ScriptConfig(surface_z=0.25), ORIGIN)
    with pytest.raises(ValueError):
        ScriptConfig(profile="square")


def test_operator_reads_the_script_and_records_the_force():
    dev = ScriptedOperator(
        origin=ORIGIN, script=ScriptConfig(surface_z=0.1), axes=(1, 0, 2), signs=(1, -1, 1),
        force_gain=0.1, max_force=3.0, uri="ignored",
    )
    with dev:
        np.testing.assert_allclose(dev.settle_and_zero(), ORIGIN)
        x, v = dev.read()
        np.testing.assert_allclose(x, ORIGIN, atol=1e-6)
        dev.write(np.array([10.0, 0.0, 0.0]))
        np.testing.assert_allclose(dev.last_command_dev, [0.0, 1.0, 0.0])  # world x -> device y
        dev.write(np.array([0.0, 0.0, 100.0]))
        np.testing.assert_allclose(dev.last_command_dev, [0.0, 0.0, 3.0])  # clamped
    assert not dev.stop_requested


def test_hand_yields_to_the_handle_force():
    from fr3_haptic.scripted import HandConfig

    cfg = ScriptConfig(surface_z=0.1, rest_s=100.0)  # holds the origin for the whole test
    dev = ScriptedOperator(origin=ORIGIN, script=cfg, force_gain=0.1, max_force=5.0,
                           hand=HandConfig(mass=0.5, stiffness=200.0, damping=20.0))
    dev.settle_and_zero()
    dev.write(np.array([0.0, 0.0, 20.0]))  # 2 N at the handle, upward
    dev.advance_hand(5.0)
    np.testing.assert_allclose(dev._x, ORIGIN + [0.0, 0.0, 2.0 / 200.0], atol=1e-6)  # script + f / k
    np.testing.assert_allclose(dev._v, 0.0, atol=1e-6)
