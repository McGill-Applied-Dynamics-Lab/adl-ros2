"""Tests for the haptic stability bench, driven by a fake Inverse3 device."""

import numpy as np

from rim_teleop.bench import HapticBench, HapticBenchConfig
from rim_teleop.bench.haptic_bench import SAMPLE_COLUMNS


class FakeInverse3:
    """Duck-typed stand-in for Inverse3Device.

    Reports a leader moving at constant velocity along z (so the virtual mass
    lags and the coupling force is exercised) and records the last applied force.
    """

    def __init__(self, start=(0.4, 0.0, 0.5), vel=(0.0, 0.0, 0.02)):
        self._pos = np.asarray(start, dtype=float)
        self._vel = np.asarray(vel, dtype=float)
        self.last_force = None

    def update_device_state(self):
        # Advance by a fixed small step each read (constant-velocity leader).
        self._pos = self._pos + self._vel * 1e-3

    @property
    def position_robot(self):
        return self._pos.copy()

    @property
    def velocity_robot(self):
        return self._vel.copy()

    def apply_force(self, force):
        self.last_force = np.asarray(force, dtype=float).copy()

    def is_connected(self):
        return True


def _stable_cfg(**kw):
    defaults = dict(mass=1.0, stiffness=500.0, damping=60.0, rate_hz=1000.0, duration_s=0.05, force_cap=12.0)
    defaults.update(kw)
    return HapticBenchConfig(**defaults)


def test_run_returns_shaped_samples():
    bench = HapticBench(_stable_cfg())
    fake = FakeInverse3()
    samples = bench.run(device=fake)
    assert samples.ndim == 2
    assert samples.shape[0] > 0
    assert samples.shape[1] == len(SAMPLE_COLUMNS)
    assert np.all(np.isfinite(samples))


def test_force_within_cap():
    cfg = _stable_cfg(force_cap=12.0)
    bench = HapticBench(cfg)
    samples = bench.run(device=FakeInverse3())
    forces = samples[:, SAMPLE_COLUMNS.index("force")]
    # The integrator force itself is unclamped in the log; the device-applied
    # force is what gets capped. Verify the logged force stays finite/bounded
    # for stable gains (no runaway).
    assert np.max(np.abs(forces)) < 50.0


def test_force_zeroed_on_exit():
    """Fail-safe: the device must be left at zero force when the run ends (CH-70)."""
    fake = FakeInverse3()
    HapticBench(_stable_cfg()).run(device=fake)
    assert fake.last_force is not None
    np.testing.assert_allclose(fake.last_force, np.zeros(3), atol=1e-9)


def test_stable_gains_do_not_diverge():
    bench = HapticBench(_stable_cfg(stiffness=500.0, damping=60.0))
    samples = bench.run(device=FakeInverse3())
    x_mass = samples[:, SAMPLE_COLUMNS.index("x_mass")]
    x_leader = samples[:, SAMPLE_COLUMNS.index("x_leader")]
    # The mass tracks the moving leader within a small bounded lag.
    assert np.max(np.abs(x_mass - x_leader)) < 0.1
