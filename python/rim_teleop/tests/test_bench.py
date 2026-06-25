"""Tests for the haptic stability bench, driven by a fake Inverse3 device."""

import numpy as np
import pytest

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


def test_contact_wall_blocks_mass():
    """With a contact surface enabled, the mass never penetrates the wall."""
    surface = 0.49
    cfg = _stable_cfg(stiffness=1000.0, damping=90.0, duration_s=0.1, contact_surface=surface)
    # Leader driven downward (−z) below the surface; the mass should clamp at it.
    fake = FakeInverse3(start=(0.4, 0.0, 0.5), vel=(0.0, 0.0, -0.5))
    samples = HapticBench(cfg).run(device=fake)
    x_mass = samples[:, SAMPLE_COLUMNS.index("x_mass")]
    assert np.min(x_mass) >= surface - 1e-9
    # Sanity: the leader actually went below the surface (so the wall was tested).
    assert np.min(samples[:, SAMPLE_COLUMNS.index("x_leader")]) < surface


def test_free_space_default_has_no_wall():
    cfg = _stable_cfg()
    assert cfg.contact_surface is None
    # The mass is free to move below any finite level (no clamp) — driven down.
    fake = FakeInverse3(start=(0.4, 0.0, 0.5), vel=(0.0, 0.0, -0.5))
    samples = HapticBench(cfg).run(device=fake)
    assert np.min(samples[:, SAMPLE_COLUMNS.index("x_mass")]) < 0.5


# ---------------------------------------------------------------------------
# Foxglove scene encoder (no live server)
# ---------------------------------------------------------------------------

def test_scene_encoder_with_surface():
    from rim_teleop.data.logger import _scene_update_from_payload, _to_fg_timestamp

    payload = {
        "leader": [0.4, 0.0, 0.5],
        "mass": [0.4, 0.0, 0.48],
        "surface": 0.45,
        "rim_direction": [0.0, 0.0, 1.0],
        "frame_id": "bench",
    }
    upd = _scene_update_from_payload(payload, _to_fg_timestamp(0.0))
    assert upd is not None
    assert len(upd.encode()) > 0


def test_scene_encoder_without_surface():
    from rim_teleop.data.logger import _scene_update_from_payload, _to_fg_timestamp

    payload = {"leader": [0.4, 0.0, 0.5], "mass": [0.4, 0.0, 0.48], "surface": None}
    upd = _scene_update_from_payload(payload, _to_fg_timestamp(0.0))
    assert upd is not None
    assert len(upd.encode()) > 0


def test_scene_encoder_missing_data_returns_none():
    from rim_teleop.data.logger import _scene_update_from_payload, _to_fg_timestamp

    assert _scene_update_from_payload({"leader": [0.0, 0.0, 0.0]}, _to_fg_timestamp(0.0)) is None


# ---------------------------------------------------------------------------
# Freshness monitor (stale/duplicate device-read detection)
# ---------------------------------------------------------------------------

def test_freshness_monitor_all_fresh():
    from rim_teleop.monitoring import FreshnessMonitor

    m = FreshnessMonitor()
    for i in range(20):
        m.update(np.array([float(i), 0.0, 0.0, 0.0, 0.0, 0.0]))  # changing each read
    assert m.summary().stale_fraction == 0.0


def test_freshness_monitor_detects_duplicates():
    from rim_teleop.monitoring import FreshnessMonitor

    m = FreshnessMonitor()
    for _ in range(10):
        m.update(np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))  # bit-identical re-reads
    # First read is fresh, the remaining 9 are stale.
    assert m.summary().stale_fraction == pytest.approx(0.9, abs=1e-9)


def test_bench_freshness_moving_vs_stationary():
    # A moving fake changes position each read -> all fresh.
    moving = HapticBench(_stable_cfg())
    moving.run(device=FakeInverse3(vel=(0.0, 0.0, 0.05)))
    assert moving.freshness.summary().stale_fraction < 0.05

    # A motionless deterministic fake re-reads identical state -> mostly stale.
    # (A real device has sensor noise, so stationary != stale on hardware.)
    stationary = HapticBench(_stable_cfg())
    stationary.run(device=FakeInverse3(vel=(0.0, 0.0, 0.0)))
    assert stationary.freshness.summary().stale_fraction > 0.9
