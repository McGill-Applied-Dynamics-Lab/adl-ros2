"""Haptic stability bench: Inverse3 coupled to a constant 1-DoF virtual mass.

Couples the Inverse3 to a constant 1-DoF virtual mass via a spring/damper and
runs the haptic loop at a configurable feedback rate.
Used to characterize the achievable (stiffness, damping, rate) stability envelope before the RIM proxy
is in the loop. The virtual mass is stepped by the ``RIMIntegrator`` the
RIM uses, so the measured envelope transfers to the RIM proxy.
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
from arm_client.teleop.inverse3_teleop import Inverse3Config, Inverse3Device
from pyrim import InterfaceFrame, RIMIntegrator, RIMModel

from ..config import InterfaceConfig
from ..data.logger import _git_info
from ..filters import LowPassFilter
from ..monitoring import FreshnessMonitor, LoopRateMonitor


@dataclass(kw_only=True)
class HapticBenchConfig:
    """Parameters for haptic stability-bench run."""

    mass: float = 1.0  # virtual mass [kg]
    stiffness: float = 1000.0  # coupling stiffness K [N/m]
    damping: float = 0.0  # coupling damping D [Ns/m]
    rate_hz: float = 100.0  # haptic feedback loop rate (the swept "feedback frequency")
    duration_s: float = 1000.0  # run length
    rim_direction: list = field(default_factory=lambda: [0.0, 0.0, 1.0])  # 1-DoF interface direction
    contact_surface: float | None = None  # wall as an OFFSET from the device's initial position along the axis (e.g. -0.05 = 5cm toward -z); None = free space
    disable_ff: bool = False
    force_scale: float = 0.1  # device force scaling (applied before the cap)
    force_cap: float = 12.0  # max force rendered to the device [N]
    vel_filter_alpha: float = 0.2  # IIR alpha on leader velocity; 1.0 = no filter
    initial_robot_position: list = field(default_factory=lambda: [0.4, 0.0, 0.5])  # device "robot" origin
    uri: str = "ws://localhost:10001"
    notes: str = ""  # free-text purpose, recorded in metadata


# Stream column order for the recorded samples array.
SAMPLE_COLUMNS = ("t", "x_leader", "v_leader", "x_mass", "v_mass", "force")


class HapticBench:
    """Run the 1-DoF haptic loop coupling the Inverse3 to a constant virtual mass.

    The mass is stepped by ``RIMIntegrator`` seeded with a constant ``RIMModel``
    (``M_eff = mass``, no Coriolis/external force). ``contact_surface`` is an
    offset from the device's initial position along the interface axis, resolved
    to an absolute wall in ``run()`` once the start position is known (None = free
    space at -inf; the integrator's tested LCP contact handles the wall).
    """

    def __init__(self, cfg: HapticBenchConfig) -> None:
        self.cfg = cfg
        self.frame = InterfaceFrame.from_direction(cfg.rim_direction)
        self.integrator = self._make_integrator(-np.inf)  # wall resolved in run() once x0 is known
        # Leader velocity IIR, formerly inside the integrator
        self._leader_vel_filter = LowPassFilter(cfg.vel_filter_alpha)
        self._samples: list[list[float]] = []
        self.freshness = FreshnessMonitor()  # reset per run()
        self._surface_abs: float | None = None  # absolute wall position, set in run()

    def _make_integrator(self, contact_surface: float) -> RIMIntegrator:
        """Build the integrator; the wall is fixed at construction (no setter in pyrim)."""
        return RIMIntegrator(
            interface_dim=1,
            dt=1.0 / self.cfg.rate_hz,
            stiffness=self.cfg.stiffness,
            damping=self.cfg.damping,
            contact_surface=contact_surface,
        )

    def _seed_mass(self, x0: float) -> None:
        """Initialize the constant virtual mass at rest at position ``x0``."""
        self.integrator.update_rim(
            RIMModel(
                m=1,
                M_eff=np.array([[self.cfg.mass]], dtype=float),
                z_i=np.zeros(1),
                f_eff=np.zeros(1),
                x=np.array([x0], dtype=float),
                v=np.zeros(1),
            )
        )

    def run(self, device: object | None = None, visualizer: object | None = None) -> np.ndarray:
        """Run the bench loop for ``duration_s`` and return the recorded samples.

        ``device`` is duck-typed (Inverse3Device API); if None, a real
        Inverse3Device is created and connected. ``visualizer`` (optional,
        BenchVisualizer API) receives scalars every tick and the 3D scene at
        ~60 Hz for live Foxglove viz. The device force is always zeroed on exit,
        even on error or KeyboardInterrupt.
        """
        owns_device = device is None
        if owns_device:
            ff_enabled = not self.cfg.disable_ff
            i3_cfg = Inverse3Config(uri=self.cfg.uri, force_cap=self.cfg.force_cap, enable_force_feedback=ff_enabled)
            device = Inverse3Device(
                initial_robot_position=np.asarray(self.cfg.initial_robot_position, dtype=float),
                config=i3_cfg,
            )

        interface = self._make_interface(device)
        self._samples = []
        # Publish the scene + rate at ~60 Hz regardless of loop rate to bound message volume.
        scene_every = max(1, int(round(self.cfg.rate_hz / 60.0)))
        rate_monitor = LoopRateMonitor(self.cfg.rate_hz)
        self.freshness = FreshnessMonitor()
        self._leader_vel_filter = LowPassFilter(self.cfg.vel_filter_alpha)

        if owns_device:
            device.start()
        if visualizer is not None:
            visualizer.start()

        try:
            # Seed the mass at the current device position to avoid a startup jump.
            interface.update()
            x0, _ = interface.get_interface_state()

            # Resolve the contact surface offset to an absolute wall along the axis.
            if self.cfg.contact_surface is not None:
                self._surface_abs = float(x0[0]) + self.cfg.contact_surface
            else:
                self._surface_abs = None
            self.integrator = self._make_integrator(-np.inf if self._surface_abs is None else self._surface_abs)
            self._seed_mass(float(x0[0]))

            dt = 1.0 / self.cfg.rate_hz
            t_start = time.perf_counter()
            next_tick = t_start
            end = t_start + self.cfg.duration_s
            tick = 0

            while True:
                now = time.perf_counter()
                if now >= end:
                    break

                interface.update()
                # Single device read per tick: full 3D leader state, then project
                # to the 1-DoF interface. Freshness = exact-duplicate detection on
                # the raw 3D state (re-reads of the cached sample are bit-identical).
                leader3, leadervel3 = interface.get_cartesian_state()
                self.freshness.update(np.concatenate([leader3, leadervel3]))
                x_l = self.frame.project(leader3)
                v_l = self.frame.project(leadervel3)
                self.integrator.add_leader_state(x_l, self._leader_vel_filter.update(v_l))
                x_m, v_m = self.integrator.step()
                force = self.integrator.get_interface_force()
                device_force = interface.set_interface_force(force)  # 3D force actually sent (scaled + capped)
                rate_monitor.tick()

                t = now - t_start
                self._samples.append([t, float(x_l[0]), float(v_l[0]), float(x_m[0]), float(v_m[0]), float(force[0])])

                if visualizer is not None:
                    visualizer.log_scalars(
                        t, float(x_l[0]), float(v_l[0]), float(x_m[0]), float(v_m[0]), float(force[0]),
                        device_force=device_force,
                    )
                    if tick % scene_every == 0:
                        mass3 = self.frame.compose(x_m, leader3)
                        visualizer.log_scene(leader3, mass3, self._surface_abs)
                        snap = rate_monitor.snapshot()
                        fresh = self.freshness.snapshot()
                        visualizer.log_rate(
                            snap.measured_hz,
                            snap.target_hz,
                            snap.mean_dt_ms,
                            snap.p95_dt_ms,
                            snap.max_dt_ms,
                            fresh.fresh_rate_hz,
                            fresh.stale_fraction,
                        )

                tick += 1
                next_tick += dt
                time.sleep(max(0.0, next_tick - time.perf_counter()))
        except KeyboardInterrupt:
            # Operator stopped the run early; keep whatever was recorded so the
            # caller can still save/plot the partial trial.
            pass

        finally:
            try:
                interface.set_interface_force(np.zeros(1))
            except Exception:
                pass
            if visualizer is not None:
                visualizer.stop()
            if owns_device:
                device.stop()

        return np.asarray(self._samples, dtype=float)

    def _make_interface(self, device: object):
        # Local import to avoid a hard import cycle at module load.
        from ..adapters import TeleopInterfaceAdapter

        iface_cfg = InterfaceConfig(force_scale=self.cfg.force_scale, force_cap=self.cfg.force_cap)
        return TeleopInterfaceAdapter(device=device, interface_cfg=iface_cfg, frame=self.frame)


def save_bench_run(cfg: HapticBenchConfig, samples: np.ndarray, output_dir: str | Path = "data/bench_runs") -> Path:
    """Persist a bench run (samples + provenance metadata) and return its directory."""
    base = Path(output_dir)
    run_name = datetime.now().strftime("bench_%Y%m%d_%H%M%S")
    run_dir = base / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    git = _git_info()
    git_diff = git.pop("git_diff", None)
    if git_diff:
        (run_dir / "uncommitted.diff").write_text(git_diff, encoding="utf-8")

    metadata = {
        "created_at": datetime.now().isoformat(),
        "notes": cfg.notes,
        **git,
        "config": asdict(cfg),
        "sample_columns": list(SAMPLE_COLUMNS),
    }
    with open(run_dir / "metadata.json", "w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)

    np.savez(run_dir / "samples.npz", samples=samples, columns=np.array(SAMPLE_COLUMNS))
    return run_dir
