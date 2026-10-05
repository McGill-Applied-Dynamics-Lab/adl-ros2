"""One haptic-teleop run on the real FR3: the per-tick logic of the haptic and plant loops.

Mirrors adl-python ``examples/04_i3_newton_fr3_coupling.py`` with the Newton plant replaced by
[FR3Plant][fr3_haptic.plant.FR3Plant]. Two loops share this object:

- [haptic_step][TeleopSession.haptic_step], at the haptic rate: read the Inverse3, feed the
  rendering method (leader state, fresh plant sample, fresh model), render the force, apply
  the gains (start ramp, staleness watchdog, guard cooldown), write the handle, log.
- [plant_step][TeleopSession.plant_step], at the plant rate: refresh the dynamics model (proxy
  methods), stream the targets to ``osc_controller``, or freeze the robot once the watchdog
  trips.

They exchange values through attributes replaced whole (``leader``, ``model``), so neither
side takes a lock. Everything hardware-specific is injected, so the logic runs against fakes.

Interface points. The coupling methods couple the leader to the controller's end-effector
(``ee_state``), so the leader's origin is that point. The RIM proxy lives at the interface
point of the dynamics model (`RobotModelAdapter`, e.g. a tool tip), so for RIM the leader's
origin is shifted onto it and the proxy target is shifted back by ``tool_correction``
(end-effector minus model interface point, in interface coordinates) before it is sent.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from haptic_teleop import PassivityObserver, RenderingMethod, SafetyMonitor, TickLog, clamp_force
from pyrim import DynModel, InterfaceFrame

from .plant import StalenessWatchdog

__all__ = ["PROXY_METHODS", "SessionConfig", "TeleopSession", "haptic_columns"]

PROXY_METHODS = ("proxy-rim", "proxy-fixed-mass")


@dataclass(kw_only=True)
class SessionConfig:
    """Run parameters the loops need, in world units.

    Args:
        method: rendering method name (see ``haptic_teleop.RenderingConfigs.build``).
        haptic_hz: haptic loop rate [Hz].
        plant_hz: plant loop rate: target streaming and model updates [Hz].
        max_force_world: clamp on the rendered interface force [world N].
        force_ramp_s: the rendered force fades in linearly over this long from the start [s].
        guard_cooldown_s: after a guard trip, zero the force this long, then reset the guard [s].
        free_axis_stiffness: handle spring toward the origin on the axes the interface does not
            use [world N/m]; keeps the hand on the interface axis. 0 disables.
        free_axis_damping: matching damper [world N·s/m].
        feedforward_cap: clamp on the feedforward force sent to the robot (proxy methods) [N].
        tool_correction: ``(dim,)`` end-effector minus model interface point [m]; proxy methods only.
    """

    method: str
    haptic_hz: float = 1000.0
    plant_hz: float = 50.0
    max_force_world: float = 10.0
    force_ramp_s: float = 1.0
    guard_cooldown_s: float = 3.0
    free_axis_stiffness: float = 0.0
    free_axis_damping: float = 0.0
    feedforward_cap: float = 15.0
    tool_correction: np.ndarray | None = None

    @property
    def proxy(self) -> bool:
        """True for the proxy methods (RIM, fixed-mass), which command the robot themselves."""
        return self.method in PROXY_METHODS


def haptic_columns(dim: int) -> tuple[str, ...]:
    """Column names of the haptic `TickLog` for an interface of dimension ``dim``."""
    vec = ("x_l", "v_l", "f", "f_raw", "x_i")
    cols = tuple(f"{name}_{k}" for name in vec for k in range(dim))
    return cols + ("plant_age_s", "device_age_s", "energy_j", "guard_tripped", "watchdog_gain")


@dataclass
class _Shared:
    """Counters and flags of one run, touched by both loops."""

    seen_sample: int = 0
    seen_model: int = 0
    model_updates: int = 0
    guard_trips: int = 0
    guard_was_tripped: bool = False
    cooldown_until: float | None = None
    ticks: int = 0
    stop: bool = False
    stop_reason: str = ""
    commands: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = field(default_factory=list)


class TeleopSession:
    """The two loops of one run. See the module docstring.

    Args:
        cfg: run parameters.
        device: a ``haptic_teleop`` 3-DoF `Device` in world (robot base) coordinates, whose
            origin is the leader's start point (``leader_origin``).
        plant: an [FR3Plant][fr3_haptic.plant.FR3Plant] (or anything with its ``sample``,
            ``sample_age_s``, ``aim``, ``command``, ``freeze``).
        method: the rendering method, at ``frame.dim``.
        frame: the interface frame.
        leader_origin: ``(3,)`` world point the device's zero maps to; the free-axis spring
            pulls toward it.
        observer: passivity observer on the handle.
        guard: safety monitor on the handle.
        watchdog: plant staleness watchdog.
        device_axes, device_signs, device_scale: the device's axis map, for the handle-frame
            passivity check (same as adl-python example 04).
        system: a ``pyrim.SystemInterface`` (proxy-rim only), updated at the plant rate.
        log: haptic tick log with [haptic_columns][fr3_haptic.session.haptic_columns].
    """

    def __init__(
        self,
        cfg: SessionConfig,
        *,
        device,
        plant,
        method: RenderingMethod,
        frame: InterfaceFrame,
        leader_origin: np.ndarray,
        observer: PassivityObserver,
        guard: SafetyMonitor,
        watchdog: StalenessWatchdog,
        device_axes: tuple[int, ...],
        device_signs: np.ndarray,
        device_scale: float,
        log: TickLog,
        system=None,
    ) -> None:
        if cfg.method == "proxy-rim" and system is None:
            raise ValueError("proxy-rim needs a system (FR3System) for its dynamics model")
        self.cfg = cfg
        self.device = device
        self.plant = plant
        self.method = method
        self.frame = frame
        self.leader_origin = np.asarray(leader_origin, dtype=float).copy()
        self.observer = observer
        self.guard = guard
        self.watchdog = watchdog
        self.system = system
        self.log = log
        self._axes = list(device_axes)
        self._signs = np.asarray(device_signs, dtype=float)
        self._scale = float(device_scale)
        self._dt = 1.0 / cfg.haptic_hz
        self._tool_correction = (
            np.zeros(frame.dim) if cfg.tool_correction is None else np.asarray(cfg.tool_correction, dtype=float)
        )
        dim = frame.dim
        self.leader: tuple[np.ndarray, np.ndarray] = (frame.project(self.leader_origin), np.zeros(dim))
        """Latest leader ``(x, v)`` in interface coordinates, replaced whole by the haptic loop."""
        self.model: tuple[int, DynModel] | None = None
        """Latest ``(updates, model)``, replaced whole by the plant loop (proxy-rim)."""
        self.shared = _Shared()

    # ------------------------------------------------------------------ control

    @property
    def stopped(self) -> bool:
        """True once either loop asked the run to end."""
        return self.shared.stop

    def request_stop(self, reason: str) -> None:
        """End the run (both loops poll [stopped][TeleopSession.stopped])."""
        if not self.shared.stop:
            self.shared.stop_reason = reason
        self.shared.stop = True

    # ------------------------------------------------------------------ haptic loop

    def haptic_step(self, tick: int, t_s: float) -> None:
        """One haptic tick. ``t_s`` is loop time from the start of the run [s]."""
        s = self.shared
        x3, v3 = self.device.read()
        x_l = self.frame.project(x3)
        v_l = self.frame.project(v3)
        self.leader = (x_l, v_l)
        self.method.add_leader_state(x_l, v_l, t_s)

        latest = self.plant.sample
        if latest is not None and latest[0] != s.seen_sample:
            s.seen_sample = latest[0]
            self.method.update_plant(latest[1])
        model = self.model
        if model is not None and model[0] != s.seen_model:
            s.seen_model = model[0]
            self.method.update_model(model[1])

        self.method.step(self._dt)
        f_raw = clamp_force(self.method.haptic_force(), self.cfg.max_force_world)

        plant_age = self.plant.sample_age_s()
        wd_gain = self.watchdog.update(plant_age, self._dt)
        ramp = min(1.0, t_s / self.cfg.force_ramp_s) if self.cfg.force_ramp_s > 0 else 1.0
        f = f_raw * (wd_gain * ramp)

        if self.guard.tripped and not s.guard_was_tripped:
            s.guard_trips += 1
            s.cooldown_until = t_s + self.cfg.guard_cooldown_s
            if self.cfg.guard_cooldown_s <= 0:
                self.request_stop(f"guard: {self.guard.reason}")
        s.guard_was_tripped = self.guard.tripped
        if s.cooldown_until is not None:
            if t_s >= s.cooldown_until and self.cfg.guard_cooldown_s > 0:
                self.guard.reset()
                self.observer.reset()
                s.cooldown_until, s.guard_was_tripped = None, False
            else:
                f = np.zeros_like(f)

        f3 = self.frame.lift(f) + self._free_axis_force(x3, v3)
        self.device.write(f3)

        # Passivity in handle units, as in adl-python example 04
        f_hand = self.device.last_command_dev[self._axes]
        v_hand = self._signs * v3 / self._scale
        self.observer.update(f_hand, v_hand, self._dt)
        self.guard.update(self.observer.energy, float(np.linalg.norm(f_hand)), self._dt, force_vec=f_hand)

        x_i = latest[1].x_i if latest is not None else np.full(self.frame.dim, np.nan)
        self.log.record(
            tick,
            *x_l,
            *v_l,
            *f,
            *f_raw,
            *x_i,
            plant_age,
            self.device.sample_age_s(),
            self.observer.energy,
            float(self.guard.tripped),
            wd_gain,
        )
        s.ticks = tick + 1

    def _free_axis_force(self, x3: np.ndarray, v3: np.ndarray) -> np.ndarray:
        k, d = self.cfg.free_axis_stiffness, self.cfg.free_axis_damping
        if k <= 0.0 and d <= 0.0:
            return np.zeros(3)
        return -k * self.frame.complement(x3 - self.leader_origin) - d * self.frame.complement(v3)

    # ------------------------------------------------------------------ plant loop

    def plant_step(self, tick: int, t_s: float) -> None:
        """One plant tick: model update, then targets (or freeze)."""
        s = self.shared
        if self.watchdog.tripped:
            self.plant.freeze()
            self.request_stop(f"plant samples stale (> {self.watchdog.max_age_s * 1e3:.0f} ms)")
            return

        if self.system is not None:
            self.system.update()
            if self.system.ready:
                s.model_updates += 1
                self.model = (s.model_updates, self.system.model)

        if self.cfg.proxy:
            x, v = self.method.rim_state
            if x is None:
                return  # proxy not running yet: the controller holds its pose
            target = x + self._tool_correction
            f_ff = clamp_force(-self.method.haptic_force(), self.cfg.feedforward_cap)
            self.plant.command(target, v, f_ff)
            s.commands.append((t_s, target.copy(), np.asarray(v, dtype=float).copy(), f_ff))
        else:
            x_l, v_l = self.leader
            self.plant.aim(x_l, v_l)
            s.commands.append((t_s, x_l.copy(), v_l.copy(), np.zeros(self.frame.dim)))

    def shutdown_robot(self) -> None:
        """Freeze the robot at the end of the run (idempotent)."""
        self.plant.freeze()
