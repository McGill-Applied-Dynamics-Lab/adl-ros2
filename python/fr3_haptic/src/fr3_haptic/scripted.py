"""A programmed operator: the leader follows a fixed script instead of a hand on the Inverse3.

[ScriptedOperator][] is a ``haptic_teleop`` device (``HapticProcessSpec.device_factory``) whose
position is a deterministic function of time since ``settle_and_zero``, in world coordinates,
starting at the leader origin. The rendered force is recorded and never applied, so every
method and delay sees the exact same operator motion: force onset, contact force and tracking
are then comparable across conditions. What it cannot show is stability against a real hand.

Profiles (``ScriptConfig.profile``):

- ``press``: from the origin, down to ``depth`` below the surface, hold, back up; ``repeats``
  times. The table is reached on the way down, at the speed the profile has there.
- ``circle``: over to the circle's start, down to ``depth`` below the surface, ``laps`` circles of
  ``radius`` around the origin's x/y at ``period_s`` per lap (smooth start and stop), up, back.

Every move is a minimum-jerk segment, so position and velocity are continuous. After the last
segment the operator holds the origin.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

__all__ = ["PROFILES", "ScriptConfig", "Script", "ScriptedOperator"]

PROFILES = ("press", "circle")


@dataclass(kw_only=True)
class ScriptConfig:
    """The programmed operator's motion.

    Args:
        profile: ``press`` or ``circle``.
        surface_z: table height in leader coordinates (world z, the same point the proxies'
            ``--contact-surface`` refers to) [m].
        depth: how far below the surface the leader goes [m]; the commanded penetration.
        speed: peak speed of the approach and retract moves [m/s].
        hold_s: ``press``: time held at the bottom [s].
        rest_s: time held at the origin before the first move, between presses and at the end [s].
        repeats: ``press``: number of presses.
        radius: ``circle``: radius [m].
        period_s: ``circle``: time per lap at full speed [s].
        laps: ``circle``: number of laps.
        ramp_s: ``circle``: time to reach full angular speed, and to stop [s].
    """

    profile: str = "press"
    surface_z: float = 0.0
    depth: float = 0.01
    speed: float = 0.05
    hold_s: float = 2.0
    rest_s: float = 1.0
    repeats: int = 3
    radius: float = 0.05
    period_s: float = 6.0
    laps: int = 2
    ramp_s: float = 1.0

    def __post_init__(self) -> None:
        if self.profile not in PROFILES:
            raise ValueError(f"profile must be one of {PROFILES}, got {self.profile!r}")
        if self.speed <= 0 or self.period_s <= 0 or self.depth < 0:
            raise ValueError("speed and period_s must be > 0, depth >= 0")
        if self.repeats < 1 or self.laps < 1:
            raise ValueError("repeats and laps must be >= 1")


@dataclass
class _Move:
    t0: float
    duration: float
    start: np.ndarray
    end: np.ndarray

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        s = np.clip((t - self.t0) / self.duration, 0.0, 1.0) if self.duration > 0 else 1.0
        p = 10 * s**3 - 15 * s**4 + 6 * s**5  # minimum jerk
        dp = (30 * s**2 - 60 * s**3 + 30 * s**4) / self.duration if self.duration > 0 else 0.0
        d = self.end - self.start
        return self.start + p * d, dp * d


@dataclass
class _Circle:
    t0: float
    duration: float
    center: np.ndarray  # (3,), the circle's plane is z = center[2]
    radius: float
    omega: float  # full angular speed [rad/s]
    ramp_s: float

    def angle(self, t: float) -> tuple[float, float]:
        """``(theta, theta_dot)``: cosine ramps up and down around a constant ``omega``."""
        tau, w, T = self.ramp_s, self.omega, self.duration
        t = float(np.clip(t - self.t0, 0.0, T))

        def ramp_integral(u: float) -> float:  # integral of (1 - cos(pi u / tau)) / 2 from 0 to u
            return u / 2 - tau / (2 * np.pi) * np.sin(np.pi * u / tau) if tau > 0 else 0.0

        if tau > 0 and t < tau:
            return w * ramp_integral(t), w * (1 - np.cos(np.pi * t / tau)) / 2
        theta_ramp = w * ramp_integral(tau)
        if tau > 0 and t > T - tau:
            u = T - t
            total = 2 * theta_ramp + w * (T - 2 * tau)
            return total - w * ramp_integral(u), w * (1 - np.cos(np.pi * u / tau)) / 2
        return theta_ramp + w * (t - tau), w

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        th, dth = self.angle(t)
        x = self.center + self.radius * np.array([np.cos(th), np.sin(th), 0.0])
        v = self.radius * dth * np.array([-np.sin(th), np.cos(th), 0.0])
        return x, v


@dataclass
class Script:
    """The scripted leader trajectory from ``origin`` (``(3,)``, world) under ``cfg``.

    [sample][Script.sample] gives ``(position, velocity)`` at ``t`` seconds from the start;
    [duration_s][Script.duration_s] is when it is back at the origin and done.
    """

    cfg: ScriptConfig
    origin: np.ndarray
    segments: list = field(init=False)
    duration_s: float = field(init=False)

    def __post_init__(self) -> None:
        self.origin = np.asarray(self.origin, dtype=float).reshape(3)
        self.segments = []
        self._t = 0.0
        self._at = self.origin.copy()
        c = self.cfg
        bottom_z = c.surface_z - c.depth
        if bottom_z >= self.origin[2]:
            raise ValueError(f"the origin (z = {self.origin[2]:.3f} m) must be above surface_z - depth ({bottom_z:.3f} m)")
        self._rest(c.rest_s)
        if c.profile == "press":
            for _ in range(c.repeats):
                self._move_to(np.array([self.origin[0], self.origin[1], bottom_z]))
                self._rest(c.hold_s)
                self._move_to(self.origin)
                self._rest(c.rest_s)
        else:
            start_air = self.origin + np.array([c.radius, 0.0, 0.0])
            start = np.array([start_air[0], start_air[1], bottom_z])
            self._move_to(start_air)
            self._move_to(start)
            ramp = min(c.ramp_s, c.laps * c.period_s / 2)
            omega = 2 * np.pi / c.period_s
            # cosine ramps lose omega * ramp / 2 of angle each, so add one ramp to close the laps
            duration = c.laps * c.period_s + ramp
            center = np.array([self.origin[0], self.origin[1], bottom_z])
            self.segments.append(_Circle(self._t, duration, center, c.radius, omega, ramp))
            self._t += duration
            self._at = start.copy()
            self._move_to(start_air)
            self._move_to(self.origin)
            self._rest(c.rest_s)
        self.duration_s = self._t

    def _rest(self, seconds: float) -> None:
        if seconds > 0:
            self.segments.append(_Move(self._t, seconds, self._at.copy(), self._at.copy()))
            self._t += seconds

    def _move_to(self, target: np.ndarray) -> None:
        dist = float(np.linalg.norm(target - self._at))
        duration = 1.875 * dist / self.cfg.speed  # minimum jerk peaks at 1.875x the mean speed
        self.segments.append(_Move(self._t, duration, self._at.copy(), np.asarray(target, dtype=float).copy()))
        self._t += duration
        self._at = np.asarray(target, dtype=float).copy()

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        for seg in self.segments:
            if t < seg.t0 + seg.duration:
                return seg.sample(t)
        return self.origin.copy(), np.zeros(3)


class ScriptedOperator:
    """A ``haptic_teleop`` device whose handle follows a [Script][] (see the module docstring).

    Args:
        origin: leader origin, world ``(3,)`` [m]; the script starts and ends there.
        script: the motion.
        dim: world axes (3).
        axes, signs: signed device axis per world axis, as for ``Inverse3Device``; only used to
            store the recorded force in device axes like the real device does.
        force_gain: handle N per world N, applied to the recorded force.
        max_force: clamp on the recorded force [N]; 0 records zero.
        **_: the other ``Inverse3Device`` arguments (``uri``, ``write_through``, …), ignored.
    """

    def __init__(
        self,
        *,
        origin,
        script: ScriptConfig,
        dim: int = 3,
        axes=(0, 1, 2),
        signs=None,
        force_gain: float = 1.0,
        max_force: float = 0.0,
        **_,
    ) -> None:
        if dim != 3:
            raise ValueError(f"ScriptedOperator is 3-D (dim = 3), got {dim}")
        self.dim = dim
        self.stop_requested = False
        self.script = Script(script, np.asarray(origin, dtype=float))
        self._axes = tuple(axes)
        self._signs = np.ones(3) if signs is None else np.asarray(signs, dtype=float)
        self.force_gain = float(force_gain)
        self.max_force = float(max_force)
        self._command_dev = np.zeros(3)
        self._t0 = time.perf_counter()

    def __enter__(self) -> "ScriptedOperator":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def settle_and_zero(self, warmup_s: float = 0.0) -> np.ndarray:
        """Start the script now; returns the origin."""
        self._t0 = time.perf_counter()
        return self.script.origin.copy()

    @property
    def elapsed_s(self) -> float:
        return time.perf_counter() - self._t0

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        return self.script.sample(self.elapsed_s)

    def write(self, force: np.ndarray) -> None:
        f = np.asarray(force, dtype=float).reshape(3) * self.force_gain
        norm = np.linalg.norm(f)
        if self.max_force <= 0:
            f = np.zeros(3)
        elif norm > self.max_force:
            f *= self.max_force / norm
        dev = np.zeros(3)
        dev[list(self._axes)] = self._signs * f
        self._command_dev = dev

    @property
    def last_command_dev(self) -> np.ndarray:
        """The last force "sent", device axes [N]; recorded, never applied."""
        return self._command_dev.copy()

    def sample_age_s(self) -> float:
        return 0.0

    def close(self) -> None:
        self._command_dev = np.zeros(3)
