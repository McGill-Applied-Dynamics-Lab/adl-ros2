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
  With ``surface_z`` above the table the circle is in the air.
- ``sine``: ``cycles`` sinusoids of ``amplitude`` at ``frequency_hz`` along ``direction``, centred
  on the origin, under a raised-cosine envelope (``ramp_cycles`` up and down). No table.
- ``approach``: down to ``pre_height`` above the surface, then at the constant ``approach_speed``
  through the surface to ``depth`` below it (short cosine speed ramps: the start one above the
  surface, the stop one below it, so the table is crossed at full speed), hold, back up;
  ``repeats`` times.
- ``line``: from the origin to ``line_end`` (a minimum-jerk move at ``speed``) with a sinusoidal
  dither of ``dither_amplitude`` at ``dither_hz`` along ``dither_direction`` (default: the line),
  under the same envelope; hold, then back to the origin. No table.

Every move is a minimum-jerk segment, so position and velocity are continuous. After the last
segment the operator holds the origin.

With a [HandConfig][] the operator is no longer a pure position source: the handle is a
mass-spring-damper (a grip) pulled along the script, and the rendered handle force acts on it.
The loop is then closed through the operator, as with a person: a late force lets the hand
overshoot into the table, and a non-passive rendering can drive it unstable. Without one, the
rendered force is recorded and never applied.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

__all__ = ["PROFILES", "HandConfig", "ScriptConfig", "Script", "ScriptedOperator"]

PROFILES = ("press", "circle", "sine", "approach", "line")
SURFACE_PROFILES = ("press", "circle", "approach")  # the ones that go to the table


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
        direction: ``sine``: direction of the motion (normalized), world.
        amplitude: ``sine``: amplitude [m].
        frequency_hz: ``sine``: frequency [Hz].
        cycles: ``sine``: number of cycles at full amplitude (plus the envelope ramps).
        ramp_cycles: ``sine`` / ``line``: envelope ramp, in cycles of the sine / dither.
        approach_speed: ``approach``: constant speed through the surface [m/s].
        pre_height: ``approach``: height above the surface where the constant speed starts [m].
        line_end: ``line``: end point, world [m].
        dither_amplitude: ``line``: dither amplitude [m].
        dither_hz: ``line``: dither frequency [Hz].
        dither_direction: ``line``: dither direction; ``None``: along the line.
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
    direction: tuple[float, float, float] = (0.0, 0.0, 1.0)
    amplitude: float = 0.05
    frequency_hz: float = 1.0
    cycles: int = 5
    ramp_cycles: float = 1.0
    approach_speed: float = 0.05
    pre_height: float = 0.03
    line_end: tuple[float, float, float] | None = None
    dither_amplitude: float = 0.01
    dither_hz: float = 1.0
    dither_direction: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        if self.profile not in PROFILES:
            raise ValueError(f"profile must be one of {PROFILES}, got {self.profile!r}")
        if self.profile == "sine" and (self.amplitude <= 0 or self.frequency_hz <= 0 or self.cycles < 1):
            raise ValueError("sine: amplitude and frequency_hz must be > 0, cycles >= 1")
        if self.profile == "approach" and (self.approach_speed <= 0 or self.pre_height <= 0 or self.depth <= 0):
            raise ValueError("approach: approach_speed, pre_height and depth must be > 0")
        if self.profile == "line" and self.line_end is None:
            raise ValueError("line: line_end is required")
        if self.speed <= 0 or self.period_s <= 0 or self.depth < 0:
            raise ValueError("speed and period_s must be > 0, depth >= 0")
        if self.repeats < 1 or self.laps < 1:
            raise ValueError("repeats and laps must be >= 1")


@dataclass(kw_only=True)
class HandConfig:
    """The operator's hand on the handle: ``m a = k (x_s - x) + b (v_s - v) + f``, with
    ``(x_s, v_s)`` the script and ``f`` the handle force (rendered world force × ``force_gain``,
    clamped). In world units, for ``scale = 1``.

    Args:
        mass: handle plus hand [kg].
        stiffness: grip stiffness toward the script [N/m].
        damping: grip damping [N·s/m].
        max_step_s: integration substep [s].
    """

    mass: float = 0.6
    stiffness: float = 200.0
    damping: float = 8.0
    max_step_s: float = 5e-4

    def __post_init__(self) -> None:
        if self.mass <= 0 or self.stiffness < 0 or self.damping < 0:
            raise ValueError("hand mass must be > 0, stiffness and damping >= 0")


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


def _envelope(t: float, duration: float, ramp: float) -> tuple[float, float]:
    """Raised-cosine envelope ``(w, dw/dt)``: 0 → 1 over ``ramp``, 1, then 1 → 0 over the last ``ramp``."""
    if t <= 0 or t >= duration:
        return 0.0, 0.0
    if ramp <= 0:
        return 1.0, 0.0
    for u, sign in ((t, 1.0), (duration - t, -1.0)):
        if u < ramp:
            return 0.5 * (1 - np.cos(np.pi * u / ramp)), sign * 0.5 * np.pi / ramp * np.sin(np.pi * u / ramp)
    return 1.0, 0.0


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = np.linalg.norm(v)
    if n == 0:
        raise ValueError("a direction must be non-zero")
    return v / n


@dataclass
class _Oscillation:
    """``base(t) + n · A · w(t) · sin(2π f (t - t0))``: a sine under the envelope, on a base segment
    (a hold for ``sine``, the line's move for ``line``)."""

    t0: float
    duration: float
    base: object  # a segment with sample(t) over the same span
    direction: np.ndarray
    amplitude: float
    frequency_hz: float
    ramp_s: float

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        x, v = self.base.sample(t)
        u = t - self.t0
        w, dw = _envelope(u, self.duration, self.ramp_s)
        om = 2 * np.pi * self.frequency_hz
        s, c = np.sin(om * u), np.cos(om * u)
        a = self.amplitude
        return x + self.direction * a * w * s, v + self.direction * a * (dw * s + w * om * c)


@dataclass
class _Cruise:
    """From ``start`` to ``end`` at constant ``speed``, with cosine speed ramps of ``ramp_s`` at both
    ends (each covering ``speed · ramp_s / 2``)."""

    t0: float
    duration: float
    start: np.ndarray
    end: np.ndarray
    speed: float
    ramp_s: float

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        d = self.end - self.start
        length = float(np.linalg.norm(d))
        n = d / length
        u = float(np.clip(t - self.t0, 0.0, self.duration))
        v, tau, T = self.speed, self.ramp_s, self.duration

        def ramp_dist(x):  # distance covered x seconds into a ramp from rest
            return v * (x / 2 - tau / (2 * np.pi) * np.sin(np.pi * x / tau))

        if u < tau:
            s, ds = ramp_dist(u), v * (1 - np.cos(np.pi * u / tau)) / 2
        elif u > T - tau:
            r = T - u
            s, ds = length - ramp_dist(r), v * (1 - np.cos(np.pi * r / tau)) / 2
        else:
            s, ds = v * tau / 2 + v * (u - tau), v
        return self.start + n * s, n * ds


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
        if c.profile in SURFACE_PROFILES and bottom_z >= self.origin[2]:
            raise ValueError(f"the origin (z = {self.origin[2]:.3f} m) must be above surface_z - depth ({bottom_z:.3f} m)")
        self._rest(c.rest_s)
        if c.profile == "sine":
            ramp = c.ramp_cycles / c.frequency_hz
            duration = c.cycles / c.frequency_hz + 2 * ramp
            hold = _Move(self._t, duration, self.origin.copy(), self.origin.copy())
            self.segments.append(_Oscillation(self._t, duration, hold, _unit(c.direction), c.amplitude, c.frequency_hz, ramp))
            self._t += duration
            self._rest(c.rest_s)
        elif c.profile == "line":
            end = np.asarray(c.line_end, dtype=float).reshape(3)
            length = float(np.linalg.norm(end - self.origin))
            n_d = _unit(c.dither_direction if c.dither_direction is not None else end - self.origin)
            duration = 1.875 * length / c.speed
            base = _Move(self._t, duration, self.origin.copy(), end)
            ramp = min(c.ramp_cycles / c.dither_hz, duration / 2)
            self.segments.append(_Oscillation(self._t, duration, base, n_d, c.dither_amplitude, c.dither_hz, ramp))
            self._t += duration
            self._at = end.copy()
            self._rest(c.rest_s)
            self._move_to(self.origin)
            self._rest(c.rest_s)
        elif c.profile == "approach":
            pre = np.array([self.origin[0], self.origin[1], c.surface_z + c.pre_height])
            bottom = np.array([self.origin[0], self.origin[1], bottom_z])
            if pre[2] >= self.origin[2]:
                raise ValueError(f"approach: the origin (z = {self.origin[2]:.3f} m) must be above surface_z + pre_height")
            v = c.approach_speed
            # each ramp (covering v tau / 2) within half its side of the surface: the table is crossed at full speed
            tau = min(0.2, min(c.depth, c.pre_height) / v)
            cruise = (c.pre_height + c.depth) / v + tau  # two ramps of tau at half speed on average
            for _ in range(c.repeats):
                self._move_to(pre)
                self.segments.append(_Cruise(self._t, cruise, pre.copy(), bottom.copy(), v, tau))
                self._t += cruise
                self._at = bottom.copy()
                self._rest(c.hold_s)
                self._move_to(self.origin)
                self._rest(c.rest_s)
        elif c.profile == "press":
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
        hand: a hand on the handle (see the module docstring); ``None``: the handle is the script.
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
        hand: HandConfig | None = None,
        **_,
    ) -> None:
        if dim != 3:
            raise ValueError(f"ScriptedOperator is 3-D (dim = 3), got {dim}")
        self.dim = dim
        self.stop_requested = False
        self.script = Script(script, np.asarray(origin, dtype=float))
        self.hand = hand
        self._axes = tuple(axes)
        self._signs = np.ones(3) if signs is None else np.asarray(signs, dtype=float)
        self.force_gain = float(force_gain)
        self.max_force = float(max_force)
        self._command_dev = np.zeros(3)
        self._f_hand = np.zeros(3)  # handle force, world axes [N]
        self._t0 = time.perf_counter()
        self._reset_hand(0.0)

    def _reset_hand(self, t: float) -> None:
        self._x, self._v = self.script.sample(t)
        self._t_hand = t

    def __enter__(self) -> "ScriptedOperator":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def settle_and_zero(self, warmup_s: float = 0.0) -> np.ndarray:
        """Start the script now; returns the origin."""
        self._t0 = time.perf_counter()
        self._f_hand = np.zeros(3)
        self._reset_hand(0.0)
        return self.script.origin.copy()

    @property
    def elapsed_s(self) -> float:
        return time.perf_counter() - self._t0

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        t = self.elapsed_s
        if self.hand is None:
            return self.script.sample(t)
        self.advance_hand(t)
        return self._x.copy(), self._v.copy()

    def advance_hand(self, t: float) -> None:
        """Integrate the hand to script time ``t`` [s] (semi-implicit Euler, the last handle force held)."""
        h = self.hand
        remaining = t - self._t_hand
        n = max(1, int(np.ceil(remaining / h.max_step_s))) if remaining > 0 else 0
        for k in range(n):
            dt = remaining / n
            x_s, v_s = self.script.sample(self._t_hand + (k + 1) * dt)
            a = (h.stiffness * (x_s - self._x) + h.damping * (v_s - self._v) + self._f_hand) / h.mass
            self._v = self._v + dt * a
            self._x = self._x + dt * self._v
        if n:
            self._t_hand = t

    def write(self, force: np.ndarray) -> None:
        f = np.asarray(force, dtype=float).reshape(3) * self.force_gain
        norm = np.linalg.norm(f)
        if self.max_force <= 0:
            f = np.zeros(3)
        elif norm > self.max_force:
            f *= self.max_force / norm
        self._f_hand = f
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
        self._f_hand = np.zeros(3)
