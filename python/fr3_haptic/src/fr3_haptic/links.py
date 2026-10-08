"""Simulated delays on the two links between the haptic loop and the robot.

Each direction gets its own ``haptic_teleop.DelayLine`` at its entry, behind a drop-in wrapper,
so the code on either side does not change:

- **feedback** (robot → haptic): [DelayedPlantWriter][] stands in for the ``PlantMailbox`` that
  `FR3Plant` writes its samples to, and [DelayedModelWriter][] for the RIM ``ModelMailbox``;
- **command** (haptic → robot): [DelayedPlant][] stands in for the `FR3Plant` the `CommandLoop`
  sends targets through. ``freeze()`` is the local safety action and is *not* delayed; it also
  drops every target still in flight, so a late one cannot undo it.

[Links][] owns the lines and delivers whatever is due: [Links.pump][] is called at 1 kHz from its
own thread, which sets the delivery resolution (1 ms). Times are ``time.monotonic()``, the clock
the haptic process measures sample ages with.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import numpy as np
from haptic_teleop import DelayConfig, DelayLine, ModelMailbox, PlantMailbox, RateTicker

__all__ = ["DelayedModelWriter", "DelayedPlant", "DelayedPlantWriter", "Links", "LinksConfig"]

PUMP_HZ = 1000.0


@dataclass(kw_only=True)
class LinksConfig:
    """The delay of each link (see ``haptic_teleop.DelayConfig``)."""

    feedback: DelayConfig = field(default_factory=DelayConfig)
    command: DelayConfig = field(default_factory=DelayConfig)

    @property
    def enabled(self) -> bool:
        return self.feedback.enabled or self.command.enabled

    def worst_delivery_gap_s(self, model_update_hz: float) -> float:
        """Rough upper bound on the gap between two feedback deliveries [s]: one update period
        plus the spread of the delay (3 sigma for a normal jitter). With a variable delay, an
        update can be superseded by the next one, so deliveries are not evenly spaced."""
        f = self.feedback
        spread = 0.0 if f.jitter_ms <= 0 else (2 * f.jitter_ms if f.jitter_dist == "uniform" else 6 * f.jitter_ms)
        return 1.0 / model_update_hz + spread * 1e-3


class DelayedPlantWriter:
    """A ``PlantMailbox`` writer whose samples reach the mailbox through a feedback delay."""

    def __init__(self, line: DelayLine, mailbox: PlantMailbox) -> None:
        self.line = line
        self.mailbox = mailbox

    def write(self, x_i, v_i, lam, t_s: float, rx_s: float | None = None) -> None:
        rx_s = time.monotonic() if rx_s is None else rx_s
        self.line.push((np.copy(x_i), np.copy(v_i), np.copy(lam), t_s, rx_s), sent_s=rx_s)


class DelayedModelWriter:
    """A ``ModelMailbox`` writer whose models reach the mailbox through a feedback delay."""

    def __init__(self, line: DelayLine, mailbox: ModelMailbox) -> None:
        self.line = line
        self.mailbox = mailbox

    def write(self, model) -> None:
        self.line.push(model, sent_s=time.monotonic())


class DelayedPlant:
    """An `FR3Plant` stand-in for the `CommandLoop`: targets go through a command delay.

    Args:
        plant: the real `FR3Plant`, which [Links.pump][] delivers the targets to.
        line: the command delay line.
    """

    def __init__(self, plant, line: DelayLine) -> None:
        self.plant = plant
        self.line = line
        self.frozen = False

    def aim(self, x, v) -> None:
        if not self.frozen:
            self.line.push(("aim", np.copy(x), np.copy(v), None), sent_s=time.monotonic())

    def command(self, x, v=None, f_ff=None) -> None:
        if not self.frozen:
            args = ("command", np.copy(x), None if v is None else np.copy(v), None if f_ff is None else np.copy(f_ff))
            self.line.push(args, sent_s=time.monotonic())

    def freeze(self) -> None:
        """Freeze now (not delayed); targets still in flight are dropped when they arrive."""
        self.frozen = True
        self.plant.freeze()

    def deliver(self, target) -> None:
        if self.frozen:
            return
        kind, x, v, f_ff = target
        if kind == "aim":
            self.plant.aim(x, v)
        else:
            self.plant.command(x, v, f_ff)


@dataclass
class _Records:
    feedback: list[tuple[float, float, float, float]] = field(default_factory=list)  # rx, delivered, delay, t_s
    command: list[tuple[float, float, float]] = field(default_factory=list)  # sent, delivered, delay


class Links:
    """The delay lines of both links and the 1 kHz thread that delivers what is due.

    Ask it for the stand-ins, in whatever order the run builds things: [plant_sink][Links.plant_sink],
    [model_sink][Links.model_sink], [command_plant][Links.command_plant]. A link with a zero delay
    returns the original object, so it is bypassed entirely.
    """

    def __init__(self, cfg: LinksConfig) -> None:
        self.cfg = cfg
        self.plant_writer: DelayedPlantWriter | None = None
        self.model_writer: DelayedModelWriter | None = None
        self.delayed_plant: DelayedPlant | None = None
        self.records = _Records()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def plant_sink(self, plant_box: PlantMailbox):
        """What `FR3Plant` should write its samples to: the mailbox, or its feedback-delayed stand-in."""
        if not self.cfg.feedback.enabled:
            return plant_box
        self.plant_writer = DelayedPlantWriter(DelayLine(self.cfg.feedback.build()), plant_box)
        return self.plant_writer

    def model_sink(self, model_box: ModelMailbox):
        """What the RIM model loop should write to. Same feedback delay distribution as the samples,
        independent draws (exact only once the model comes from the same controller tick)."""
        if not self.cfg.feedback.enabled:
            return model_box
        cfg = self.cfg.feedback
        seed = None if cfg.seed is None else cfg.seed + 1000  # its own stream of draws
        self.model_writer = DelayedModelWriter(DelayLine(DelayConfig(**{**cfg.__dict__, "seed": seed}).build()), model_box)
        return self.model_writer

    def command_plant(self, plant):
        """What the `CommandLoop` should send targets through: the plant, or its command-delayed stand-in."""
        if not self.cfg.command.enabled:
            return plant
        self.delayed_plant = DelayedPlant(plant, DelayLine(self.cfg.command.build()))
        return self.delayed_plant

    def pump(self, now_s: float | None = None) -> None:
        """Deliver everything due by ``now_s`` (default: now) on both links."""
        now = time.monotonic() if now_s is None else now_s
        if self.plant_writer is not None:
            got = self.plant_writer.line.poll(now)
            if got is not None:
                (x_i, v_i, lam, t_s, rx_s), _, delay = got
                self.plant_writer.mailbox.write(x_i, v_i, lam, t_s, rx_s=rx_s, delivered_s=now)
                self.records.feedback.append((rx_s, now, delay, t_s))
        if self.model_writer is not None:
            got = self.model_writer.line.poll(now)
            if got is not None:
                self.model_writer.mailbox.write(got[0])
        if self.delayed_plant is not None:
            got = self.delayed_plant.line.poll(now)
            if got is not None:
                target, sent, delay = got
                self.delayed_plant.deliver(target)
                self.records.command.append((sent, now, delay))

    def start(self) -> None:
        """Start the delivery thread (only if a link has a delay)."""
        if not self.cfg.enabled or self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="links", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        ticker = RateTicker(PUMP_HZ)  # not run_loop: it preallocates one slot per tick
        while not self._stop.is_set():
            self.pump()
            ticker.sleep()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def summary(self) -> str:
        """Per link: delay drawn (median, max), items delivered and superseded."""
        parts = []
        for name, line in (
            ("feedback", self.plant_writer.line if self.plant_writer else None),
            ("command", self.delayed_plant.line if self.delayed_plant else None),
        ):
            if line is None:
                parts.append(f"{name}: no delay")
                continue
            d = np.array(line.stats.delays_s) * 1e3
            stats = f"median {np.median(d):.1f} ms, max {d.max():.1f} ms" if len(d) else "nothing sent"
            parts.append(f"{name}: {stats}, {line.stats.delivered} delivered, {line.stats.superseded} superseded")
        return " | ".join(parts)
