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

import csv
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from haptic_teleop import DelayConfig, DelayLine, ModelMailbox, PlantMailbox, RateTicker

__all__ = ["DelayedModelWriter", "DelayedPlant", "DelayedPlantWriter", "Links", "LinksConfig", "load_delay_trace"]

PUMP_HZ = 1000.0
TRACE_5G = Path(__file__).resolve().parents[4] / "src/teleop/network_sim/data/5G-data.csv"


def load_delay_trace(source: str, filters: dict[str, str] | None = None, scale: float = 0.5) -> tuple[float, ...]:
    """Delays [ms] from a recorded latency trace, times ``scale``.

    Args:
        source: a CSV with a ``latency`` column [ms], or ``5g`` for the lab's 5G round-trip
            measurements (``src/teleop/network_sim/data/5G-data.csv``: columns ``condition``
            (5G, 5GE), ``server``, ``location``, ``record_time``).
        filters: keep only the rows whose columns equal these values, e.g. ``{"condition": "5G"}``.
        scale: factor on every latency; 0.5 turns a round trip into one link's delay.

    Raises:
        ValueError: if no row is left.
    """
    path = TRACE_5G if source.lower() == "5g" else Path(source).expanduser()
    with open(path, newline="") as fh:
        rows = [r for r in csv.DictReader(fh) if all(r.get(k) == v for k, v in (filters or {}).items())]
    if not rows:
        raise ValueError(f"no rows in {path} match {filters}")
    return tuple(scale * float(r["latency"]) for r in rows)


@dataclass(kw_only=True)
class LinksConfig:
    """The delay of each link (see ``haptic_teleop.DelayConfig``)."""

    feedback: DelayConfig = field(default_factory=DelayConfig)
    command: DelayConfig = field(default_factory=DelayConfig)

    @property
    def enabled(self) -> bool:
        return self.feedback.enabled or self.command.enabled

    @property
    def variable(self) -> bool:
        """True when the feedback delay varies (jitter or a trace): delivery gaps are then irregular."""
        f = self.feedback
        return f.enabled and (f.jitter_ms > 0 or f.trace_ms is not None)

    @staticmethod
    def describe(cfg: DelayConfig) -> dict | None:
        """One link's delay for the run metadata: the config, with a trace summarized instead of listed."""
        if not cfg.enabled:
            return None
        d = dict(vars(cfg))
        trace = d.pop("trace_ms")
        if trace is not None:
            t = np.asarray(trace)
            d["trace"] = {"samples": len(t), "median_ms": float(np.median(t)), "p95_ms": float(np.percentile(t, 95)),
                          "max_ms": float(t.max())}
        return d

    @staticmethod
    def label(cfg: DelayConfig) -> str:
        """A one-line description of one link's delay."""
        text = f"{cfg.base_ms:g} ms + {cfg.jitter_dist} jitter {cfg.jitter_ms:g} ms"
        if cfg.trace_ms is not None:
            t = np.asarray(cfg.trace_ms)
            text += (f" + trace ({cfg.trace_mode}, {len(t)} samples, median {np.median(t):.1f} ms, "
                     f"p95 {np.percentile(t, 95):.1f} ms, max {t.max():.1f} ms)")
        return f"{text} (seed {cfg.seed})"

    def drain_s(self, margin_s: float = 0.1) -> float:
        """How long to wait after setup before the haptic loop starts [s]: the longest feedback
        delay plus ``margin_s``, so a sample gap left by a setup stall has been delivered (and
        is not replayed into the run). 0 without a feedback delay."""
        f = self.feedback
        if not f.enabled:
            return 0.0
        spread = f.jitter_ms if f.jitter_dist == "uniform" else 3 * f.jitter_ms
        trace = max(f.trace_ms) if f.trace_ms else 0.0
        return (f.base_ms + spread + trace) * 1e-3 + margin_s

    def feedback_gaps_s(self, model_update_hz: float, seconds: float) -> tuple[np.ndarray, float]:
        """Simulated gaps between feedback deliveries [s] over ``seconds``, and the fraction of
        samples superseded.

        With a variable delay a sample that draws a short delay supersedes every older one still
        in flight, so deliveries come in irregular gaps, longer than the update period. This
        replays the run's link: a sample every update period, the delays from the same seed, the
        newest due sample delivered on each 1 ms pump tick ([Links.pump][]). Empty without a
        feedback delay.
        """
        f = self.feedback
        if not f.enabled:
            return np.zeros(0), 0.0
        model = f.build()
        sent = np.arange(0.0, seconds, 1.0 / model_update_hz)
        delays = np.array([model.sample_s(t) for t in sent])
        tick = np.ceil((sent + delays) * PUMP_HZ - 1e-9)  # the pump tick each sample is due on
        order = np.lexsort((sent, tick))
        newest = np.maximum.accumulate(sent[order])  # newest sample sent, among those due by then
        advances = np.r_[True, newest[1:] > newest[:-1]]
        delivered_ticks = np.unique(tick[order][advances])
        return np.diff(delivered_ticks) / PUMP_HZ, 1.0 - len(delivered_ticks) / len(sent)


class DelayedPlantWriter:
    """A ``PlantMailbox`` writer whose samples reach the mailbox through a feedback delay."""

    def __init__(self, line: DelayLine, mailbox: PlantMailbox) -> None:
        self.line = line
        self.mailbox = mailbox

    def write(self, x_i, v_i, lam, t_s: float, rx_s: float | None = None, energy_in: float = 0.0) -> None:
        rx_s = time.monotonic() if rx_s is None else rx_s
        self.line.push((np.copy(x_i), np.copy(v_i), np.copy(lam), t_s, rx_s, energy_in), sent_s=rx_s)


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

    def aim(self, x, v, energy_in: float | None = None, tick: int = -1) -> None:
        if not self.frozen:
            self.line.push(("aim", np.copy(x), np.copy(v), energy_in, tick), sent_s=time.monotonic())

    def command(self, x, v=None, f_ff=None, tick: int = -1) -> None:
        if not self.frozen:
            args = ("command", np.copy(x), None if v is None else np.copy(v), None if f_ff is None else np.copy(f_ff), tick)
            self.line.push(args, sent_s=time.monotonic())

    def freeze(self) -> None:
        """Freeze now (not delayed); targets still in flight are dropped when they arrive."""
        self.frozen = True
        self.plant.freeze()

    def deliver(self, target) -> None:
        if self.frozen:
            return
        kind, x, v, extra, tick = target
        if kind == "aim":
            if extra is None:
                self.plant.aim(x, v, tick=tick)
            else:
                self.plant.aim(x, v, energy_in=extra, tick=tick)  # the operator end's energy, with the target
        else:
            self.plant.command(x, v, extra, tick=tick)


@dataclass
class _Records:
    feedback: list[tuple[float, float, float, float]] = field(default_factory=list)  # rx, delivered, delay, t_s
    command: list[tuple[float, float, float, int]] = field(default_factory=list)  # sent, delivered, delay, haptic tick


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
        """What the RIM model loop should write to. The model travels on the feedback link: same
        delay config and seed as the samples, so a constant (or time-indexed) delay is the same
        for both; a per-item random draw (jitter, ``iid`` trace) is not."""
        if not self.cfg.feedback.enabled:
            return model_box
        self.model_writer = DelayedModelWriter(DelayLine(self.cfg.feedback.build()), model_box)
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
                (x_i, v_i, lam, t_s, rx_s, energy_in), _, delay = got
                self.plant_writer.mailbox.write(x_i, v_i, lam, t_s, rx_s=rx_s, delivered_s=now, energy_in=energy_in)
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
                self.records.command.append((sent, now, delay, target[-1]))

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
