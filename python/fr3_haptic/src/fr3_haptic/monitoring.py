"""Rate and jitter monitoring for multi-rate loops."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import statistics
import time

import numpy as np


@dataclass
class LoopSnapshot:
    """Latest loop timing summary."""

    target_hz: float
    measured_hz: float
    mean_dt_ms: float
    p95_dt_ms: float
    max_dt_ms: float


class LoopRateMonitor:
    """Collect and summarize dt statistics from a loop."""

    def __init__(self, target_hz: float, window_size: int = 300) -> None:
        self.target_hz = target_hz
        self._periods: deque[float] = deque(maxlen=window_size)
        self._last_tick: float | None = None

    def tick(self) -> None:
        now = time.perf_counter()
        if self._last_tick is not None:
            self._periods.append(now - self._last_tick)
        self._last_tick = now

    def snapshot(self) -> LoopSnapshot:
        if not self._periods:
            return LoopSnapshot(self.target_hz, 0.0, 0.0, 0.0, 0.0)
        periods = list(self._periods)
        mean_dt = statistics.mean(periods)
        sorted_periods = sorted(periods)
        p95_idx = max(0, int(0.95 * (len(sorted_periods) - 1)))
        p95_dt = sorted_periods[p95_idx]
        max_dt = max(sorted_periods)
        return LoopSnapshot(
            target_hz=self.target_hz,
            measured_hz=(1.0 / mean_dt) if mean_dt > 0.0 else 0.0,
            mean_dt_ms=1000.0 * mean_dt,
            p95_dt_ms=1000.0 * p95_dt,
            max_dt_ms=1000.0 * max_dt,
        )


@dataclass
class FreshnessSnapshot:
    """Effective fresh-sample rate and the fraction of stale (duplicate) reads."""

    fresh_rate_hz: float
    stale_fraction: float


class FreshnessMonitor:
    """Detect stale device reads to measure the effective fresh-sample rate.

    Devices like the Inverse3 publish state from a background thread; a polling
    loop that runs faster than that thread re-reads the same cached sample. Two
    consecutive reads with bit-identical state therefore mean no new sample
    arrived in between — real samples always differ by sensor noise. (A truly
    motionless, noise-free source would be miscounted as stale, but that does
    not happen with a live device.)

    Tracks a windowed rate for live streaming and whole-run totals for a
    per-run summary.
    """

    def __init__(self, window_size: int = 300) -> None:
        self._fresh: deque[bool] = deque(maxlen=window_size)
        self._times: deque[float] = deque(maxlen=window_size)
        self._prev: np.ndarray | None = None
        self._total = 0
        self._total_fresh = 0
        self._t_first: float | None = None
        self._t_last: float = 0.0

    def update(self, state: np.ndarray) -> bool:
        """Record a device read (e.g. concatenated position+velocity). Returns is_fresh."""
        now = time.perf_counter()
        state = np.asarray(state, dtype=float)
        is_fresh = self._prev is None or not np.array_equal(state, self._prev)
        self._prev = state
        self._fresh.append(is_fresh)
        self._times.append(now)
        self._total += 1
        self._total_fresh += int(is_fresh)
        if self._t_first is None:
            self._t_first = now
        self._t_last = now
        return is_fresh

    def snapshot(self) -> FreshnessSnapshot:
        """Windowed fresh-sample rate + stale fraction (recent window)."""
        n = len(self._fresh)
        if n < 2:
            return FreshnessSnapshot(0.0, 0.0)
        fresh_count = sum(self._fresh)
        span = self._times[-1] - self._times[0]
        fresh_rate = fresh_count / span if span > 0 else 0.0
        return FreshnessSnapshot(fresh_rate, 1.0 - fresh_count / n)

    def summary(self) -> FreshnessSnapshot:
        """Whole-run fresh-sample rate + stale fraction."""
        span = self._t_last - self._t_first if self._t_first is not None else 0.0
        if self._total < 2 or span <= 0.0:
            return FreshnessSnapshot(0.0, 0.0)
        return FreshnessSnapshot(self._total_fresh / span, 1.0 - self._total_fresh / self._total)
