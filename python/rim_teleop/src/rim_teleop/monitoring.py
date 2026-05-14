"""Rate and jitter monitoring for multi-rate loops."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import statistics
import time


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
