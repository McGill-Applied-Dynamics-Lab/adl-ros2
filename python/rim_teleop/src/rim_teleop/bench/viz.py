"""Live Foxglove visualization for the haptic bench.

Thin wrapper over ``ExperimentLogger`` (foxglove sink only) so the bench can
stream scalars (for live Plot panels) and a 3D scene (I3 + virtual mass spheres
and the contact surface) without touching the hot loop directly. All publishing
goes through ``log_sample`` — non-blocking (drops when the queue is full) and
served from the logger's own thread, so it never stalls the 1 kHz haptic loop.
"""

from __future__ import annotations

import time

import numpy as np

from ..config import FileSinkConfig, FoxgloveSinkConfig, LoggingConfig
from ..data.logger import ExperimentLogger


class BenchVisualizer:
    """Stream bench state to Foxglove (scalars + 3D scene)."""

    def __init__(
        self,
        rim_direction: list | np.ndarray,
        *,
        port: int = 8765,
        topic_prefix: str = "/bench",
        frame_id: str = "bench",
    ) -> None:
        self.rim_direction = list(np.asarray(rim_direction, dtype=float).reshape(3))
        self.frame_id = frame_id
        logging_cfg = LoggingConfig(
            enabled=True,
            sinks=["foxglove_sink"],
            file_sink=FileSinkConfig(enabled=False),
            foxglove_sink=FoxgloveSinkConfig(enabled=True, port=port, topic_prefix=topic_prefix),
        )
        self.logger = ExperimentLogger(logging_cfg, full_config=None)

    def start(self) -> None:
        self.logger.start()

    def stop(self) -> None:
        self.logger.stop()

    def log_scalars(
        self,
        t: float,
        x_leader: float,
        v_leader: float,
        x_mass: float,
        v_mass: float,
        force: float,
        timestamp_s: float | None = None,
    ) -> None:
        """Stream the 1-DoF scalars for live Plot panels."""
        self.logger.log_sample(
            "bench/state",
            {
                "t": t,
                "x_leader": x_leader,
                "v_leader": v_leader,
                "x_mass": x_mass,
                "v_mass": v_mass,
                "force": force,
            },
            timestamp_s=timestamp_s,
        )

    def log_rate(
        self,
        measured_hz: float,
        target_hz: float,
        mean_dt_ms: float,
        p95_dt_ms: float,
        max_dt_ms: float,
        fresh_rate_hz: float = 0.0,
        stale_fraction: float = 0.0,
        timestamp_s: float | None = None,
    ) -> None:
        """Stream the achieved loop rate + jitter + device fresh-sample rate for live Plot panels."""
        self.logger.log_sample(
            "bench/rate",
            {
                "measured_hz": measured_hz,
                "target_hz": target_hz,
                "mean_dt_ms": mean_dt_ms,
                "p95_dt_ms": p95_dt_ms,
                "max_dt_ms": max_dt_ms,
                "fresh_rate_hz": fresh_rate_hz,
                "stale_fraction": stale_fraction,
            },
            timestamp_s=timestamp_s,
        )

    def log_scene(
        self,
        leader3: np.ndarray,
        mass3: np.ndarray,
        surface: float | None,
        timestamp_s: float | None = None,
    ) -> None:
        """Stream the 3D scene (leader + mass spheres, optional contact surface)."""
        self.logger.log_sample(
            "viz/scene",
            {
                "leader": np.asarray(leader3, dtype=float).reshape(3).tolist(),
                "mass": np.asarray(mass3, dtype=float).reshape(3).tolist(),
                "surface": None if surface is None else float(surface),
                "rim_direction": self.rim_direction,
                "frame_id": self.frame_id,
            },
            timestamp_s=timestamp_s if timestamp_s is not None else time.time(),
        )
