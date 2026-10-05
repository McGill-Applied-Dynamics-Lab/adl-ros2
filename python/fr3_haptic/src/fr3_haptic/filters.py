"""Simple reusable filters used by model estimation."""

from __future__ import annotations

import numpy as np


class LowPassFilter:
    """First-order low-pass filter: y[k] = a*x[k] + (1-a)*y[k-1]."""

    def __init__(self, alpha: float) -> None:
        self.alpha = float(np.clip(alpha, 0.0, 1.0))
        self.value: np.ndarray | None = None

    def update(self, measurement: np.ndarray) -> np.ndarray:
        measurement = np.asarray(measurement)
        if self.value is None:
            self.value = measurement.copy()
        else:
            self.value = self.alpha * measurement + (1.0 - self.alpha) * self.value
        return self.value.copy()
