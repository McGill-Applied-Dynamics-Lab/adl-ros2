import abc
from typing import Any
import numpy as np


class BaseTeleop(abc.ABC):
    """Base interface for all teleoperation devices."""

    @property
    @abc.abstractmethod
    def target_position(self) -> np.ndarray:
        """The desired end-effector translation (x, y, z) in base frame."""

    @property
    @abc.abstractmethod
    def target_orientation(self) -> np.ndarray:
        """The desired end-effector orientation (w, x, y, z) in base frame."""

    @abc.abstractmethod
    def start(self) -> None:
        """Start the teleoperation loop/stream."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Stop the teleoperation loop/stream."""
