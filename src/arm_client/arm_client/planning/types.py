"""Typed planning data models."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class CartesianWaypoint:
    """A Cartesian waypoint sampled along a path."""

    position: np.ndarray
    orientation: Rotation
    s: float


@dataclass
class PlannedJointTrajectory:
    """A time-parameterized joint trajectory."""

    joint_names: list[str]
    time_from_start: list[float]
    joint_positions: np.ndarray
