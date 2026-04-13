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
    """A time-parameterized joint trajectory.

    For s points and N joints.

    Leaving the value to NaN for joint_velocities or accels will let the controller interpolate
    """

    joint_names: list[str]
    time_from_start: list[float]  # List with the time from start for each Point (s)
    joint_positions: np.ndarray  # Array of joint positions (s, N)
    joint_velocities: np.ndarray  # Array of joint velocities (s, N)
    joint_accelerations: np.ndarray  # Array of joint accelerations (s, N)
