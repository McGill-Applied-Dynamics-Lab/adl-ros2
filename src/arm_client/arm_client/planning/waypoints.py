"""Cartesian waypoint generators for trajectory planning."""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from arm_client.planning.types import CartesianWaypoint


def generate_linear_waypoints(
    start_position: np.ndarray,
    start_orientation: Rotation,
    end_position: np.ndarray,
    end_orientation: Rotation,
    num_waypoints: int = 10,
) -> list[CartesianWaypoint]:
    """Generate linear Cartesian waypoints between start and end poses."""
    if num_waypoints < 2:
        raise ValueError("num_waypoints must be >= 2")

    slerp = Slerp(
        [0.0, 1.0],
        Rotation.concatenate(
            [
                start_orientation,
                end_orientation,
            ]
        ),
    )

    waypoints: list[CartesianWaypoint] = []
    for i in range(num_waypoints):
        s = i / (num_waypoints - 1)
        position = start_position * (1.0 - s) + end_position * s
        orientation = slerp(s)
        waypoints.append(
            CartesianWaypoint(
                position=np.array(position, dtype=float),
                orientation=orientation,
                s=float(s),
            )
        )

    return waypoints


def generate_spherical_waypoints(
    start_position: np.ndarray,
    start_orientation: Rotation,
    radius: float,
    theta_deg: float,
    phi_deg: float,
    num_waypoints: int = 10,
) -> list[CartesianWaypoint]:
    """Generate spherical waypoints around a center point."""
    if num_waypoints < 2:
        raise ValueError("num_waypoints must be >= 2")

    local_z = start_orientation.apply(np.array([0.0, 0.0, 1.0]))
    center = start_position + local_z * radius

    local_x = start_orientation.apply(np.array([1.0, 0.0, 0.0]))
    local_y = start_orientation.apply(np.array([0.0, 1.0, 0.0]))

    phi_rad = np.deg2rad(phi_deg)
    rot_axis = local_x * np.cos(phi_rad) + local_y * np.sin(phi_rad)
    rot_axis = rot_axis / np.linalg.norm(rot_axis)

    theta_rad = np.deg2rad(theta_deg)

    waypoints: list[CartesianWaypoint] = []
    vec_center_to_start = start_position - center
    for i in range(num_waypoints):
        s = i / (num_waypoints - 1)
        current_angle = s * theta_rad
        r_step = Rotation.from_rotvec(current_angle * rot_axis)

        new_vec = r_step.apply(vec_center_to_start)
        position = center + new_vec
        orientation = r_step * start_orientation

        waypoints.append(
            CartesianWaypoint(
                position=np.array(position, dtype=float),
                orientation=orientation,
                s=float(s),
            )
        )

    return waypoints
