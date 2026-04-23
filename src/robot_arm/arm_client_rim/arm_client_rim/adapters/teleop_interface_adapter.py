"""Device-agnostic teleop interface adapter for reduced RIM interaction space."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import InterfaceConfig
from arm_client.teleop.inverse3_teleop import Inverse3Device


_AXIS_TO_INDEX = {"x": 0, "y": 1, "z": 2}


@dataclass
class TeleopInterfaceAdapter:
    """Convert 3D teleop device state/forces to reduced interface signals.

    Expected device API (duck-typed):
      - update_device_state() -> None
      - position_robot -> np.ndarray shape (3,)
      - velocity_robot -> np.ndarray shape (3,)
      - apply_force(np.ndarray shape (3,)) -> None
      - is_connected() -> bool
    """

    device: Inverse3Device | object
    interface_cfg: InterfaceConfig

    def __post_init__(self) -> None:
        if self.interface_cfg.axis not in _AXIS_TO_INDEX:
            raise ValueError(f"Unsupported interface axis: {self.interface_cfg.axis}")
        self._axis = _AXIS_TO_INDEX[self.interface_cfg.axis]
        self._initial_position: np.ndarray | None = None

    def update(self) -> None:
        """Refresh underlying device state."""
        self.device.update_device_state()

    def get_interface_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Return reduced interface position and velocity, both shape (1,)."""
        position_robot = np.asarray(self.device.position_robot, dtype=float).reshape(3)
        velocity_robot = np.asarray(self.device.velocity_robot, dtype=float).reshape(3)

        if self._initial_position is None:
            self._initial_position = position_robot.copy()

        # delta = self.interface_cfg.interface_workspace_scale * (position_robot - self._initial_position)
        # velocity_scaled = self.interface_cfg.interface_workspace_scale * velocity_robot

        return (
            np.array([position_robot[self._axis]], dtype=float),
            np.array([velocity_robot[self._axis]], dtype=float),
        )

    def set_interface_force(self, force: np.ndarray) -> None:
        """Map reduced force (shape (1,)) to 3D robot-frame/device-frame force."""
        f = np.asarray(force, dtype=float).reshape(-1)
        if f.size == 0:
            scalar = 0.0
        else:
            scalar = float(f[0])

        scalar = float(
            np.clip(
                self.interface_cfg.force_scale * scalar,
                -self.interface_cfg.force_cap,
                self.interface_cfg.force_cap,
            )
        )

        force_3d = np.zeros(3, dtype=float)
        force_3d[self._axis] = scalar
        self.device.apply_force(force_3d)

    def is_connected(self) -> bool:
        """Forward connected state from underlying device."""
        return bool(self.device.is_connected())
