"""Device-agnostic teleop interface adapter for the RIM interaction space."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from arm_client.teleop.inverse3_teleop import Inverse3Device
from pyrim import InterfaceFrame

from ..config import InterfaceConfig


@dataclass
class TeleopInterfaceAdapter:
    """Map a 3D teleop device to/from the RIM interface subspace via an InterfaceFrame.

    Expected device API (duck-typed):
      - update_device_state() -> None
      - position_robot -> np.ndarray shape (3,)
      - velocity_robot -> np.ndarray shape (3,)
      - apply_force(np.ndarray shape (3,)) -> None
      - is_connected() -> bool
    """

    device: Inverse3Device | object
    interface_cfg: InterfaceConfig
    frame: InterfaceFrame

    def update(self) -> None:
        """Refresh underlying device state."""
        self.device.update_device_state()

    def _device_state(self) -> tuple[np.ndarray, np.ndarray]:
        position_robot = np.asarray(self.device.position_robot, dtype=float).reshape(3)
        velocity_robot = np.asarray(self.device.velocity_robot, dtype=float).reshape(3)
        return position_robot, velocity_robot

    def get_interface_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Return the device position/velocity projected onto the RIM subspace, shape (k,)."""
        position_robot, velocity_robot = self._device_state()
        return self.frame.project(position_robot), self.frame.project(velocity_robot)

    def get_cartesian_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Return the full 3D device position/velocity in the robot frame, shape (3,)."""
        return self._device_state()

    def set_interface_force(self, force: np.ndarray) -> None:
        """Scale/cap the subspace force and lift it into a 3D device force."""
        f = np.asarray(force, dtype=float).reshape(-1)
        scalar = 0.0 if f.size == 0 else float(f[0])

        scalar = float(
            np.clip(
                self.interface_cfg.force_scale * scalar,
                -self.interface_cfg.force_cap,
                self.interface_cfg.force_cap,
            )
        )

        force_3d = self.frame.lift(scalar)
        self.device.apply_force(force_3d)

    def is_connected(self) -> bool:
        """Forward connected state from underlying device."""
        return bool(self.device.is_connected())
