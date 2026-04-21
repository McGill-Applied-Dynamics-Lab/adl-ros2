"""Thread-safe RIM integrator with unilateral contact projection."""

from __future__ import annotations

import threading
import numpy as np

from .rim_types import RIM


class DelayRIM:
    """
    DelayRIM Algorithm. Estimates the current interface forces from the RIM model and previous states.
    Integrates the reduced dynamics at high rate.
    """

    def __init__(
        self,
        interface_dim: int,
        dt: float,
        stiffness: float,
        damping: float,
        contact_surface: float,
    ) -> None:
        self._m = interface_dim
        self._dt = dt
        self._k = stiffness
        self._d = damping
        self._contact_surface = contact_surface
        self._lock = threading.Lock()

        self._leader_pos = np.zeros(self._m)
        self._leader_vel = np.zeros(self._m)

        self._rim: RIM | None = None
        self._mass_factor: np.ndarray | None = None
        self._inv_aug: np.ndarray | None = None
        self._interface_force = np.zeros(self._m)

    def add_leader_state(self, position: np.ndarray, velocity: np.ndarray) -> None:
        with self._lock:
            self._leader_pos = position.reshape(self._m).copy()
            self._leader_vel = velocity.reshape(self._m).copy()

    def update_rim(self, rim: RIM) -> None:
        with self._lock:
            if self._rim is None:
                self._rim = rim
            else:
                self._rim.M_eff = rim.M_eff
                self._rim.z_i = rim.z_i
                self._rim.f_eff = rim.f_eff
                # Preserve integrated state and only refresh model terms.
            eye = np.eye(self._m)
            aug = self._rim.M_eff + self._dt * (self._d + self._dt * self._k) * eye
            self._inv_aug = np.linalg.inv(aug)
            self._mass_factor = self._inv_aug @ self._rim.M_eff

    def step(self) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
        with self._lock:
            if self._rim is None or self._mass_factor is None or self._inv_aug is None:
                return None, None

            x_rim = self._rim.x
            v_rim = self._rim.v

            phi = x_rim - self._leader_pos
            regular = self._rim.f_eff - self._k * phi + (self._d + self._dt * self._k) * self._leader_vel

            v_next = self._mass_factor @ v_rim + self._dt * self._inv_aug @ regular
            x_next = x_rim + self._dt * v_rim

            if x_next[0] < self._contact_surface:
                x_next[0] = self._contact_surface
                if v_next[0] < 0.0:
                    v_next[0] = 0.0

            self._rim.x = x_next
            self._rim.v = v_next

            self._interface_force = self._k * (x_next - self._leader_pos) + self._d * (v_next - self._leader_vel)
            return x_next.copy(), v_next.copy()

    def get_interface_force(self) -> np.ndarray:
        with self._lock:
            return self._interface_force.copy()

    def get_rim_state(self) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
        with self._lock:
            if self._rim is None:
                return None, None
            return self._rim.x.copy(), self._rim.v.copy()
