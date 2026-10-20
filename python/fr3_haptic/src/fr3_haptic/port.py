r"""Two-port TDPA, the robot end (the operator end is ``haptic_teleop.TDPA2Rendering``).

Runs where the robot's state is fresh: after the command delay, next to ``osc_controller``. It
observes the robot port on every controller tick and damps the robot, through the OSC's
feedforward wrench, when the energy it has put out exceeds what the operator end has put in
(received with the commands, delayed). Its own input energy goes back with the plant samples.

With $F$ the OSC's task force on the robot (projected on the interface; it includes the
feedforward wrench, so the damping applied here is booked by the observer itself) and $v$ the
robot velocity, the power into the network is $-F \cdot v$:

$$E^{in}_s \mathrel{+}= \max(-F\cdot v\,\Delta t, 0), \quad E^{out}_s \mathrel{+}= \max(F\cdot v\,\Delta t, 0), \quad
  W_s = E^{in}_m(t - T_c) - E^{out}_s .$$

When $W_s < 0$ the next command carries the damping wrench $-\alpha v$, with
$\alpha = \min(-W_s / (T\, v \cdot v), \alpha_{max})$ and $T$ the command period.
"""

from __future__ import annotations

import threading

import numpy as np

__all__ = ["RobotPort"]


class RobotPort:
    """The robot end of a two-port TDPA. Thread-safe: observed from the plant's callback thread,
    asked for the wrench from the command thread.

    Args:
        max_damping: cap on the damping $\\alpha$ [N·s/m].
        min_speed: below this robot speed [m/s] no damping is applied (the deficit waits).
        record: keep ``(t_s, E_in, E_out, remote E_in, alpha)`` per command, for logging.
    """

    def __init__(self, max_damping: float = 50.0, min_speed: float = 1e-3, record: bool = True) -> None:
        if max_damping < 0 or min_speed < 0:
            raise ValueError("max_damping and min_speed must not be negative")
        self.max_damping = float(max_damping)
        self.min_speed = float(min_speed)
        self._lock = threading.Lock()
        self._e_in = 0.0
        self._e_out = 0.0
        self._remote_in = 0.0
        self._t_prev: float | None = None
        self._v = None
        self._alpha = 0.0
        self._last_cmd_s: float | None = None
        self.history: list[tuple[float, float, float, float, float]] | None = [] if record else None

    @property
    def energy_in(self) -> float:
        """$E^{in}_s$ [J], sent back to the operator end with the plant samples."""
        with self._lock:
            return self._e_in

    @property
    def energy_out(self) -> float:
        with self._lock:
            return self._e_out

    @property
    def po_energy(self) -> float:
        """$W_s$ [J]; ``>= 0`` while this end has put out no more than the operator end put in."""
        with self._lock:
            return self._remote_in - self._e_out

    @property
    def damping(self) -> float:
        """The damping in the last wrench [N·s/m]."""
        return self._alpha

    def observe(self, force: np.ndarray, velocity: np.ndarray, t_s: float) -> None:
        """Book one controller tick: task force on the robot ``force`` [N], robot ``velocity``
        [m/s] (both on the interface), controller time ``t_s`` [s]."""
        with self._lock:
            if self._t_prev is not None and t_s > self._t_prev:
                p = -float(np.dot(force, velocity)) * (t_s - self._t_prev)
                if p > 0:
                    self._e_in += p
                else:
                    self._e_out -= p
            self._t_prev = t_s
            self._v = np.asarray(velocity, dtype=float).copy()

    def receive(self, remote_energy_in: float) -> None:
        """The operator end's input energy, as delivered with a command (never goes back)."""
        if remote_energy_in is None or not np.isfinite(remote_energy_in):
            return
        with self._lock:
            self._remote_in = max(self._remote_in, float(remote_energy_in))

    def wrench(self, now_s: float) -> np.ndarray | None:
        """The damping wrench for the command sent at ``now_s`` [s] (local clock), or ``None``
        before the first observed tick."""
        with self._lock:
            v = self._v
            if v is None:
                return None
            period = 1e-3 if self._last_cmd_s is None else max(now_s - self._last_cmd_s, 1e-4)
            self._last_cmd_s = now_s
            w = self._remote_in - self._e_out
            vv = float(v @ v)
            alpha = 0.0
            if w < 0 and vv > self.min_speed**2:
                alpha = min(-w / (period * vv), self.max_damping)
            self._alpha = alpha
            if self.history is not None:
                self.history.append((now_s, self._e_in, self._e_out, self._remote_in, alpha))
            return -alpha * v
