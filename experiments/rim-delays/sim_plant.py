"""A simulated FR3 for running fr3_teleop's pipeline without the robot.

The arm at the end effector is a 3-D point mass with the FR3's task-space inertia at home,
driven by what ``osc_controller`` does with decoupling off: a per-axis PD toward the target
pose with twist feedforward, plus the feedforward wrench of the proxy methods. Gravity is
compensated, as on the robot. The table is a stiff spring-damper below ``table_z`` with
regularized Coulomb friction, and the arm has some viscous joint-like damping.

[SimFR3][] stands in for both robot-side objects of a run:

- `FR3Plant` (command side): ``aim``, ``command``, ``freeze``; samples
  ``(x_i, v_i, lam, t_s)`` written to a ``PlantMailbox`` (or its delayed stand-in) at the model
  update rate. ``lam`` is the negative task force on the interface, as on the robot.
- `FR3System` (RIM model): ``update``, ``ready``, ``model``. The model is the point mass itself
  (``M`` = the inertia, no bias), so RIM sees the exact plant.

Physics runs on its own thread at ``physics_hz``, in real time (the haptic process is real time).
Ground truth (positions, task and contact forces) is recorded at every ``record_every`` steps.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np
from haptic_teleop import RateTicker
from pyrim import DynModel, InterfaceFrame

__all__ = ["PRESET_K", "TRUTH_COLS", "SimConfig", "SimFR3", "osc_gains"]

TASK_INERTIA = (11.4, 4.6, 5.4)  # FR3 Lambda_ii at home [kg], experiments/benchmarks/osc_controller/FINDINGS.md
PRESET_K = (1500.0, 1500.0, 3500.0)  # configs/osc_teleop.yaml held-axis stiffness [N/m]
DAMPING_RATIO = 0.7


def osc_gains(
    axes: str, coupling: tuple[float, float] | None, mass=TASK_INERTIA
) -> tuple[np.ndarray, np.ndarray]:
    """``(K, D)`` per base axis as fr3_teleop leaves ``osc_controller``: the teleop preset, with the
    interface axes set to the coupling ``(kv, dv)`` for the coupling methods (``None`` for the
    proxy methods, which keep the preset). Preset damping: ``2 zeta sqrt(K m)``."""
    K = np.array(PRESET_K, dtype=float)
    D = 2 * DAMPING_RATIO * np.sqrt(K * np.asarray(mass, dtype=float))
    if coupling is not None:
        for a in axes:
            i = "xyz".index(a)
            K[i], D[i] = coupling
    return K, D


@dataclass(kw_only=True)
class SimConfig:
    """Simulated plant parameters.

    Args:
        mass: task-space inertia per base axis [kg].
        start: initial end-effector position, base frame [m].
        table_z: table surface height [m].
        table_stiffness: [N/m]. table_damping: [N·s/m], on penetration velocity, pushing only.
        friction: Coulomb coefficient on the table; ``friction_v``: regularization speed [m/s].
        joint_damping: viscous damping on every axis [N·s/m].
        physics_hz: integration rate [Hz].
        record_every: ground-truth record decimation (steps).
        record_s: ground truth kept for this long [s]; preallocated, later steps are not recorded.
    """

    mass: tuple[float, float, float] = TASK_INERTIA
    start: tuple[float, float, float] = (0.4, 0.0, 0.2)
    table_z: float = 0.1
    table_stiffness: float = 3.0e4
    table_damping: float = 300.0
    friction: float = 0.2
    friction_v: float = 0.005
    joint_damping: float = 5.0
    physics_hz: float = 2000.0
    record_every: int = 2
    record_s: float = 600.0


TRUTH_COLS = ("t",) + tuple(f"{n}_{k}" for n in ("x", "v", "target", "f_task", "f_contact") for k in "xyz")


class SimFR3:
    """See the module docstring.

    Args:
        cfg: plant parameters.
        frame: the interface frame.
        gains: ``(K, D)`` per base axis (`osc_gains`).
        sink: ``PlantMailbox`` or a delayed stand-in (``write(x_i, v_i, lam, t_s, rx_s=...)``).
        sample_hz: plant samples per second to ``sink``.
        port: the robot end of a two-port TDPA (`fr3_haptic.port.RobotPort`), as in `FR3Plant`.
    """

    def __init__(self, cfg: SimConfig, frame: InterfaceFrame, gains, *, sink, sample_hz: float, port=None) -> None:
        self.cfg = cfg
        self.frame = frame
        self.K, self.D = (np.asarray(g, dtype=float) for g in gains)
        self.sink = sink
        self.port = port
        self.dt = 1.0 / cfg.physics_hz
        self._sample_every = max(1, round(cfg.physics_hz / sample_hz))
        self.m = np.asarray(cfg.mass, dtype=float)
        self.x = np.asarray(cfg.start, dtype=float).copy()
        self.v = np.zeros(3)
        self.hold = self.x.copy()
        self._target = (self.x.copy(), np.zeros(3), np.zeros(3))  # pose, twist, wrench feedforward
        self._frozen = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        # preallocated: a growing list of rows makes the collector stall every thread
        self._truth = np.full((int(cfg.record_s * cfg.physics_hz / cfg.record_every) + 1, len(TRUTH_COLS)), np.nan)
        self._n_truth = 0
        self.f_task = np.zeros(3)
        self.f_contact = np.zeros(3)
        self._model: DynModel | None = None
        self.samples = 0
        self.targets: list = []  # as FR3Plant.targets: (time.monotonic(), haptic tick, position, x, v, f_ff)
        self.t0 = time.monotonic()

    # -- FR3Plant surface -------------------------------------------------------------------
    @property
    def dim(self) -> int:
        return self.frame.dim

    def aim(self, x, v, energy_in: float | None = None, tick: int = -1) -> None:
        f_ff = None
        if self.port is not None:
            if energy_in is not None:
                self.port.receive(energy_in)
            f_ff = self.port.wrench(time.monotonic())
        self._set(x, v, f_ff, tick)

    def command(self, x, v=None, f_ff=None, tick: int = -1) -> None:
        self._set(x, np.zeros(self.dim) if v is None else v, f_ff, tick)

    def freeze(self) -> None:
        with self._lock:
            self._frozen = True
            self._target = (self.x.copy(), np.zeros(3), np.zeros(3))

    def _set(self, x, v, f_ff, tick: int = -1) -> None:
        pose = self.frame.compose(np.asarray(x, dtype=float), self.hold)
        twist = self.frame.lift(np.asarray(v, dtype=float))
        wrench = np.zeros(3) if f_ff is None else self.frame.lift(np.asarray(f_ff, dtype=float))
        with self._lock:
            if not self._frozen:
                self._target = (pose, twist, wrench)
                f = None if f_ff is None else np.asarray(f_ff, dtype=float)
                self.targets.append((time.monotonic(), int(tick), pose.copy(), np.asarray(x, dtype=float).copy(),
                                     np.asarray(v, dtype=float).copy(), f))

    # -- FR3System surface ------------------------------------------------------------------
    def update(self) -> None:
        with self._lock:
            x, v = self.x.copy(), self.v.copy()
        B = self.frame.basis
        self._model = DynModel(
            n=3, m=self.dim, q=x, q_dot=v, x_i=B.T @ x, v_i=B.T @ v, M=np.diag(self.m), c=np.zeros(3),
            J_i=B.T.copy(), b_i=np.zeros(self.dim), stamp_s=time.monotonic(),
        )

    @property
    def ready(self) -> bool:
        return self._model is not None

    @property
    def model(self) -> DynModel:
        if self._model is None:
            raise RuntimeError("no model yet: call update()")
        return self._model

    # -- physics ----------------------------------------------------------------------------
    def contact_force(self, x: np.ndarray, v: np.ndarray) -> np.ndarray:
        c = self.cfg
        pen = c.table_z - x[2]
        if pen <= 0:
            return np.zeros(3)
        fn = max(c.table_stiffness * pen - c.table_damping * v[2], 0.0)
        vt = v[:2]
        speed = np.linalg.norm(vt)
        ft = -c.friction * fn * np.tanh(speed / c.friction_v) * (vt / speed) if speed > 1e-9 else np.zeros(2)
        return np.array([ft[0], ft[1], fn])

    def step(self) -> tuple[np.ndarray, ...]:
        """One semi-implicit Euler step."""
        with self._lock:
            pose, twist, wrench = self._target
            f_task = self.K * (pose - self.x) + self.D * (twist - self.v) + wrench
            f_contact = self.contact_force(self.x, self.v)
            a = (f_task + f_contact - self.cfg.joint_damping * self.v) / self.m
            self.v = self.v + self.dt * a
            self.x = self.x + self.dt * self.v
            self.f_task, self.f_contact = f_task, f_contact
            x, v = self.x.copy(), self.v.copy()
        return x, v, pose, f_task, f_contact

    def _run(self) -> None:
        ticker = RateTicker(self.cfg.physics_hz)
        k = 0
        B = self.frame.basis
        while not self._stop.is_set():
            x, v, pose, f_task, f_contact = self.step()
            now = time.monotonic()
            if self.port is not None:
                self.port.observe(B.T @ f_task, B.T @ v, now)
            if k % self._sample_every == 0:
                energy = self.port.energy_in if self.port is not None else 0.0
                self.sink.write(B.T @ x, B.T @ v, -(B.T @ f_task), now, rx_s=now, energy_in=energy)
                self.samples += 1
            if k % self.cfg.record_every == 0 and self._n_truth < len(self._truth):
                self._truth[self._n_truth] = (now, *x, *v, *pose, *f_task, *f_contact)
                self._n_truth += 1
            k += 1
            ticker.sleep()

    @property
    def truth(self) -> np.ndarray:
        """Ground truth so far, one row per record, columns `TRUTH_COLS` (``t`` is ``time.monotonic()``)."""
        return self._truth[: self._n_truth]

    def start(self) -> None:
        self.t0 = time.monotonic()
        self._thread = threading.Thread(target=self._run, name="sim_fr3", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
