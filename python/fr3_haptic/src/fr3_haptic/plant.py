"""The real FR3 as a ``haptic_teleop`` plant.

``haptic_teleop`` rendering methods never talk to a robot: they are fed a `PlantState`
(interface position, velocity, coupling force, plant time) at the plant rate and, for RIM, a
``pyrim.DynModel`` at the control rate. This module produces both from the live FR3 and sends
the leader (or RIM proxy) back as ``osc_controller`` targets.

Roles, matching adl-python ``examples/04_i3_newton_fr3_coupling.py``:

- the follower-side coupling spring runs on franka-pc, in ``osc_controller`` at 1 kHz
  (``inertia_decoupling: false``, interface-axis gains = the coupling ``K``, ``D``);
- [FR3Plant.aim][] streams the leader state as ``target_pose`` + ``target_twist`` (ZOH,
  linear, TDPA); [FR3Plant.command][] streams the proxy state and a feedforward wrench
  (RIM, fixed-mass);
- each [FR3PlantSample][] is built from ``/fr3/osc/ee_state`` and ``/fr3/osc/task_wrench``
  published by the same controller tick, paired by their identical header stamp.

Threading: subscription callbacks run on the `Robot` executor and replace
[FR3Plant.sample][] whole, as an ``(updates, sample)`` tuple. The haptic thread reads it without
a lock and detects a fresh sample by the update counter, as in example 04.

Clocks: ``t_s`` is the controller's stamp (franka-pc clock). It is only differenced against
other stamps from the same clock. Staleness uses the local receive time (``rx_s``) instead,
because the two PCs are only NTP-synchronised.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np
from arm_client.robot import Pose, Robot, Twist
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from pyrim import DynModel, InterfaceFrame
from rclpy.qos import qos_profile_sensor_data

from .adapters import RobotModelAdapter

__all__ = ["FR3Plant", "FR3PlantSample", "FR3System", "StalenessWatchdog"]


@dataclass(frozen=True)
class FR3PlantSample:
    """One controller tick of the real FR3, for a rendering method to consume.

    Satisfies ``haptic_teleop.PlantState`` structurally (``x_i``, ``v_i``, ``lam``, ``t_s``).
    The 3-D fields are for logging.

    Attributes:
        x_i: ``(m,)`` interface position [m].
        v_i: ``(m,)`` interface velocity [m/s].
        lam: ``(m,)`` coupling force in the operator-should-feel sign [N]: the negative of
            the task force the controller applied to the robot, projected on the interface.
        t_s: controller time of the tick [s] (franka-pc clock).
        rx_s: local ``time.monotonic()`` when the sample was completed [s].
        ee_position: ``(3,)`` measured EE position, base frame [m].
        ee_velocity: ``(3,)`` measured EE linear velocity, base frame [m/s].
        task_force: ``(3,)`` task force applied to the robot, base frame [N].
    """

    x_i: np.ndarray
    v_i: np.ndarray
    lam: np.ndarray
    t_s: float
    rx_s: float
    ee_position: np.ndarray
    ee_velocity: np.ndarray
    task_force: np.ndarray


def _stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class FR3Plant:
    """Plant state from the live FR3, and interface targets back to ``osc_controller``.

    Puts ``robot`` in streaming mode on construction (its periodic target republishing would
    otherwise send a second, held copy of the targets); call [close][FR3Plant.close] to leave it.

    Args:
        robot: a ready `Robot` (``wait_until_ready`` done). Its node and executor are reused.
        frame: the interface frame; ``frame.dim`` is the method's ``dim``.
        hold_pose: the pose held on the axes the interface does not control, and the
            orientation target. Usually the pose at the start of the run.
        ee_state_topic: ``osc_controller`` EE state topic (``nav_msgs/Odometry``).
        task_wrench_topic: ``osc_controller`` task wrench topic (``WrenchStamped``).
        sample_period_s: keep at most one sample per this many seconds of controller time,
            to emulate a slower plant. 0 keeps every controller tick.
        interface_limits: ``(low, high)`` bounds on the commanded interface coordinate [m],
            applied to every component. ``None`` disables them.
    """

    _MAX_PENDING = 16

    def __init__(
        self,
        robot: Robot,
        frame: InterfaceFrame,
        hold_pose: Pose,
        ee_state_topic: str = "/fr3/osc/ee_state",
        task_wrench_topic: str = "/fr3/osc/task_wrench",
        sample_period_s: float = 0.0,
        interface_limits: tuple[float, float] | None = None,
    ) -> None:
        if interface_limits is not None and interface_limits[0] >= interface_limits[1]:
            raise ValueError(f"interface_limits must be (low, high) with low < high, got {interface_limits}")
        self._robot = robot
        self._frame = frame
        self._hold_pose = hold_pose.copy()
        self._sample_period_ns = round(float(sample_period_s) * 1e9)
        self._limits = interface_limits

        self.sample: tuple[int, FR3PlantSample] | None = None
        """Latest ``(updates, sample)``, replaced whole; ``None`` before the first one."""

        self._updates = 0
        self._last_kept_ns: int | None = None
        self._lock = threading.Lock()
        self._pending_ee: dict[int, Odometry] = {}
        self._pending_wrench: dict[int, WrenchStamped] = {}

        robot.set_target_streaming(True)
        self._subs = [
            robot.node.create_subscription(Odometry, ee_state_topic, self._on_ee_state, qos_profile_sensor_data),
            robot.node.create_subscription(
                WrenchStamped, task_wrench_topic, self._on_task_wrench, qos_profile_sensor_data
            ),
        ]

    # ---------------------------------------------------------------- state side

    @property
    def dim(self) -> int:
        """Interface dimension."""
        return self._frame.dim

    def sample_age_s(self, now_s: float | None = None) -> float:
        """Seconds since the latest sample was received (local clock), ``inf`` before the first."""
        latest = self.sample
        if latest is None:
            return float("inf")
        now_s = time.monotonic() if now_s is None else now_s
        return now_s - latest[1].rx_s

    def _on_ee_state(self, msg: Odometry) -> None:
        self._on_part(_stamp_ns(msg), ee=msg)

    def _on_task_wrench(self, msg: WrenchStamped) -> None:
        self._on_part(_stamp_ns(msg), wrench=msg)

    def _on_part(self, key: int, ee: Odometry | None = None, wrench: WrenchStamped | None = None) -> None:
        """Pair the two halves of a controller tick by stamp; publish the sample when both are in."""
        with self._lock:
            if ee is not None:
                wrench = self._pending_wrench.pop(key, None)
                if wrench is None:
                    self._store(self._pending_ee, key, ee)
                    return
            else:
                ee = self._pending_ee.pop(key, None)
                if ee is None:
                    self._store(self._pending_wrench, key, wrench)
                    return
            # Halves of older ticks will never be completed
            for pending in (self._pending_ee, self._pending_wrench):
                for k in [k for k in pending if k < key]:
                    del pending[k]

            if self._last_kept_ns is not None and key - self._last_kept_ns < self._sample_period_ns:
                return
            self._last_kept_ns = key
            sample = self._make_sample(ee, wrench, key * 1e-9)
            self._updates += 1
            self.sample = (self._updates, sample)

    def _store(self, pending: dict, key: int, msg) -> None:
        pending[key] = msg
        while len(pending) > self._MAX_PENDING:
            del pending[next(iter(pending))]

    def _make_sample(self, ee: Odometry, wrench: WrenchStamped, t_s: float) -> FR3PlantSample:
        p = ee.pose.pose.position
        v = ee.twist.twist.linear
        f = wrench.wrench.force
        position = np.array([p.x, p.y, p.z])
        velocity = np.array([v.x, v.y, v.z])
        force = np.array([f.x, f.y, f.z])
        return FR3PlantSample(
            x_i=self._frame.project(position),
            v_i=self._frame.project(velocity),
            lam=-self._frame.project(force),
            t_s=t_s,
            rx_s=time.monotonic(),
            ee_position=position,
            ee_velocity=velocity,
            task_force=force,
        )

    # ---------------------------------------------------------------- command side

    def aim(self, x: np.ndarray, v: np.ndarray) -> None:
        """Stream the leader state as the coupling target (ZOH, linear, TDPA).

        Args:
            x: ``(dim,)`` leader interface position [m].
            v: ``(dim,)`` leader interface velocity [m/s].
        """
        self._publish(x, v, None)

    def command(self, x: np.ndarray, v: np.ndarray | None = None, f_ff: np.ndarray | None = None) -> None:
        """Stream a proxy state and a feedforward force (RIM, fixed-mass).

        Args:
            x: ``(dim,)`` proxy interface position [m].
            v: ``(dim,)`` proxy interface velocity [m/s]; zero if ``None``.
            f_ff: ``(dim,)`` feedforward force on the robot along the interface [N]; not
                published if ``None`` (the controller zeroes a stale one after its timeout).
        """
        self._publish(x, np.zeros(self.dim) if v is None else v, f_ff)

    def freeze(self) -> None:
        """Hold the robot where it is: last measured interface position, zero velocity and force."""
        latest = self.sample
        x = latest[1].x_i if latest is not None else self._frame.project(self._hold_pose.position)
        self._publish(x, np.zeros(self.dim), np.zeros(self.dim))

    def _publish(self, x: np.ndarray, v: np.ndarray, f_ff: np.ndarray | None) -> None:
        x = np.asarray(x, dtype=float)
        v = np.asarray(v, dtype=float).copy()
        if self._limits is not None:
            low, high = self._limits
            x_clipped = np.clip(x, low, high)
            # No velocity pushing further out of the limits
            v[(x_clipped <= low) & (v < 0.0)] = 0.0
            v[(x_clipped >= high) & (v > 0.0)] = 0.0
            x = x_clipped
        pose = Pose(self._frame.compose(x, self._hold_pose.position), self._hold_pose.orientation)
        twist = Twist(self._frame.lift(v), np.zeros(3))
        force = None if f_ff is None else self._frame.lift(np.asarray(f_ff, dtype=float))
        self._robot.publish_target(pose=pose, twist=twist, force=force)

    def close(self) -> None:
        """Destroy the subscriptions and leave streaming mode."""
        for sub in self._subs:
            self._robot.node.destroy_subscription(sub)
        self._subs = []
        self._robot.set_target_streaming(False)


class StalenessWatchdog:
    """Gain in ``[0, 1]`` that ramps the haptic force down when the plant goes quiet.

    Feed it the sample age every haptic tick and multiply the rendered force by the result.
    While the age exceeds ``max_age_s`` the gain ramps to 0 over ``ramp_s``; once fresh again
    it ramps back up, unless ``latch`` is set, in which case the first trip holds it at 0 for
    the rest of the run (a stale trial is invalid anyway).

    Args:
        max_age_s: largest acceptable sample age [s].
        ramp_s: time to ramp the gain fully down or up [s].
        latch: keep the gain at 0 after the first trip.
    """

    def __init__(self, max_age_s: float, ramp_s: float = 0.1, latch: bool = True) -> None:
        if max_age_s <= 0 or ramp_s <= 0:
            raise ValueError(f"max_age_s and ramp_s must be positive, got {max_age_s}, {ramp_s}")
        self.max_age_s = float(max_age_s)
        self.ramp_s = float(ramp_s)
        self.latch = latch
        self.gain = 0.0
        self.stale = True
        self.tripped = False
        self.trips = 0

    def update(self, age_s: float, dt: float) -> float:
        """Advance by ``dt`` [s] with the current sample age [s]; return the gain."""
        stale = age_s > self.max_age_s
        if stale and not self.stale and not self.tripped:
            self.trips += 1
            self.tripped = self.latch
        self.stale = stale
        step = dt / self.ramp_s
        if stale or self.tripped:
            self.gain = max(0.0, self.gain - step)
        else:
            self.gain = min(1.0, self.gain + step)
        return self.gain


class FR3System:
    """``pyrim.SystemInterface`` over the Pinocchio model of the live FR3 (for RIM).

    Note the interface point: `RobotModelAdapter` computes ``x_i`` at its configured EE /
    tool-tip frame, while [FR3PlantSample][] uses ``osc_controller``'s end-effector frame
    (libfranka ``kEndEffector``). They must describe the same point for RIM and the coupling
    methods to be comparable.

    Args:
        adapter: a `RobotModelAdapter` on the live robot.
    """

    def __init__(self, adapter: RobotModelAdapter) -> None:
        self._adapter = adapter
        self._model: DynModel | None = None

    def update(self) -> None:
        """Recompute the model from the latest robot state; keeps the previous one if no state yet."""
        model = self._adapter.compute()
        if model is not None:
            self._model = model

    @property
    def ready(self) -> bool:
        """True once a model has been computed."""
        return self._model is not None

    @property
    def model(self) -> DynModel:
        """The latest model. Raises ``RuntimeError`` before the first successful `update`."""
        if self._model is None:
            raise RuntimeError("no DynModel yet: robot state not received")
        return self._model
