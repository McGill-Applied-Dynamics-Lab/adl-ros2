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

Threading: the two 1 kHz subscriptions live on the plant's own node, spun by its own
``SingleThreadedExecutor`` thread. Not on the `Robot` node: rclpy's ``MultiThreadedExecutor``
(which `Robot` uses) delivered only ~14 Hz of a 1 kHz best-effort topic on the lab PC, against
the full 1 kHz with a single-threaded executor. Callbacks replace [FR3Plant.sample][] whole, as
an ``(updates, sample)`` tuple; the haptic thread reads it without a lock and detects a fresh
sample by the update counter, as in example 04.

Clocks: ``t_s`` is the controller's stamp (franka-pc clock). It is only differenced against
other stamps from the same clock. Staleness uses the local receive time (``rx_s``) instead,
because the two PCs are only NTP-synchronised.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from arm_client.robot import Pose, Robot, Twist
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from pyrim import DynModel, InterfaceFrame
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation

from .adapters import RobotModelAdapter

__all__ = ["FR3Plant", "FR3PlantSample", "FR3System"]


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
        ee_orientation: ``(4,)`` measured EE orientation quaternion ``(x, y, z, w)``, base frame.
        task_force: ``(3,)`` task force applied to the robot, base frame [N].
    """

    x_i: np.ndarray
    v_i: np.ndarray
    lam: np.ndarray
    t_s: float
    rx_s: float
    ee_position: np.ndarray
    ee_velocity: np.ndarray
    ee_orientation: np.ndarray
    task_force: np.ndarray

    @property
    def ee_pose(self) -> Pose:
        """The measured EE pose, in the controller's own end-effector frame."""
        return Pose(self.ee_position.copy(), Rotation.from_quat(self.ee_orientation))


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
            orientation target. ``None`` until set with [set_hold_pose][FR3Plant.set_hold_pose],
            typically from the first sample (``wait_for_sample().ee_pose``), which is in the
            controller's own end-effector frame.
        ee_state_topic: ``osc_controller`` EE state topic (``nav_msgs/Odometry``).
        task_wrench_topic: ``osc_controller`` task wrench topic (``WrenchStamped``).
        sample_period_s: keep at most one sample per this many seconds of controller time,
            to emulate a slower plant. 0 keeps every controller tick.
        interface_limits: ``(low, high)`` bounds on the commanded interface coordinate [m],
            applied to every component. ``None`` disables them.
        record: keep every sample (after decimation) in [history][FR3Plant.history], for logging.
        node: subscribe on this node, which the caller spins. ``None`` (the default) creates
            the plant's own node and a single-threaded executor thread for it.
        mailbox: also publish every kept sample here (a ``haptic_teleop.PlantMailbox``), for a
            haptic loop in another process.
    """

    _MAX_PENDING = 16

    def __init__(
        self,
        robot: Robot,
        frame: InterfaceFrame,
        hold_pose: Pose | None = None,
        ee_state_topic: str = "/fr3/osc/ee_state",
        task_wrench_topic: str = "/fr3/osc/task_wrench",
        sample_period_s: float = 0.0,
        interface_limits: tuple[float, float] | None = None,
        record: bool = False,
        node=None,
        mailbox=None,
    ) -> None:
        self._robot = robot
        self._frame = frame
        self._hold_pose = None if hold_pose is None else hold_pose.copy()
        self._sample_period_ns = round(float(sample_period_s) * 1e9)
        self._limits: tuple[float, float] | None = None
        self.set_interface_limits(interface_limits)

        self._frozen_x: np.ndarray | None = None
        self._record = record
        self._mailbox = mailbox
        self.history: list[FR3PlantSample] = []
        """Every sample, in order, when ``record`` is set."""
        self.sample: tuple[int, FR3PlantSample] | None = None
        """Latest ``(updates, sample)``, replaced whole; ``None`` before the first one."""

        self._updates = 0
        self._last_kept_ns: int | None = None
        self._lock = threading.Lock()
        self._pending_ee: dict[int, Odometry] = {}
        self._pending_wrench: dict[int, WrenchStamped] = {}

        robot.set_target_streaming(True)
        self._executor = None
        self._spin_thread: threading.Thread | None = None
        if node is None:
            node = rclpy.create_node("fr3_plant")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(node)
            self._spin_thread = threading.Thread(target=self._spin, name="fr3_plant_spin", daemon=True)
        self._node = node
        self._subs = [
            node.create_subscription(Odometry, ee_state_topic, self._on_ee_state, qos_profile_sensor_data),
            node.create_subscription(WrenchStamped, task_wrench_topic, self._on_task_wrench, qos_profile_sensor_data),
        ]
        if self._spin_thread is not None:
            self._spin_thread.start()

    # ---------------------------------------------------------------- state side

    @property
    def dim(self) -> int:
        """Interface dimension."""
        return self._frame.dim

    @property
    def node(self):
        """The node the plant subscribes on (its own, unless one was passed in)."""
        return self._node

    @property
    def hold_pose(self) -> Pose | None:
        """The pose held on the free axes, and the orientation target."""
        return None if self._hold_pose is None else self._hold_pose.copy()

    def set_hold_pose(self, pose: Pose) -> None:
        """Set the pose held on the free axes and the orientation target."""
        self._hold_pose = pose.copy()

    @property
    def interface_limits(self) -> tuple[float, float] | None:
        """``(low, high)`` bounds on the commanded interface coordinate [m], or ``None``."""
        return self._limits

    def set_interface_limits(self, limits: tuple[float, float] | None) -> None:
        """Set ``(low, high)`` bounds on the commanded interface coordinate [m]; ``None`` removes them."""
        if limits is not None and not limits[0] < limits[1]:
            raise ValueError(f"interface_limits must be (low, high) with low < high, got {limits}")
        self._limits = None if limits is None else (float(limits[0]), float(limits[1]))

    def wait_for_sample(self, timeout_s: float = 2.0, poll_s: float = 0.005) -> FR3PlantSample:
        """Block until a sample arrives and return it.

        Raises:
            TimeoutError: if none arrives within ``timeout_s`` (is ``osc_controller`` active and
                publishing ``ee_state`` and ``task_wrench``?).
        """
        deadline = time.monotonic() + timeout_s
        while self.sample is None:
            if time.monotonic() > deadline:
                raise TimeoutError(f"no plant sample within {timeout_s} s")
            time.sleep(poll_s)
        return self.sample[1]

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
            if self._record:
                self.history.append(sample)
            if self._mailbox is not None:
                self._mailbox.write(sample.x_i, sample.v_i, sample.lam, sample.t_s, sample.rx_s)

    def _store(self, pending: dict, key: int, msg) -> None:
        pending[key] = msg
        while len(pending) > self._MAX_PENDING:
            del pending[next(iter(pending))]

    def _make_sample(self, ee: Odometry, wrench: WrenchStamped, t_s: float) -> FR3PlantSample:
        p = ee.pose.pose.position
        o = ee.pose.pose.orientation
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
            ee_orientation=np.array([o.x, o.y, o.z, o.w]),
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

    @property
    def frozen(self) -> bool:
        """True after [freeze][FR3Plant.freeze], until [release][FR3Plant.release]."""
        return self._frozen_x is not None

    def freeze(self) -> None:
        """Hold the robot where it is, with zero velocity and feedforward force.

        The first call latches the last measured interface position (the hold pose before any
        sample) and every call publishes that same target, so repeated calls do not let the
        robot drift with its own tracking error.
        """
        if self._frozen_x is None:
            latest = self.sample
            if latest is not None:
                self._frozen_x = latest[1].x_i.copy()
            elif self._hold_pose is not None:
                self._frozen_x = self._frame.project(self._hold_pose.position)
            else:
                return  # nothing measured or held yet: the controller still holds its activation pose
        self._publish(self._frozen_x, np.zeros(self.dim), np.zeros(self.dim))

    def release(self) -> None:
        """Forget the frozen target; the next [aim][FR3Plant.aim] / [command][FR3Plant.command] takes over."""
        self._frozen_x = None

    def _publish(self, x: np.ndarray, v: np.ndarray, f_ff: np.ndarray | None) -> None:
        if self._hold_pose is None:
            raise RuntimeError("FR3Plant has no hold pose: call set_hold_pose() before streaming targets")
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

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001 - the context is shut down under a spinning executor at exit
            if rclpy.ok():
                raise

    def close(self) -> None:
        """Destroy the subscriptions (and the plant's own node), and leave streaming mode."""
        for sub in self._subs:
            self._node.destroy_subscription(sub)
        self._subs = []
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
            if self._spin_thread is not None:
                self._spin_thread.join(timeout=1.0)
            self._node.destroy_node()
            self._executor = None
        self._robot.set_target_streaming(False)


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
