"""Record robot-side topics during a run, decimated, into preallocated tables for the logger.

Every numeric field of each message is kept, flattened to dotted column names
(``measured_joint_state.position_3``, ``o_f_ext_hat_k.wrench.force.z``, ...). Strings, joint
names, frame ids, covariances and nested headers are dropped; the top-level header stamp becomes
the ``t_s`` column and the local receive time ``rx_s``.

Cost, which is why this records at a reduced rate: decoding one ``FrankaRobotState`` in rclpy
takes ~370 µs (~480 µs with flattening), all of it holding the GIL the 1 kHz haptic loop needs.
The subscriptions are therefore raw: every message arrives as bytes, and only the ones kept by
the decimation (one per ``period_s`` of local time) are decoded. At 50 Hz that is ~24 ms of GIL
per second; at the topics' 1 kHz it would be ~40 %.

Like ``haptic_teleop.TickLog``: tables are allocated up front, nothing grows during the run, and
the rows go to the logger only after it, with [RobotStateRecorder.to_logger][].
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

__all__ = ["DEFAULT_TOPICS", "RecordedTopic", "RobotStateRecorder", "StreamLog", "flatten_message"]

_SKIP_FIELDS = {"frame_id", "name"}


def flatten_message(msg, prefix: str = "") -> list[tuple[str, float]]:
    """Every numeric field of a ROS message as ``(dotted_name, value)``, in field order.

    Arrays expand to ``name_0, name_1, ...``; booleans become 0/1. Strings, ``name`` lists,
    ``frame_id``, covariances and all headers are skipped (the caller keeps the top-level stamp).
    """
    out: list[tuple[str, float]] = []
    _flatten(msg, prefix, out)
    return out


def _flatten(msg, prefix: str, out: list[tuple[str, float]]) -> None:
    for field in msg.get_fields_and_field_types():
        if field in _SKIP_FIELDS or field == "header" or "covariance" in field:
            continue
        value = getattr(msg, field)
        name = f"{prefix}{field}"
        if hasattr(value, "get_fields_and_field_types"):
            _flatten(value, f"{name}.", out)
        elif isinstance(value, (bool, int, float)):
            out.append((name, float(value)))
        elif isinstance(value, str):
            continue
        else:
            try:
                array = np.asarray(value, dtype=float).ravel()
            except (TypeError, ValueError):
                continue  # sequences of messages or strings
            out.extend((f"{name}_{i}", float(x)) for i, x in enumerate(array))


class StreamLog:
    """A preallocated table of rows ``(t_s, rx_s, *values)``; columns fixed by the first row.

    Rows past ``capacity`` are counted in ``dropped`` and not stored. A later row with a
    different set of columns (e.g. a JointState that suddenly has fewer joints) is matched by
    name; missing values are NaN, unknown ones ignored.
    """

    def __init__(self, capacity: int) -> None:
        self.capacity = int(capacity)
        self.columns: tuple[str, ...] = ()
        self._index: dict[str, int] = {}
        self._data: np.ndarray | None = None
        self.rows = 0
        self.dropped = 0

    def record(self, t_s: float, rx_s: float, values: list[tuple[str, float]]) -> None:
        """Store one row."""
        if self._data is None:
            self.columns = ("t_s", "rx_s", *(name for name, _ in values))
            self._index = {name: i for i, name in enumerate(self.columns)}
            self._data = np.full((self.capacity, len(self.columns)), np.nan)
        if self.rows >= self.capacity:
            self.dropped += 1
            return
        row = self._data[self.rows]
        row[0], row[1] = t_s, rx_s
        same_layout = (
            len(values) == len(self.columns) - 2
            and (not values or (values[0][0] == self.columns[2] and values[-1][0] == self.columns[-1]))
        )
        if same_layout:
            row[2:] = [v for _, v in values]
        else:
            for name, v in values:
                i = self._index.get(name)
                if i is not None:
                    row[i] = v
        self.rows += 1

    @property
    def data(self) -> np.ndarray:
        """The recorded rows, ``(rows, len(columns))``."""
        return np.zeros((0, len(self.columns))) if self._data is None else self._data[: self.rows]

    def column(self, name: str) -> np.ndarray:
        """One recorded column."""
        return self.data[:, self._index[name]]

    def to_logger(self, logger, stream: str, t0: float) -> None:
        """Log every row as one sample of ``stream``, timestamped ``rx_s - t0`` (local clock)."""
        names = self.columns[2:]
        for row in self.data:
            values = {"t_s": row[0], **dict(zip(names, row[2:]))}
            logger.log_sample(stream, values, timestamp_s=float(row[1] - t0))


@dataclass(frozen=True)
class RecordedTopic:
    """One topic to record: where, its message type, and the logger stream it becomes."""

    topic: str
    msg_type: type
    stream: str


DEFAULT_TOPICS = (
    RecordedTopic("/fr3/franka_robot_state_broadcaster/robot_state", FrankaRobotState, "robot_state"),
    RecordedTopic("/fr3/osc/joint_torques", JointState, "joint_torques_cmd"),
    RecordedTopic("/fr3/osc/task_error", TwistStamped, "task_error"),
    RecordedTopic("/fr3/osc/ee_state", Odometry, "ee_state"),
    RecordedTopic("/fr3/osc/task_wrench", WrenchStamped, "task_wrench"),
)


class RobotStateRecorder:
    """Decimated recording of ``topics`` on ``node``, into one [StreamLog][] per topic.

    Args:
        node: the node to subscribe on; it must be spun by a single-threaded executor (the
            ``FR3Plant`` node is). Callbacks only copy numbers into the tables.
        rate_hz: keep at most this many messages per second per topic (local receive time).
        seconds: run length; sizes the tables (with 20 % margin).
        topics: what to record.
        clock: local time source, for tests.
    """

    def __init__(
        self,
        node,
        rate_hz: float,
        seconds: float,
        topics: tuple[RecordedTopic, ...] = DEFAULT_TOPICS,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be positive, got {rate_hz}")
        self._node = node
        self._period_s = 1.0 / rate_hz
        self._clock = clock
        capacity = int(np.ceil(seconds * rate_hz * 1.2)) + 10
        self.logs: dict[str, StreamLog] = {}
        self._subs = []
        for spec in topics:
            log = StreamLog(capacity)
            self.logs[spec.stream] = log
            callback = self._make_callback(spec.msg_type, log)
            self._subs.append(node.create_subscription(spec.msg_type, spec.topic, callback, qos_profile_sensor_data, raw=True))

    def _make_callback(self, msg_type: type, log: StreamLog) -> Callable[[bytes], None]:
        next_keep = [-np.inf]
        period = self._period_s

        def callback(raw: bytes) -> None:
            now = self._clock()
            if now < next_keep[0]:
                return  # not decoded: decoding is the expensive part
            # Keep on a fixed schedule (a "time since last kept" test drops one tick per period
            # to rounding and runs slow); restart it after a gap longer than a period.
            next_keep[0] = next_keep[0] + period if now - next_keep[0] < period else now + period
            msg = deserialize_message(raw, msg_type)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            log.record(stamp, now, flatten_message(msg))

        return callback

    def close(self) -> None:
        """Stop recording (destroy the subscriptions); the tables are kept."""
        for sub in self._subs:
            self._node.destroy_subscription(sub)
        self._subs = []

    def summary(self) -> str:
        """Rows (and drops) per stream, for the end-of-run report."""
        return ", ".join(
            f"{name} {log.rows}" + (f" (+{log.dropped} dropped)" if log.dropped else "") for name, log in self.logs.items()
        )

    def to_logger(self, logger, t0: float) -> None:
        """Log every stream; timestamps are local receive time minus ``t0``."""
        for name, log in self.logs.items():
            log.to_logger(logger, name, t0)
