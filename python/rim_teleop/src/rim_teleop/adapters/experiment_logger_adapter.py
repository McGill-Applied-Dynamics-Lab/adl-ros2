"""Experiment logging with pluggable sinks."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from datetime import datetime
import json
from pathlib import Path
import queue
import threading
import time
from typing import Any, Protocol

import numpy as np
import foxglove
from foxglove import messages as fg_msgs


def _to_json_bytes(obj: Any) -> bytes:
    def _default(v: Any) -> Any:
        if isinstance(v, np.ndarray):
            return v.tolist()
        if isinstance(v, np.floating):
            return float(v)
        if isinstance(v, np.integer):
            return int(v)
        raise TypeError(f"Not JSON serializable: {type(v)}")
    return json.dumps(obj, default=_default).encode()

from ..config import FileSinkConfig, FoxgloveSinkConfig, LoggingConfig, RIMTeleopConfig


class _SampleSink(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def publish(self, sample: dict[str, Any]) -> None: ...


class _NullSink:
    def start(self) -> None:
        return

    def stop(self) -> None:
        return

    def publish(self, sample: dict[str, Any]) -> None:
        del sample
        return


_PB_ENCODING = "protobuf"


def _to_fg_timestamp(ts_s: float | None) -> fg_msgs.Timestamp:
    ts = float(time.time() if ts_s is None else ts_s)
    sec = int(ts)
    nsec = int((ts - sec) * 1e9)
    return fg_msgs.Timestamp(sec=sec, nsec=nsec)


def _to_vector3(values: Any) -> fg_msgs.Vector3 | None:
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return None
    try:
        return fg_msgs.Vector3(x=float(values[0]), y=float(values[1]), z=float(values[2]))
    except Exception:
        return None


def _to_quaternion_xyzw(values: Any) -> fg_msgs.Quaternion | None:
    if not isinstance(values, (list, tuple)) or len(values) < 4:
        return None
    try:
        return fg_msgs.Quaternion(x=float(values[0]), y=float(values[1]), z=float(values[2]), w=float(values[3]))
    except Exception:
        return None


def _typed_records_for_sample(sample: dict[str, Any], topic_prefix: str) -> list[tuple[str, foxglove.Schema, bytes]]:
    """Build typed Foxglove records for selected streams."""
    stream = str(sample.get("stream", "samples"))
    values = sample.get("values", {})
    if not isinstance(values, dict):
        return []

    ts = _to_fg_timestamp(sample.get("ts"))
    prefix = topic_prefix.rstrip("/")

    if stream == "robot/joint_states":
        names = values.get("name", [])
        positions = values.get("position", [])
        velocities = values.get("velocity", [])
        efforts = values.get("effort", [])

        if not isinstance(names, list):
            return []

        joints: list[fg_msgs.JointState] = []
        for i, name in enumerate(names):
            p = float(positions[i]) if isinstance(positions, list) and i < len(positions) else None
            v = float(velocities[i]) if isinstance(velocities, list) and i < len(velocities) else None
            e = float(efforts[i]) if isinstance(efforts, list) and i < len(efforts) else None
            joints.append(fg_msgs.JointState(name=str(name), position=p, velocity=v, effort=e))

        msg = fg_msgs.JointStates(timestamp=ts, joints=joints)
        return [
            (
                f"{prefix}/{stream}",
                fg_msgs.JointStates.get_schema(),
                msg.encode(),
            )
        ]

    if stream == "robot/end_effector/pose":
        position = _to_vector3(values.get("position"))
        orientation = _to_quaternion_xyzw(values.get("orientation_xyzw"))
        if position is None or orientation is None:
            return []

        msg = fg_msgs.PoseInFrame(
            timestamp=ts,
            frame_id=str(values.get("frame_id", "")),
            pose=fg_msgs.Pose(position=position, orientation=orientation),
        )
        return [
            (
                f"{prefix}/{stream}",
                fg_msgs.PoseInFrame.get_schema(),
                msg.encode(),
            )
        ]

    if stream == "robot/end_effector/velocity":
        linear = _to_vector3(values.get("linear"))
        angular = _to_vector3(values.get("angular"))
        records: list[tuple[str, foxglove.Schema, bytes]] = []
        if linear is not None:
            records.append(
                (
                    f"{prefix}/{stream}/linear",
                    fg_msgs.Vector3.get_schema(),
                    linear.encode(),
                )
            )
        if angular is not None:
            records.append(
                (
                    f"{prefix}/{stream}/angular",
                    fg_msgs.Vector3.get_schema(),
                    angular.encode(),
                )
            )
        return records

    if stream == "robot/end_effector/force":
        force = _to_vector3(values.get("force"))
        torque = _to_vector3(values.get("torque"))
        records: list[tuple[str, foxglove.Schema, bytes]] = []
        if force is not None:
            records.append(
                (
                    f"{prefix}/{stream}/force",
                    fg_msgs.Vector3.get_schema(),
                    force.encode(),
                )
            )
        if torque is not None:
            records.append(
                (
                    f"{prefix}/{stream}/torque",
                    fg_msgs.Vector3.get_schema(),
                    torque.encode(),
                )
            )
        return records

    return []


# def _foxglove_message_from_sample(sample: dict[str, Any]) -> dict[str, Any]:
#     msg: dict[str, Any] = {
#         "timestamp_s": float(sample.get("ts", time.time())),
#         "stream": str(sample.get("stream", "samples")),
#     }

#     values = sample.get("values", {})
#     if isinstance(values, dict):
#         msg.update(_flatten_for_foxglove(values))
#     else:
#         msg["values"] = values

#     return msg


class _FileSink:
    """Local MCAP sink using Foxglove SDK."""

    def __init__(self, cfg: FileSinkConfig, full_config: RIMTeleopConfig | None = None):
        self.cfg = cfg
        self._full_config = full_config
        self._run_dir: Path | None = None
        self._context = None
        self._writer = None
        self._channels: dict[str, Any] = {}
        self._next_flush = 0.0

    @property
    def run_dir(self) -> Path | None:
        return self._run_dir

    def start(self) -> None:
        if not self.cfg.enabled:
            return

        base = Path(self.cfg.output_dir)
        run_name = self.cfg.run_name or datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self._run_dir = base / run_name
        self._run_dir.mkdir(parents=True, exist_ok=True)

        metadata = {
            "created_at": datetime.now().isoformat(),
            "file_sink": ExperimentLogger._to_jsonable(asdict(self.cfg)),
        }
        if self._full_config is not None:
            metadata["config"] = ExperimentLogger._to_jsonable(asdict(self._full_config))

        with open(self._run_dir / "metadata.json", "w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2)

        mcap_path = self._run_dir / "samples.mcap"
        self._context = foxglove.Context()
        self._writer = foxglove.open_mcap(mcap_path, allow_overwrite=True, context=self._context)
        self._writer.write_metadata(
            "rim_teleop",
            {
                "created_at": metadata["created_at"],
                "logger": "ExperimentLogger",
            },
        )
        self._next_flush = time.perf_counter() + 1.0 / max(self.cfg.flush_hz, 1.0)

        print(f"[ExperimentLogger] File sink started. Logging to {self._run_dir}")

    def stop(self) -> None:
        for channel in self._channels.values():
            try:
                channel.close()
            except Exception:
                pass
        self._channels.clear()

        if self._writer is not None:
            try:
                self._writer.close()
            except Exception:
                pass
            self._writer = None

        self._context = None

    def publish(self, sample: dict[str, Any]) -> None:
        if self._writer is None or self._context is None:
            return

        import foxglove

        log_time = int(float(sample.get("ts", time.time())) * 1e9)

        # Publish typed records for known robot telemetry streams.
        typed_topics: set[str] = set()
        for topic, schema, payload in _typed_records_for_sample(sample, topic_prefix="/rim"):
            typed_channel = self._channels.get(topic)
            if typed_channel is None:
                typed_channel = foxglove.Channel(
                    topic,
                    schema=schema,
                    message_encoding=_PB_ENCODING,
                    context=self._context,
                )
                self._channels[topic] = typed_channel
            typed_channel.log(payload, log_time=log_time)
            typed_topics.add(topic)

        stream = str(sample.get("stream", "samples"))
        topic = f"/rim/{stream}"
        if topic not in typed_topics:
            channel = self._channels.get(topic)
            if channel is None:
                channel = foxglove.Channel(topic, message_encoding="json", context=self._context)
                self._channels[topic] = channel
            channel.log(_to_json_bytes(sample), log_time=log_time)

        now = time.perf_counter()
        if now >= self._next_flush:
            # SDK writer flushes internally; keep periodic cadence for symmetry.
            self._next_flush = now + 1.0 / max(self.cfg.flush_hz, 1.0)


class _FoxgloveSink:
    """Live Foxglove SDK sink."""

    def __init__(self, cfg: FoxgloveSinkConfig):
        self.cfg = cfg
        self._server = None
        self._context = None
        self._channels: dict[str, Any] = {}

    def start(self) -> None:
        if not self.cfg.enabled:
            return

        try:
            import foxglove
        except Exception:
            print("[rim_teleop] foxglove package unavailable; foxglove_sink disabled.")
            self._server = None
            return

        # Quickstart-style setup: explicit context + websocket sink attached to it.
        self._context = foxglove.Context()
        self._server = foxglove.start_server(
            name="rim_teleop",
            host=self.cfg.host,
            port=self.cfg.port,
            context=self._context,
        )

        print(f"[ExperimentLogger] Foxglove sink started. Publishing to port {self.cfg.port}")

    def stop(self) -> None:
        for channel in self._channels.values():
            try:
                channel.close()
            except Exception:
                pass
        self._channels.clear()

        if self._server is not None:
            try:
                self._server.stop()
            except Exception:
                pass
            self._server = None

    def publish(self, sample: dict[str, Any]) -> None:
        if self._server is None or self._context is None:
            return

        log_time = int(float(sample.get("ts", time.time())) * 1e9)

        # Publish typed records for known robot telemetry streams.
        typed_topics: set[str] = set()
        for topic, schema, payload in _typed_records_for_sample(sample, topic_prefix=self.cfg.topic_prefix):
            typed_channel = self._channels.get(topic)
            if typed_channel is None:
                typed_channel = foxglove.Channel(
                    topic,
                    schema=schema,
                    message_encoding=_PB_ENCODING,
                    context=self._context,
                )
                self._channels[topic] = typed_channel
            typed_channel.log(payload, log_time=log_time)
            typed_topics.add(topic)

        stream = str(sample.get("stream", "samples"))
        topic = f"{self.cfg.topic_prefix.rstrip('/')}/{stream}"
        if topic not in typed_topics:
            channel = self._channels.get(topic)
            if channel is None:
                channel = foxglove.Channel(topic, message_encoding="json", context=self._context)
                self._channels[topic] = channel
            vals = sample.get("values", {})
            channel.log(_to_json_bytes(self._flatten_dict(vals)), log_time=log_time)

    def _flatten_dict(self, value: Any, prefix: str = "") -> dict[str, Any]:
        out: dict[str, Any] = {}

        if isinstance(value, dict):
            for key, child in value.items():
                key_str = str(key)
                child_prefix = f"{prefix}_{key_str}" if prefix else key_str
                out.update(self._flatten_dict(child, child_prefix))
            return out

        if isinstance(value, (list, tuple)):
            if len(value) == 1 and not isinstance(value[0], (dict, list, tuple)):
                out[prefix] = value[0]
                return out
            for i, child in enumerate(value):
                child_prefix = f"{prefix}_{i}" if prefix else str(i)
                if isinstance(child, (dict, list, tuple)):
                    out.update(self._flatten_dict(child, child_prefix))
                else:
                    out[child_prefix] = child
            return out

        if prefix:
            out[prefix] = value
        return out


class ExperimentLogger:
    """Collect timestamped samples from multiple loops and fan-out to sinks."""

    def __init__(self, cfg: LoggingConfig, full_config: RIMTeleopConfig | None = None):
        self.cfg = cfg
        self._full_config = full_config
        self._running = False
        self._queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=20000)
        self._writer_thread: threading.Thread | None = None
        self._sinks: list[_SampleSink] = self._build_sinks()
        self._file_sink: _FileSink | None = next((s for s in self._sinks if isinstance(s, _FileSink)), None)

    @property
    def run_dir(self) -> Path | None:
        return None if self._file_sink is None else self._file_sink.run_dir

    def start(self) -> None:
        if not self.cfg.enabled:
            return

        self._running = True
        for sink in self._sinks:
            sink.start()

        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()

    def stop(self) -> None:
        if not self.cfg.enabled:
            return

        self._running = False
        if self._writer_thread and self._writer_thread.is_alive():
            self._writer_thread.join(timeout=2.0)

        for sink in self._sinks:
            sink.stop()

    def log_sample(self, stream: str, values: dict[str, Any], timestamp_s: float | None = None) -> None:
        if not self.cfg.enabled:
            return

        sample = {
            "ts": float(time.time() if timestamp_s is None else timestamp_s),
            "stream": stream,
            "values": self._to_jsonable(values),
        }
        try:
            self._queue.put_nowait(sample)
        except queue.Full:
            return

    def _writer_loop(self) -> None:
        flush_period = 1.0 / max(self.cfg.file_sink.flush_hz, 1.0)
        next_flush = time.perf_counter() + flush_period

        while self._running or not self._queue.empty():
            try:
                sample = self._queue.get(timeout=0.05)
            except queue.Empty:
                sample = None

            if sample is not None:
                for sink in self._sinks:
                    sink.publish(sample)

            now = time.perf_counter()
            if now >= next_flush:
                next_flush = now + flush_period

    @staticmethod
    def _to_jsonable(value: Any) -> Any:
        if value is None:
            return None
        if is_dataclass(value):
            return {k: ExperimentLogger._to_jsonable(v) for k, v in asdict(value).items()}
        if isinstance(value, dict):
            return {str(k): ExperimentLogger._to_jsonable(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [ExperimentLogger._to_jsonable(v) for v in value]
        if hasattr(value, "tolist"):
            try:
                return value.tolist()
            except Exception:
                pass
        if isinstance(value, (str, int, float, bool)):
            return value
        return str(value)

    def _build_sinks(self) -> list[_SampleSink]:
        configured = {str(s).lower() for s in self.cfg.sinks}
        sinks: list[_SampleSink] = []

        if "file_sink" in configured and self.cfg.file_sink.enabled:
            sinks.append(_FileSink(self.cfg.file_sink, full_config=self._full_config))

        if "foxglove_sink" in configured and self.cfg.foxglove_sink.enabled:
            sinks.append(_FoxgloveSink(self.cfg.foxglove_sink))

        if not sinks:
            sinks.append(_NullSink())

        return sinks
