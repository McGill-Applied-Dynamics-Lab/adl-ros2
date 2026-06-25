"""Experiment logging with pluggable sinks."""

from __future__ import annotations

import json
import queue
import subprocess
import threading
import time
from dataclasses import asdict, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Protocol

import foxglove
import numpy as np
from foxglove import messages as fg_msgs


def _git_info() -> dict[str, Any]:
    """Capture the code provenance: commit SHA, dirty flag, and (if dirty) the diff.

    Run against the repo containing this file so it works regardless of cwd.
    Failures are non-fatal — provenance is best-effort, never blocks a run.
    """
    repo_dir = Path(__file__).resolve().parent

    def _git(*args: str) -> str | None:
        try:
            out = subprocess.run(
                ["git", *args],
                cwd=repo_dir,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            if out.returncode != 0:
                return None
            return out.stdout
        except Exception:
            return None

    sha = _git("rev-parse", "HEAD")
    status = _git("status", "--porcelain")
    info: dict[str, Any] = {
        "git_sha": sha.strip() if sha else None,
        "git_dirty": bool(status.strip()) if status is not None else None,
    }
    if info["git_dirty"]:
        info["git_diff"] = _git("diff", "HEAD") or ""
    return info


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


_IDENTITY_QUAT = fg_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


def _scene_update_from_payload(values: dict[str, Any], ts: fg_msgs.Timestamp) -> "fg_msgs.SceneUpdate | None":
    """Build a SceneUpdate from a ``viz/scene`` payload.

    Payload: ``{leader:[x,y,z], mass:[x,y,z], surface: float|None,
    rim_direction:[x,y,z], frame_id: str}``. Renders the leader and mass as
    spheres and (when ``surface`` is set) the contact surface as a thin slab
    perpendicular to ``rim_direction``.
    """
    leader = _to_vector3(values.get("leader"))
    mass = _to_vector3(values.get("mass"))
    if leader is None or mass is None:
        return None
    frame_id = str(values.get("frame_id", "bench"))

    spheres = [
        fg_msgs.SpherePrimitive(
            pose=fg_msgs.Pose(position=leader, orientation=_IDENTITY_QUAT),
            size=fg_msgs.Vector3(x=0.03, y=0.03, z=0.03),
            color=fg_msgs.Color(r=0.1, g=0.4, b=1.0, a=1.0),  # leader = blue
        ),
        fg_msgs.SpherePrimitive(
            pose=fg_msgs.Pose(position=mass, orientation=_IDENTITY_QUAT),
            size=fg_msgs.Vector3(x=0.03, y=0.03, z=0.03),
            color=fg_msgs.Color(r=1.0, g=0.5, b=0.0, a=1.0),  # virtual mass = orange
        ),
    ]

    cubes: list[fg_msgs.CubePrimitive] = []
    surface = values.get("surface")
    if surface is not None:
        n = np.asarray(values.get("rim_direction", [0.0, 0.0, 1.0]), dtype=float)
        norm = float(np.linalg.norm(n))
        n = n / norm if norm > 0 else np.array([0.0, 0.0, 1.0])
        lead = np.asarray(values.get("leader"), dtype=float)
        # Plane point nearest the leader: project the leader onto the surface.
        center = lead - (float(lead @ n) - float(surface)) * n
        # Thin dimension along the dominant axis of n (bench directions are axis-aligned).
        size = [0.4, 0.4, 0.4]
        size[int(np.argmax(np.abs(n)))] = 0.002
        cubes.append(
            fg_msgs.CubePrimitive(
                pose=fg_msgs.Pose(
                    position=fg_msgs.Vector3(x=float(center[0]), y=float(center[1]), z=float(center[2])),
                    orientation=_IDENTITY_QUAT,
                ),
                size=fg_msgs.Vector3(x=size[0], y=size[1], z=size[2]),
                color=fg_msgs.Color(r=0.6, g=0.6, b=0.6, a=0.4),  # contact surface = translucent gray
            )
        )

    entity = fg_msgs.SceneEntity(timestamp=ts, frame_id=frame_id, id="bench", spheres=spheres, cubes=cubes)
    return fg_msgs.SceneUpdate(entities=[entity])


def _typed_records_for_sample(sample: dict[str, Any], topic_prefix: str) -> list[tuple[str, foxglove.Schema, bytes]]:
    """Build typed Foxglove records for selected streams."""
    stream = str(sample.get("stream", "samples"))
    values = sample.get("values", {})
    if not isinstance(values, dict):
        return []

    ts = _to_fg_timestamp(sample.get("ts"))
    prefix = topic_prefix.rstrip("/")

    if stream == "viz/scene":
        upd = _scene_update_from_payload(values, ts)
        if upd is None:
            return []
        frame_id = str(values.get("frame_id", "bench"))
        # Publish an identity transform so the 3D panel registers `frame_id` as a
        # selectable Display frame — SceneEntity.frame_id alone does not add a frame.
        tf = fg_msgs.FrameTransform(
            timestamp=ts,
            parent_frame_id="world",
            child_frame_id=frame_id,
            translation=fg_msgs.Vector3(x=0.0, y=0.0, z=0.0),
            rotation=_IDENTITY_QUAT,
        )
        return [
            (f"{prefix}/tf", fg_msgs.FrameTransform.get_schema(), tf.encode()),
            (f"{prefix}/scene", fg_msgs.SceneUpdate.get_schema(), upd.encode()),
        ]

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

        git = _git_info()
        # Keep the (potentially large) diff out of the JSON; write it alongside.
        git_diff = git.pop("git_diff", None)
        if git_diff:
            with open(self._run_dir / "uncommitted.diff", "w", encoding="utf-8") as handle:
                handle.write(git_diff)

        metadata = {
            "created_at": datetime.now().isoformat(),
            "notes": self.cfg.notes,
            **git,
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
        typed_records = list(_typed_records_for_sample(sample, topic_prefix="/rim"))
        for topic, schema, payload in typed_records:
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

        if not typed_records:
            stream = str(sample.get("stream", "samples"))
            topic = f"/rim/{stream}"
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
        typed_records = list(_typed_records_for_sample(sample, topic_prefix=self.cfg.topic_prefix))
        for topic, schema, payload in typed_records:
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

        if not typed_records:
            stream = str(sample.get("stream", "samples"))
            topic = f"{self.cfg.topic_prefix.rstrip('/')}/{stream}"
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
