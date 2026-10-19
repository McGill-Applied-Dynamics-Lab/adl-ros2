"""LiveView: the run's state streamed to Foxglove, without hardware."""

from __future__ import annotations

import asyncio
import json
import socket
import struct
import time
from types import SimpleNamespace

import numpy as np
import pytest
from experiment_logger import ExperimentLogger, LoggingConfig
from fr3_haptic.live import LiveView, live_logger, write_layout
from haptic_teleop import HapticStateMailbox, PlantMailbox, StatusMailbox, TelemetryMailbox


@pytest.fixture
def boxes():
    made = []

    def make(box):
        made.append(box)
        return box

    yield make
    for box in made:
        box.close()
        box.unlink()


def fake_plant(n_samples: int = 3, n_targets: int = 2):
    sample = SimpleNamespace(
        ee_position=np.array([0.4, 0.0, 0.2]),
        ee_velocity=np.zeros(3),
        x_i=np.array([0.2]),
        v_i=np.array([0.0]),
        lam=np.array([1.5]),
        task_force=np.array([0.0, 0.0, -1.5]),
    )
    target = (n_targets, np.array([0.4, 0.0, 0.21]), np.array([0.21]), np.array([0.0]), None)
    return SimpleNamespace(sample=(n_samples, sample), last_target=target)


def sources(boxes):
    haptic_state, status, telemetry, plant_box = (
        boxes(HapticStateMailbox(1)),
        boxes(StatusMailbox()),
        boxes(TelemetryMailbox(1)),
        boxes(PlantMailbox(1)),
    )
    haptic_state.write(10, 0.01, np.array([0.21]), np.array([0.0]), force=np.array([-1.0]))
    status.write(ticks=10, t_s=0.01, watchdog_gain=1.0, watchdog_tripped=0, watchdog_trips=0, guard_tripped=0, guard_trips=0)
    scalars = dict.fromkeys(TelemetryMailbox.SCALARS, 0.0) | {"haptic_hz": 1000.0, "tdpa_energy_j": np.nan}
    telemetry.write(np.array([-0.9]), np.array([0.0, 0.0, -0.09]), **scalars)
    plant_box.write(np.array([0.2]), np.array([0.0]), np.array([1.5]), t_s=1.0)
    return dict(haptic_state=haptic_state, status=status, telemetry=telemetry, plant_box=plant_box)


def test_one_sample_of_every_stream(boxes):
    with ExperimentLogger(LoggingConfig.in_memory()) as logger:
        commands = [(0.01, np.array([0.21]), np.array([0.0]), np.zeros(1))]
        view = LiveView(logger, 50.0, plant=fake_plant(), commands=commands, **sources(boxes))
        view.step(now_s=100.0)
        logger.flush()
        by_stream = {s["stream"]: s["values"] for s in logger.samples}
    assert set(by_stream) == {"haptic", "robot", "target", "safety", "rates"}
    assert by_stream["haptic"]["f"] == [-0.9] and by_stream["haptic"]["proxy_x"] == [None]  # NaN → null
    assert by_stream["robot"]["lam"] == [1.5]
    assert by_stream["target"]["robot_x"] == [0.21] and by_stream["target"]["sent_x"] == [0.21]
    assert by_stream["target"]["robot_f_ff"] == [None]  # coupling methods: no feedforward
    assert by_stream["safety"]["tdpa_energy_j"] is None
    assert by_stream["rates"]["haptic"] == 1000.0


def test_rates_from_counters(boxes):
    with ExperimentLogger(LoggingConfig.in_memory()) as logger:
        commands: list = []
        plant = fake_plant()
        view = LiveView(logger, 50.0, plant=plant, commands=commands)
        for k in range(51):  # 1 s at 50 Hz: 500 robot samples, 250 commands per second
            plant.sample = (k * 10, plant.sample[1])
            commands.extend([(0.0, np.zeros(1), np.zeros(1), np.zeros(1))] * 5)
            view.step(now_s=100.0 + k / 50.0)
        logger.flush()
        rates = [s["values"] for s in logger.samples if s["stream"] == "rates"][-1]
    assert rates["plant"] == pytest.approx(500.0)
    assert rates["command"] == pytest.approx(250.0)
    assert rates["live"] == pytest.approx(50.0)


def test_streams_keep_their_fields_as_sources_appear(boxes):
    """The channel schema comes from a stream's first sample: later ones must not add fields."""
    with ExperimentLogger(LoggingConfig.in_memory()) as logger:
        commands: list = []
        plant = fake_plant()
        plant.last_target = None
        view = LiveView(logger, 50.0, plant=plant, commands=commands)
        view.step(now_s=1.0)  # nothing sent or published yet
        commands.append((0.01, np.array([0.21]), np.array([0.0]), np.zeros(1)))
        plant.last_target = fake_plant().last_target
        view.step(now_s=1.02)
        logger.flush()
        fields: dict[str, list[set]] = {}
        for sample in logger.samples:
            fields.setdefault(sample["stream"], []).append(set(sample["values"]))
    for stream, per_sample in fields.items():
        assert len(per_sample) == 2 and per_sample[0] == per_sample[1], stream
    first_target = logger.samples[[s["stream"] for s in logger.samples].index("target")]["values"]
    assert first_target["sent_x"] == [None] and first_target["robot_position"] == [None] * 3


def _layout_tree_ids(node) -> list[str]:
    if isinstance(node, str):
        return [node]
    return _layout_tree_ids(node["first"]) + _layout_tree_ids(node["second"])


def test_layout_references_only_logged_fields(tmp_path, boxes):
    path = write_layout(tmp_path / "layout.json")
    layout = json.loads(path.read_text())
    configs = layout["configById"]
    assert set(layout) >= {"configById", "layout"} and layout["layout"] in configs
    tabs = configs[layout["layout"]]["tabs"]
    assert [t["title"] for t in tabs] == ["Teleop", "Robot", "Timing"]
    placed = [pid for t in tabs for pid in _layout_tree_ids(t["layout"])]
    plots = {pid for pid in configs if pid.startswith("Plot!")}
    assert sorted(placed) == sorted(plots)  # every panel placed once, none missing a config
    series = [p["value"] for pid in plots for p in configs[pid]["paths"]]
    with ExperimentLogger(LoggingConfig.in_memory()) as logger:
        commands = [(0.01, np.array([0.21]), np.array([0.0]), np.zeros(1))]
        LiveView(logger, 50.0, plant=fake_plant(), commands=commands, **sources(boxes)).step(now_s=1.0)
        logger.flush()
        logged = {(s["stream"], k) for s in logger.samples for k in s["values"]}
    assert series
    for path_ in series:  # "/live/<stream>.values.<field>[i]"
        stream, _, field = path_.removeprefix("/live/").partition(".values.")
        assert (stream, field.split("[")[0]) in logged, path_


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


async def _receive(port: int, want: int) -> tuple[dict[str, dict], dict[str, dict]]:
    """Subscribe to every advertised channel: the first message of ``want`` topics (strict-JSON
    parsed) and each advertised channel's JSON schema, by topic."""
    import websockets

    def strict(s):
        raise ValueError(f"not JSON: {s}")

    got: dict[str, dict] = {}
    schemas: dict[str, dict] = {}
    async with websockets.connect(f"ws://127.0.0.1:{port}", subprotocols=["foxglove.sdk.v1"], close_timeout=0.5) as ws:
        topics: dict[int, str] = {}
        deadline = time.monotonic() + 5.0
        while len(got) < want and time.monotonic() < deadline:
            msg = await asyncio.wait_for(ws.recv(), timeout=5.0)
            if isinstance(msg, str):
                op = json.loads(msg)
                if op.get("op") == "advertise":
                    new = [c for c in op["channels"] if c["id"] not in topics]
                    for c in new:
                        topics[c["id"]] = c["topic"]
                        assert c["encoding"] == "json" and c["schemaEncoding"] == "jsonschema", c
                        schemas[c["topic"]] = json.loads(c["schema"])
                    subs = [{"id": c["id"], "channelId": c["id"]} for c in new]
                    await ws.send(json.dumps({"op": "subscribe", "subscriptions": subs}))
            elif msg[0] == 0x01:  # message data: opcode, u32 subscription id, u64 time, payload
                (sub_id,) = struct.unpack_from("<I", msg, 1)
                got[topics[sub_id]] = json.loads(msg[13:], parse_constant=strict)
    return got, schemas


def test_foxglove_client_receives_valid_json(boxes):
    port = _free_port()
    commands = [(0.01, np.array([0.21]), np.array([0.0]), np.zeros(1))]
    view = LiveView(live_logger(port, host="127.0.0.1"), 100.0, plant=fake_plant(), commands=commands, **sources(boxes))
    view.start()
    try:
        got, schemas = asyncio.run(_receive(port, want=5))
    finally:
        view.stop()
    assert set(got) == {f"/live/{s}" for s in ("haptic", "robot", "target", "safety", "rates")}
    assert got["/live/haptic"]["values"]["proxy_x"] == [None]
    for topic, message in got.items():  # the app lists every field from the schema
        fields = schemas[topic]["properties"]["values"]["properties"]
        assert set(fields) == set(message["values"]), topic
    assert schemas["/live/robot"]["properties"]["values"]["properties"]["ee_position"] == {
        "type": "array",
        "items": {"type": "number"},
    }
