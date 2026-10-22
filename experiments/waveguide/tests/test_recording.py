"""Tests of the run recording (recording.py), with the in-memory logger and the frames of calibration_dummy.db."""

import json
import multiprocessing as mp
import sqlite3
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("serial")  # sensor.py
experiment_logger = pytest.importorskip("experiment_logger")
import recording  # noqa: E402
import sensor  # noqa: E402

CALIBRATION_DIR = Path(__file__).resolve().parents[1] / "calibration"


@pytest.fixture
def logger():
    log = experiment_logger.ExperimentLogger(experiment_logger.LoggingConfig.in_memory())
    log.start()
    yield log
    log.stop()


def frames_by_key():
    rows = sqlite3.connect(CALIBRATION_DIR / "calibration_dummy.db").execute(
        "SELECT key_vector, signal_blob FROM signals ORDER BY id"
    ).fetchall()
    frames = {}
    for key_vector, blob in rows:
        frames.setdefault(json.loads(key_vector)[0], []).append(np.frombuffer(blob, dtype="<u2").copy())
    return frames


def test_sensor_recorder(logger, tmp_path):
    frames = frames_by_key()
    detector = sensor.HoleDetector(
        CALIBRATION_DIR / "calibration_dummy.npz", replace(sensor.DETECTION_CFG, n_background_frames=5)
    )
    recorder = recording.SensorRecorder(logger.log_sample, tmp_path)
    t = time.monotonic()
    for i, frame in enumerate(frames[0][:5] + frames[12][:5]):
        pressed = detector.update(frame)
        recorder.frame(t + 0.05 * i, detector, pressed)
    swipe = sensor.Swipe(t + 1.0, 12, 15, 0.4, sensor.MoveCommand("x", -0.06, 0.15))
    recorder.swipe(swipe)
    logger.flush()

    streams = logger.to_dataframes()
    assert len(streams["sensor/background"]) == 1
    frame_columns = streams["sensor/frame"].filter(regex=r"^frame_\d+$")
    assert len(streams["sensor/frame"]) == 5 and frame_columns.shape[1] == sensor.FRAME_SIZE
    detection = streams["sensor/detection"]
    assert len(detection) == 5 and detection["detected"].iloc[-1] and detection["hole"].iloc[-1] == 12
    assert streams["sensor/swipe"]["axis"].iloc[0] == "x" and streams["sensor/swipe"]["end_hole"].iloc[0] == 15
    calibration = streams["sensor/calibration"]
    assert calibration["file"].iloc[0] == "calibration_1.npz" and (tmp_path / "calibration_1.npz").exists()


def test_queue_log_and_drain(logger):
    samples = mp.get_context("spawn").Queue()
    log = recording.QueueLog(samples)
    for i in range(3):
        log("sensor/detection", {"t_mono": float(i), "frame": np.arange(4)}, 100.0 + i)
    drain = recording.QueueDrain(samples, logger)
    drain.start()
    drain.stop()
    logger.flush()
    detection = logger.to_dataframes()["sensor/detection"]
    assert list(detection["t_mono"]) == [0.0, 1.0, 2.0]
    assert list(detection.filter(regex=r"^frame_\d+$").iloc[0]) == [0, 1, 2, 3]


def test_robot_recorder(logger):
    state = (np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.4, 0.0, 0.3]))
    recorder = recording.RobotRecorder(logger, lambda: state, rate_hz=200.0)
    recorder.start()
    recorder.command(sensor.MoveCommand("y", 0.05, 0.1), "executed", target=[0.4, 0.05, 0.3])
    recorder.command(None, "home")
    time.sleep(0.1)
    recorder.stop()
    logger.flush()
    streams = logger.to_dataframes()
    commands = streams["robot/command"]
    assert list(commands["status"]) == ["executed", "home"] and commands["axis"].iloc[0] == "y"
    state = streams["robot/state"]
    assert len(state) >= 5 and list(state[["position_0", "position_1", "position_2"]].iloc[0]) == [0.4, 0.0, 0.3]
