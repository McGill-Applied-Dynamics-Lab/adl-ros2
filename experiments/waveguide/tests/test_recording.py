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


def test_plot_run_reads_a_recorded_run(tmp_path, monkeypatch):
    """A run written by the logger (MCAP file) is read back and plotted by plot_run, which does not use it."""
    import matplotlib

    matplotlib.use("Agg")
    import plot_run

    monkeypatch.setattr(recording, "DATA_DIR", tmp_path)
    logger = recording.start_logger("teleop", "test", {"gesture": {"debounce_s": 0.3}})
    t = time.monotonic()
    log = logger.log_sample
    log("sensor/background", {"t_mono": t, "frame": np.full(1500, 2000.0)}, 1e9)
    log("sensor/calibration", {"t_mono": t, "threshold": 20.0, "hole_ids": [1, 2]}, 1e9)
    for i in range(10):
        ts = t + 0.05 * (i + 1)
        log("sensor/frame", {"t_mono": ts, "frame": np.full(1500, 2000.0 + i)}, 1e9 + i)
        log("sensor/detection", {"t_mono": ts, "detected": i > 5, "hole": 3, "hole2": None, "amplitude": 10.0 * i,
                                 "echo_time_ms": 1.5, "distance_cm": 27.0, "pressed": [3]}, 1e9 + i)
        log("robot/state", {"t_mono": ts, "position": [0.4, 0.0, 0.3 + 0.01 * i], "orientation": [1, 0, 0, 0],
                            "target": [0.4, 0.0, 0.35]}, 1e9 + i)
    log("sensor/swipe", {"t_mono": t + 0.8, "kind": "swipe", "start_hole": 3, "end_hole": 6, "duration_s": 0.2,
                         "axis": "x", "distance": -0.03, "speed": 0.1, "ignored_reason": ""}, 1e9 + 20)
    log("robot/command", {"t_mono": t + 0.85, "status": "executed", "detail": "", "axis": "x", "distance": -0.03,
                          "speed": 0.1, "t_created": t + 0.8, "target": [0.37, 0.0, 0.35]}, 1e9 + 21)
    logger.stop()

    run = plot_run.load_run(logger.run_dir)
    assert plot_run.stack(run["sensor/frame"], "frame").shape == (10, 1500)
    assert plot_run.stack(run["sensor/detection"], "detected", dtype=bool).sum() == 4
    assert plot_run.latest_run(tmp_path) == logger.run_dir
    assert "x -3.0 cm" in plot_run.summary(run)
    fig = plot_run.plot_run(run, title="test")
    assert len(fig.axes) == 4
