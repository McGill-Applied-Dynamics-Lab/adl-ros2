"""Tests of the guided calibration (live_calibration.py), with the frames of calibration/calibration_dummy.db."""

import json
import sqlite3
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("scipy")  # wg_helpers
import live_calibration as lc  # noqa: E402
import wg_helpers  # noqa: E402

DUMMY_DB = Path(__file__).resolve().parents[1] / "calibration" / "calibration_dummy.db"
FRAMES_PER_PRESS = wg_helpers.CYCLES_PER_PRESS


@pytest.fixture(scope="module")
def recording():
    """{key: frames} of the first pass of the dummy recording."""
    rows = sqlite3.connect(DUMMY_DB).execute("SELECT key_vector, signal_blob FROM signals ORDER BY id").fetchall()
    frames: dict[int, list[np.ndarray]] = {}
    for key_vector, blob in rows:
        key = json.loads(key_vector)[0]
        if len(frames.get(key, [])) < FRAMES_PER_PRESS:
            frames.setdefault(key, []).append(np.frombuffer(blob, dtype="<u2").copy())
    return frames


def test_calibration_holes():
    assert lc.calibration_holes(51, 11) == (1, 6, 11, 16, 21, 26, 31, 36, 41, 46, 51)
    assert lc.calibration_holes(51, 3, always=(23, 24, 99)) == (1, 23, 24, 26, 51)
    assert lc.calibration_holes(5, 10) == (1, 2, 3, 4, 5)


class Clock:
    def __init__(self):
        self.t = 0.0

    def feed(self, recorder, frames, period=0.05):
        for frame in frames:
            self.t += period
            recorder.update(self.t, frame)


def run_session(recorder, recording, holes, n_passes, clock=None):
    """Untouched frames until the background is recorded, then for each hole: untouched, pressed, released."""
    clock = clock or Clock()
    background = recording[0]
    for p in range(n_passes):
        while recorder.key == 0 and not recorder.done:
            clock.feed(recorder, background[:1])
        assert recorder.pass_background is not None
        for hole in holes:
            assert recorder.key == hole and recorder.phase == "waiting"
            clock.feed(recorder, background[:3])
            clock.feed(recorder, recording[hole] * 2)  # Held: steady_frames + frames_per_press
            assert recorder.phase == "release"
            clock.feed(recorder, background[:3])
    return clock


def test_recorder_walks_through_the_passes(recording):
    holes = (1, 16, 31)
    recorder = lc.CalibrationRecorder(holes, n_passes=2)
    assert (recorder.key, recorder.phase, recorder.n_steps) == (0, "quiet", 8)
    run_session(recorder, recording, holes, 2)
    assert recorder.done
    assert [(p.pass_idx, p.key) for p in recorder.presses] == [(p, k) for p in range(2) for k in (0, *holes)]
    assert all(len(p.frames) == FRAMES_PER_PRESS for p in recorder.presses)
    # The hole presses are the pressed frames, not untouched ones
    first_hole = recorder.presses[1]
    assert lc.change(np.mean(first_hole.frames, axis=0), recorder.pass_background) > recorder.threshold


def test_background_waits_for_quiet(recording):
    recorder = lc.CalibrationRecorder((1,), n_passes=1)
    clock = Clock()
    clock.feed(recorder, recording[0][:1] * 10, period=0.05)  # 0.5 s < quiet_s
    assert recorder.phase == "quiet"
    clock.feed(recorder, recording[0][:1] * 20)
    assert recorder.key == 1 and recorder.phase == "waiting"


def test_record_now(recording):
    recorder = lc.CalibrationRecorder((1, 2), n_passes=1)
    recorder.record_now()  # Background right away
    Clock().feed(recorder, recording[0])
    assert recorder.key == 1 and recorder.phase == "waiting"
    recorder.record_now()  # Hole 1, whatever the signal
    Clock().feed(recorder, recording[0])
    assert recorder.phase == "release" and len(recorder.presses) == 2
    recorder.record_now()  # While waiting for the release: next hole
    assert recorder.key == 2 and recorder.phase == "waiting"


def test_short_touch_is_not_recorded(recording):
    recorder = lc.CalibrationRecorder((1,), n_passes=1)
    recorder.record_now()
    clock = Clock()
    clock.feed(recorder, recording[0])
    clock.feed(recorder, recording[1][:4])  # steady_frames, then 1 frame recorded
    clock.feed(recorder, recording[0][:1])  # Released too early
    assert recorder.phase == "waiting" and len(recorder.presses) == 1


def test_save_and_fit(recording, tmp_path):
    holes = tuple(range(1, 32, 5))
    recorder = lc.CalibrationRecorder(holes, n_passes=2)
    run_session(recorder, recording, holes, 2)
    db, npz = tmp_path / "cal.db", tmp_path / "cal.npz"
    db.write_bytes(b"old")
    calibration = lc.save_and_fit(recorder.presses, db, npz, first_hole_cm=25.0, hole_spacing_cm=1.0)
    assert list(calibration["hole_ids"]) == list(holes)
    assert npz.exists() and wg_helpers.load_calibration(npz)["threshold"] == pytest.approx(calibration["threshold"])
    assert [p.read_bytes() for p in tmp_path.glob("cal_backup_*.db")] == [b"old"]


def test_failed_fit_restores_the_previous_files(tmp_path):
    db, npz = tmp_path / "cal.db", tmp_path / "cal.npz"
    db.write_bytes(b"old db")
    npz.write_bytes(b"old npz")
    presses = [lc.Press(1, 0, (np.zeros(1500, dtype=np.uint16),) * FRAMES_PER_PRESS)]  # No background: fails
    with pytest.raises(AssertionError):
        lc.save_and_fit(presses, db, npz, 25.0, 1.0)
    assert db.read_bytes() == b"old db" and npz.read_bytes() == b"old npz"
    assert not list(tmp_path.glob("*_backup_*"))
