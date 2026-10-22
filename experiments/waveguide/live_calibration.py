"""Guided calibration from the live frames (the live plot's calibration panel), and its fit.

`CalibrationRecorder` is fed every frame and walks through the steps of each pass: the background (nothing
covered), then each hole. It moves on by itself:
- background: recorded after `quiet_s` without touching
- hole: once the frame differs from the untouched reference for `steady_frames` frames, the next
  `frames_per_press` frames are recorded; then it waits for the release
`record_now()` (Space in the plot) records the current step right away, e.g. for a hole too weak to detect.

The change is the largest envelope (`wg_helpers.ema`) of |frame - reference| after the transmit ring-down, with
the reference the mean of the latest untouched frames (it follows the slow drift of the background). A press is
a change above max(`noise_factor` x the noise of the pass background, `min_change`).

`save_and_fit` writes the presses in the SAVE_TO_SQLITE.py format and fits them (`wg_helpers.calibrate`).

No Qt, no ROS: tested without the sensor.
"""

import json
import sqlite3
import threading
import time
from collections import deque
from contextlib import closing
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import wg_helpers


@dataclass(frozen=True)
class AutoRecordConfig:
    """How `CalibrationRecorder` decides to record and to move on."""

    frames_per_press: int = wg_helpers.CYCLES_PER_PRESS  # Frames recorded per step (wg_helpers.calibrate averages them)
    quiet_s: float = 1.0  # Background: recorded after this long in the step, nothing must touch the waveguide
    change_start: int = wg_helpers.SEARCH_START  # Samples before are the transmit ring-down, ignored
    # A press: a change above noise_factor x the noise of the pass background, and above min_change (ADC,
    # envelope). The far holes give ~25: higher misses them; the rolling reference absorbs the background drift
    noise_factor: float = 2.0
    min_change: float = 15.0
    steady_frames: int = 3  # Frames in a row above (press) or below (release) the threshold before acting
    n_reference_frames: int = 5  # Latest untouched frames averaged into the reference


def calibration_holes(n_total: int, n: int, always: tuple[int, ...] = ()) -> tuple[int, ...]:
    """`n` holes evenly spaced over holes 1..`n_total` (first and last included), plus the `always` ones."""
    n = max(2, min(n, n_total))
    spaced = np.round(np.linspace(1, n_total, n)).astype(int)
    return tuple(sorted(set(spaced.tolist()) | {h for h in always if 1 <= h <= n_total}))


def change(frame: np.ndarray, reference: np.ndarray, start: int = wg_helpers.SEARCH_START) -> float:
    """Largest envelope of |frame - reference| after `start` (the transmit ring-down)."""
    return float(wg_helpers.ema(np.abs(np.asarray(frame, dtype=float) - reference))[start:].max())


@dataclass(frozen=True)
class Press:
    """One recorded step: key 0 (background) or a hole, its pass, and its frames (uint16)."""

    key: int
    pass_idx: int
    frames: tuple[np.ndarray, ...]


class CalibrationRecorder:
    """Walks through the calibration steps from the live frames (see the module docstring).

    `update(t, frame)` is called for every frame (from the sensor thread); the plot reads the state: `key` (0:
    background), `pass_idx`, `phase` ("quiet", "waiting", "recording", "release" or "done"), `step_idx` /
    `n_steps`, `last_change`, `threshold`, `pass_background` and `presses`.
    """

    def __init__(self, holes: tuple[int, ...], n_passes: int, cfg: AutoRecordConfig = AutoRecordConfig()):
        if not holes:
            raise ValueError("No hole to calibrate")
        self.holes, self.n_passes, self.cfg = tuple(holes), n_passes, cfg
        self.steps = [(p, key) for p in range(n_passes) for key in (0, *self.holes)]
        self.presses: list[Press] = []
        self.pass_background: np.ndarray | None = None
        self.threshold: float | None = None
        self.last_change = 0.0
        self._lock = threading.Lock()
        self._reference = deque(maxlen=cfg.n_reference_frames)  # Latest untouched frames
        self._step_idx = -1
        self._next_step()

    # --- State, for the plot
    @property
    def n_steps(self) -> int:
        return len(self.steps)

    @property
    def step_idx(self) -> int:
        return self._step_idx

    @property
    def done(self) -> bool:
        return self.phase == "done"

    @property
    def pass_idx(self) -> int:
        return self.steps[min(self._step_idx, self.n_steps - 1)][0]

    @property
    def key(self) -> int:
        return self.steps[min(self._step_idx, self.n_steps - 1)][1]

    def recorded_holes(self) -> set[int]:
        """Holes recorded in the current pass."""
        return {p.key for p in self.presses if p.pass_idx == self.pass_idx and p.key > 0}

    # --- Control
    def record_now(self) -> None:
        """Record the current step from the next frames, whatever the change; while waiting for a release, move on."""
        with self._lock:
            if self.phase == "release":
                self._next_step()
            elif self.phase in ("quiet", "waiting"):
                self._start_recording(forced=True)

    def update(self, t: float, frame: np.ndarray) -> None:
        with self._lock:
            self._update(t, np.asarray(frame))

    # --- Steps
    def _next_step(self) -> None:
        self._step_idx += 1
        self._t_step = None  # Time of the step's first frame
        self._count = 0  # Frames in a row past the threshold
        self._recording: list[np.ndarray] = []
        self._forced = False
        if self._step_idx >= self.n_steps:
            self.phase = "done"
        else:
            self.phase = "quiet" if self.key == 0 else "waiting"

    def _start_recording(self, forced: bool = False) -> None:
        self.phase, self._recording, self._forced = "recording", [], forced

    def _reference_frame(self) -> np.ndarray | None:
        if self._reference:
            return np.mean(self._reference, axis=0)
        return self.pass_background

    def _update(self, t: float, frame: np.ndarray) -> None:
        if self.phase == "done":
            return
        cfg = self.cfg
        reference = self._reference_frame()
        self.last_change = change(frame, reference, cfg.change_start) if reference is not None else 0.0
        touched = self.threshold is not None and self.last_change > self.threshold

        if self._t_step is None:
            self._t_step = t
        if self.phase == "quiet":  # Background: after quiet_s (and untouched, once a threshold is known)
            if touched:
                self._t_step = t  # Restart the wait
            elif t - self._t_step >= cfg.quiet_s:
                self._start_recording()
            return

        if self.phase == "waiting":
            if touched:
                self._count += 1
                if self._count >= cfg.steady_frames:
                    self._start_recording()
            else:
                self._count = 0
                self._reference.append(frame.astype(float))
            return

        if self.phase == "recording":
            if self.key > 0 and not self._forced and not touched:  # Released too early: wait for the press again
                self.phase, self._count = "waiting", 0
                return
            self._recording.append(frame.copy())
            if len(self._recording) == cfg.frames_per_press:
                self._finish_recording()
            return

        if self.phase == "release":
            if touched:
                self._count = 0
            else:
                self._count += 1
                if self._count >= cfg.steady_frames:
                    self._next_step()

    def _finish_recording(self) -> None:
        self.presses.append(Press(self.key, self.pass_idx, tuple(self._recording)))
        if self.key == 0:
            frames = np.array(self._recording, dtype=float)
            self.pass_background = frames.mean(axis=0)
            noise = max(change(f, self.pass_background, self.cfg.change_start) for f in frames)
            self.threshold = max(self.cfg.noise_factor * noise, self.cfg.min_change)
            self._reference.clear()
            self._reference.extend(frames)
            self._next_step()
        else:
            self.phase, self._count = "release", 0


# --- Save and fit
def create_signals_table(conn: sqlite3.Connection) -> None:
    """The SAVE_TO_SQLITE.py table, read by `wg_helpers.calibrate`."""
    conn.execute(
        """CREATE TABLE IF NOT EXISTS signals (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            key_vector TEXT NOT NULL,
            run_number INTEGER NOT NULL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            signal_blob BLOB NOT NULL
        )"""
    )


def write_recording(db_path: Path, presses: list[Press]) -> None:
    """The presses as a SAVE_TO_SQLITE.py database, one run per press, in order."""
    with closing(sqlite3.connect(db_path)) as conn:
        create_signals_table(conn)
        conn.executemany(
            "INSERT INTO signals (key_vector, run_number, signal_blob) VALUES (?, ?, ?)",
            [
                (json.dumps([press.key]), run, np.asarray(frame, dtype="<u2").tobytes())
                for run, press in enumerate(presses)
                for frame in press.frames
            ],
        )
        conn.commit()


def save_and_fit(presses: list[Press], db_path: Path, npz_path: Path, first_hole_cm: float, hole_spacing_cm: float) -> dict:
    """Write the recording to `db_path` and fit it to `npz_path` (`wg_helpers.calibrate`); returns the calibration.

    The previous files are kept as <name>_backup_<date>_<time>; they are put back if the fit fails.
    """
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backups = {}
    for path in (db_path, npz_path):
        if path.exists():
            backups[path] = path.with_name(f"{path.stem}_backup_{stamp}{path.suffix}")
            path.rename(backups[path])
    try:
        db_path.parent.mkdir(parents=True, exist_ok=True)
        write_recording(db_path, presses)
        return wg_helpers.calibrate(db_path, first_hole_cm, hole_spacing_cm, save_path=npz_path)
    except BaseException:
        for path in (db_path, npz_path):
            path.unlink(missing_ok=True)
        for path, backup in backups.items():
            backup.rename(path)
        raise
