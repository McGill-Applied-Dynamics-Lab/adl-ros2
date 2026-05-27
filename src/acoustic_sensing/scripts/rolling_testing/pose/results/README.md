# Rolling-contact results

Each rolling-contact experiment writes one **trial directory** under this folder.
A trial directory holds one pickle per `(speed, repeat)` combination produced by
`live_rolling_view.py` / `rolling_contact_experiment_fr3pose.py` /
`rolling_contact_experiment.py`.

## Folder layout

```
results/
├── <trial-dir>/                 # e.g. trial_07, 2mm_depth_dia24mm, …
│   ├── rolling_speed_0005.0_mm_s_rep_00.pkl
│   ├── rolling_speed_0005.0_mm_s_rep_01.pkl
│   ├── rolling_speed_0010.0_mm_s_rep_00.pkl
│   └── …
├── README.md                    # this file
└── simple_reader.py             # standalone example reader
```

File name convention:

```
rolling_speed_{speed_mm_s:06.1f}_mm_s_rep_{repeat_idx:02d}.pkl
                  └ slide speed in mm/s       └ 0-indexed repeat
```

## Pickle schema

Each `.pkl` is a Python dict with two top-level keys: `trial` and `session`.

```python
{
  "trial":   { ... per-trial data, see below ... },
  "session": { ... metadata shared across all trials in this dir ... },
}
```

### `trial`

| Key | Type | Description |
| --- | --- | --- |
| `speed_m_s` | float | Commanded slide speed (m/s) |
| `speed_idx` | int | Index into `session["speeds_m_s"]` |
| `repeat_idx` | int | 0..`N_REPEATS-1` |
| `roll_distance_m` | float | Effective slide length (m). Equals `finray_length_m * roll_length_percent/100 - roll_end_offset_m` |
| `roll_end_offset_m` | float | Safety margin subtracted from the sensor tip (m) |
| `roller_diameter_m` | float | Roller cylinder diameter (m) |
| `roller_radius_m` | float | `roller_diameter_m / 2` |
| `compress_duration_s` | float | Planned duration of the compress segment (s) |
| `slide_duration_s` | float | Planned duration of the slide segment (s) |
| `decompress_duration_s` | float | Planned duration of the decompress segment (s) |
| `rf_window_duration_s` | float | Window over which RF was streamed during this trial (s) |
| `approach_position` | list[3] | Cartesian XYZ of the approach pose (m, world frame) |
| `contact_position` | list[3] | Measured EE position at first contact (m) |
| `compressed_start_position` | list[3] | EE position at slide start (m, after compress) |
| `compressed_end_position` | list[3] | EE position at slide end (m) |
| `surface_end_position` | list[3] | EE position after decompress (m) |
| `frames` | list[(rf, t)] | Rolling-phase RF frames; see **RF frames** below |
| `ee_poses` | list[dict] | EE telemetry samples during the whole trial; see **EE samples** |
| `ee_poses_slide_only` | list[dict] | Subset of `ee_poses` filtered to the slide-phase Z plane (±1 mm) |
| `rf_capture_summary` | dict | `{"frame_count": int, "coverage_ratio": float}` over `rf_window_duration_s` |
| `static_rf` | dict | Stationary baseline captured before any rolling; see **Static RF baseline** |

### `session`

| Key | Type | Description |
| --- | --- | --- |
| `controller` | str | ROS 2 controller name used (`fr3_pose_controller` or `joint_trajectory_controller`) |
| `roller_diameter_m` | float | Same as per-trial; kept for convenience |
| `compress_depth_m` | float | Z compression applied at contact (m) |
| `finray_length_m` | float | Sensor active length (m) |
| `roll_length_percent` | float | Percentage of fin-ray length rolled |
| `roll_distance_m` | float | Effective slide length (m) |
| `speeds_m_s` | list[float] | All speeds in this session |
| `expected_rf_samples` | int | Samples per RF frame per channel (always 1000 in current firmware) |
| `trial_dir` | str | Absolute path of the trial directory at acquisition time |
| `stationary_baseline_frames` | int (opt) | Number of static RF frames requested before rolling started |
| `baseline_extra_height_m` | float (opt) | Extra Z lift above approach used for the stationary baseline (m) |

### RF frames

Both `trial["frames"]` and `trial["static_rf"]["frames"]` are lists of tuples:

```python
(rf, t_seconds_since_capture_start)
```

- `rf` is a list of **4** `numpy.ndarray` (`dtype=float32`, shape `(1000,)`),
  ordered `[S0, S1, S2, S3]`.
- `t` is a `float` — seconds since the RF stream was started for that capture
  (i.e. timestamps in `static_rf["frames"]` and `trial["frames"]` use
  **different t=0 origins**; do not mix).

A single frame typically spans ~22 ms on the wire (Teensy fires the four
waveguides sequentially, 1000 samples each).

### EE samples

Each entry in `ee_poses` and `ee_poses_slide_only` is:

```python
{
  "t":                            float,      # s since trial start
  "position":                     [x, y, z],  # m, world frame
  "orientation_quat":             [x, y, z, w],
  "orientation_euler_deg_xyz":    [rx, ry, rz],  # degrees
  "force_xyz":                    [fx, fy, fz],  # N, EE frame
  "torque_xyz":                   [tx, ty, tz],  # Nm
  "velocity_xyz":                 [vx, vy, vz] | None,  # None on the first sample
  "acceleration_xyz":             [ax, ay, az] | None,  # None on the first two samples
}
```

Sampled at ~50 Hz (period `EE_SAMPLE_PERIOD_SEC` in `rolling_config.py`).

### Static RF baseline

```python
trial["static_rf"] = {
  "frames":              [(rf, t), ...],     # may be empty if STATIONARY_BASELINE_FRAMES = 0
  "n_frames_requested":  int,
  "ee_pose":             {"position": [..], "orientation_quat": [..], "orientation_euler_deg_xyz": [..]} | None,
  "expected_rf_samples": int,                # 1000
}
```

These frames are captured with the robot held still ~80 mm above the cylinder
(approach + `baseline_extra_height_m`), with **no contact**. Use them as a
per-session no-contact reference (e.g. for background subtraction or to detect
electronics drift between sessions).

## Quickstart

```bash
# Plot one file with the bundled viewer:
../view_results.sh <trial-dir>/rolling_speed_0010.0_mm_s_rep_00.pkl

# Just inspect the contents from Python:
python simple_reader.py <trial-dir>/rolling_speed_0010.0_mm_s_rep_00.pkl
```

`simple_reader.py` is a ~60-line, dependency-free (numpy only) example that
prints the schema, the static-RF stats, the slide RF stats, and a small EE
summary — copy it into your own analysis script as a starting point.
