# Rolling Testing

This directory contains the rolling-contact experiment workflow, replay tools, and Teensy-side utilities for the fin-ray RF rolling tests.

## Main Files

- `rolling_contact_experiment_fr3pose.py` ← **primary experiment script**
  Runs the rolling experiment using the `fr3_pose_controller` (Cartesian pose tracking).
  Smooth, jerk-free motion via `execute_trajectory` with quintic interpolation.
- `rolling_contact_experiment.py`
  Legacy script that uses `joint_trajectory_controller` (joint-space IK planning).
  Kept for reference; the fr3_pose version is preferred.
- `rolling_config.py`
  Central configuration for robot motion, RF serial settings, roller geometry, and experiment tuning.
- `read_rolling_results.py`
  Replays a saved rolling result file. Dark-themed dashboard with RF plots, contact
  visualization, and a dedicated EEF state panel (position + rotation always visible).
- `rolling_testing_teensy_firmware/rolling_testing_teensy_firmware.ino`
  Teensy firmware for serial-controlled RF streaming.
- `rolling_testing_teensy_firmware/live_plot_send_67_stop_69.py`
  Host-side Qt live viewer that sends `67` on startup and `69` on shutdown.

## Current Experiment Flow (`rolling_contact_experiment_fr3pose.py`)

Per trial, the robot does:

1. Switch to `fr3_pose_controller`.
2. Move to approach pose (above contact point).
3. Descend to contact pose via `execute_trajectory`.
4. Send `0x43` to Teensy; wait for the first RF frame.
5. Compress in `-Z` via `execute_trajectory` (smooth, quintic).
6. Short settle hold.
7. Slide in `-Y` via `execute_trajectory`.
8. Decompress in `+Z` via `execute_trajectory`.
9. Send `0x45` to stop RF.
10. Return to approach pose.
11. Switch back to `joint_trajectory_controller`.

All contact-phase motion (compress / slide / decompress) uses `robot.execute_trajectory`
so the controller performs quintic polynomial interpolation internally — no Python-side
timing jitter.

## Why `fr3_pose_controller`?

The joint-space version (`joint_trajectory_controller`) suffers from Z drift (~2 mm)
during the slide because joint-space cubic interpolation between IK waypoints does not
follow a perfectly straight Cartesian line.  The fr3_pose controller tracks Cartesian
targets natively, giving ±0.03 mm Z stability during the slide.

## Trajectory Timing

`execute_trajectory` sleeps 0.5 s before publishing the message, so the header
timestamp is already 0.5 s old when the controller receives it.  If waypoint times are
not shifted forward, the first waypoint appears in the past and the controller triggers a
`cartesian_motion_generator_joint_acceleration_discontinuity` reflex.

Fix: `EXEC_TRAJ_TIME_OFFSET = 0.6` is added to every waypoint timestamp inside
`_build_contact_trajectory`.  The `wait_for_trajectory_completion` timeout is padded by
the same amount.

## Result Replay (`read_rolling_results.py`)

Run interactively (prompts for trial and speed file):

```bash
python read_rolling_results.py
```

Or pass a path directly:

```bash
python read_rolling_results.py results/trial_XX/rolling_speed_5.0_mm_s.pkl
```

The replay window shows:
- **Left**: four RF channel waveforms (S0–S3), animated frame-by-frame.
- **Top right**: contact visualization — a band tracks from base to tip as the slide progresses; phase label is color-coded (yellow = compress, green = slide, orange = decompress).
- **Bottom right**: EEF state panel — always-visible text showing time, phase durations, XYZ position, and RPY rotation.

## Important Behavior Notes

- `DRY_RUN = True` skips serial/RF only; the robot still moves.
- Results save under `results/trial_XX/`, one `.pkl` per speed.
- RF handshake: PC sends `0x43` (`CMD_START`) to start streaming, `0x45` (`CMD_STOP`) to stop.
- Python waits for the first RF frame before starting the contact motion.

## Key Configuration (`rolling_config.py`)

| Parameter | Purpose |
|---|---|
| `CONTACT_POSITION_M` | World-frame XYZ of sensor at φ=0 (cylinder top). |
| `COMPRESS_DEPTH_M` | How far below contact surface to press (preload). |
| `FINRAY_LENGTH_M` / `ROLL_LENGTH_PERCENT` | Determines slide travel distance. |
| `ROLL_SPEEDS_M_S` | List of speeds; one full trial per speed. |
| `APPROACH_SPEED_M_S` / `RETURN_SPEED_M_S` | Speeds for non-contact phases. |
| `EXEC_TRAJ_TIME_OFFSET` | Timestamp forward-shift to avoid acceleration reflex. |

## Useful Files To Check First

- `rolling_config.py` — all tunable parameters
- `rolling_contact_experiment_fr3pose.py` — primary experiment script
- `results/trial_XX/...` — saved results
