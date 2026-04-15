# Rolling Testing Change History

This is a compact local summary of the major changes made during the rolling-testing work in this directory.

## Core Script Evolution

- Started from a rolling-contact script that originally used a different motion model and simpler save layout.
- Switched the experiment to a straight slide along `Y` instead of the older rolling/arc-style motion.
- Reversed the slide direction to `-Y` after observing the real motion direction.
- Moved experiment parameters into `rolling_config.py`.
- Changed interpolation from fixed waypoint counts to spacing-based interpolation.
- Changed result saving to one folder per session and one file per rolling speed.
- Added per-trial replay support with `read_rolling_results.py`.

## RF / Teensy Changes

- Added explicit serial handshake:
  - `0x43` starts streaming
  - `0x45` stops streaming
- Changed experiment flow so RF starts after contact and waits for the first RF frame before continuing.
- Added Teensy firmware under an Arduino-style sketch folder.
- Added host-side live RF viewer in the firmware folder.
- Updated channel pin mapping to match the latest intended wiring, including swapping channels 3 and 4 after bench observation.
- Updated sample-count assumptions to match the larger acquisition window used by the newer firmware lineage.

## Saving / Replay Changes

- Results now save under:
  - `results/trial_XX/rolling_speed_XXXX.X_mm_s.pkl`
- Replay tool redesigned with dark theme (`read_rolling_results.py`):
  - Four RF channel charts on the left (animated, dark panel)
  - Contact visualization top-right: rounded sensor body, color-coded phase label (yellow/green/orange), band tracks from base to tip during slide
  - **Dedicated EEF state panel** bottom-right: always-visible monospace display of time, phase durations, XYZ position, and RPY rotation (no longer blocked by the contact visualization)

## Planning / Motion Changes

- Switched experiment execution to use `plan_joint_trajectory_sequence(...)` and `execute_sequence(...)`.
- Tried a merged `compress + slide + decompress` plan for better continuity.
- Later reverted to stop-by-stop replanning because of joint inconsistency and pose drift.
- Added measured-state replanning at each segment boundary.
- Removed forced orientation reset during contact motion after it caused confusing corrections.
- Added wanted-vs-actual pose prints for each segment.
- Added a narrow first-waypoint fix:
  - first planned joint sample is anchored to current measured `q`
  - only the first interval is expanded if needed

## fr3_pose_controller Migration

- Created `rolling_contact_experiment_fr3pose.py` to replace the joint-space version as the primary experiment script.
- Uses `fr3_pose_controller` which tracks Cartesian targets natively, eliminating Z drift during the slide phase.
- Z stability improved from ~2 mm drift to ±0.03 mm during slide.
- Orientation drift reduced from ~1° (joint-space) to <0.1° (Cartesian).
- All contact phases (compress / slide / decompress) and the approach/return phases use `robot.execute_trajectory`, giving smooth quintic interpolation with no Python-side timing jitter.
- `_build_contact_trajectory` interpolates linearly across multi-segment paths and builds `(waypoints, times)` for `execute_trajectory`.
- `EXEC_TRAJ_TIME_OFFSET = 0.6` shifts all waypoint timestamps forward to compensate for the 0.5 s internal sleep inside `execute_trajectory`, preventing the `cartesian_motion_generator_joint_acceleration_discontinuity` libfranka reflex.
- `rf_window_duration` accounts for per-phase overhead: `(0.5 + EXEC_TRAJ_TIME_OFFSET + 2.0) × 3` phases.

## Current State

The primary workflow is:

1. Switch to `fr3_pose_controller`.
2. Approach and descend via `execute_trajectory`.
3. Start RF, wait for first frame.
4. Compress → settle → slide → decompress, each via `execute_trajectory`.
5. Stop RF.
6. Return via `execute_trajectory`.
7. Switch back to `joint_trajectory_controller`.

## Practical Debugging Advice

If the robot goes to a pose that does not make sense:

1. Check the printed `start wanted` and `end wanted` values for the segment.
2. Compare them against the printed `actual` pose after execution.
3. If the target itself is wrong, the planning inputs are wrong.
4. If the target is right but the actual pose is far off, the issue is execution / IK / controller behavior.

If `cartesian_motion_generator_joint_acceleration_discontinuity` fires on compress:

- The most likely cause is `EXEC_TRAJ_TIME_OFFSET` being too small.
- The controller received a trajectory whose first waypoint timestamp is already in the past.
- Increase `EXEC_TRAJ_TIME_OFFSET` slightly (e.g. 0.6 → 0.7).
