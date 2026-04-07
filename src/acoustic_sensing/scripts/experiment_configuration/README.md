# Experiment Configuration

This folder contains the current self-contained 4RF waveguide probing workflow.

## Files

- `precompute_probe_sequence_4rf_teensy_sync.py`
  - Loads grids/landmarks from this folder.
  - Uses a lifted landmark-home startup pose plus `robot.config.home_config` as the assumed start state.
  - The saved sequence begins by descending vertically from the lifted startup pose to the landmark home pose.
  - The first move from the landmark home pose to the first probe location is protected with `up -> across -> down`.
  - Later inter-location moves remain direct.
  - Precomputes either:
    - approach-only motion
    - or full approach+plunge+retract motion
  - Saves planned joint trajectories to `artifacts/`.
  - Prints total wall-clock precompute time after saving the artifact.

- `run_precomputed_probe_sequence_4rf_teensy_sync.py`
  - Loads the approach-only precomputed artifact.
  - Moves to the saved sequence start joint configuration.
  - Executes the saved startup descent to the landmark home pose before location 1.
  - Executes precomputed approach motion.
  - Computes plunge/retract online, waits for Teensy at the bottom, then retracts.
  - Records RF, pose, and wrench data.
  - Prints the saved startup pose, landmark home pose, and startup transition indices when loading the artifact.

- `run_precomputed_waypoints_only.py`
  - Test runner with no Teensy/RF.
  - Asks whether to use:
    - fully precomputed approach+probe motion
    - or precomputed approach plus live-computed plunge/retract
  - Executes the saved startup descent to the landmark home pose before location 1.
  - In the live-computed probe mode, probe strokes are planned in the background with a planner-only CPU JAX IK path and execution waits if the next location is not ready yet.
  - Each location executes as one combined sequence block to preserve approach/probe continuity.
  - Returns home at the end.
  - Prints the saved startup pose, landmark home pose, and startup transition indices when loading the artifact.

- `run_live_grid_probe_waypoints_only.py`
  - Builds one continuous live sequence directly from the grid.
  - No precomputed artifact and no Teensy wait.

- `run_grid_probe_no_preplan.py`
  - Live grid probe runner with no full-sequence preplanning.
  - Switches to `fr3_pose_controller` and refuses to continue if that controller is not active.
  - Opens the Teensy serial port and calls `common.acquire_rf_burst()` at each probe depth.
  - Captures robot end-effector pose/orientation/force samples during each plunge.
  - Saves results incrementally after each successful location to `results/`.
  - Prints per-burst RF debug progress plus a short SHA-256 fingerprint so repeated/stale RF data can be detected.
  - Skips a location if the Teensy burst times out instead of blocking the entire run.

- `common.py`
  - Shared geometry/planning utilities and shared settings.
  - Source of truth for:
    - `PROBE_DEPTH`
    - `PROBE_STEP_SIZE`
    - `APPROACH_LIFT_HEIGHT`
    - `ACTUATOR_SURFACE_Z_OFFSET`
    - durations
    - orientation
    - artifacts/results paths
  - Implements the fixed-burst Teensy reader:
    - clears stale serial input
    - sends `67` / `0x43`
    - parses newline-delimited `S0` to `S3` RF frames
    - treats a raw `69` / `0x45` byte received after frame data starts as end-of-burst
    - ignores stale repeated `69` bytes before frame data starts

- `waveguide_gripper_grid_generator.py`
  - Local grid generator for this workflow only.
  - Reads/writes `results/grids/` inside this folder.
  - Stores reproducibility metadata in `grids.pkl` under `_metadata`.
  - Prints and stores `_metadata["sha256"]` and `_metadata["sha256_short"]` for the generated grid and grid parameters.

## Workflow

0. If you want a minimal setup helper, `examples/00_home.py` now only:
   - homes the robot
   - sets the end-effector orientation to `(180, 0, 0)`
   - stops
1. Populate `results/grids/landmarks.txt` in this folder.
2. Run `waveguide_gripper_grid_generator.py`.
3. Set `PROBE_DEPTH` / `PROBE_STEP_SIZE` in `common.py`.
   - If the actuator surface sits below the landmark contact height, set `ACTUATOR_SURFACE_Z_OFFSET` in `common.py`.
4. Run `precompute_probe_sequence_4rf_teensy_sync.py`.
   - The script saves `_new.pkl` artifacts for comparison with older artifacts.
   - The script prints the saved lifted startup pose, landmark home pose, startup descent distance, and startup transition indices.
5. Choose one runner:
   - `run_precomputed_probe_sequence_4rf_teensy_sync.py` for Teensy-synced probing
   - `run_grid_probe_no_preplan.py` for live pose-controller probing with RF capture and no full-sequence preplanning
   - `run_precomputed_waypoints_only.py` for no-RF testing
   - `run_live_grid_probe_waypoints_only.py` for full live planning

## Teensy Fixed-Burst Handshake

The current fixed-burst experiment path expects the Teensy sketch `10RC_finish.ino`.

- Host sends `67` / `0x43` / `'C'` to start one RF burst.
- Teensy streams 10 frames.
- Each frame contains `S0`, `S1`, `S2`, and `S3` channel blocks terminated by `T`.
- Teensy sends raw `69` / `0x45` / `'E'` after the 10th frame.
- Teensy continues sending `69` while waiting for the next `67`.
- `common.acquire_rf_burst()` detects the raw byte directly, so `69` does not need a newline.

To verify the firmware without moving the robot:

```bash
python3 src/acoustic_sensing/teensy_firmware/debug_teensy_serial.py --burst-test
```

Expected output includes:

```text
TX dec=67 hex=0x43 ascii='C'
RX frame data started
RX received end byte 69 / 0x45 / 'E'
```

To inspect continuous live RF frames without moving the robot:

```bash
python3 src/acoustic_sensing/teensy_firmware/live_plot_teensy_qt.py
```

The plotter sends `TEST`, parses the same `S0` to `S3` frame format, and sends `TESTEND` when the window closes.

This test-mode workflow has been exercised after flashing the fixed-burst firmware and is the recommended first check before running robot motion.

To stop test mode manually, send ASCII `TESTEND` over serial:

```python
ser.write(b"TESTEND")
ser.flush()
```

## Debugging And Reproducibility

- Use the per-burst SHA line from `run_grid_probe_no_preplan.py` to check whether RF data is changing:
  - Different SHA values mean the returned RF data is not exactly identical.
  - Repeated SHA values across different locations are a strong signal to inspect Teensy triggering, sensor conditions, or parser state.
- Use the grid SHA from `waveguide_gripper_grid_generator.py` to confirm that a `grids.pkl` file came from the expected landmarks and grid parameters.
- The debug serial tool can also show raw idle `69` traffic:

```bash
python3 src/acoustic_sensing/teensy_firmware/debug_teensy_serial.py
```

If the Teensy is between bursts, repeated lines like `dec=[69] hex=[45] ascii='E'` indicate that it is waiting for the next `67`.

## Artifacts

- Approach-only precompute:
  - `artifacts/precomputed_<SET>_4RF_joint_sequence_new.pkl`
- Approach+probe precompute:
  - `artifacts/precomputed_<SET>_4RF_joint_sequence_with_probe_new.pkl`
- Live no-preplan RF results:
  - `results/<N>_grid_<SET>_RF_<NN>.pkl`
- Generated grid file:
  - `results/grids/grids.pkl`
  - Contains `GRIPPER_FRAME`, `WORLD_FRAME`, and `_metadata` with `sha256` fields.

## Notes

- The precompute script asks for `train/test`.
- If you choose to precompute plunge/retract too, it asks for confirmation of the current `PROBE_DEPTH` and `PROBE_STEP_SIZE` from `common.py`.
- Vertical motions use the finer `PROBE_STEP_SIZE` spacing, including probe plunge/retract and the startup lift/lower around the landmark home pose.
- Landmark contact and actuator probing are separated:
  - landmark touch uses `landmarks["z"] + Z_OFFSET`
  - actuator probing uses `landmarks["z"] + Z_OFFSET + ACTUATOR_SURFACE_Z_OFFSET`
- The precomputed runners expect regenerated `_new` artifacts if you want the explicit startup descent to the landmark home pose.
- The experiment-config workflow no longer uses `src/acoustic_sensing/scripts/results`; it uses `src/acoustic_sensing/scripts/experiment_configuration/results`.
- Older exploratory scripts remain under `src/acoustic_sensing/scripts/testing`.
