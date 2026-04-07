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

- `common.py`
  - Shared geometry/planning utilities and shared settings.
  - Source of truth for:
    - `PROBE_DEPTH`
    - `PROBE_STEP_SIZE`
    - `APPROACH_LIFT_HEIGHT`
    - durations
    - orientation
    - artifacts/results paths

- `waveguide_gripper_grid_generator.py`
  - Local grid generator for this workflow only.
  - Reads/writes `results/grids/` inside this folder.

## Workflow

0. If you want a minimal setup helper, `examples/00_home.py` now only:
   - homes the robot
   - sets the end-effector orientation to `(180, 0, 0)`
   - stops
1. Populate `results/grids/landmarks.txt` in this folder.
2. Run `waveguide_gripper_grid_generator.py`.
3. Set `PROBE_DEPTH` / `PROBE_STEP_SIZE` in `common.py`.
4. Run `precompute_probe_sequence_4rf_teensy_sync.py`.
   - The script saves `_new.pkl` artifacts for comparison with older artifacts.
   - The script prints the saved lifted startup pose, landmark home pose, startup descent distance, and startup transition indices.
5. Choose one runner:
   - `run_precomputed_probe_sequence_4rf_teensy_sync.py` for Teensy-synced probing
   - `run_precomputed_waypoints_only.py` for no-RF testing
   - `run_live_grid_probe_waypoints_only.py` for full live planning

## Artifacts

- Approach-only precompute:
  - `artifacts/precomputed_<SET>_4RF_joint_sequence_new.pkl`
- Approach+probe precompute:
  - `artifacts/precomputed_<SET>_4RF_joint_sequence_with_probe_new.pkl`

## Notes

- The precompute script asks for `train/test`.
- If you choose to precompute plunge/retract too, it asks for confirmation of the current `PROBE_DEPTH` and `PROBE_STEP_SIZE` from `common.py`.
- Vertical motions use the finer `PROBE_STEP_SIZE` spacing, including probe plunge/retract and the startup lift/lower around the landmark home pose.
- The precomputed runners expect regenerated `_new` artifacts if you want the explicit startup descent to the landmark home pose.
- The experiment-config workflow no longer uses `src/acoustic_sensing/scripts/results`; it uses `src/acoustic_sensing/scripts/experiment configuration/results`.
- Older exploratory scripts remain under `src/acoustic_sensing/scripts/testing`.
