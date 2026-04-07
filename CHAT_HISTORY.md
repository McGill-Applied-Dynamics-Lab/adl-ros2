# Chat History

## Global Changes

- Updated the fixed-burst Teensy acquisition workflow so `10RC_finish.ino` starts a 10-frame RF burst on host byte `67`, then sends and repeats finish byte `69` after the burst while waiting for the next `67`.
- Added `src/acoustic_sensing/teensy_firmware/debug_teensy_serial.py` to inspect raw Teensy serial bytes and run a `--burst-test` handshake check.
- Reworked `common.acquire_rf_burst()` in the experiment-configuration workflow to read raw byte chunks, ignore stale pre-frame `69` bytes, parse `S0` to `S3` frame text, and end a burst when raw `69` is received after frame data starts.
- Added RF acquisition debug prints and short per-burst SHA-256 fingerprints to `run_grid_probe_no_preplan.py` to detect repeated or stale RF data.
- Added grid reproducibility metadata and SHA-256 fields to `waveguide_gripper_grid_generator.py` output under `grids.pkl["_metadata"]`.
- Updated experiment-configuration and Teensy firmware READMEs to document the fixed-burst handshake, debug workflow, RF SHA checks, and grid SHA metadata.
- Added a `TEST` / `TESTEND` mode to `10RC_finish.ino` for continuous RF frame streaming without changing the normal `67` fixed-burst / `69` finish behavior.
- Added `src/acoustic_sensing/teensy_firmware/live_plot_teensy_qt.py`, a Qt/pyqtgraph live plotter that sends `TEST`, plots `S0` to `S3` frames, and sends `TESTEND` on close.
- Documented that test mode can be stopped by closing the Qt plotter or manually sending ASCII `TESTEND` over serial.
- Updated the Teensy firmware documentation to match the active firmware pin mapping and current file layout.
- Corrected the 4RF Teensy firmware pin definitions and kept the README aligned with the active `.ino` source.
- Updated the waveguide gripper probing work for the 180-degree end-effector orientation.
- Adjusted the waveguide grid generation during iteration to match the current physical probing layout.
- Added an example-based probing path using `execute_sequence()` in `examples/12_probe_sequence_180.py`.
- Reworked the `src` 4RF probing logic toward joint-trajectory planning and sequence execution.
- Added visible preplanning progress output and worker-count reporting for the offline planning stage.
- Split the 4RF probing workflow into an offline precompute phase and a runtime execution phase.
- Added a dedicated experiment-configuration area for precomputed probing workflows.
- Moved the older ad hoc probing scripts under `src/acoustic_sensing/scripts/testing`.
- Switched experiment-config grids/landmarks/results to live entirely under `src/acoustic_sensing/scripts/experiment_configuration/results`.
- Added a live grid runner and expanded the precomputed runner options for approach-only vs full probe motion.
- Added a lifted-landmark startup model for the precomputed waypoint planner and explicit startup descent to the landmark home pose.
- Changed the first move of the precomputed plan to be the only protected `up -> across -> down` travel; later inter-location moves remain direct.
- Tightened vertical waypoint spacing for probe up/down and startup lift/lower to `0.5 mm` step size.
- Switched the non-Teensy waypoint runner back to background probe planning using a planner-only IK path instead of a second ROS `Robot` publisher.
- Added explicit execution of the saved startup descent in the precomputed runners so the tool touches the landmark home pose before location 1.
- Forced the background probe planner to use the CPU JAX path to avoid concurrent GPU/cuSolver failures.
- Added total wall-clock timing output to the precompute script after the artifact is saved.
- Simplified `examples/00_home.py` so it only homes the robot, sets the `(180, 0, 0)` end-effector orientation, and stops.
- Added startup diagnostics to precompute and the precomputed runners so the saved lifted-start pose, landmark home pose, and startup transition indices are printed explicitly.
- Separated landmark touch height from actuator probing height with a shared actuator-surface Z offset in experiment-config workflows.

## Implemented Decisions

- Fixed-burst RF acquisition now uses the protocol: host sends `67`; Teensy streams 10 RF frames; Teensy waits briefly, sends `69`, and continues sending `69` until the next `67`.
- Host-side RF parsing treats raw `69` as the end marker only after frame data starts, so idle repeated `69` bytes do not terminate a new acquisition before it begins.
- `run_grid_probe_no_preplan.py` records live pose-controller probe data without full-sequence preplanning and logs RF frame counts plus SHA summaries for each burst.
- `waveguide_gripper_grid_generator.py` stores a deterministic grid SHA to make generated grid files traceable to their landmarks and generator parameters.
- Firmware test mode is explicitly separate from experiment acquisition: `TEST` starts continuous plotting/debug streaming, `TESTEND` stops it, and normal acquisition continues to use `67` and `69`.
- Updated the 4RF probing work to use a 180-degree end-effector orientation.
- Kept the world-frame probing direction as vertical up/down motion.
- Adjusted the waveguide grid generation to reflect the current physical orientation updates during iteration.
- Reworked the probing logic toward joint-trajectory planning instead of the earlier pose-controller workflow.
- Added preplanning progress output and worker-count reporting for the precompute stage.
- Switched the main probe execution model to sequence-based execution with explicit plunge and retract phases.
- Preserved chained joint continuity in planning by using segment-by-segment planning with the previous segment's final joint configuration as the next seed.
- Added a split workflow:
  - offline precompute of the probing sequence
  - runtime execution with a single startup bridge from the actual robot state to the assumed precomputed start state
- Added a dedicated experiment-configuration folder with separate scripts for precompute and runtime execution.
- Changed the experiment-config precompute start state to the landmark-based home pose with `robot.config.home_config`.
- Centralized shared probe settings in `src/acoustic_sensing/scripts/experiment_configuration/common.py`.
- Added an optional precompute mode that includes plunge/retract probe motion and saves a separate artifact.
- Updated the waypoint-only runner so it can either:
  - execute a fully precomputed approach+probe artifact
  - or use precomputed approach motion and compute plunge/retract live
- Added continuity metadata for each saved approach pose so probing can transition into the next approach pose without a discontinuous jump when using live-computed probe strokes.
- Changed precompute so the saved sequence starts above the landmark, descends vertically to the landmark home pose, then begins the first protected move to the first probe location.
- Changed the precompute script to save comparison artifacts with `_new` suffixes instead of overwriting the older names.
- Updated the no-RF waypoint runner so each location executes as one combined sequence block:
  - precomputed approach segment(s), if needed
  - live-planned plunge/retract
  - optional transition to the next approach pose
- Updated both precomputed runners so they explicitly execute the stored startup descent segment before beginning location 1.
- Updated the no-RF waypoint runner so its background planner uses a planner-only CPU JAX path and execution waits until each location block is ready.
- Updated experiment-config precompute and live runners so landmark touch uses the landmark home Z while actual actuator probing uses the actuator-surface Z offset.

## Final Structure

- `src/acoustic_sensing/teensy_firmware/README.md`
- `src/acoustic_sensing/teensy_firmware/debug_teensy_serial.py`
- `src/acoustic_sensing/teensy_firmware/live_plot_teensy_qt.py`
- `src/acoustic_sensing/teensy_firmware/10RC_finish/10RC_finish.ino`
- `src/acoustic_sensing/scripts/testing/waveguide_gripper_grid_generator.py`
- `src/acoustic_sensing/scripts/testing/probing_waveguide_gripper_4rf_6769.py`
- `examples/12_probe_sequence_180.py`
- `src/acoustic_sensing/scripts/experiment_configuration/precompute_probe_sequence_4rf_teensy_sync.py`
- `src/acoustic_sensing/scripts/experiment_configuration/run_precomputed_probe_sequence_4rf_teensy_sync.py`
- `src/acoustic_sensing/scripts/experiment_configuration/run_precomputed_waypoints_only.py`
- `src/acoustic_sensing/scripts/experiment_configuration/run_live_grid_probe_waypoints_only.py`
- `src/acoustic_sensing/scripts/experiment_configuration/run_grid_probe_no_preplan.py`
- `src/acoustic_sensing/scripts/experiment_configuration/waveguide_gripper_grid_generator.py`
- `src/acoustic_sensing/scripts/experiment_configuration/common.py`
- `src/acoustic_sensing/scripts/experiment_configuration/README.md`
