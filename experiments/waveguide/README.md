# Waveguide Experiments
For the experiments using the **Waveguide with holes** device.

## Environment
The files here run from the `wg` pixi environment. To activate
```bash
pixi shell -e wg
```

Or directly run the files:
```bash
pixi run -e wg python <file>
```

## Sensor Testing
The main file for reading the sensor is `sensor.py`.

Standalone (the port defaults to the connected Teensy sending frames, see --port):
```
    pixi run -e wg python experiments/waveguide/sensor.py               # frame rate, pressed holes, commands
    pixi run -e wg python experiments/waveguide/sensor.py --live-plot   # raw frames
    pixi run -e wg python experiments/waveguide/sensor.py --calibrate   # guided calibration, see CalibrationConfig
    pixi run -e wg python experiments/waveguide/sensor.py --live-match  # live trace vs the newest calibration/reference_*.csv
    pixi run -e wg python experiments/waveguide/sensor.py --live-match calibration/reference_10_14_2026.csv
```


### Calibrating from the live plot
After a restart or reconnection, recalibrate from the `--live-plot` window (also in `wg_teleop.py --live-plot`):
open the ⚙ tab on the right (or press C), choose the number of holes (evenly spaced, plus the `GestureConfig` z
holes) and of passes, then Start. The waveguide shows the hole to press in green; each step moves on by itself
(hold until it changes, then release). Space records the current step now (e.g. a weak hole), Esc cancels.
Detection and teleop commands are paused meanwhile. The fit is saved to `CalibrationConfig.npz_path` (the previous
files kept as `*_backup_<date>`) and used right away; the background is then recorded again: don't touch.

## Teloperation
To teleop the franka with the sensor, use the `wg_teleop.py` file.

```bash
    pixi run -e wg python experiments/waveguide/wg_teleop.py                    # Waveguide, the Teensy sending frames (--port to pick one)
    pixi run -e wg python experiments/waveguide/wg_teleop.py --live-plot        # Same, with the sensor live plot
    pixi run -e wg python experiments/waveguide/wg_teleop.py --input terminal   # Commands typed in the terminal
```

It's possible to test the teleoperation without the waveguide, inputing the commands via the terminal.

## Recording
`--record` (with an optional `--notes "..."`) saves the run with the lab's `experiment_logger`, in
`~/data/waveguide/teleop/<date-time>/` for `wg_teleop.py` and `~/data/waveguide/sensor/<date-time>/` for `sensor.py`
(default and `--live-plot` modes):

```bash
    pixi run -e wg python experiments/waveguide/wg_teleop.py --live-plot --record --notes "demo 1, take 3"
    pixi run -e wg python experiments/waveguide/sensor.py --live-plot --record
```

The folder holds `samples.mcap` (open it in Foxglove, or `experiment_logger.load_streams(run_dir)` for one DataFrame
per stream), `metadata.json` (configs, args, git commit, notes) and a copy of each calibration used. Streams:
`sensor/frame` (the averaged frame of every estimate), `sensor/detection` (every frame), `sensor/background`,
`sensor/calibration`, `sensor/swipe` (ignored ones too), and with the robot `robot/command` (executed, ignored while
moving, invalid) and `robot/state` (end-effector pose and target, 50 Hz). See `recording.py` for the fields.

To read and plot a run, `plot_run.py` is standalone: it only needs numpy, matplotlib and mcap
(`pip install numpy matplotlib mcap`), so it can be copied anywhere. Its `load_run` / `stack` give the data as plain
Python and numpy (see its docstring).

```bash
    python experiments/waveguide/plot_run.py                                  # newest run in ~/data/waveguide/*/
    python experiments/waveguide/plot_run.py ~/data/waveguide/teleop/20261022-154716 --start 10 --end 40 --save run.png
```

## Demos with on-board processing
For these demos the Teensy runs the hole detection itself (TEENSY_BLUETOOTH firmware, one or two holes with the
"multi" method) and sends a result every cycle (~20/s), read by `touch.py` through `wg_helpers` (Bluetooth section).
- Link: BLE to the HC-08 by default (`--link ble --ble-name WAVEGUIDE`), or a serial port (`--link serial --port ...`)
- Sensor calibration (`--sensor-calibration`, a `wg_helpers.calibrate()` .npz): uploaded when the Teensy has none,
  i.e. after a power cycle (about a minute over BLE); `--upload` replaces it
- The background is recorded at every start: don't touch the waveguide then
- Placing the sensor in the waveguide: with the Teensy's USB cable plugged in, `sensor.py --live-match` (and the other
  `sensor.py` modes) work as with TEENSY_FIRMWARE: the Teensy sends its raw frames over USB while the port is open, the
  Bluetooth side keeps running

To check the link and the detection alone (the BLE counterpart of `sensor.py`: covered holes and results/s):
```bash
    pixi run -e wg python experiments/waveguide/touch.py --sensor-calibration experiments/waveguide/calibration/demo_calibration2.npz
```

`wg_teleop.py` still reads the raw frames over USB serial (TEENSY_FIRMWARE) and detects the holes on the computer.

### Demo 2 - Gripper (`wg_gripper_demo.py`)
The waveguide is on the Franka Hand fingers: grasp an egg at `EGG_POSITION` (hard-coded), close until a hole is
covered, then squeeze by a set distance or until the amplitude rose by a set amount. The gripper closes with a
non-blocking action, cancelled (the gripper stops) as soon as the waveguide shows the stop condition.

```bash
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py                                    # squeeze 1 mm after contact
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --squeeze-mm 2
    pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --mode amplitude --amplitude-rise 50
```

### Demo 3 - Robot body (`wg_body_demo.py`)
The waveguide is on link 5: holding holes accelerates the arm along their direction (`BodyDemoConfig.direction_holes`),
touching against the motion stops it, and it stays in an absolute bounding box. Streams to `osc_pd_controller`.
Holes between two calibrated ones get an interpolated direction (`interpolate_holes`); with `fixed_mapping=False`
the directions follow the rotation of link 5 since the calibration (see the `BodyDemoConfig` options).

To run on the robot:
```bash
    python wg_body_demo.py --live-plot --method leading_edge --sensor-calibration calibration/calibration_large_w_endcap.npz 
```

Other examples:

```bash
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --calibrate        # first: touch the holes of +x, -x, ... in turn (arm at home)
    pixi run -e wg python experiments/waveguide/wg_body_demo.py                    # then: uses calibration/body_direction_holes.json
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --input terminal   # covered holes typed in the terminal ("+x", "3 4", "t -y")
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --live-plot --sensor-calibration experiments/waveguide/calibration/<file>.npz
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --live-plot --no-robot   # sensor + plot on the bench, the arm simulated
```

`--live-plot` (`body_live_plot.py`, from the Teensy results): the touches against time, the waveguide with each hole
in the color of its direction (interpolated holes lighter, blended; the names over the calibrated runs; covered holes
outlined), and the arm in its bounding box (top and side views, velocity). The ⚙ tab (or C) shows the mapping and
calibrates it: Calibrate, then for each direction touch its holes and press Space (Backspace redoes it, Esc cancels).
The arm stays stopped meanwhile; the mapping is saved (previous one kept as a backup) and used right away. The holes
drawn are those of `--sensor-calibration`: give the one of this waveguide.

Tests of the demos logic: `pixi run -e wg pytest experiments/waveguide/tests`
