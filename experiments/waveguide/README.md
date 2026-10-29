# Waveguide experiments

Experiments with the **waveguide with holes**: an acoustic waveguide (a tube with holes) with an ultrasonic sensor
(MaxSonar) at one end, read by a Teensy. Covering a hole changes the echo, and the hole is found from the echo time.
The sensor drives the Franka FR3 in three ways:

| Script | What it does | Teensy firmware / link |
|---|---|---|
| `wg_body_demo.py` | **Collision demo**: waveguide on link 5. Holding holes pushes the arm, touching against the motion stops it | TEENSY_BLUETOOTH, BLE |
| `wg_gripper_demo.py` | **Gripper demo**: waveguide on the gripper fingers. Grasps an egg, stops closing at the contact | TEENSY_BLUETOOTH, BLE |
| `wg_teleop.py` | **Teleoperation**: swipes along the waveguide move the arm along x / y / z | raw frames over USB |
| `sensor.py` | Sensor tools: live plot, sensor calibration, `--live-match` to place the sensor | raw frames over USB |
| `touch.py` | Prints what the Teensy detects over BLE (link check) | TEENSY_BLUETOOTH, BLE |
| `plot_run.py` | Plots a run recorded with `--record` | none (offline) |

All the commands below are run **from the repository root** (`adl-ros2/`).

---

## 1. Hardware and firmware

- **Teensy 4.x** wired to the MaxSonar (trigger: pin 2, PW sync: pin 3, raw analog: A0) and, for Bluetooth, to an
  **HC-08 BLE module** (HC-08 TXD -> pin 0, RXD -> pin 1, 3.3 V, GND).
- The firmware lives in the [WAVEGUIDE_HOLES_PROJECT](https://github.com/macrobotics-lab/WAVEGUIDE_HOLES_PROJECT)
  repository, in `firmware/`. Flash it with the Arduino IDE and Teensyduino.
  - **`TEENSY_BLUETOOTH`** (use this one): runs the hole detection on the Teensy and sends a result about 20 times a
    second over BLE (HC-08, device name `WAVEGUIDE`). When a computer opens its **USB** port, it also sends every
    raw frame there, in the same format as TEENSY_FIRMWARE, so `sensor.py` and `wg_teleop.py` work with it too.
  - `TEENSY_FIRMWARE`: raw frames over USB only (the original firmware).
- **BLE vs USB:** BLE is slow (about 2 kB/s). It carries the detection results, not the raw frames (one frame takes
  ~1.4 s). Anything that needs the raw echo (live plot of the echo, `--live-match`, sensor calibration, recording
  the raw frames) needs the **USB cable**.
- Only one program can open the Teensy's USB port at a time. Close `sensor.py` before starting a demo with
  `--record`, which opens the port to record the frames.

## 2. Setup (once)

```bash
pixi install -e wg                # the wg environment: ROS 2 Humble + the waveguide dependencies
pixi run setup-colcon             # once per machine
pixi run -e wg build              # builds arm_client and the other ROS packages (needed for the robot demos)
```

Then either open a shell in the environment (`pixi shell -e wg`) and run `python ...`, or prefix each command with
`pixi run -e wg`, as below. After a build, open a new shell so the newly built packages are found.

The robot itself (franka-server on the mini-PC, robot unlocked, FCI on) is started as in
[docs/getting-started.md](../../docs/getting-started.md) (sections 1 and 2).

Check that everything imports, without hardware:
```bash
pixi run -e wg pytest experiments/waveguide/tests
pixi run -e wg python experiments/waveguide/wg_body_demo.py --live-plot --no-robot --input terminal   # type "1", Enter, "q"
```

## 3. Before each session

1. **Power up the Teensy.** The MaxSonar calibrates itself on power-up: keep the sensor **disconnected from the
   waveguide** while it powers up, then put it back.
2. **Put the sensor back in the exact position it had during the sensor calibration.** This matters: the
   calibration only holds for that position. Plug in the Teensy's USB cable and match the live echo to the saved
   reference:
   ```bash
   pixi run -e wg python experiments/waveguide/sensor.py --live-match                     # against the newest calibration/reference_*.csv
   pixi run -e wg python experiments/waveguide/sensor.py --live-match experiments/waveguide/calibration/reference_10_22.csv
   ```
   Move or twist the sensor until the blue curve (live, averaged) lies on the orange one (reference): the bottom
   plot (difference) is flat, and the RMS difference in the title is small and its correlation close to 1. Drag
   the shaded region to compare only part of the trace. Press **S** to save the current trace as a new reference
   (it becomes the default for the next `--live-match`). Close the window when done.
3. **Connect over Bluetooth.** Do **not** pair the HC-08 in the system Bluetooth settings: it does not support
   pairing (`AuthenticationFailed`), and the scripts connect to it directly. Its LED blinks when it is free (no
   connection). Check the link and the detection:
   ```bash
   pixi run -e wg python experiments/waveguide/touch.py --sensor-calibration experiments/waveguide/calibration/calibration_large_w_endcap.npz
   ```
   After a power cycle it first uploads the sensor calibration (about a minute), then records the background
   (**don't touch the waveguide**), then prints the covered holes and about 20 results/s. Touch a few holes, then
   Ctrl+C.
4. **Start the robot** (franka-server running, see section 2) for the robot demos.

## 4. Collision demo: robot body (`wg_body_demo.py`)

The waveguide is wrapped around **link 5**. Each hole has a direction (+x, -x, +y, -y, +z, -z in the robot base
frame):
- **holding** holes accelerates the arm along their direction; on release it keeps its velocity;
- **touching against the motion** (a hole more than 120° from the velocity) stops the arm at once, and it stays
  stopped until every hole is released;
- the arm stays in a **bounding box**: reaching it stops the arm.

The end-effector keeps its orientation. The arm is driven by `osc_pd_controller`, and goes home at the start.

### Run it

```bash
pixi run -e wg python experiments/waveguide/wg_body_demo.py --live-plot --method leading_edge \
    --sensor-calibration experiments/waveguide/calibration/calibration_large_w_endcap.npz
```

Add `--record --notes "collision, take 1"` to record the run (section 7). Close the window or press Ctrl+C to stop:
the arm holds its last position.

### The live plot

- **Touches**: the covered holes over the last 10 s, in the color of their direction. Stops are marked at the top.
- **Waveguide**: every hole, colored by its direction. Calibrated holes are drawn in full color with the direction
  name above them; holes in between (interpolated) are lighter; unmapped holes are grey. Covered holes are
  outlined. The large status line shows what the arm does (Moving, STOP, Bounding box reached, ...).
- **Top / side views**: the end-effector in the bounding box, its trail and its velocity.
- The holes drawn come from `--sensor-calibration`: give the one of this waveguide.

### Calibrating the hole -> direction mapping

Do it once per mounting of the waveguide on the arm, **with the arm at home** (where it starts):
1. In the live plot, open the **⚙ tab** on the right (or press **C**). It lists the holes of each direction.
2. Press **Calibrate**. The arm stays stopped during the calibration.
3. For each direction in turn (+x, -x, +y, -y, +z, -z, shown in the status line): touch **all the holes whose touch
   should move the arm that way** (one by one, or slide a finger over them), then press **Space**. **Backspace**
   redoes the current direction, **Esc** cancels.
4. After -z, the mapping is saved to `calibration/body_direction_holes.json` (the previous one is kept as
   `body_direction_holes_backup_<date>.json`) and used right away.

Holes between two calibrated holes get an interpolated direction (`interpolate_holes`), except between opposite
directions. Without the live plot, `--calibrate` does the same calibration in the terminal.

### Other options

```bash
pixi run -e wg python experiments/waveguide/wg_body_demo.py --live-plot --no-robot ...   # no arm (no ROS needed): the motion is simulated, to try the sensor and the plot
pixi run -e wg python experiments/waveguide/wg_body_demo.py --input terminal            # no waveguide: type the covered holes ("3 4", "+x", "t -y" = tap, empty = release, q = quit)
pixi run -e wg python experiments/waveguide/wg_body_demo.py --calibrate                 # mapping calibration in the terminal
```

**To check before running on the arm** (`BodyDemoConfig` at the top of `wg_body_demo.py`): `box_lower` / `box_upper`
(the bounding box, in metres in the base frame; it must contain the home position), `acceleration`, `max_speed` and
`stop_angle_deg`.

## 5. Gripper demo (`wg_gripper_demo.py`)

The waveguide is on the **Franka Hand fingers**. The demo:
1. moves above the egg at `EGG_POSITION`, then down to it;
2. closes fast to `fast_close_width` (the egg is not touched yet);
3. closes slowly until a hole is covered (the contact);
4. squeezes: by `--squeeze-mm` after the contact (`--mode squeeze`, default), or until the amplitude rose by
   `--amplitude-rise` (`--mode amplitude`), never more than `max_squeeze_mm`;
5. lifts the egg, holds it, puts it down, releases it and goes home.

The gripper stops within about 100 ms of the contact.

```bash
pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --sensor-calibration experiments/waveguide/calibration/<gripper waveguide>.npz
pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --squeeze-mm 2 ...
pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --mode amplitude --amplitude-rise 50 ...
```

**To check before running**: `EGG_POSITION` at the top of `wg_gripper_demo.py` (still a placeholder: measure the
TCP position for grasping the egg), and the widths and speeds in `GripperDemoConfig` (`fast_close_width` must be
larger than the egg).

## 6. Teleoperation and sensor tools (USB)

These read the raw frames over USB and do the detection on the computer. They use the sensor calibration in
`CalibrationConfig` (top of `sensor.py`, currently `calibration/demo_calibration2.npz`).

```bash
pixi run -e wg python experiments/waveguide/wg_teleop.py --live-plot          # swipes move the arm (x / y / z segments, see GestureConfig)
pixi run -e wg python experiments/waveguide/wg_teleop.py --input terminal     # commands typed in the terminal, no waveguide

pixi run -e wg python experiments/waveguide/sensor.py                         # frame rate, pressed holes, commands
pixi run -e wg python experiments/waveguide/sensor.py --live-plot             # echo, envelope and waveguide, live
pixi run -e wg python experiments/waveguide/sensor.py --live-match            # place the sensor (section 3)
```

`--port` picks the Teensy if several are plugged in (default: the one sending frames).

### Making a new sensor calibration

A sensor calibration (`.npz`) holds, for one waveguide and one sensor position, the hole positions, the detection
threshold and the templates. The demos need one (`--sensor-calibration`), and the Teensy gets it uploaded.
1. In `CalibrationConfig` (top of `sensor.py`), set `filename` (the new recording, e.g. `my_waveguide.db`),
   `n_holes`, `first_hole_cm` (distance from the sensor to hole 1) and `hole_spacing_cm`.
2. With the USB cable plugged in, run `sensor.py --live-plot`, open the **⚙ tab** (or C), choose the number of holes
   (evenly spaced) and of passes, and press Start. The hole to press is shown in green: hold it until the next one
   is shown, then release. Space records the current step now (e.g. a weak hole), Esc cancels. Or run
   `sensor.py --calibrate` for the guided calibration in the terminal.
3. The fit is saved next to the recording (`my_waveguide.npz`; the previous files are kept as `*_backup_<date>`).
4. Save a reference trace for `--live-match` right after (press S in `--live-match`), so the sensor can be put back
   in the same position later.
5. For the demos, run them once with `--upload` so the Teensy replaces the calibration it already has.

## 7. Recording and plotting

Add `--record` (and optionally `--notes "..."`) to any of the scripts:

```bash
pixi run -e wg python experiments/waveguide/wg_body_demo.py --live-plot --record --notes "collision, take 1" ...
pixi run -e wg python experiments/waveguide/wg_gripper_demo.py --record --notes "egg 1, squeeze 1 mm" ...
pixi run -e wg python experiments/waveguide/wg_teleop.py --live-plot --record --notes "teleop, take 3"
pixi run -e wg python experiments/waveguide/sensor.py --live-plot --record
```

A run is saved to `~/data/waveguide/<body|gripper|teleop|sensor>/<date-time>/`, the path being printed at the end:
- `samples.mcap`: the data; open it in Foxglove, or read it with `plot_run.load_run` /
  `experiment_logger.load_streams`;
- `metadata.json`: the notes, the command-line arguments, the configs and the git commit;
- copies of the calibrations used (`sensor_calibration.npz`, `mapping_<n>.json` for the body demo, ...).

What the demos record:
- `sensor/result`: every Teensy result (hole, second hole, distance, amplitude, ...);
- `sensor/calibration`: the sensor calibration and the Teensy settings;
- `sensor/frame`: the raw frames, **only if the Teensy's USB cable is plugged in** (BLE cannot carry them). The
  frames recorded before the first result are the Teensy's background;
- `robot/state` (50 Hz): the end-effector pose. The body demo also records the commanded position, velocity and
  state; the gripper demo the gripper width;
- `demo/event`: what the demo did and when. Body demo: moving, stop, bounding box, new mapping. Gripper demo:
  contact (width, amplitude), squeeze, lift, release, ...

`wg_teleop.py` and `sensor.py` record the averaged frames, the detections, the backgrounds, the swipes, and the
robot commands and state. All the fields are described at the top of `recording.py`.

**Plotting a run**: `plot_run.py` is standalone (it only needs `numpy`, `matplotlib` and `mcap`), so it can be
copied to any computer:

```bash
python experiments/waveguide/plot_run.py                                          # the newest run
python experiments/waveguide/plot_run.py ~/data/waveguide/body/20261029-162640 --start 10 --end 40 --save run.png
```

It plots, against time, the echo (if the raw frames were recorded), the detected holes, the amplitude against the
detection threshold, the end-effector position, the gripper width (gripper demo) and the demo events (dotted
lines). It also prints a summary of the run. To use the data in Python:

```python
from plot_run import load_run, stack
run = load_run("~/data/waveguide/body/20261029-162640")    # {stream: [sample dict, ...]}
holes = stack(run["sensor/result"], "hole")                 # numpy array
t = stack(run["sensor/result"], "t_mono")                   # time [s], shared by all the streams
```

## 8. Where to change what

| What | Where |
|---|---|
| Teensy link (BLE name, serial port), detection method, sensor calibration | command line: `--link`, `--ble-name`, `--port`, `--method`, `--sensor-calibration`, `--upload` (defaults: `TeensyConfig` in `touch.py`) |
| Body demo: bounding box, speed, acceleration, stop angle, mapping file | `BodyDemoConfig` in `wg_body_demo.py` |
| Body demo: hole -> direction mapping | ⚙ tab of the live plot, saved in `calibration/body_direction_holes.json` |
| Gripper demo: egg position, widths, speeds, maximum squeeze | `EGG_POSITION` and `GripperDemoConfig` in `wg_gripper_demo.py` |
| Teleop: speeds, workspace, gestures | top of `wg_teleop.py`, `GestureConfig` in `sensor.py` |
| Sensor calibration used by `sensor.py` / `wg_teleop.py` | `CalibrationConfig` in `sensor.py` |
| Live plot look (colors, fonts, window size) | `PlotStyle` in `sensor.py` |
| Where the runs are saved | `DATA_DIR` in `recording.py` |

## 9. Troubleshooting

| Problem | Fix |
|---|---|
| `Failed to pair: org.bluez.Error.AuthenticationFailed` | Don't pair the HC-08. Remove it (`bluetoothctl`, then `remove <address>`) and let the script connect. |
| `No BLE device named 'WAVEGUIDE' found` | The HC-08 is still connected to something (its LED is on, not blinking): close the other script or `bluetoothctl disconnect`. Check that the Teensy is powered. |
| Connects, but no reply / timeouts | The HC-08 baud rate does not match the firmware (`HC08_BAUD`, 115200; a new HC-08 runs at 9600): see `CONFIGURE_HC08` in TEENSY_BLUETOOTH.ino. In the Arduino serial monitor (USB), type `d` for the debug counters (`f` brings back the frames). |
| Holes detected wrong or not at all | The sensor moved: redo the `--live-match` (section 3). The Teensy may also hold another waveguide's calibration: run once with `--upload`. Don't touch the waveguide while the background is recorded (at every start). |
| The start takes about a minute | Normal after a power cycle: the sensor calibration is uploaded over BLE. |
| `No touch data: stopping` | The BLE results stopped (link lost, Teensy reset). Restart the script. |
| `Start position ... is outside the bounding box` | Adjust `box_lower` / `box_upper` in `BodyDemoConfig` so they contain the home position. |
| The live plot shows the wrong number of holes | Give the right `--sensor-calibration`: the holes drawn are those of that file. |
| `No Teensy sending frames` (`sensor.py`) | Plug in the USB cable; another program may have the port open (another `sensor.py`, a demo with `--record`, the Arduino serial monitor). |
| `ModuleNotFoundError: arm_client` | Build (`pixi run -e wg build`) and open a new shell. |

## 10. Files

| File | Role |
|---|---|
| `wg_body_demo.py` | Collision demo (link 5): jog logic, mapping calibration, `--live-plot`, `--record` |
| `body_live_plot.py` | Live plot of the collision demo, with the mapping calibration (⚙ tab) |
| `wg_gripper_demo.py` | Gripper demo (egg grasp) |
| `wg_teleop.py` | Teleoperation with swipes (raw frames over USB) |
| `touch.py` | Touch sources of the demos: the Teensy over BLE / serial, or the terminal; `--record` helpers |
| `sensor.py` | Raw frame reading, detection on the computer, gestures, live plot, `--live-match`, `--calibrate` |
| `live_calibration.py` | Sensor calibration from the live plot (⚙ tab of `sensor.py --live-plot`) |
| `wg_helpers.py` | Signal processing, calibration fit, and the Teensy Bluetooth protocol (`TeensyController`, `BLEReceiver`) |
| `recording.py` | `--record`: run folders and the recorded streams |
| `plot_run.py` | Standalone reader and plots of a recorded run |
| `commands.py` | Teleop command type |
| `calibration/` | Sensor calibrations (`.db` recordings, `.npz` fits), `--live-match` references (`reference_*.csv`), the body demo mapping (`body_direction_holes.json`) |
| `tests/` | Tests of the demo logic, without hardware: `pixi run -e wg pytest experiments/waveguide/tests` |
| `LIVE_PLOT.py`, `LIVE_PLOT_MATCH.py` | Older standalone plot scripts (superseded by `sensor.py --live-plot` / `--live-match`) |
