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

Standalone (the port defaults to /dev/ttyACM0, see --port):
```
    pixi run -e wg python experiments/waveguide/sensor.py               # frame rate, pressed holes, commands
    pixi run -e wg python experiments/waveguide/sensor.py --live-plot   # raw frames
    pixi run -e wg python experiments/waveguide/sensor.py --calibrate   # guided calibration, see CalibrationConfig
```


## Teloperation
To teleop the franka with the sensor, use the `wg_teleop.py` file.

```bash
    pixi run -e wg python experiments/waveguide/wg_teleop.py                    # Waveguide with Teensy on /dev/ttyACM0 (--port)
    pixi run -e wg python experiments/waveguide/wg_teleop.py --live-plot        # Same, with the sensor live plot
    pixi run -e wg python experiments/waveguide/wg_teleop.py --input terminal   # Commands typed in the terminal
```

It's possible to test the teleoperation without the waveguide, inputing the commands via the terminal.

## Demos with on-board processing
For these demos the Teensy runs the hole detection itself (TEENSY_BLUETOOTH firmware, one or two holes with the
"multi" method) and sends a result every cycle (~20/s), read by `touch.py` through `wg_helpers` (Bluetooth section).
- Link: BLE to the HC-08 by default (`--link ble --ble-name WAVEGUIDE`), or a serial port (`--link serial --port ...`)
- Sensor calibration (`--sensor-calibration`, a `wg_helpers.calibrate()` .npz): uploaded when the Teensy has none,
  i.e. after a power cycle (about a minute over BLE); `--upload` replaces it
- The background is recorded at every start: don't touch the waveguide then

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

```bash
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --calibrate        # first: touch the holes of +x, -x, ... in turn (arm at home)
    pixi run -e wg python experiments/waveguide/wg_body_demo.py                    # then: uses calibration/body_direction_holes.json
    pixi run -e wg python experiments/waveguide/wg_body_demo.py --input terminal   # covered holes typed in the terminal ("+x", "3 4", "t -y")
```

Tests of the demos logic: `pixi run -e wg pytest experiments/waveguide/tests`
