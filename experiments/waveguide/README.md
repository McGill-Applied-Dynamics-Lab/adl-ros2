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