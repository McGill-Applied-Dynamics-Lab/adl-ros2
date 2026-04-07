# Teensy 4.1 4x MB1340 Firmware

## Overview

This project captures raw analog waveforms from 4 MaxBotix MB1340 ultrasonic rangefinders on a Teensy 4.1 and streams them to a host PC over USB serial.

There are currently two supported sketches:

- `teensy4_1_MB1340_4x/teensy4_1_MB1340_4x.ino`: continuous streaming. Send `67` to start, send `69` to stop after the current 4-sensor frame.
- `10RC_finish/10RC_finish.ino`: fixed-burst experiment firmware with a test mode. Send `67` to start one 10-frame RF burst. After the burst, the Teensy waits briefly, then repeatedly sends `69` until the next `67` is received. Send ASCII `TEST` to stream frames continuously for debugging, and send ASCII `TESTEND` to stop test streaming after the current frame.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Teensy 4.1 |
| Sensors | 4x MaxBotix MB1340 |
| Trigger pins | D2, D3, D4, D5 |
| Analog input pins | A0, A1, A2, A3 |
| ADC resolution | 12-bit |
| ADC averaging | 2 samples |
| Measured sample rate | ~178,804 Hz |
| Baud rate | 3,000,000 |

### Sensor Pin Mapping

The firmware defines the pins per sensor as:

| Sensor | Trigger pin | Analog input pin |
|--------|-------------|------------------|
| MB0 / S0 | D4 | A2 |
| MB1 / S1 | D5 | A3 |
| MB2 / S2 | D3 | A1 |
| MB3 / S3 | D2 | A0 |

This mapping is shared by the current 4RF firmware sketches.

## Buffer Sizing

The buffer size is derived from the target ranging distance:

```cpp
BUFFER_SIZE = (SAMPLE_RATE / C) * 2 * RANGING_DISTANCE * 1.2
```

With:

- `SAMPLE_RATE = 178804.2`
- `C = 343.0 m/s`
- `RANGING_DISTANCE = 0.6 m`

This evaluates to `750` samples per sensor.

## Serial Protocols

### Fixed-Burst Protocol: `10RC_finish.ino`

This is the protocol used by the fixed-burst experiment acquisition path in `src/acoustic_sensing/scripts/experiment_configuration/common.py`.

| Byte | Hex | ASCII | Direction | Action |
|------|-----|-------|-----------|--------|
| 67 | `0x43` | `'C'` | Host to Teensy | Start one 10-frame RF burst |
| 69 | `0x45` | `'E'` | Teensy to host | Burst finished; repeated while waiting for the next `67` |
| `TEST` | ASCII | | Host to Teensy | Start continuous test streaming |
| `TESTEND` | ASCII | | Host to Teensy | Stop continuous test streaming after the current frame |

After the 10th frame, the Teensy waits `POST_BURST_FINISH_DELAY_MS` to give the host time to drain received data, sends one `69`, then keeps sending `69` every `FINISH_ANNOUNCE_INTERVAL_MS` until another `67` arrives.

Host readers should treat any raw `0x45` byte received after RF frame data has started as the end-of-burst marker. Do not depend on `69` being newline-terminated.

In test mode, the Teensy continuously captures and streams full 4RF frames until `TESTEND` is received. The firmware checks for `TESTEND` between frames, not during an active sensor capture.

### Continuous-Streaming Protocol: `teensy4_1_MB1340_4x.ino`

The continuous-streaming sketch uses:

| Byte | Hex | ASCII | Action |
|------|-----|-------|--------|
| 67 | `0x43` | `'C'` | Start continuous streaming |
| 69 | `0x45` | `'E'` | Stop streaming after the current frame |

### Stream Format

Each streamed frame contains 4 channel dumps:

```text
S0
<750 samples, one per line>
T
S1
<750 samples>
T
S2
<750 samples>
T
S3
<750 samples>
T
```

When a stop command is received by the continuous-streaming sketch, the Teensy finishes the current frame and then prints:

```text
STREAM_END
```

## Behavior Notes

- Sensors are triggered sequentially to avoid acoustic cross-talk.
- Both sketches are idle until they receive `67`.
- In `10RC_finish.ino`, `69` is a Teensy-to-host finish marker, not a host-to-Teensy stop command.
- In `teensy4_1_MB1340_4x.ino`, `69` is a host-to-Teensy stop command.
- The firmware streams immediately after capture; it does not keep a separate "capture then dump later" state.

## Repository Layout

```text
teensy_firmware/
├── README.md
├── debug_teensy_serial.py
├── live_plot_teensy_qt.py
├── receive_fast.py
├── 10RC_finish/
│   └── 10RC_finish.ino
├── teensy4_1_MB1340_4x/
│   └── teensy4_1_MB1340_4x.ino
```

## Usage

1. Flash the sketch for the workflow you are running:
   - Fixed-burst experiment acquisition: [10RC_finish.ino](/home/sni22/Documents/sim2real_adlros/src/acoustic_sensing/teensy_firmware/10RC_finish/10RC_finish.ino)
   - Continuous streaming: [teensy4_1_MB1340_4x.ino](/home/sni22/Documents/sim2real_adlros/src/acoustic_sensing/teensy_firmware/teensy4_1_MB1340_4x/teensy4_1_MB1340_4x.ino)
2. Connect the Teensy over USB and confirm it prints `Ready` on boot.
3. Activate the Conda environment:

```bash
conda activate teensy_firmware
```

4. To manually test the fixed-burst protocol:

```bash
python3 src/acoustic_sensing/teensy_firmware/debug_teensy_serial.py --burst-test
```

Expected behavior:

```text
TX dec=67 hex=0x43 ascii='C'
RX frame data started
RX received end byte 69 / 0x45 / 'E'
```

5. To manually control the continuous-streaming protocol from Python:

```python
ser.write(b"C")  # start streaming
ser.write(b"E")  # stop after the current frame
```

6. To live-plot the fixed-burst firmware's test mode:

```bash
python3 src/acoustic_sensing/teensy_firmware/live_plot_teensy_qt.py
```

The plotter sends `TEST` when it opens and sends `TESTEND` when the window closes. It requires `pyqtgraph` and a Qt binding such as PyQt5, PyQt6, or PySide6.

To stop test mode:

- Close the Qt plotter window; it automatically sends `TESTEND`.
- Or send ASCII `TESTEND` manually from a serial client:

```python
ser.write(b"TESTEND")
ser.flush()
```

## Notes On Host Scripts

- [debug_teensy_serial.py](/home/sni22/Documents/sim2real_adlros/src/acoustic_sensing/teensy_firmware/debug_teensy_serial.py) can print raw Teensy bytes or run a fixed-burst handshake test with `--burst-test`.
- [live_plot_teensy_qt.py](/home/sni22/Documents/sim2real_adlros/src/acoustic_sensing/teensy_firmware/live_plot_teensy_qt.py) starts `TEST` mode and plots the 4 RF channels in real time.
- [receive_fast.py](/home/sni22/Documents/sim2real_adlros/src/acoustic_sensing/teensy_firmware/receive_fast.py) currently implements the earlier capture/dump workflow and does not match the protocols described above.
