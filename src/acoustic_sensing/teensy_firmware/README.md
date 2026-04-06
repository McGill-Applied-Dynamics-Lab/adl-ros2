# Teensy 4.1 4x MB1340 Streaming Firmware

## Overview

This project captures raw analog waveforms from 4 MaxBotix MB1340 ultrasonic rangefinders on a Teensy 4.1 and streams them to a host PC over USB serial.

The active firmware is command-gated:

- Send `67` / `0x43` / `'C'` to start continuous streaming.
- Send `69` / `0x45` / `'E'` to stop after the current 4-sensor frame completes.

While streaming, the Teensy behaves like the original free-running version: it captures all 4 sensors sequentially and transmits each frame immediately.

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

This mapping comes directly from the active firmware in `teensy4_1_MB1340_4x.ino`.

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

## Serial Protocol

### Commands

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

When a stop command is received, the Teensy finishes the current frame and then prints:

```text
STREAM_END
```

## Behavior Notes

- Sensors are triggered sequentially to avoid acoustic cross-talk.
- The device is idle until it receives `67`.
- Stop latency is one full frame because `69` is checked between frames, not during a frame.
- The firmware streams immediately after capture; it does not keep a separate “capture then dump later” state.

## Repository Layout

```text
teensy_firmware/
├── README.md
├── receive_fast.py
├── teensy4_1_MB1340_4x/
│   └── teensy4_1_MB1340_4x.ino
```

## Usage

1. Flash [teensy4_1_MB1340_4x.ino](/home/sni22/Documents/sim2real_adlros/src/acoustic_sensing/teensy_firmware/teensy4_1_MB1340_4x/teensy4_1_MB1340_4x.ino) to the Teensy 4.1.
2. Connect the Teensy over USB and confirm it prints `Ready` on boot.
3. Activate the Conda environment:

```bash
conda activate teensy_firmware
```

4. To manually control the protocol from Python:

```python
ser.write(b"C")  # start streaming
ser.write(b"E")  # stop after the current frame
```

## Notes On Host Scripts

- [receive_fast.py](/home/sni22/Documents/sim2real_adlros/src/acoustic_sensing/teensy_firmware/receive_fast.py) currently implements the earlier capture/dump workflow and does not match the continuous-streaming firmware protocol described above.
