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
├── test_serial.py
├── teensy4_1_MB1340_4x/
│   └── teensy4_1_MB1340_4x.ino
├── teensy_original_4RF/
│   ├── teensy4_1_MB1340_4x.ino
│   └── receive_fast.py
└── teensy_67_69/
    ├── teensy_tx.ino
    └── twist_v2.py
```

## Usage

1. Flash [teensy4_1_MB1340_4x.ino](/home/sni22/Downloads/teensy_firmware/teensy4_1_MB1340_4x/teensy4_1_MB1340_4x.ino) to the Teensy 4.1.
2. Connect the Teensy over USB and confirm it prints `Ready` on boot.
3. Activate the Conda environment:

```bash
conda activate teensy_firmware
```

4. For a quick smoke test, run:

```bash
python test_serial.py
```

5. To manually control the protocol from Python:

```python
ser.write(b"C")  # start streaming
ser.write(b"E")  # stop after the current frame
```

## Notes On Host Scripts

- [test_serial.py](/home/sni22/Downloads/teensy_firmware/test_serial.py) is a minimal protocol smoke test.
- [receive_fast.py](/home/sni22/Downloads/teensy_firmware/receive_fast.py) currently implements the earlier capture/dump workflow and does not match the new continuous-streaming firmware protocol.
