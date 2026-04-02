import matplotlib.pyplot as plt
import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
SERIAL_TIMEOUT_SEC = 10

CMD_START = bytes([0x43])  # 'C' - trigger ranging capture
CMD_DUMP = bytes([0x45])   # 'E' - dump buffered data

EXPECTED_SAMPLES = 750
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")


def read_line_or_raise(ser: serial.Serial) -> str:
    raw = ser.readline()
    if raw == b"":
        raise TimeoutError("Timed out waiting for serial data from Teensy")
    return raw.decode("ascii", errors="ignore").strip()


def wait_for_marker(ser: serial.Serial, expected: str) -> None:
    while True:
        line = read_line_or_raise(ser)
        if line == expected:
            return


def read_channel(ser: serial.Serial, channel_marker: str) -> list[int]:
    wait_for_marker(ser, channel_marker)

    samples = []
    while True:
        line = read_line_or_raise(ser)
        if line == "T":
            break
        try:
            samples.append(int(line))
        except ValueError as exc:
            raise ValueError(
                f"Received non-integer sample {line!r} while reading {channel_marker}"
            ) from exc

    if len(samples) != EXPECTED_SAMPLES:
        raise ValueError(
            f"{channel_marker} expected {EXPECTED_SAMPLES} samples, got {len(samples)}"
        )

    return samples


def capture_once(ser: serial.Serial) -> list[list[int]]:
    ser.reset_input_buffer()
    ser.write(CMD_START)
    wait_for_marker(ser, "Data Read")

    ser.write(CMD_DUMP)
    return [read_channel(ser, marker) for marker in CHANNEL_MARKERS]


def plot_capture(channels: list[list[int]]) -> None:
    fig, ax = plt.subplots(nrows=1, ncols=4, figsize=(16, 5))

    for idx, channel in enumerate(channels):
        ax[idx].plot(channel)
        ax[idx].set_title(f"Rangefinder {idx}")

    plt.suptitle("Captured Signal")
    plt.show(block=False)


def main() -> None:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC)

    try:
        while True:
            channels = capture_once(ser)
            plot_capture(channels)
            input()
            plt.close()
    finally:
        ser.close()


if __name__ == "__main__":
    main()
