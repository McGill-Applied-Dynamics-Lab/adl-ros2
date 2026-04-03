import threading

import matplotlib.pyplot as plt
import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
SERIAL_TIMEOUT_SEC = 10
EXPECTED_SAMPLES = 750
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")

CMD_START = b"C"
CMD_STOP = b"E"
STREAM_END = "STREAM_END"


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
        if line == STREAM_END:
            raise RuntimeError("Streaming stopped before the expected channel marker arrived")


def read_channel(ser: serial.Serial, channel_marker: str) -> list[int]:
    wait_for_marker(ser, channel_marker)

    samples = []
    while True:
        line = read_line_or_raise(ser)
        if line == "T":
            break
        if line == STREAM_END:
            raise RuntimeError("Streaming stopped before the current channel finished")
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


def read_frame(ser: serial.Serial) -> list[list[int]] | None:
    channels = []
    for idx, marker in enumerate(CHANNEL_MARKERS):
        try:
            channels.append(read_channel(ser, marker))
        except RuntimeError:
            if idx == 0:
                return None
            raise
    return channels


def plot_frame(channels: list[list[int]], frame_idx: int) -> tuple[plt.Figure, list]:
    fig, ax = plt.subplots(nrows=1, ncols=4, figsize=(16, 5))
    for idx, channel in enumerate(channels):
        ax[idx].plot(channel)
        ax[idx].set_title(f"Rangefinder {idx}")
    plt.suptitle(f"Captured Signal, Frame {frame_idx}")
    plt.show(block=False)
    return fig, ax


def wait_for_stop(stop_event: threading.Event) -> None:
    input("Press Enter to stop streaming after the current frame...\n")
    stop_event.set()


def main() -> None:
    stop_event = threading.Event()
    stop_thread = threading.Thread(target=wait_for_stop, args=(stop_event,), daemon=True)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC)
    latest_fig = None

    try:
        ser.reset_input_buffer()
        ser.write(CMD_START)
        stop_thread.start()

        frame_idx = 0
        while True:
            channels = read_frame(ser)
            if channels is None:
                break

            frame_idx += 1
            if latest_fig is not None:
                plt.close(latest_fig)
            latest_fig, _ = plot_frame(channels, frame_idx)

            if stop_event.is_set():
                ser.write(CMD_STOP)
                stop_event.clear()

        if latest_fig is not None:
            plt.show()
    finally:
        ser.close()


if __name__ == "__main__":
    main()
