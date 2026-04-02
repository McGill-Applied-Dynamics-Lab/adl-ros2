import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
TIMEOUT_SEC = 10
LINES_TO_PRINT = 20


def read_line_or_raise(ser: serial.Serial) -> str:
    raw = ser.readline()
    if raw == b"":
        raise TimeoutError("Timed out waiting for serial data from Teensy")
    return raw.decode("ascii", errors="ignore").strip()


def main() -> None:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_SEC)

    try:
        ser.reset_input_buffer()
        ser.write(b"C")

        while True:
            line = read_line_or_raise(ser)
            print(line)
            if line == "Data Read":
                break

        ser.write(b"E")

        for _ in range(LINES_TO_PRINT):
            print(read_line_or_raise(ser))
    finally:
        ser.close()


if __name__ == "__main__":
    main()
