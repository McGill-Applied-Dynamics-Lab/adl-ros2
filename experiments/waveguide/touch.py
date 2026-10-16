"""Touch input of the waveguide demos with on-board processing (gripper, robot body).

For these demos the Teensy processes the echo frames itself (hole detection, multiple holes) and sends,
for each frame, the covered holes with their touch distance and signal amplitude. A `TouchSource` turns
that stream into `TouchFrame`s, and keeps the latest one for the demo loop.

Sources:
- `TeensyTouchSource`: the Teensy over serial. The packet format is not defined yet: see `decode_packet`
- `TerminalTouchSource`: covered holes typed in the terminal, to test without the waveguide

Nothing here imports ROS.
"""

import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path


@dataclass(frozen=True)
class HoleTouch:
    """A covered hole: its index, the touch distance along the waveguide [cm] and the signal amplitude."""

    idx: int
    distance_cm: float
    amplitude: float


@dataclass(frozen=True)
class TouchFrame:
    """The holes covered at time `t` (CLOCK_MONOTONIC) [s]; empty if nothing is touched."""

    holes: tuple[HoleTouch, ...] = ()
    t: float = field(default_factory=time.monotonic)

    @property
    def covered(self) -> frozenset[int]:
        return frozenset(h.idx for h in self.holes)

    @property
    def amplitude(self) -> float:
        """Largest amplitude of the covered holes, 0 if none."""
        return max((h.amplitude for h in self.holes), default=0.0)


class TouchSource:
    """Interface of a touch source: `latest()` is the last frame received, without waiting."""

    def start(self) -> None:
        pass

    def latest(self) -> TouchFrame | None:
        """The last frame received, or None if none yet. Raises RuntimeError if the source stopped."""
        raise NotImplementedError

    def close(self) -> None:
        pass


class _ThreadedSource(TouchSource):
    """Runs `_run` in a background thread; an exception there ends the source, `latest()` re-raises it."""

    def __init__(self):
        self._lock = threading.Lock()
        self._frame: TouchFrame | None = None
        self._error: BaseException | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run_guarded, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def latest(self) -> TouchFrame | None:
        if self._error is not None:
            raise RuntimeError(f"Touch source stopped: {self._error!r}") from self._error
        with self._lock:
            return self._frame

    def close(self) -> None:
        self._stop.set()

    def _publish(self, frame: TouchFrame) -> None:
        with self._lock:
            self._frame = frame

    def _run_guarded(self) -> None:
        try:
            self._run()
        except BaseException as e:  # noqa: BLE001, reported by latest()
            self._error = e

    def _run(self) -> None:
        raise NotImplementedError


# --- Teensy ---
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 3_000_000
SERIAL_TIMEOUT = 0.1  # s, so the reader checks the stop event even without data


def decode_packet(ser) -> TouchFrame | None:
    """Read the next packet from the serial port `ser`, or None if none arrived within its timeout.

    TODO: the packet format of the on-board processing is not defined yet. Expected content, per frame:
    the covered holes, each with its index, distance [cm] and amplitude (an empty frame when nothing is
    touched also tells the stream is alive).
    """
    raise NotImplementedError("The Teensy packet format is not defined yet (touch.decode_packet)")


class TeensyTouchSource(_ThreadedSource):
    """Touch frames processed on board the Teensy, read over serial."""

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE):
        super().__init__()
        if not Path(port).exists():
            raise FileNotFoundError(f"Serial port {port} not found: is the Teensy plugged in? (--port to use another)")
        self._port, self._baudrate = port, baudrate

    def _run(self) -> None:
        import serial  # Not at the top: only needed with the Teensy

        with serial.Serial(self._port, self._baudrate, timeout=SERIAL_TIMEOUT) as ser:
            ser.reset_input_buffer()
            while not self._stop.is_set():
                frame = decode_packet(ser)
                if frame is not None:
                    self._publish(frame)


# --- Terminal ---
class TerminalTouchSource(_ThreadedSource):
    """Covered holes typed in the terminal, held until the next line.

    `aliases` maps names (e.g. "+x") to holes, so a line can name them instead of their index.
    """

    TAP_S = 0.2  # How long `t ...` covers the holes
    TOUCH_AMPLITUDE = 1.0
    TOUCH_DISTANCE_CM = 0.0

    def __init__(self, aliases: dict[str, tuple[int, ...]] | None = None):
        super().__init__()
        self._aliases = aliases or {}
        self.quit_requested = threading.Event()
        self._publish(TouchFrame())

    def help(self) -> str:
        names = f" or a name in {list(self._aliases)}" if self._aliases else ""
        return f"""Terminal touch input (each line replaces the covered holes):
    <hole> [<hole> ...]     cover these holes until the next line; a hole is an index{names}
    t <hole> [<hole> ...]   tap: cover them for {self.TAP_S} s
    (empty line)            release all
    q                       quit"""

    def parse(self, line: str) -> tuple[frozenset[int], bool] | None:
        """The holes of `line` and whether it is a tap; None for `q`. ValueError if a hole is unknown."""
        parts = line.strip().lower().split()
        if parts in (["q"], ["quit"], ["exit"]):
            return None
        tap = bool(parts) and parts[0] == "t"
        holes = set()
        for part in parts[1:] if tap else parts:
            if part in self._aliases:
                holes.update(self._aliases[part])
            else:
                try:
                    holes.add(int(part))
                except ValueError:
                    raise ValueError(f"Unknown hole '{part}'") from None
        return frozenset(holes), tap

    def latest(self) -> TouchFrame | None:
        # The typed holes stay covered until the next line: they are covered now
        frame = super().latest()
        return None if frame is None else replace(frame, t=time.monotonic())

    def _frame(self, holes: frozenset[int]) -> TouchFrame:
        return TouchFrame(
            tuple(HoleTouch(i, self.TOUCH_DISTANCE_CM, self.TOUCH_AMPLITUDE) for i in sorted(holes)),
        )

    def _run(self) -> None:
        print(self.help())
        while not self._stop.is_set():
            try:
                line = input("touch> ")
            except EOFError:
                line = "q"
            try:
                parsed = self.parse(line)
            except ValueError as e:
                print(e)
                continue
            if parsed is None:
                self._publish(TouchFrame())
                self.quit_requested.set()
                return
            holes, tap = parsed
            self._publish(self._frame(holes))
            if tap:
                time.sleep(self.TAP_S)
                self._publish(TouchFrame())
