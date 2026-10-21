"""Touch input of the waveguide demos with on-board processing (gripper, robot body).

For these demos the Teensy processes the echo frames itself (hole detection, multiple holes) and sends,
for each frame, the covered holes with their touch distance and signal amplitude. A `TouchSource` turns
that stream into `TouchFrame`s, and keeps the latest one for the demo loop.

Sources:
- `TeensyTouchSource`: the Teensy running the hole detection itself (TEENSY_BLUETOOTH firmware, `wg_helpers`
  Bluetooth section), over BLE (HC-08) or a serial port. One or two holes per frame ("multi" method)
- `TerminalTouchSource`: covered holes typed in the terminal, to test without the waveguide

Nothing here imports ROS.
"""

import argparse
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
@dataclass(frozen=True)
class TeensyConfig:
    """On-board detection settings (TEENSY_BLUETOOTH firmware)."""

    link: str = "ble"  # "ble" (HC-08 over Bluetooth) or "serial" (HC-08 bridged to a serial port)
    ble_name: str = "WAVEGUIDE"
    port: str = "/dev/ttyUSB0"  # serial link
    baudrate: int = 115200  # serial link
    # Calibration from wg_helpers.calibrate(), for this waveguide and set-up
    calibration_path: Path = Path(__file__).resolve().parent / "calibration" / "calibration_dummy.npz"
    force_upload: bool = False  # False: upload only if the Teensy has none (it keeps it until a power cycle)
    method: str = "multi"  # wg_helpers.METHODS; "multi" reports one or two holes
    average: int = 1  # Cycles averaged per result (1: a result every cycle, ~20/s)
    background_cycles: int = 10  # Recorded at start, nothing covered
    link_timeout_s: float = 20.0


def frame_from_result(result: dict, t: float | None = None) -> TouchFrame:
    """TouchFrame from a Teensy result packet (`TeensyController.results`). The second hole ("multi") has no
    distance of its own (nan) and the amplitude of the frame."""
    if not result["detected"]:
        return TouchFrame(t=time.monotonic() if t is None else t)
    holes = [HoleTouch(int(result["hole"]), float(result["distance_cm"]), float(result["amplitude"]))]
    if result.get("hole2") is not None:
        holes.append(HoleTouch(int(result["hole2"]), float("nan"), float(result["amplitude"])))
    return TouchFrame(tuple(holes), t=time.monotonic() if t is None else t)


class TeensyTouchSource(_ThreadedSource):
    """Hole detection on board the Teensy (TEENSY_BLUETOOTH firmware), over BLE or a serial port.

    `start()` (blocking, a minute if the calibration is uploaded over BLE) connects, sets the Teensy up and
    records the background: nothing must touch the waveguide then.
    """

    RESULT_TIMEOUT_S = 0.1  # The reader checks the stop event at least this often

    def __init__(self, cfg: TeensyConfig = TeensyConfig()):
        super().__init__()
        if cfg.link not in ("ble", "serial"):
            raise ValueError(f"Unknown link '{cfg.link}', expected 'ble' or 'serial'")
        if cfg.link == "serial" and not Path(cfg.port).exists():
            raise FileNotFoundError(f"Serial port {cfg.port} not found (--port to use another)")
        if not cfg.calibration_path.exists():
            raise FileNotFoundError(f"No calibration at {cfg.calibration_path} (--sensor-calibration)")
        self.cfg = cfg
        self._link = self._teensy = None

    def start(self) -> None:
        import wg_helpers  # Not at the top: scipy, only needed with the Teensy

        cfg = self.cfg
        if cfg.link == "ble":
            print(f"Connecting to {cfg.ble_name} over BLE...")
            self._link = wg_helpers.BLEReceiver(name=cfg.ble_name)
        else:
            print(f"Connecting over {cfg.port}...")
            self._link = wg_helpers.SerialReceiver(cfg.port, cfg.baudrate)
        try:
            self._link.start(timeout=cfg.link_timeout_s)
        except TimeoutError as e:
            raise RuntimeError(f"{e}: {getattr(self._link, 'error', None) or 'no details'}") from None
        self._teensy = wg_helpers.TeensyController(self._link)
        self._teensy.stream(False)  # Still streaming from a previous run: the commands' ACKs get through anyway

        status = self._teensy.status()
        if cfg.force_upload or not (status["config_ok"] and status["templates_ok"]):
            print(f"Uploading the calibration {cfg.calibration_path.name}...")
            self._teensy.upload_calibration(wg_helpers.load_calibration(cfg.calibration_path))
        else:
            print("The Teensy already has a calibration (--upload to replace it)")
        print(f"Recording the background ({cfg.background_cycles} cycles): don't touch the waveguide")
        self._teensy.record_background(cfg.background_cycles)
        self._teensy.set_method(cfg.method)
        self._teensy.set_average(cfg.average)
        self._teensy.stream(True)
        print(f"Teensy streaming ({cfg.method}, {cfg.average} cycle(s) per result)")
        super().start()

    def _run(self) -> None:
        while not self._stop.is_set():
            result = self._link.get("result", timeout=self.RESULT_TIMEOUT_S)
            if result is not None:
                self._publish(frame_from_result(result))

    def close(self) -> None:
        super().close()
        if self._link is None:
            return
        try:
            self._teensy.stream(False)
        except (TimeoutError, RuntimeError) as e:
            print(f"Could not stop the Teensy stream: {e}")
        self._link.stop()


def add_teensy_args(parser: argparse.ArgumentParser, default: TeensyConfig = TeensyConfig()) -> None:
    """Command line options of `TeensyConfig`; `teensy_config(args)` reads them back."""
    group = parser.add_argument_group("Teensy (on-board detection)")
    group.add_argument("--link", choices=["ble", "serial"], default=default.link)
    group.add_argument("--ble-name", default=default.ble_name, help=f"BLE device name (default: {default.ble_name})")
    group.add_argument("--port", default=default.port, help=f"serial link port (default: {default.port})")
    group.add_argument("--baudrate", type=int, default=default.baudrate)
    group.add_argument(
        "--sensor-calibration", type=Path, default=default.calibration_path, help="wg_helpers.calibrate() .npz"
    )
    group.add_argument("--upload", action="store_true", help="upload the calibration even if the Teensy has one")
    group.add_argument("--method", default=default.method, help=f"detection method (default: {default.method})")


def teensy_config(args: argparse.Namespace, default: TeensyConfig = TeensyConfig()) -> TeensyConfig:
    return replace(
        default,
        link=args.link,
        ble_name=args.ble_name,
        port=args.port,
        baudrate=args.baudrate,
        calibration_path=args.sensor_calibration,
        force_upload=args.upload,
        method=args.method,
    )


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
