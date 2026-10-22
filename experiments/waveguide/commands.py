"""Teleop commands and the sources producing them.

A source turns some input (terminal, waveguide sensor) into commands for the robot loop in
`wg_teleop.py`. Every source has the same interface, so the robot loop does not depend on
where the commands come from.

Nothing here imports ROS: `sensor.py` (run in its own process) also uses `MoveCommand`.
"""

import enum
import time
from dataclasses import dataclass, field


@dataclass(frozen=True)
class MoveCommand:
    """Move the end-effector by `distance` [m] along a base-frame `axis` at `speed` [m/s]."""

    axis: str
    distance: float
    speed: float
    t_created: float = field(default_factory=time.monotonic)  # CLOCK_MONOTONIC, shared across processes


class Control(enum.Enum):
    HOME = "home"
    QUIT = "quit"


Command = MoveCommand | Control


class CommandSource:
    """Interface of a command source."""

    def start(self) -> None:
        pass

    def get_command(self, timeout: float) -> Command | None:
        """Return the next command, or None if none arrived within `timeout` [s]."""
        raise NotImplementedError

    def close(self) -> None:
        pass


class TerminalSource(CommandSource):
    """Commands typed in the terminal: `<axis> <distance> <speed>`, `home` or `q`.

    Blocks on the prompt, ignoring `timeout`: nothing else needs the loop meanwhile.
    """

    HELP = """Terminal commands:
    <axis> <distance> <speed>   axis in {x, y, z}, distance in m (signed), speed in m/s
    home                        go back to the home position
    q                           quit"""

    def start(self) -> None:
        print(self.HELP)

    def get_command(self, timeout: float) -> Command | None:
        try:
            line = input("> ").strip().lower()
        except EOFError:
            return Control.QUIT
        if not line:
            return None
        if line in ("q", "quit", "exit"):
            return Control.QUIT
        if line == "home":
            return Control.HOME

        parts = line.split()
        try:
            if len(parts) != 3:
                raise ValueError("Expected '<axis> <distance> <speed>'")
            return MoveCommand(parts[0], float(parts[1]), float(parts[2]))
        except ValueError as e:
            print(f"Invalid command: {e}")
            return None


class WaveguideSource(CommandSource):
    """Commands from the waveguide sensor, processed in a separate process (see `sensor.py`).

    With `plot`, that process also shows the sensor live plot. With `logger` (a started `ExperimentLogger`), it
    records what it sees into it (`recording.SensorRecorder`).
    """

    def __init__(self, port: str, baudrate: int, plot: bool = False, logger=None):
        from sensor import SensorProcess  # Not at the top: sensor.py imports this module

        self._sensor = SensorProcess(port, baudrate, plot=plot, logger=logger)

    def start(self) -> None:
        self._sensor.start()
        print("Waveguide sensor started, waiting for swipes (Ctrl+C to quit)")

    def get_command(self, timeout: float) -> Command | None:
        if not self._sensor.is_alive():
            raise RuntimeError("Waveguide sensor process ended (see its error above, or the plot window was closed)")
        return self._sensor.get_command(timeout)

    def close(self) -> None:
        self._sensor.stop()
