"""Live view of a teleoperation run: its state streamed to Foxglove while it runs.

[LiveView][] samples the latest values both processes share (the haptic loop's mailboxes, the
robot plant, the command loop) at its own rate, on its own thread, and logs them through an
``experiment_logger`` Foxglove sink. Connect the Foxglove app to ``ws://<host>:<port>`` and
import the layout [write_layout][] makes. Topics (``/live/<stream>``, JSON, fields under
``values``):

- ``haptic``: leader ``x_l``/``v_l``, proxy ``proxy_x``/``proxy_v`` (proxy methods), the
  rendered force ``force`` and the force sent ``f`` [world N], the handle force ``f_handle``
  [device N, per world axis];
- ``robot``: measured ``ee_position``/``ee_velocity`` [base frame], ``x_i``/``v_i`` on the
  interface, ``lam`` (the coupling force the operator should feel), ``task_force`` [N];
- ``target``: what the haptic side sent (``sent_x``, ``sent_v``, ``sent_f_ff``), and what was
  published to the robot after any command delay (``robot_position`` [base frame], ``robot_x``,
  ``robot_v``, ``robot_f_ff``);
- ``rates`` [Hz]: ``haptic``, ``plant`` (robot samples kept), ``feedback`` (delivered to the
  haptic loop), ``command`` (targets sent), ``robot_target`` (published), ``live``;
- ``safety``: ``energy_j`` (passivity observer at the handle), ``tdpa_energy_j``,
  ``tdpa_damping``, ``watchdog_gain``, ``guard_tripped``, trip counts, and the plant / delivery /
  device ages [ms].

Non-finite values are sent as ``null``: JSON has no NaN.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
from experiment_logger import ExperimentLogger, LoggingConfig
from experiment_logger.config import FoxgloveSinkConfig
from haptic_teleop import RateTicker

__all__ = ["LiveView", "live_logger", "write_layout"]

TOPIC_PREFIX = "/live"


def live_logger(port: int, host: str = "0.0.0.0") -> ExperimentLogger:
    """An ``ExperimentLogger`` with only a Foxglove websocket sink (topics under ``/live``)."""
    return ExperimentLogger(
        LoggingConfig(
            sinks=["foxglove_sink"],
            foxglove_sink=FoxgloveSinkConfig(enabled=True, host=host, port=port, topic_prefix=TOPIC_PREFIX),
        )
    )


def _clean(value):
    """``value`` as JSON-safe floats/lists, non-finite → ``None``."""
    if value is None:
        return None
    if isinstance(value, np.ndarray) or isinstance(value, (list, tuple)):
        return [_clean(v) for v in np.asarray(value, dtype=float).ravel()]
    v = float(value)
    return v if math.isfinite(v) else None


class _Rate:
    """Events per second from a monotonic counter, over a sliding window."""

    def __init__(self, window_s: float = 1.0) -> None:
        self.window_s = window_s
        self._points: deque[tuple[float, float]] = deque()

    def update(self, count: float, now_s: float) -> float:
        pts = self._points
        pts.append((now_s, float(count)))
        while len(pts) > 2 and now_s - pts[0][0] > self.window_s:
            pts.popleft()
        if len(pts) < 2 or pts[-1][0] <= pts[0][0]:
            return 0.0
        return (pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0])


class LiveView:
    """Streams the run's latest state to a Foxglove logger at ``hz``, on its own thread.

    Every source is optional (``None`` skips what it feeds). Reads only: the latest values, never
    the histories, so a slow tick costs this thread, not the loops.

    Args:
        logger: a started-on-[start][LiveView.start] ``ExperimentLogger`` ([live_logger][]).
        hz: sample rate [Hz].
        plant: the `FR3Plant` (robot samples, targets published).
        plant_box: the ``PlantMailbox`` the haptic loop reads (feedback delivered).
        haptic_state: ``HapticStateMailbox``.
        telemetry: ``TelemetryMailbox``.
        status: ``StatusMailbox``.
        commands: the command loop's list of ``(t_s, x, v, f_ff)`` sent (appended by it).
    """

    def __init__(
        self,
        logger: ExperimentLogger,
        hz: float,
        *,
        plant=None,
        plant_box=None,
        haptic_state=None,
        telemetry=None,
        status=None,
        commands: list | None = None,
    ) -> None:
        if hz <= 0:
            raise ValueError(f"hz must be > 0, got {hz}")
        self.logger = logger
        self.hz = float(hz)
        self.plant = plant
        self.plant_box = plant_box
        self.haptic_state = haptic_state
        self.telemetry = telemetry
        self.status = status
        self.commands = commands
        self.ticks = 0
        self._rates = {k: _Rate() for k in ("plant", "feedback", "command", "robot_target", "live")}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._t0 = time.monotonic()

    def start(self) -> None:
        """Start the logger (its websocket server) and the sampling thread."""
        self.logger.start()
        self._t0 = time.monotonic()
        self._thread = threading.Thread(target=self._run, name="live_view", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop sampling and close the server."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.logger.stop()

    def _run(self) -> None:
        ticker = RateTicker(self.hz)
        while not self._stop.is_set():
            try:
                self.step()
            except Exception as e:  # noqa: BLE001 - a display must never take the run down
                print(f"[live] sample failed: {e!r}", flush=True)
            ticker.sleep()

    def step(self, now_s: float | None = None) -> None:
        """Log one sample of every stream."""
        now = time.monotonic() if now_s is None else now_s
        t = now - self._t0
        self.ticks += 1
        log = self.logger.log_sample
        rates = {"live": self._rates["live"].update(self.ticks, now)}

        telemetry = self.telemetry.read() if self.telemetry is not None else None
        haptic = self.haptic_state.read() if self.haptic_state is not None else None
        if haptic is not None:
            values = {k: _clean(haptic[k]) for k in ("x_l", "v_l", "proxy_x", "proxy_v", "force")}
            if telemetry is not None:
                values.update(f=_clean(telemetry["f"]), f_handle=_clean(telemetry["f_handle"]))
            log("haptic", {"t": t, "haptic_t": haptic["t_s"], **values})

        if self.plant is not None:
            latest = self.plant.sample
            if latest is not None:
                updates, s = latest
                rates["plant"] = self._rates["plant"].update(updates, now)
                log(
                    "robot",
                    {
                        "t": t,
                        "ee_position": _clean(s.ee_position),
                        "ee_velocity": _clean(s.ee_velocity),
                        "x_i": _clean(s.x_i),
                        "v_i": _clean(s.v_i),
                        "lam": _clean(s.lam),
                        "task_force": _clean(s.task_force),
                    },
                )
            target = getattr(self.plant, "last_target", None)
            values = {"t": t}
            if target is not None:
                n, position, x, v, f_ff = target
                rates["robot_target"] = self._rates["robot_target"].update(n, now)
                values.update(robot_position=_clean(position), robot_x=_clean(x), robot_v=_clean(v), robot_f_ff=_clean(f_ff))
            if self.commands:
                _, x, v, f_ff = self.commands[-1]  # appended by the command loop; [-1] is atomic
                values.update(sent_x=_clean(x), sent_v=_clean(v), sent_f_ff=_clean(f_ff))
            if len(values) > 1:
                log("target", values)
        if self.commands is not None:
            rates["command"] = self._rates["command"].update(len(self.commands), now)
        if self.plant_box is not None:
            delivered = self.plant_box.sample
            if delivered is not None:
                rates["feedback"] = self._rates["feedback"].update(delivered[0], now)

        if telemetry is not None:
            rates["haptic"] = telemetry["haptic_hz"]
            status = self.status.read() if self.status is not None else None
            log(
                "safety",
                {
                    "t": t,
                    "energy_j": _clean(telemetry["energy_j"]),
                    "tdpa_energy_j": _clean(telemetry["tdpa_energy_j"]),
                    "tdpa_damping": _clean(telemetry["tdpa_damping"]),
                    "watchdog_gain": _clean(telemetry["watchdog_gain"]),
                    "guard_tripped": _clean(telemetry["guard_tripped"]),
                    "watchdog_trips": _clean(status["watchdog_trips"]) if status else None,
                    "guard_trips": _clean(status["guard_trips"]) if status else None,
                    "plant_age_ms": _clean(telemetry["plant_age_s"] * 1e3),
                    "delivery_age_ms": _clean(telemetry["delivery_age_s"] * 1e3),
                    "device_age_ms": _clean(telemetry["device_age_s"] * 1e3),
                },
            )
        log("rates", {"t": t, **{k: _clean(v) for k, v in rates.items()}})


def write_layout(path: str | Path, interface_axis: str = "z", window_s: float = 10.0) -> Path:
    """Write a Foxglove layout for the ``/live`` topics to ``path`` (import it once in the app).

    Args:
        path: the ``.json`` file to write.
        interface_axis: ``x``, ``y`` or ``z``, for the titles.
        window_s: the plots' sliding time window [s].
    """
    import foxglove.layouts as fl

    if interface_axis not in ("x", "y", "z"):
        raise ValueError(f"interface_axis must be x, y or z, got {interface_axis!r}")

    def plot(title: str, series: list[tuple[str, str]], y_label: str) -> fl.PlotPanel:
        return fl.PlotPanel(
            title=title,
            config=fl.PlotConfig(
                paths=[fl.PlotSeries(value=f"{TOPIC_PREFIX}/{v}", label=label, enabled=True) for v, label in series],
                show_legend=True,
                legend_display="top",
                time_window_mode="sliding",
                following_view_width=window_s,
                y_axis_label=y_label,
            ),
        )

    position = plot(
        f"Position on {interface_axis} [m]",
        [
            ("haptic.values.x_l[0]", "leader"),
            ("robot.values.x_i[0]", "robot (measured)"),
            ("target.values.sent_x[0]", "target sent"),
            ("target.values.robot_x[0]", "target at robot"),
            ("haptic.values.proxy_x[0]", "proxy"),
        ],
        "m",
    )
    force = plot(
        "Force on the interface [world N]",
        [
            ("haptic.values.force[0]", "rendered"),
            ("haptic.values.f[0]", "sent to device"),
            ("robot.values.lam[0]", "robot λ (measured)"),
            ("target.values.robot_f_ff[0]", "feedforward at robot"),
        ],
        "N",
    )
    handle = plot("Handle force [device N]", [(f"haptic.values.f_handle[{i}]", a) for i, a in enumerate("xyz")], "N")
    ee = plot("EE position [m]", [(f"robot.values.ee_position[{i}]", a) for i, a in enumerate("xyz")], "m")
    ee_force = plot("EE task force [N]", [(f"robot.values.task_force[{i}]", a) for i, a in enumerate("xyz")], "N")
    energy = plot(
        "Energy / passivity",
        [
            ("safety.values.energy_j", "observer at handle [J]"),
            ("safety.values.tdpa_energy_j", "TDPA observer [J]"),
            ("safety.values.tdpa_damping", "TDPA damping [N·s/m]"),
        ],
        "",
    )
    rates = plot(
        "Rates [Hz]",
        [
            ("rates.values.haptic", "haptic"),
            ("rates.values.plant", "robot samples"),
            ("rates.values.feedback", "feedback delivered"),
            ("rates.values.command", "commands sent"),
            ("rates.values.robot_target", "targets at robot"),
        ],
        "Hz",
    )
    ages = plot(
        "Ages [ms] / watchdog",
        [
            ("safety.values.plant_age_ms", "plant sample"),
            ("safety.values.delivery_age_ms", "since delivery"),
            ("safety.values.device_age_ms", "device"),
            ("safety.values.watchdog_gain", "watchdog gain"),
        ],
        "ms",
    )

    def column(*panels):
        return fl.SplitContainer(direction="column", items=[fl.SplitItem(content=p) for p in panels])

    layout = fl.Layout(
        content=fl.TabContainer(
            tabs=[
                fl.TabItem(
                    title="Teleop",
                    content=fl.SplitContainer(
                        direction="row",
                        items=[
                            fl.SplitItem(proportion=2, content=column(position, force)),
                            fl.SplitItem(proportion=1, content=column(energy, handle)),
                        ],
                    ),
                ),
                fl.TabItem(title="Robot", content=column(ee, ee_force)),
                fl.TabItem(title="Timing", content=column(rates, ages)),
            ]
        )
    )
    path = Path(path)
    path.write_text(layout.to_json())
    return path
