"""Live view of a teleoperation run: its state streamed to Foxglove while it runs.

[LiveView][] samples the latest values both processes share (the haptic loop's mailboxes, the
robot plant, the command loop) at its own rate, on its own thread, and logs them through an
``experiment_logger`` Foxglove sink. Connect the Foxglove app to ``ws://<host>:<port>`` and
import the layout [write_layout][] makes. Topics (``/live/<stream>``, JSON with a schema, fields
under ``values``):

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

Every stream has the same fields from its first sample on (the channel schema is inferred from
it): a value not available yet is ``null``, and so is a non-finite one (JSON has no NaN).
"""

from __future__ import annotations

import json
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
RATES = ("haptic", "plant", "feedback", "command", "robot_target", "live")


def live_logger(port: int, host: str = "0.0.0.0") -> ExperimentLogger:
    """An ``ExperimentLogger`` with only a Foxglove websocket sink (topics under ``/live``)."""
    return ExperimentLogger(
        LoggingConfig(
            sinks=["foxglove_sink"],
            foxglove_sink=FoxgloveSinkConfig(enabled=True, host=host, port=port, topic_prefix=TOPIC_PREFIX),
        )
    )


def _num(value) -> float | None:
    """A JSON-safe float; ``None`` for a missing or non-finite value."""
    if value is None:
        return None
    v = float(value)
    return v if math.isfinite(v) else None


def _vec(value, n: int) -> list[float | None]:
    """``n`` JSON-safe floats; all ``None`` for a missing value."""
    if value is None:
        return [None] * n
    return [_num(v) for v in np.asarray(value, dtype=float).ravel()]


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

    Every source is optional (``None`` leaves its fields ``null``, or skips a stream it alone
    feeds). Reads only the latest values, never the histories, so a slow tick costs this thread,
    not the loops.

    Args:
        logger: an ``ExperimentLogger`` ([live_logger][]), started by [start][LiveView.start].
        hz: sample rate [Hz].
        dim: interface dimension.
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
        dim: int = 1,
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
        self.dim = int(dim)
        self.plant = plant
        self.plant_box = plant_box
        self.haptic_state = haptic_state
        self.telemetry = telemetry
        self.status = status
        self.commands = commands
        self.ticks = 0
        self._rates = {k: _Rate() for k in RATES if k != "haptic"}
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
        d = self.dim
        log = self.logger.log_sample
        rates: dict[str, float | None] = dict.fromkeys(RATES)
        rates["live"] = self._rates["live"].update(self.ticks, now)

        telemetry = self.telemetry.read() if self.telemetry is not None else None
        haptic = self.haptic_state.read() if self.haptic_state is not None else None
        if haptic is not None:
            log(
                "haptic",
                {
                    "t": t,
                    "haptic_t": _num(haptic["t_s"]),
                    **{k: _vec(haptic[k], d) for k in ("x_l", "v_l", "proxy_x", "proxy_v", "force")},
                    "f": _vec(telemetry["f"] if telemetry else None, d),
                    "f_handle": _vec(telemetry["f_handle"] if telemetry else None, 3),
                },
            )

        if self.plant is not None:
            latest = self.plant.sample
            if latest is not None:
                updates, s = latest
                rates["plant"] = self._rates["plant"].update(updates, now)
                log(
                    "robot",
                    {
                        "t": t,
                        "ee_position": _vec(s.ee_position, 3),
                        "ee_velocity": _vec(s.ee_velocity, 3),
                        "x_i": _vec(s.x_i, d),
                        "v_i": _vec(s.v_i, d),
                        "lam": _vec(s.lam, d),
                        "task_force": _vec(s.task_force, 3),
                    },
                )
        if self.plant is not None or self.commands is not None:
            target = getattr(self.plant, "last_target", None)
            n, position, x, v, f_ff = target if target is not None else (None, None, None, None, None)
            if n is not None:
                rates["robot_target"] = self._rates["robot_target"].update(n, now)
            # appended by the command loop; [-1] is atomic
            sent = self.commands[-1] if self.commands else (None, None, None, None)
            log(
                "target",
                {
                    "t": t,
                    "sent_x": _vec(sent[1], d),
                    "sent_v": _vec(sent[2], d),
                    "sent_f_ff": _vec(sent[3], d),
                    "robot_position": _vec(position, 3),
                    "robot_x": _vec(x, d),
                    "robot_v": _vec(v, d),
                    "robot_f_ff": _vec(f_ff, d),
                },
            )
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
                    **{k: _num(telemetry[k]) for k in ("energy_j", "tdpa_energy_j", "tdpa_damping")},
                    "watchdog_gain": _num(telemetry["watchdog_gain"]),
                    "guard_tripped": _num(telemetry["guard_tripped"]),
                    "watchdog_trips": _num(status["watchdog_trips"]) if status else None,
                    "guard_trips": _num(status["guard_trips"]) if status else None,
                    "plant_age_ms": _num(telemetry["plant_age_s"] * 1e3),
                    "delivery_age_ms": _num(telemetry["delivery_age_s"] * 1e3),
                    "device_age_ms": _num(telemetry["device_age_s"] * 1e3),
                },
            )
        log("rates", {"t": t, **{k: _num(v) for k, v in rates.items()}})


def _plot(title: str, series: list[tuple[str, str]], window_s: float) -> dict:
    """A Plot panel config (the format the app exports): ``series`` are ``(path, label)``."""
    return {
        "paths": [{"value": f"{TOPIC_PREFIX}/{path}", "enabled": True, "timestampMethod": "receiveTime", "label": label} for path, label in series],
        "showLegend": True,
        "legendDisplay": "top",
        "showPlotValuesInLegend": True,
        "showXAxisLabels": True,
        "showYAxisLabels": True,
        "isSynced": True,
        "xAxisVal": "timestamp",
        "followingViewWidth": window_s,
        "foxglovePanelTitle": title,
    }


def layout_panels(interface_axis: str = "z", window_s: float = 10.0) -> dict[str, list[tuple[str, dict]]]:
    """The layout's tabs: per tab title, its ``(panel id, Plot config)`` in order."""
    if interface_axis not in ("x", "y", "z"):
        raise ValueError(f"interface_axis must be x, y or z, got {interface_axis!r}")
    xyz = list(enumerate("xyz"))
    return {
        "Teleop": [
            (
                "Plot!position",
                _plot(
                    f"Position on {interface_axis} [m]",
                    [
                        ("haptic.values.x_l[0]", "leader"),
                        ("robot.values.x_i[0]", "robot (measured)"),
                        ("target.values.sent_x[0]", "target sent"),
                        ("target.values.robot_x[0]", "target at robot"),
                        ("haptic.values.proxy_x[0]", "proxy"),
                    ],
                    window_s,
                ),
            ),
            (
                "Plot!force",
                _plot(
                    "Force on the interface [world N]",
                    [
                        ("haptic.values.force[0]", "rendered"),
                        ("haptic.values.f[0]", "sent to device"),
                        ("robot.values.lam[0]", "robot λ (measured)"),
                        ("target.values.robot_f_ff[0]", "feedforward at robot"),
                    ],
                    window_s,
                ),
            ),
            (
                "Plot!energy",
                _plot(
                    "Energy / passivity",
                    [
                        ("safety.values.energy_j", "observer at handle [J]"),
                        ("safety.values.tdpa_energy_j", "TDPA observer [J]"),
                        ("safety.values.tdpa_damping", "TDPA damping [N·s/m]"),
                    ],
                    window_s,
                ),
            ),
            ("Plot!handle", _plot("Handle force [device N]", [(f"haptic.values.f_handle[{i}]", a) for i, a in xyz], window_s)),
        ],
        "Robot": [
            ("Plot!ee_position", _plot("EE position [m]", [(f"robot.values.ee_position[{i}]", a) for i, a in xyz], window_s)),
            ("Plot!ee_force", _plot("EE task force [N]", [(f"robot.values.task_force[{i}]", a) for i, a in xyz], window_s)),
        ],
        "Timing": [
            (
                "Plot!rates",
                _plot(
                    "Rates [Hz]",
                    [
                        ("rates.values.haptic", "haptic"),
                        ("rates.values.plant", "robot samples"),
                        ("rates.values.feedback", "feedback delivered"),
                        ("rates.values.command", "commands sent"),
                        ("rates.values.robot_target", "targets at robot"),
                    ],
                    window_s,
                ),
            ),
            (
                "Plot!ages",
                _plot(
                    "Ages [ms] / watchdog",
                    [
                        ("safety.values.plant_age_ms", "plant sample"),
                        ("safety.values.delivery_age_ms", "since delivery"),
                        ("safety.values.device_age_ms", "device"),
                        ("safety.values.watchdog_gain", "watchdog gain"),
                    ],
                    window_s,
                ),
            ),
        ],
    }


def _split(ids: list[str], direction: str):
    """A mosaic tree splitting ``ids`` evenly along ``direction`` (``row`` or ``column``)."""
    if len(ids) == 1:
        return ids[0]
    return {
        "first": ids[0],
        "second": _split(ids[1:], direction),
        "direction": direction,
        "splitPercentage": 100.0 / len(ids),
    }


def write_layout(path: str | Path, interface_axis: str = "z", window_s: float = 10.0) -> Path:
    """Write a Foxglove layout for the ``/live`` topics to ``path``: import it once in the app
    (Layouts → Import from file).

    Args:
        path: the ``.json`` file to write.
        interface_axis: ``x``, ``y`` or ``z``, for the titles.
        window_s: the plots' time window [s].
    """
    tabs = layout_panels(interface_axis, window_s)
    config_by_id: dict[str, dict] = {}
    tab_items = []
    for title, panels in tabs.items():
        ids = [pid for pid, _ in panels]
        config_by_id.update(dict(panels))
        if title == "Teleop":  # two columns: position and force on the left, energy and handle on the right
            tree = {
                "first": _split(ids[:2], "column"),
                "second": _split(ids[2:], "column"),
                "direction": "row",
                "splitPercentage": 65.0,
            }
        else:
            tree = _split(ids, "column")
        tab_items.append({"title": title, "layout": tree})
    config_by_id["Tab!live"] = {"activeTabIdx": 0, "tabs": tab_items}
    layout = {
        "configById": config_by_id,
        "globalVariables": {},
        "userNodes": {},
        "playbackConfig": {"speed": 1},
        "layout": "Tab!live",
    }
    path = Path(path)
    path.write_text(json.dumps(layout, indent=2, ensure_ascii=False) + "\n")
    return path
