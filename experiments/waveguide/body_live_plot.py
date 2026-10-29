"""Live plot of the robot body demo (`wg_body_demo.py --live-plot`), with the hole -> direction calibration.

Made from the Teensy results (BLE), not the raw frames. Panels (look: `sensor.PlotStyle`):
1. Touches: the covered holes of each result against time (last `history_s`), in the color of their direction,
   and the stops (vermillion) and bounding box hits (orange) at the top
2. Waveguide: the holes along the waveguide (layout of the sensor calibration), each in the color of its
   direction (`DIRECTION_COLORS`): calibrated holes in full, interpolated ones lighter and blending the colors of
   their components, unmapped ones grey. The direction names are over the calibrated runs. Covered holes are
   filled and outlined. Above: what the arm does (moving, stop, bounding box)
3. Arm: the end-effector in the bounding box, top (x-y) and side (x-z) views, with its trail and its velocity
   (the line it covers in `arrow_s`)

Calibration panel (the ⚙ tab on the right, or C): Start, then for each direction (+x, -x, ...) touch all its
holes (one by one or a swipe) and press Space for the next one; Backspace redoes the current direction, Esc
cancels. The arm stays stopped meanwhile: calibrate with it at home. The holes counted for the current direction
are in its color. At the end the mapping is saved (`BodyDemoConfig.calibration_path`, the previous one kept as a
backup) and used right away.

Qt runs in the main thread, the control loop (`wg_body_demo.run`) in another: they share a `DemoMonitor`.
"""

import signal
import time
from collections import Counter, deque
from pathlib import Path

import numpy as np
from sensor import PLOT_STYLE, PlotStyle, _PlotTools
from touch import TouchFrame, TouchSource
from wg_body_demo import (
    DIRECTIONS,
    BodyDemoConfig,
    DemoMonitor,
    DirectionCalibration,
    HoleMapping,
    hole_vectors,
    save_mapping,
)

# Okabe-Ito (colorblind-safe); -z is a darker yellow, readable on white
DIRECTION_COLORS = {
    "+x": "#0072B2",
    "-x": "#56B4E9",
    "+y": "#CC79A7",
    "-y": "#E69F00",
    "+z": "#009E73",
    "-z": "#B59F00",
}


def hex_rgb(color: str) -> np.ndarray:
    return np.array([int(color[i : i + 2], 16) for i in (1, 3, 5)], dtype=float)


def hole_colors(vectors: dict[int, np.ndarray]) -> dict[int, np.ndarray]:
    """RGB of each mapped hole: the color of its direction; for one in between (interpolated), the blend of the
    colors of its components, weighted by their square."""
    colors = {}
    for hole, vector in vectors.items():
        weights = {name: max(0.0, float(np.dot(vector, d))) ** 2 for name, d in DIRECTIONS.items()}
        total = sum(weights.values())
        colors[hole] = sum(w * hex_rgb(DIRECTION_COLORS[name]) for name, w in weights.items()) / total
    return colors


def runs(holes) -> list[tuple[int, int]]:
    """Consecutive runs (first, last) of the sorted hole indices."""
    result = []
    for hole in sorted(holes):
        if result and hole == result[-1][1] + 1:
            result[-1] = (result[-1][0], hole)
        else:
            result.append((hole, hole))
    return result


def hole_layout(sensor_calibration: Path, direction_holes: dict[str, tuple[int, ...]]) -> tuple[np.ndarray, np.ndarray]:
    """Hole indices and their distance along the waveguide [cm], from the sensor calibration (.npz); without
    one, the mapped holes 1 cm apart."""
    if sensor_calibration.exists():
        with np.load(sensor_calibration) as data:
            return data["hole_ids"].astype(int), data["hole_cm"].astype(float)
    ids = np.arange(1, max((h for holes in direction_holes.values() for h in holes), default=1) + 1)
    return ids, ids.astype(float)


def mapping_text(direction_holes: dict[str, tuple[int, ...]]) -> str:
    """HTML list of the holes of each direction, in its color."""
    lines = []
    for name in DIRECTIONS:
        holes = direction_holes.get(name, ())
        text = ", ".join(f"{a}" if a == b else f"{a}-{b}" for a, b in runs(holes)) or "none"
        lines.append(f'<span style="color:{DIRECTION_COLORS[name]}; font-weight:bold">{name}</span>&nbsp; {text}')
    return "<br>".join(lines)


def show(
    touch: TouchSource,
    monitor: DemoMonitor,
    cfg: BodyDemoConfig,
    hole_ids: np.ndarray,
    hole_cm: np.ndarray,
    link=None,
    style: PlotStyle = PLOT_STYLE,
    history_s: float = 10.0,
    trail_s: float = 5.0,
    arrow_s: float = 1.0,
) -> None:
    """The live plot (see the module docstring), refreshed at ~60 Hz, until the window is closed (or Ctrl+C) or
    `monitor.stop_event` is set. `link` (`wg_body_demo.LinkOrientation`): its orientation is saved with a
    calibration. Raises RuntimeError if the touch source stopped."""
    tools = _PlotTools(style, "Waveguide body demo")
    pg, QtCore, QtGui, c, app = tools.pg, tools.QtCore, tools.QtGui, tools.c, tools.app
    from pyqtgraph.Qt import QtWidgets

    hole_ids, hole_cm = np.asarray(hole_ids, dtype=int), np.asarray(hole_cm, dtype=float)
    cm_of = dict(zip(hole_ids.tolist(), hole_cm.tolist()))
    idle_rgb, touch_rgb = tools.rgb(c["hole_idle"]), tools.rgb(c["touch"])
    names = list(DIRECTIONS)
    state_colors = dict(
        starting=c["muted"],
        idle=c["muted"],
        moving=c["command"],
        stop=c["touch"],
        box=c["reference"],
        nodata=c["muted"],
    )
    errors = []

    window = QtWidgets.QWidget()
    window.setWindowTitle("Waveguide body demo")
    window.setStyleSheet(f"background-color: {c['background']}; color: {c['foreground']};")
    window_layout = QtWidgets.QHBoxLayout(window)
    window_layout.setContentsMargins(0, 0, 0, 0)
    window_layout.setSpacing(0)
    win = pg.GraphicsLayoutWidget()
    window_layout.addWidget(win, 1)
    win.ci.setContentsMargins(16, 12, 24, 12)
    win.ci.setSpacing(18)

    # 1. Touches
    touches = win.addPlot(colspan=2)
    tools.set_title(touches, "Touches")
    tools.style_axes(touches, x_label="Time (s)", y_label="Hole")
    touches.setXRange(-history_s, 0, padding=0)
    top_hole = float(hole_ids.max())
    touches.setYRange(hole_ids.min() - 1, top_hole + 2, padding=0)
    touches.setMouseEnabled(x=False, y=False)
    touch_points = pg.ScatterPlotItem(size=8, pen=None)
    event_marks = pg.ScatterPlotItem(symbol="t", size=14, pen=None)
    touches.addItem(touch_points)
    touches.addItem(event_marks)

    # 2. Waveguide
    win.nextRow()
    guide = win.addPlot(colspan=2)
    tools.style_axes(guide, x_label="Distance along the waveguide (cm)")
    guide.hideAxis("left")
    guide.showGrid(x=False, y=False)
    guide.setMouseEnabled(x=False, y=False)
    guide.setYRange(-0.7, 0.8)
    margin = 2 * float(np.min(np.diff(hole_cm))) if len(hole_cm) > 1 else 1.0
    guide.setXRange(hole_cm.min() - margin, hole_cm.max() + margin, padding=0)

    def set_status(text, color):
        tools.set_title(guide, text, color=color, size=style.status_pt)

    # The spots are only recoloured, never moved or resized (see sensor.run_with_live_plot)
    idle_pen, covered_pen = pg.mkPen(c["hole_edge"], width=1.2), pg.mkPen(c["hole_covered_edge"], width=3)
    holes = pg.ScatterPlotItem(x=hole_cm, y=np.zeros(len(hole_cm)), size=20, brush=pg.mkBrush(*idle_rgb), pen=idle_pen)
    holes.setZValue(5)
    guide.addItem(holes)
    for i, (hole, x) in enumerate(zip(hole_ids, hole_cm)):
        if i > 0 and hole % style.hole_label_every:
            continue
        label = tools.text_item(str(hole), c["muted"], anchor=(0.5, 0.0), font_=tools.small_font)
        label.setPos(x, -0.22)
        guide.addItem(label)
    direction_labels = []
    shown_holes = None

    def show_holes(colors: np.ndarray, outlined: set[int]) -> None:
        nonlocal shown_holes
        key = (frozenset(outlined), colors.tobytes())
        if key == shown_holes:
            return
        holes.setBrush([pg.mkBrush(*color) for color in colors])
        holes.setPen([covered_pen if hole in outlined else idle_pen for hole in hole_ids])
        shown_holes = key

    # 3. Arm: top and side views of the bounding box
    win.nextRow()
    lower, upper = np.array(cfg.box_lower), np.array(cfg.box_upper)
    views = []
    for title, (i, j) in (("Top view", (0, 1)), ("Side view", (0, 2))):
        view = win.addPlot()
        tools.set_title(view, title)
        tools.style_axes(view, x_label=f"{'xyz'[i]} (m)", y_label=f"{'xyz'[j]} (m)")
        view.setAspectLocked(True)
        for axis in ("left", "bottom"):
            view.getAxis(axis).enableAutoSIPrefix(False)
        view.setMouseEnabled(x=False, y=False)
        pad = 0.05
        view.setXRange(lower[i] - pad, upper[i] + pad, padding=0)
        view.setYRange(lower[j] - pad, upper[j] + pad, padding=0)
        view.plot(
            [lower[i], upper[i], upper[i], lower[i], lower[i]],
            [lower[j], lower[j], upper[j], upper[j], lower[j]],
            pen=tools.pen(c["muted"], width=1.5, dashed=True),
        )
        trail = view.plot(pen=tools.pen(tools.tint(c["signal"], 110), width=1.5))
        velocity_line = view.plot()
        dot = pg.ScatterPlotItem(size=14, brush=pg.mkBrush(c["signal"]), pen=pg.mkPen(c["background"], width=1.5))
        view.addItem(dot)
        views.append(((i, j), trail, velocity_line, dot))

    win.ci.layout.setRowStretchFactor(0, 2)
    win.ci.layout.setRowStretchFactor(1, 2)
    win.ci.layout.setRowStretchFactor(2, 3)

    # --- Panel: the mapping, and its calibration; collapsed to the ⚙ tab (out of the recordings), C toggles it
    panel_width = 270
    panel = QtWidgets.QWidget()
    panel_layout = QtWidgets.QHBoxLayout(panel)
    panel_layout.setContentsMargins(0, 8, 4, 8)
    panel_layout.setSpacing(4)
    toggle = QtWidgets.QToolButton()
    toggle.setText("⚙")
    toggle.setToolTip("Hole mapping (C)")
    toggle.setCheckable(True)
    toggle.setAutoRaise(True)
    toggle.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    toggle.setStyleSheet(f"color: {c['muted']}; font-size: {style.font_pt}pt;")
    toggle_column = QtWidgets.QVBoxLayout()
    toggle_column.addWidget(toggle)
    toggle_column.addStretch(1)
    panel_layout.addLayout(toggle_column)

    content = QtWidgets.QWidget()
    content.setFixedWidth(panel_width)
    content.setStyleSheet(
        f"QWidget {{ font-family: {style.font_family}; font-size: {style.font_pt - 2}pt; }}"
        f"QPushButton {{ border: 1px solid {c['muted']}; border-radius: 4px; padding: 4px 8px; }}"
        f"QPushButton:disabled {{ color: {c['muted']}; }}"
    )
    form = QtWidgets.QVBoxLayout(content)
    title = QtWidgets.QLabel("Hole mapping")
    title.setStyleSheet(f"font-size: {style.title_pt}pt; font-weight: bold;")
    form.addWidget(title)
    mapping_label = QtWidgets.QLabel()
    mapping_label.setWordWrap(True)
    form.addWidget(mapping_label)
    start_button = QtWidgets.QPushButton("Calibrate")
    next_button = QtWidgets.QPushButton("Next direction (Space)")
    redo_button = QtWidgets.QPushButton("Redo direction (Backspace)")
    cancel_button = QtWidgets.QPushButton("Cancel (Esc)")
    for button in (start_button, next_button, redo_button, cancel_button):
        button.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)  # Space is for the next direction
        form.addWidget(button)
    info_label = QtWidgets.QLabel(
        "With the arm at home: for each direction, touch all the holes that should move the arm that way (one by "
        "one or a swipe), then Space. The arm stays stopped meanwhile."
    )
    info_label.setWordWrap(True)
    form.addWidget(info_label)
    form.addStretch(1)
    content.hide()
    panel_layout.addWidget(content)
    window_layout.addWidget(panel)

    # --- The mapping shown, and its colors
    mapping = {}

    def set_mapping(direction_holes: dict[str, tuple[int, ...]]) -> None:
        nonlocal shown_holes
        colors = hole_colors(hole_vectors(direction_holes, cfg.interpolate_holes))
        calibrated = {h for hs in direction_holes.values() for h in hs}
        mapping.update(direction_holes=direction_holes, colors=colors, calibrated=calibrated)
        mapping["brushes"] = {h: pg.mkBrush(*np.round(rgb).astype(int)) for h, rgb in colors.items()}
        mapping["unmapped_brush"] = pg.mkBrush(c["muted"])
        for label in direction_labels:
            guide.removeItem(label)
        direction_labels.clear()
        for name, hs in direction_holes.items():
            for first, last in runs(h for h in hs if h in cm_of):
                label = tools.text_item(name, DIRECTION_COLORS[name], anchor=(0.5, 1.0))
                label.setPos((cm_of[first] + cm_of[last]) / 2, 0.45)
                guide.addItem(label)
                direction_labels.append(label)
        mapping_label.setText(mapping_text(direction_holes))
        shown_holes = None

    set_mapping(dict(cfg.direction_holes))

    def mix(rgb, amount):
        return idle_rgb + amount * (np.asarray(rgb) - idle_rgb)

    def mapping_colors(covered: set[int]) -> np.ndarray:
        """Calibrated holes in their color (lighter when not covered), interpolated ones lighter still."""
        colors = []
        for hole in hole_ids:
            rgb = mapping["colors"].get(int(hole))
            if rgb is None:
                colors.append(touch_rgb if hole in covered else idle_rgb)
            elif hole in covered:
                colors.append(rgb)
            else:
                colors.append(mix(rgb, 0.6 if hole in mapping["calibrated"] else 0.3))
        return np.round(colors).astype(int)

    # --- Calibration
    session = {"calibration": None, "step": 0, "rotation": None}

    def counted(name: str) -> set[int]:
        counts = session["calibration"].counts.get(name, Counter())
        return {h for h, n in counts.items() if n >= cfg.calibration_min_frames}

    def set_buttons(calibrating: bool) -> None:
        start_button.setEnabled(not calibrating)
        for button in (next_button, redo_button, cancel_button):
            button.setEnabled(calibrating)

    set_buttons(False)

    def start_calibration() -> None:
        monitor.paused.set()
        rotation = None if link is None else link.rotation()
        session.update(calibration=DirectionCalibration(cfg.calibration_min_frames), step=0, rotation=rotation)
        session["calibration"].add(names[0], TouchFrame())
        set_buttons(True)
        win.setFocus()

    def end_calibration(message: str) -> None:
        session["calibration"] = None
        monitor.paused.clear()
        set_buttons(False)
        info_label.setText(message)

    def next_direction() -> None:
        calibration = session["calibration"]
        if calibration is None:
            return
        session["step"] += 1
        if session["step"] < len(names):
            calibration.add(names[session["step"]], TouchFrame())  # The direction exists even if nothing is touched
            return
        direction_holes = calibration.direction_holes()
        warnings = [
            f"hole {h} touched for {', '.join(ds)}: most frequent kept" for h, ds in calibration.conflicts().items()
        ]
        warnings += [f"no hole for {name}" for name, hs in direction_holes.items() if not hs]
        backup = save_mapping(HoleMapping(direction_holes, session["rotation"]), cfg.calibration_path, backup=True)
        monitor.set_mapping(direction_holes)
        set_mapping(direction_holes)
        kept = f" (the previous one kept as {backup.name})" if backup is not None else ""
        warned = "".join(f"<br>⚠ {w}" for w in warnings)
        print(f"Hole mapping saved to {cfg.calibration_path}{kept}: {direction_holes}")
        end_calibration(f"Saved to {cfg.calibration_path.name}{kept} and used now.{warned}")

    def redo_direction() -> None:
        if session["calibration"] is not None:
            session["calibration"].counts[names[session["step"]]] = Counter()

    def cancel_calibration() -> None:
        if session["calibration"] is not None:
            end_calibration("Cancelled: the previous mapping is still used.")

    def set_panel(visible: bool) -> None:
        if visible == content.isVisible():
            return
        content.setVisible(visible)
        toggle.setChecked(visible)
        window.resize(window.width() + (1 if visible else -1) * (panel_width + 4), window.height())

    toggle.toggled.connect(set_panel)
    start_button.clicked.connect(start_calibration)
    next_button.clicked.connect(next_direction)
    redo_button.clicked.connect(redo_direction)
    cancel_button.clicked.connect(cancel_calibration)
    shortcuts = []
    for key, action in (
        ("C", lambda: set_panel(not content.isVisible())),
        ("Space", next_direction),
        ("Backspace", redo_direction),
        ("Esc", cancel_calibration),
    ):
        shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), window)
        shortcut.activated.connect(action)
        shortcuts.append(shortcut)

    window.resize(*style.window_size)
    window.show()

    history: deque[tuple[float, tuple[int, ...]]] = deque()  # (t, covered holes) of each result
    trail: deque[tuple[float, np.ndarray]] = deque()
    last = {"frame": None, "n_frames": 0, "n_rate": 0, "t_rate": time.monotonic()}

    def update():
        if monitor.stop_event.is_set():
            app.quit()
            return
        try:
            frame = touch.latest()
        except RuntimeError as e:
            errors.append(e)
            app.quit()
            return
        now = time.monotonic()
        if frame is not None and frame is not last["frame"]:
            last["frame"] = frame
            last["n_frames"] += 1
            history.append((frame.t, tuple(sorted(frame.covered))))
            if session["calibration"] is not None:
                session["calibration"].add(names[session["step"]], frame)
        fresh = frame is not None and now - frame.t <= cfg.touch_timeout_s
        covered = set(frame.covered) if fresh else set()

        # 1. Touches
        while history and now - history[0][0] > history_s:
            history.popleft()
        xs = [t - now for t, hs in history for _ in hs]
        ys = [h for _, hs in history for h in hs]
        brushes = [mapping["brushes"].get(h, mapping["unmapped_brush"]) for _, hs in history for h in hs]
        touch_points.setData(x=xs, y=ys, brush=brushes)
        snapshot = monitor.snapshot()
        events = [(t, kind) for t, kind in snapshot["events"] if now - t <= history_s]
        event_marks.setData(
            x=[t - now for t, _ in events],
            y=[top_hole + 1] * len(events),
            brush=[pg.mkBrush(state_colors[kind]) for _, kind in events],
        )

        # 2. Waveguide
        calibration = session["calibration"]
        if calibration is not None:
            name = names[session["step"]]
            colors, current = [], counted(name)
            done = {h: n for n in names[: session["step"]] for h in counted(n)}
            for hole in hole_ids:
                if hole in current:
                    colors.append(hex_rgb(DIRECTION_COLORS[name]))
                elif hole in done:
                    colors.append(mix(hex_rgb(DIRECTION_COLORS[done[hole]]), 0.45))
                else:
                    colors.append(idle_rgb)
            show_holes(np.round(colors).astype(int), covered)
            set_status(
                f"Touch all the {name} holes, then Space  ({session['step'] + 1}/{len(names)})", DIRECTION_COLORS[name]
            )
            info_label.setText(
                f"<b>{name}</b>: holes {', '.join(f'{a}' if a == b else f'{a}-{b}' for a, b in runs(current)) or '-'}"
                "<br>Space: next direction · Backspace: redo · Esc: cancel"
            )
        else:
            show_holes(mapping_colors(covered), covered)
            state, message = snapshot["state"], snapshot["message"]
            speed = float(np.linalg.norm(snapshot["velocity"]))
            if state == "moving" and speed > 0:
                set_status(f"Moving  ·  {speed * 100:.1f} cm/s", state_colors[state])
            elif state in ("stop", "box", "starting", "nodata"):
                set_status(message.split(" at ")[0], state_colors[state])
            elif not fresh:
                set_status("No touch data", c["muted"])
            elif covered:
                set_status(f"Holes {', '.join(map(str, sorted(covered)))}", c["touch"])
            else:
                set_status("Ready: touch the waveguide", c["muted"])

        # 3. Arm
        position = snapshot["position"]
        if position is not None:
            trail.append((now, position))
            while trail and now - trail[0][0] > trail_s:
                trail.popleft()
            points = np.array([p for _, p in trail])
            tip = position + snapshot["velocity"] * arrow_s
            color = state_colors.get(snapshot["state"], c["signal"])
            for (i, j), trail_curve, velocity_line, dot in views:
                trail_curve.setData(points[:, i], points[:, j])
                if abs(tip[i] - position[i]) + abs(tip[j] - position[j]) > 1e-6:
                    velocity_line.setData([position[i], tip[i]], [position[j], tip[j]], pen=tools.pen(color, width=4))
                    velocity_line.show()
                else:  # A zero-length line is drawn across the whole view
                    velocity_line.hide()
                dot.setData([position[i]], [position[j]])

        if now - last["t_rate"] >= 1.0:
            rate = (last["n_frames"] - last["n_rate"]) / (now - last["t_rate"])
            window.setWindowTitle(f"Waveguide body demo ({rate:.0f} results/s)")
            last["n_rate"], last["t_rate"] = last["n_frames"], now

    signal.signal(signal.SIGINT, lambda *_: app.quit())  # Ctrl+C closes the plot (Python runs between the timer ticks)
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(16)
    app.exec()
    if errors:
        raise errors[0]
