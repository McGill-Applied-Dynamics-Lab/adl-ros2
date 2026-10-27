"""Operator guide: a reference height to follow with the handle, in a window next to the run.

For hand-in-the-loop contact runs (E6). The operator sees only the local, undelayed state: the
table, the leader (the hand) and, for a proxy method, the proxy; and a reference height that
scrolls in from the right, so the next seconds are visible ahead of time. Every method and delay
then gets the same motion: same contact times, speeds and depth. In steady contact the felt force
is about K × depth for every method, so the reference depth also sets the force.

[Guide][] is the reference, a function of run time (``time.monotonic() - t0``, the haptic log's
clock): a lead-in at the hover height, then per contact of ``pattern`` a descent below the table,
a hold (``hold``) or none (``tap``), a lift back to the hover height and a pause. It is rebuilt
offline from the run's ``args`` ([Guide.from_args][]).

[GuideWindow][] draws it in its own interpreter (OpenCV), reading the haptic loop's
``HapticStateMailbox``; it waits for the run start (``t0``) and closes at the end.
"""

from __future__ import annotations

import base64
import math
import os
import pickle
import subprocess
import sys
import time
from dataclasses import asdict, dataclass

import numpy as np

KINDS = ("tap", "hold")


@dataclass(frozen=True)
class GuideConfig:
    """The reference profile. Heights are relative to the table [m], speeds [m/s], times [s]."""

    pattern: tuple[str, ...] = ("tap", "hold", "tap", "hold", "tap", "hold")
    hover: float = 0.03  # height between contacts
    depth: float = 0.01  # below the table at the bottom of a contact
    hold_s: float = 4.0  # at depth, for a hold
    tap_speed: float = 0.08  # descent of a tap (an Inverse3 hand follows ~8 cm/s; 20 was too fast)
    tap_dwell_s: float = 0.3  # at depth, for a tap: a touch, short enough to stay a tap
    press_speed: float = 0.05  # descent of a hold
    lift_speed: float = 0.10
    pause_s: float = 2.0  # at the hover height after each contact
    lead_s: float = 3.0  # at the hover height before the first contact

    def __post_init__(self) -> None:
        bad = [k for k in self.pattern if k not in KINDS]
        if bad:
            raise ValueError(f"guide pattern: {bad} not in {KINDS}")


class Guide:
    """The reference height ``z(t)`` [m, base frame] and its contacts, from a [GuideConfig][]."""

    def __init__(self, cfg: GuideConfig, surface: float):
        self.cfg, self.surface = cfg, float(surface)
        top, bottom = surface + cfg.hover, surface - cfg.depth
        t, knots = cfg.lead_s, [(0.0, top), (cfg.lead_s, top)]
        self.contacts: list[dict] = []
        for kind in cfg.pattern:
            speed = cfg.tap_speed if kind == "tap" else cfg.press_speed
            t_down = t
            t += (top - bottom) / speed
            knots.append((t, bottom))
            t_bottom = t
            t += cfg.hold_s if kind == "hold" else cfg.tap_dwell_s
            knots.append((t, bottom))
            t_up = t
            t += (top - bottom) / cfg.lift_speed
            knots.append((t, top))
            self.contacts.append({"kind": kind, "t_down": t_down, "t_cross": t_down + cfg.hover / speed,
                                  "t_bottom": t_bottom, "t_up": t_up, "t_end": t})
            t += cfg.pause_s
            knots.append((t, top))
        self._t, self._z = np.array(knots).T
        self.duration_s = float(t)

    @classmethod
    def from_args(cls, args) -> Guide | None:
        """The guide of a run's arguments (an ``argparse.Namespace`` or the metadata's ``args``
        dict), ``None`` without ``--guide``."""
        a = args if isinstance(args, dict) else vars(args)
        if not a.get("guide") or a.get("contact_surface") is None:
            return None
        pattern = tuple(k.strip() for k in str(a["guide"]).split(",") if k.strip())
        # a run from before a flag existed used that flag's value of the time (taps 20 cm/s, no dwell)
        cfg = GuideConfig(pattern=pattern, hold_s=float(a.get("guide_hold_s", GuideConfig.hold_s)),
                          depth=1e-3 * float(a.get("guide_depth_mm", 1e3 * GuideConfig.depth)),
                          hover=1e-3 * float(a.get("guide_hover_mm", 1e3 * GuideConfig.hover)),
                          tap_speed=1e-2 * float(a.get("guide_tap_cms", 20.0)),
                          tap_dwell_s=float(a.get("guide_tap_dwell_s", 0.0)))
        return cls(cfg, float(a["contact_surface"]))

    def z(self, t) -> np.ndarray:
        """Reference height at run times ``t`` (the hover height outside the profile)."""
        return np.interp(t, self._t, self._z)

    def contact_at(self, t: float) -> tuple[int, dict] | None:
        """The contact under way or next at ``t`` (its index and entry), ``None`` after the last."""
        for i, c in enumerate(self.contacts):
            if t < c["t_end"]:
                return i, c
        return None

    def describe(self) -> dict:
        return {**asdict(self.cfg), "surface": self.surface, "duration_s": self.duration_s}


# ------------------------------------------------------------------ the window

# the categorical role colors of the run figures (analysis/run_plots.py), BGR
LEADER, PROXY, REFERENCE = (214, 120, 42), (52, 104, 235), (122, 175, 27)
INK, INK_2, INK_3, GRID, SURFACE = (11, 11, 11), (78, 81, 82), (132, 137, 138), (224, 229, 230), (251, 252, 252)


@dataclass(frozen=True)
class WindowConfig:
    past_s: float = 1.5  # history shown left of "now"
    future_s: float = 3.5  # reference shown ahead
    width: int = 1100
    height: int = 600
    fps: float = 30.0  # the reference moves a few cm/s: 30 fps is smooth, and half the load of 60
    margin_mm: float = 15.0  # above the hover height and below the depth


class GuideWindow:
    """The guide in its own interpreter (as ``HapticProcess``: its own resource tracker, nothing of
    this process in it): ``start()`` before the operator's prompt, ``begin(t0)`` at the run start
    (``time.monotonic()``), ``stop()`` at the end. Its stdin is the channel: the pickled spec, then
    the ``t0`` line; closing it ends the window."""

    def __init__(self, guide: Guide, haptic_state, basis: np.ndarray, show_proxy: bool, title: str,
                 cfg: WindowConfig | None = None):
        self._spec = (guide.cfg, guide.surface, haptic_state, np.asarray(basis, float), show_proxy, title,
                      cfg or WindowConfig())
        self._proc: subprocess.Popen | None = None

    def start(self) -> None:
        self._proc = subprocess.Popen([sys.executable, "-c", "from fr3_haptic.guide import main; main()"],
                                      stdin=subprocess.PIPE)
        self._send(base64.b64encode(pickle.dumps(self._spec)) + b"\n")

    def begin(self, t0: float) -> None:
        self._send(f"{t0!r}\n".encode())

    def stop(self) -> None:
        if self._proc is None:
            return
        try:
            self._proc.stdin.close()
        except OSError:
            pass
        try:
            self._proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._proc.terminate()

    def _send(self, line: bytes) -> None:
        try:
            self._proc.stdin.write(line)
            self._proc.stdin.flush()
        except OSError:  # the window was closed: the run goes on without it
            pass


def main() -> None:
    """The guide window's process: reads its spec and ``t0`` from stdin (see [GuideWindow][])."""
    import threading

    import cv2

    gcfg, surface, haptic_state, basis, show_proxy, title, wcfg = pickle.loads(
        base64.b64decode(sys.stdin.buffer.readline()))
    t0 = [math.nan]
    stop = threading.Event()

    def control():
        for line in sys.stdin.buffer:
            if line.strip():
                t0[0] = float(line)
        stop.set()  # stdin closed: the run is over

    threading.Thread(target=control, daemon=True).start()
    # stay out of the haptic loop's way (default scheduling, same machine): OpenCV's thread pool woke
    # every core per drawing call, 9 % of 1 kHz ticks late instead of 6 % (p99 7.8 vs 4.2 ms)
    cv2.setNumThreads(1)
    os.nice(10)
    guide = Guide(gcfg, surface)
    name = f"guide: {title}"
    cv2.namedWindow(name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(name, wcfg.width, wcfg.height)
    hist_t, hist_l, hist_p = [], [], []
    period = 1.0 / wcfg.fps
    while not stop.is_set():
        tick = time.monotonic()
        started = math.isfinite(t0[0])
        t = tick - t0[0] if started else 0.0
        state = haptic_state.read() if started else None
        z_l = z_p = math.nan
        if state is not None:
            z_l = float((basis @ np.asarray(state["x_l"], float))[2])
            if show_proxy:
                z_p = float((basis @ np.asarray(state["proxy_x"], float))[2])
            hist_t.append(t)
            hist_l.append(z_l)
            hist_p.append(z_p)
            while hist_t and hist_t[0] < t - wcfg.past_s:
                hist_t.pop(0)
                hist_l.pop(0)
                hist_p.pop(0)
        cv2.imshow(name, _draw(guide, wcfg, t, started, hist_t, hist_l, hist_p, z_l, z_p, show_proxy, title))
        cv2.waitKey(max(1, int(1e3 * (period - (time.monotonic() - tick)))))
        if cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1:  # closed by hand
            break
    cv2.destroyAllWindows()


def _draw(guide, w, t, started, hist_t, hist_l, hist_p, z_l, z_p, show_proxy, title) -> np.ndarray:
    import cv2

    W, H = w.width, w.height
    img = np.full((H, W, 3), SURFACE, np.uint8)
    top_px, bottom_px, left_px, right_px = 90, H - 40, 70, W - 30
    lo = -1e3 * guide.cfg.depth - w.margin_mm
    hi = 1e3 * guide.cfg.hover + w.margin_mm
    t_lo, t_hi = t - w.past_s, t + w.future_s

    def px(tt, z):  # run time [s], height [m, base] -> pixel
        u = left_px + (np.asarray(tt) - t_lo) / (t_hi - t_lo) * (right_px - left_px)
        v = bottom_px - (1e3 * (np.asarray(z) - guide.surface) - lo) / (hi - lo) * (bottom_px - top_px)
        return u, v

    aa = cv2.LINE_AA
    # the table: below it shaded
    _, v_table = px(t, guide.surface)
    cv2.rectangle(img, (left_px, int(v_table)), (right_px, bottom_px), GRID, -1)
    cv2.line(img, (left_px, int(v_table)), (right_px, int(v_table)), INK_3, 2, aa)
    _put(img, "table", (left_px + 6, int(v_table) + 20), 0.55, INK_2)
    # height axis
    for mm in range(int(math.ceil(lo / 10) * 10), int(hi) + 1, 10):
        _, v = px(t, guide.surface + 1e-3 * mm)
        cv2.line(img, (left_px - 6, int(v)), (left_px, int(v)), INK_3, 1, aa)
        _put(img, f"{mm:+d}" if mm else "0", (8, int(v) + 5), 0.5, INK_2)
    _put(img, "mm", (8, top_px - 12), 0.5, INK_2)
    # the reference: past thin, ahead thick
    tt = np.linspace(t_lo, t_hi, 400)
    u, v = px(tt, guide.z(tt))
    pts = np.stack([u, v], 1).astype(np.int32)
    ahead = tt >= t
    cv2.polylines(img, [pts[~ahead]], False, REFERENCE, 2, aa)
    cv2.polylines(img, [pts[ahead]], False, REFERENCE, 5, aa)
    # labels over the contacts ahead
    for i, c in enumerate(guide.contacts):
        if t_lo <= c["t_down"] <= t_hi or t_lo <= c["t_up"] <= t_hi:
            label = "TAP" if c["kind"] == "tap" else f"HOLD {guide.cfg.hold_s:g} s"
            u0, v0 = px(max(c["t_down"], t_lo), guide.surface + guide.cfg.hover)
            _put(img, f"{i + 1}. {label}", (int(u0), int(v0) - 16), 0.6, INK)
    # now
    u_now, _ = px(t, 0.0)
    cv2.line(img, (int(u_now), top_px - 30), (int(u_now), bottom_px), INK_3, 1, aa)
    v_ref = px(t, guide.z(t))[1]
    cv2.circle(img, (int(u_now), int(v_ref)), 11, REFERENCE, 2, aa)  # the target: put the hand in the ring
    # the hand and the proxy: their last past_s, a marker now
    for hist, z_now, color, marker in ((hist_p, z_p, PROXY, "D"), (hist_l, z_l, LEADER, "o")):
        if hist is hist_p and not show_proxy:
            continue
        if len(hist_t) > 1:
            hu, hv = px(np.array(hist_t), np.array(hist))
            ok = np.isfinite(hv)
            cv2.polylines(img, [np.stack([hu[ok], hv[ok]], 1).astype(np.int32)], False, color, 3, aa)
        if math.isfinite(z_now):
            _marker(img, (int(u_now), int(px(t, z_now)[1])), marker, color, 9)
    # header: what to do now
    if not started:
        head = "waiting for Enter in the terminal: hand at the hover height (+%g mm)" % (1e3 * guide.cfg.hover)
    elif t > guide.duration_s:
        head = "done: hold the hand still until the run ends"
    else:
        at = guide.contact_at(t)
        if at is None:
            head = "done"
        else:
            i, c = at
            kind = "TAP" if c["kind"] == "tap" else "HOLD"
            head = (f"{i + 1}/{len(guide.contacts)} {kind}: down in {c['t_down'] - t:.1f} s" if t < c["t_down"]
                    else f"{i + 1}/{len(guide.contacts)} {kind}")
    _put(img, head, (left_px, 34), 0.8, INK)
    _put(img, title, (left_px, 62), 0.55, INK_2)
    # legend
    items = (("reference", REFERENCE, None), ("hand", LEADER, "o")) + ((("proxy", PROXY, "D"),) if show_proxy else ())
    widths = [28 + cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0][0] + 24 for label, _, _ in items]
    cx = right_px - sum(widths)
    for (label, color, marker), width in zip(items, widths):
        if marker is None:
            cv2.line(img, (cx, 56), (cx + 22, 56), color, 5, aa)
        else:
            _marker(img, (cx + 11, 56), marker, color, 7)
        _put(img, label, (cx + 28, 62), 0.55, INK_2)
        cx += width
    return img


def _marker(img, center, marker: str, color, r: int) -> None:
    import cv2

    u, v = center
    if marker == "o":
        cv2.circle(img, (u, v), r + 2, SURFACE, -1, cv2.LINE_AA)
        cv2.circle(img, (u, v), r, color, -1, cv2.LINE_AA)
        return
    for rr, col in ((r + 3, SURFACE), (r, color)):
        pts = np.array([(u, v - rr), (u + rr, v), (u, v + rr), (u - rr, v)], np.int32)
        cv2.fillPoly(img, [pts], col, cv2.LINE_AA)


def _put(img, text, org, fs, color) -> None:
    import cv2

    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fs, color, 1, cv2.LINE_AA)
