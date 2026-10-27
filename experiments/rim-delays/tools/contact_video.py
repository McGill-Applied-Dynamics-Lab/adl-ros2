r"""Contact video: where the hand, the proxy and the robot are, frame by frame, for a talk clip.

Under delay the operator's hand (the leader), what it pushes against (the proxy; for M1 the robot as
last received, ``x_i``) and the robot are in different places. This draws them, from a run's logs,
in one of three modes:

- ``panel``: an animated panel alone, over run time: a height gauge (each one's height above the
  table, now), their heights over the last ``--window`` seconds, and the felt force against the
  robot's table force.
- ``side``: the camera clip and the panel side by side (``--video``, synced).
- ``overlay``: markers drawn on the clip itself, at each one's position (``--video``, synced,
  ``--calib``), with the table line and a caption.

Sync: ``--offset S`` (the video time at run t = 0), or ``--sync-contact V`` (the video time at which
the robot first touches the table, read off the clip; the robot's contacts in run time are printed).

Overlay calibration, a side view (camera looking along the base y axis): the tool tip's pixel at two
or more video times, ``--calib V,U,PX ...`` (video time [s], pixel column, pixel row); the robot's
logged position at those times gives the mapping. Two points (a tap is vertical) fix the scale from
their height difference, ``--x-sign`` the direction of base +x in the image; three or more points
spread in x fit a full affine map. ``--pick V ...`` shows those frames, you click the tool tip, and
it prints the ``--calib`` argument.

    pixi run -e humble python experiments/rim-delays/tools/contact_video.py RUN --mode panel
    pixi run -e humble python experiments/rim-delays/tools/contact_video.py RUN --mode side --video clip.mp4 --sync-contact 7.4
    pixi run -e humble python experiments/rim-delays/tools/contact_video.py RUN --video clip.mp4 --sync-contact 7.4 --pick 3 9
    pixi run -e humble python experiments/rim-delays/tools/contact_video.py RUN --mode overlay --video clip.mp4 \
        --sync-contact 7.4 --calib 3,812,240 9,815,402

Output: ``<run>/analysis/contact_<mode>.mp4`` (``--out`` to change it), H.264.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

RIM = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(RIM), str(RIM / "analysis")]
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import metrics as M  # noqa: E402
from run_plots import LEADER, PROXY, ROBOT, TARGET  # noqa: E402
from runs import GRID, INK, INK_2, INK_3, LABELS, SURFACE, load_run, surface_of  # noqa: E402

TOUCH_M = 0.002  # the robot touches the table when its tool point is this close to it [m]


# ------------------------------------------------------------------ the signals


class Signals:
    """The run's positions in the base frame [m], interpolated at any run time: ``hand`` (leader),
    ``felt`` (the proxy, or for M1 the robot as last received), ``command`` (the target as it
    reached the robot), ``robot``; the felt force ``f_z`` and the robot's table force ``table``
    (``-F_osc,z``, now) [N]."""

    def __init__(self, run: dict):
        self.run, meta = run, run["meta"]
        self.s = surface_of(meta)
        self.method = meta.get("args", {}).get("method", "?")
        self.proxy = self.method.startswith("proxy")
        B = M.basis(run)
        h, r, tg = run["haptic"], run["robot"], run["target"]
        lift = lambda name: np.stack([h[f"{name}_{k}"].to_numpy(float) for k in range(B.shape[1])], 1) @ B.T  # noqa: E731
        self.th = h.ts.to_numpy(float)
        self.hand = lift("x_l")
        self.felt = lift("proxy_x" if self.proxy else "x_i")
        self.f_z = (lift("f"))[:, 2]
        self.tr = r.ts.to_numpy(float)
        self.robot = r[["x", "y", "z"]].to_numpy(float)
        self.table = -r.fz.to_numpy(float)
        self.tt = tg.ts.to_numpy(float) if tg is not None else np.zeros(0)
        self.command = tg[["position_0", "position_1", "position_2"]].to_numpy(float) if tg is not None else None
        self.names = {"hand": "hand", "felt": "proxy" if self.proxy else "robot, as felt", "command": "command at robot",
                      "robot": "robot"}
        self.colors = {"hand": LEADER, "felt": PROXY, "command": TARGET, "robot": ROBOT}
        self.markers = {"hand": "o", "felt": "D", "command": "^", "robot": "s"}
        d = meta.get("delays") or {}
        rtt = sum((d.get(k) or {}).get("base_ms", 0.0) for k in ("feedback", "command"))
        trace = any((d.get(k) or {}).get("trace") for k in ("feedback", "command")) or meta.get("args", {}).get("delay_trace")
        self.delay = "5G" if trace else f"T = {rtt / 1e3:g} s"
        self.title = f"{LABELS.get(self.method, self.method)} · {self.delay}"

    def at(self, key: str, t) -> np.ndarray:
        """``(len(t), 3)`` position of ``key`` at run times ``t`` (NaN outside its log)."""
        src = {"hand": (self.th, self.hand), "felt": (self.th, self.felt), "robot": (self.tr, self.robot),
               "command": (self.tt, self.command)}[key]
        ts, x = src
        t = np.atleast_1d(np.asarray(t, float))
        if x is None or not len(ts):
            return np.full((len(t), 3), np.nan)
        return np.stack([np.interp(t, ts, x[:, k], left=np.nan, right=np.nan) for k in range(3)], 1)

    def force(self, key: str, t) -> np.ndarray:
        ts, y = (self.th, self.f_z) if key == "felt" else (self.tr, self.table)
        return np.interp(np.atleast_1d(t), ts, y, left=np.nan, right=np.nan)

    def robot_contacts(self, gap_s: float = 0.3) -> np.ndarray:
        """Run times at which the robot's tool point reached the table (within ``TOUCH_M``)."""
        on = self.robot[:, 2] < self.s + TOUCH_M
        t = self.tr[1:][np.diff(on.astype(int)) == 1]
        keep = np.r_[True, np.diff(t) > gap_s] if len(t) else np.zeros(0, bool)
        return t[keep]

    def keys(self, command: bool) -> list[str]:
        return ["hand", "felt"] + (["command"] if command else []) + ["robot"]


# ------------------------------------------------------------------ the panel


class Panel:
    """The animated panel: a height gauge (now), heights and forces over the last ``window`` s."""

    def __init__(self, sig: Signals, height_px: int, window_s: float, command: bool, t_span: tuple[float, float]):
        self.sig, self.window, self.keys = sig, window_s, sig.keys(command)
        dpi = 100
        h_in = max(height_px, 360) / dpi
        self.fig = plt.figure(figsize=(1.25 * h_in, h_in), dpi=dpi)
        gs = self.fig.add_gridspec(2, 2, width_ratios=(1, 2.6), height_ratios=(1.5, 1), left=0.09, right=0.97,
                                   bottom=0.09, top=0.88, wspace=0.28, hspace=0.32)
        self.ax_g = self.fig.add_subplot(gs[:, 0])
        self.ax_z = self.fig.add_subplot(gs[0, 1])
        self.ax_f = self.fig.add_subplot(gs[1, 1], sharex=self.ax_z)
        self.title = self.fig.suptitle("", x=0.03, ha="left", fontsize=13, color=INK)
        # height range [mm above the table], from the whole span
        ts = np.arange(*t_span, 0.01)
        zs = np.concatenate([1e3 * (sig.at(k, ts)[:, 2] - sig.s) for k in self.keys])
        lo, hi = np.nanpercentile(zs, [0.5, 99.5]) if np.isfinite(zs).any() else (-20.0, 50.0)
        lo, hi = min(lo, -5.0), max(hi, 10.0)
        pad = 0.08 * (hi - lo)
        self.zlim = (lo - pad, hi + pad)
        fs = np.concatenate([sig.force("felt", ts), sig.force("robot", ts)])
        flo, fhi = np.nanpercentile(fs, [0.5, 99.5]) if np.isfinite(fs).any() else (-5.0, 20.0)
        fpad = 0.1 * (max(fhi, 1.0) - min(flo, 0.0))
        self.flim = (min(flo, 0.0) - fpad, max(fhi, 1.0) + fpad)
        self._static()
        # the gauge: one lane per signal, a marker at its height now
        self.dots = {}
        for i, k in enumerate(self.keys):
            (self.dots[k],) = self.ax_g.plot([i], [np.nan], self.sig.markers[k], ms=13, color=sig.colors[k],
                                             mec=SURFACE, mew=2, zorder=3)
        self.lines_z = {k: self.ax_z.plot([], [], color=sig.colors[k], lw=2, label=sig.names[k])[0] for k in self.keys}
        self.line_felt = self.ax_f.plot([], [], color=LEADER, lw=2, label="felt f_z")[0]
        self.line_table = self.ax_f.plot([], [], color=ROBOT, lw=2, label="robot force −F_osc,z")[0]
        self.ax_z.legend(loc="upper left", fontsize=8, frameon=False, ncol=2)
        self.ax_f.legend(loc="upper left", fontsize=8, frameon=False)

    def _static(self) -> None:
        g, sig = self.ax_g, self.sig
        g.set_xlim(-0.6, len(self.keys) - 0.4)
        g.set_ylim(*self.zlim)
        g.set_xticks(range(len(self.keys)), [sig.names[k].replace(", ", ",\n").replace(" at ", "\nat ")
                                             for k in self.keys], fontsize=8, color=INK_2)
        g.set_ylabel("height above the table [mm]", color=INK_2)
        g.axhspan(self.zlim[0], 0, color=GRID, zorder=0)
        g.axhline(0, color=INK_3, lw=1.5, zorder=1)
        g.text(-0.5, 0, " table", va="bottom", fontsize=8, color=INK_3)
        for i in range(len(self.keys)):
            g.axvline(i, color=GRID, lw=1, zorder=0)
        for ax in (self.ax_z, self.ax_f):
            ax.grid(True, color=GRID, lw=0.8)
            ax.axhline(0, color=INK_3, lw=1, zorder=1)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
        g.spines[["top", "right"]].set_visible(False)
        self.ax_z.set_ylim(*self.zlim)
        self.ax_z.set_ylabel("height [mm]", color=INK_2)
        self.ax_f.set_ylim(*self.flim)
        self.ax_f.set_ylabel("force [N]", color=INK_2)
        self.ax_f.set_xlabel("run time [s]", color=INK_2)
        self.now = [ax.axvline(0, color=INK_2, lw=1, ls="--") for ax in (self.ax_z, self.ax_f)]

    def frame(self, t: float) -> np.ndarray:
        sig = self.sig
        tw = np.linspace(t - self.window, t, 400)
        for i, k in enumerate(self.keys):
            z = 1e3 * (sig.at(k, tw)[:, 2] - sig.s)
            self.lines_z[k].set_data(tw, z)
            self.dots[k].set_data([i], [z[-1]])
        felt, table = sig.force("felt", tw), sig.force("robot", tw)
        self.line_felt.set_data(tw, felt)
        self.line_table.set_data(tw, table)
        self.ax_z.set_xlim(t - self.window, t + 0.02 * self.window)
        for ln in self.now:
            ln.set_xdata([t, t])
        self.title.set_text(f"{sig.title}    t = {t:5.1f} s    felt {felt[-1]:5.1f} N · robot {table[-1]:5.1f} N")
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3].copy()


# ------------------------------------------------------------------ the overlay


def _hex_bgr(c: str) -> tuple[int, int, int]:
    return int(c[5:7], 16), int(c[3:5], 16), int(c[1:3], 16)


class Calibration:
    """Base-frame (x, z) → pixel (u, v), from the tool tip's pixel at known robot positions."""

    def __init__(self, xz: np.ndarray, uv: np.ndarray, x_sign: float):
        xz, uv = np.asarray(xz, float), np.asarray(uv, float)
        if len(xz) >= 3 and np.ptp(xz[:, 0]) > 0.02:  # spread in x: a full affine map
            A = np.c_[xz, np.ones(len(xz))]
            self.P = np.linalg.lstsq(A, uv, rcond=None)[0].T  # (2, 3)
        else:  # a side view with square pixels: scale from the height difference
            dz = np.ptp(xz[:, 1])
            if len(xz) < 2 or dz < 0.01:
                raise SystemExit("--calib: give at least two points at heights 1 cm or more apart")
            k = np.ptp(uv[:, 1]) / dz  # [px/m]
            x0, z0 = xz.mean(0)
            u0, v0 = uv.mean(0)
            self.P = np.array([[x_sign * k, 0.0, u0 - x_sign * k * x0], [0.0, -k, v0 + k * z0]])
        res = uv - self.px(xz)
        print(f"[contact_video] calibration: {1e-3 * abs(self.P[1, 1]):.2f} px/mm, residual "
              f"{np.sqrt(np.mean(res**2)):.1f} px RMS over {len(xz)} points")

    def px(self, xz: np.ndarray) -> np.ndarray:
        xz = np.atleast_2d(xz)
        return xz @ self.P[:, :2].T + self.P[:, 2]


def draw_overlay(img: np.ndarray, sig: Signals, cal: Calibration, t: float, keys: list[str]) -> np.ndarray:
    import cv2

    out = img.copy()
    h, w = out.shape[:2]
    scale = max(h / 720, 0.6)
    # the table: a line along base x at its height, around the robot
    xr = np.nanmean(sig.robot[:, 0])
    a, b = cal.px(np.array([[xr - 0.25, sig.s], [xr + 0.25, sig.s]]))
    cv2.line(out, tuple(int(v) for v in a), tuple(int(v) for v in b), (200, 200, 200), max(1, int(2 * scale)), cv2.LINE_AA)
    r = int(10 * scale)
    lane = 3.2 * r  # markers side by side at their own heights (a tap is vertical): none hides another
    for i, k in enumerate(keys):
        p = sig.at(k, t)[0]
        if not np.isfinite(p).all():
            continue
        u, v = cal.px(p[[0, 2]])[0]
        _marker(out, (int(round(u + (i - (len(keys) - 1) / 2) * lane)), int(round(v))), sig.markers[k],
                _hex_bgr(sig.colors[k]), r)
    caption = (f"{sig.title.replace(' · ', ' | ')}   t = {t:5.1f} s   felt {sig.force('felt', t)[0]:5.1f} N   "
               f"robot {sig.force('robot', t)[0]:5.1f} N")
    x0, y0 = int(16 * scale), int(36 * scale)
    _text(out, caption, (x0, y0), 0.75 * scale)
    # legend, left to right in the markers' lane order
    for i, k in enumerate(keys):
        y = y0 + int((34 + 30 * i) * scale)
        _marker(out, (x0 + r, y), sig.markers[k], _hex_bgr(sig.colors[k]), int(0.8 * r))
        _text(out, sig.names[k], (x0 + 3 * r, y + int(7 * scale)), 0.6 * scale)
    return out


def _marker(img, center, marker: str, color, r: int) -> None:
    """A filled marker with a white ring: circle, diamond, triangle or square (``o D ^ s``)."""
    import cv2

    u, v = center
    if marker == "o":
        cv2.circle(img, (u, v), r + 2, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (u, v), r, color, -1, cv2.LINE_AA)
        return
    pts = {"D": [(0, -1), (1, 0), (0, 1), (-1, 0)], "^": [(0, -1), (0.9, 0.7), (-0.9, 0.7)],
           "s": [(-0.8, -0.8), (0.8, -0.8), (0.8, 0.8), (-0.8, 0.8)]}[marker]
    for rr, col in ((r + 3, (255, 255, 255)), (r, color)):
        cv2.fillPoly(img, [np.array([(u + rr * dx, v + rr * dy) for dx, dy in pts], np.int32)], col, cv2.LINE_AA)


def _text(img, text, org, fs):
    import cv2

    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fs, (20, 20, 20), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), 1, cv2.LINE_AA)


# ------------------------------------------------------------------ video I/O


class Writer:
    """H.264 through ffmpeg, RGB frames in."""

    def __init__(self, path: Path, size: tuple[int, int], fps: float):
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise SystemExit("ffmpeg not found (run through pixi)")
        self.w, self.h = (s - s % 2 for s in size)
        self.proc = subprocess.Popen(
            [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{self.w}x{self.h}",
             "-r", f"{fps:g}", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(path)],
            stdin=subprocess.PIPE)

    def write(self, rgb: np.ndarray) -> None:
        self.proc.stdin.write(np.ascontiguousarray(rgb[: self.h, : self.w]).tobytes())

    def close(self) -> None:
        self.proc.stdin.close()
        if self.proc.wait():
            raise SystemExit("ffmpeg failed")


def open_video(path: Path):
    import cv2

    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"cannot open {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    return cap, fps, n


def frame_at(path: Path, tv: float) -> np.ndarray:
    import cv2

    cap, fps, _ = open_video(path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(tv * fps)))
    ok, img = cap.read()
    cap.release()
    if not ok:
        raise SystemExit(f"no frame at {tv} s in {path}")
    return img


def pick(path: Path, times: list[float]) -> list[tuple[float, int, int]]:
    """Show the frame at each video time; a click on the tool tip records it (any key: next)."""
    import cv2

    points = []
    for tv in times:
        img = frame_at(path, tv)
        clicked = []

        def on_click(event, u, v, *_):
            if event == cv2.EVENT_LBUTTONDOWN:
                clicked[:] = [(u, v)]
                shown = img.copy()
                cv2.drawMarker(shown, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
                cv2.imshow(win, shown)

        win = f"click the tool tip at {tv:g} s, then any key"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win, on_click)
        cv2.imshow(win, img)
        cv2.waitKey(0)
        cv2.destroyWindow(win)
        if clicked:
            points.append((tv, *clicked[0]))
    return points


# ------------------------------------------------------------------ main


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("run", type=Path)
    p.add_argument("--mode", choices=("panel", "side", "overlay"), default="panel")
    p.add_argument("--video", type=Path, help="the camera clip (side, overlay, --pick)")
    p.add_argument("--offset", type=float, help="video time at run t = 0 [s]")
    p.add_argument("--sync-contact", type=float, help="video time of the robot's first touch of the table [s]")
    p.add_argument("--calib", nargs="+", default=None, help="overlay: V,U,PX per point (video time, pixel column, row)")
    p.add_argument("--x-sign", type=float, default=1.0, help="overlay, two points: +1 if base +x points right in the image")
    p.add_argument("--pick", type=float, nargs="+", help="click the tool tip at these video times, print --calib")
    p.add_argument("--t0", type=float, help="start [run time, s]")
    p.add_argument("--t1", type=float, help="end [run time, s]")
    p.add_argument("--window", type=float, default=5.0, help="history shown in the panel's traces [s]")
    p.add_argument("--command", action="store_true", help="also show the command as it reached the robot")
    p.add_argument("--fps", type=float, default=30.0, help="panel mode frame rate")
    p.add_argument("--height", type=int, default=720, help="panel mode height [px]")
    p.add_argument("--out", type=Path)
    a = p.parse_args(argv)

    run = load_run(a.run.expanduser())
    if run is None or run.get("haptic") is None or run.get("robot") is None:
        raise SystemExit(f"{a.run}: no complete run")
    sig = Signals(run)
    touches = sig.robot_contacts()
    print(f"[contact_video] {sig.title}: robot table contacts at run t = {np.round(touches, 2).tolist()} s")

    offset = a.offset
    if a.sync_contact is not None:
        if not len(touches):
            raise SystemExit("--sync-contact: the robot never touched the table")
        offset = a.sync_contact - float(touches[0])
    needs_video = a.mode != "panel" or a.pick
    if needs_video and (a.video is None or offset is None):
        raise SystemExit("--video and --offset or --sync-contact are needed for side, overlay and --pick")
    if needs_video:
        print(f"[contact_video] video time = run time + {offset:.3f} s")

    if a.pick:
        pts = pick(a.video, a.pick)
        print("--calib " + " ".join(f"{tv:g},{u},{v}" for tv, u, v in pts))
        return

    span = (max(sig.th[0], sig.tr[0]), min(sig.th[-1], sig.tr[-1]))
    t0 = a.t0 if a.t0 is not None else span[0]
    t1 = a.t1 if a.t1 is not None else span[1]
    out = a.out or a.run.expanduser() / "analysis" / f"contact_{a.mode}.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    keys = sig.keys(a.command)

    if a.mode == "panel":
        panel = Panel(sig, a.height, a.window, a.command, (t0, t1))
        times = np.arange(t0, t1, 1.0 / a.fps)
        first = panel.frame(times[0])
        w = Writer(out, (first.shape[1], first.shape[0]), a.fps)
        for k, t in enumerate(times):
            w.write(first if k == 0 else panel.frame(t))
        w.close()
        print(f"[contact_video] {len(times)} frames -> {out}")
        return

    import cv2

    cap, fps, n = open_video(a.video)
    v0 = max(0.0, t0 + offset)
    v1 = min(n / fps, t1 + offset)
    if v1 <= v0:
        raise SystemExit(f"the clip ({n / fps:.1f} s) and the run [{t0:.1f}, {t1:.1f}] s do not overlap with this sync")
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(v0 * fps)))
    cal = None
    if a.mode == "overlay":
        if not a.calib:
            raise SystemExit("--mode overlay needs --calib (get it with --pick)")
        pts = np.array([[float(x) for x in c.split(",")] for c in a.calib])
        rob = np.array([sig.at("robot", tv - offset)[0][[0, 2]] for tv in pts[:, 0]])
        if not np.isfinite(rob).all():
            raise SystemExit(f"--calib: video times {pts[~np.isfinite(rob).all(1), 0].tolist()} are outside the robot log "
                             f"(video {sig.tr[0] + offset:.1f}-{sig.tr[-1] + offset:.1f} s)")
        cal = Calibration(rob, pts[:, 1:], a.x_sign)
    panel, w = None, None
    k0 = int(round(v0 * fps))
    for k in range(k0, int(v1 * fps)):
        ok, img = cap.read()
        if not ok:
            break
        t = k / fps - offset
        if a.mode == "overlay":
            frame = cv2.cvtColor(draw_overlay(img, sig, cal, t, keys), cv2.COLOR_BGR2RGB)
        else:
            if panel is None:
                panel = Panel(sig, img.shape[0], a.window, a.command, (t0, t1))
            right = panel.frame(t)
            right = cv2.resize(right, (int(right.shape[1] * img.shape[0] / right.shape[0]), img.shape[0]))
            frame = np.hstack([cv2.cvtColor(img, cv2.COLOR_BGR2RGB), right])
        if w is None:
            w = Writer(out, (frame.shape[1], frame.shape[0]), fps)
        w.write(frame)
    cap.release()
    if w is not None:
        w.close()
    print(f"[contact_video] {a.mode} -> {out}")


if __name__ == "__main__":
    main()
