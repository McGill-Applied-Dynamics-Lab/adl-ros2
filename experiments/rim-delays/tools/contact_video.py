r"""Contact video: where the hand, the proxy and the robot are, frame by frame, for a talk clip.

Under delay the operator's hand (the leader), what it pushes against (the proxy; for M1 the robot as
last received, ``x_i``) and the robot are in different places. This draws them, from a run's logs,
in one of three modes:

- ``panel``: an animated panel alone, over run time: a height gauge (each one's height above the
  table, now), their heights over the last ``--window`` seconds, and the felt force against the
  robot's table force.
- ``side``: the camera clip and the panel side by side (``--video``, synced).
- ``overlay``: drawn on the clip itself (``--video``, synced, ``--calib``): the hand to the left of
  the tool and what it pulls on (the proxy; M1: the robot as felt) to its right, each at its own
  height with a level line to the tool, the table plane, and a card with the method, the delay and
  the felt against the robot's force. The robot is the video.

Several runs on one clip (one long take): give them all; each gets its own sync, and one output per
run (``<clip>_<method>_<delay>_<mode>.mp4``, ``--pad`` seconds around it) next to the clip, or in
``--out`` (a directory).

Sync, per run: ``--offset S ...`` (the video time at run t = 0, one per run), ``--sync-audio`` (the
impact clicks in the clip's sound, matched to the robot's table contacts, searched ``--audio-window``
around a guess from the clocks: the clip's creation time and length, the run's start; minus
``--audio-lead``, the sound running ahead of the picture), or
``--sync-contact V`` (one run: the video time at which the robot first touches the table, read off
the clip; the robot's contacts in run time are printed).

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
    pixi run -e humble python experiments/rim-delays/tools/contact_video.py RUN1 RUN2 RUN3 --mode overlay \
        --video long.mp4 --sync-audio --calib ...

Output: one run: ``<run>/analysis/contact_<mode>.mp4`` (``--out`` to change it); several: see above. H.264.
"""

from __future__ import annotations

import argparse
import datetime
import json
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
REFERENCE = "#1baf7a"  # the guide's reference: the categorical slot after the leader's and the proxy's
TRAIL_S = 1.5  # overlay: how long the reference and hand trails last before fading out [s]
TALK_NAMES = {"zoh": "Position-position (no model)", "tdpa-zoh": "TDPA", "proxy-god-object": "God-object",
              "proxy-fixed-mass": "Fixed-mass proxy", "proxy-rim": "RIM (ours)"}  # overlay card, ASCII


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
        self.talk_name = TALK_NAMES.get(self.method, self.method)
        self.talk_delay = "5G network" if trace else ("no delay" if rtt == 0 else f"{rtt / 1e3:g} s round-trip delay")
        self.span = (max(self.th[0], self.tr[0]), min(self.th[-1], self.tr[-1]))
        self.label = f"{self.method}_{'5g' if trace else f'rtt{rtt:g}'}"
        meta_args = meta.get("args", {})
        from fr3_haptic.guide import Guide

        self.guide = Guide.from_args(meta_args)  # the reference the operator followed (fr3_teleop --guide), or None
        self.t0_wall = meta.get("t0_wall_s")  # the run start, wall clock [s since the epoch]; older runs: estimated
        if self.t0_wall is None and meta.get("created_at"):
            saved = datetime.datetime.fromisoformat(meta["created_at"]).timestamp()
            self.t0_wall = saved - float(meta_args.get("seconds", self.span[1])) - 1.0  # metadata written ~1 s after

    def at(self, key: str, t) -> np.ndarray:
        """``(len(t), 3)`` position of ``key`` at run times ``t`` (NaN outside its log)."""
        src = {"hand": (self.th, self.hand), "felt": (self.th, self.felt), "robot": (self.tr, self.robot),
               "command": (self.tt, self.command)}[key]
        ts, x = src
        t = np.atleast_1d(np.asarray(t, float))
        if x is None or not len(ts):
            return np.full((len(t), 3), np.nan)
        return np.stack([np.interp(t, ts, x[:, k], left=np.nan, right=np.nan) for k in range(3)], 1)

    def reference(self, t) -> np.ndarray:
        """The guide's reference height [m, base] at run times ``t``; NaN without a guide or outside the run."""
        t = np.atleast_1d(np.asarray(t, float))
        if self.guide is None:
            return np.full(len(t), np.nan)
        return np.where((t >= self.span[0]) & (t <= self.span[1]), self.guide.z(t), np.nan)

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


def draw_overlay(img: np.ndarray, sig: Signals, cal: Calibration, t: float, keys: list[str],
                 f_max: float, reference: bool = True) -> np.ndarray:
    """The hand left of the tool and what it pulls on right of it (``keys`` without the robot: it is
    the video), each at its own height with a level line to the tool; the table plane; a card with
    the method, the delay and the felt against the robot's force (bars to ``f_max``)."""
    import cv2

    out = img.copy()
    h, w = out.shape[:2]
    sc = h / 1080
    aa = cv2.LINE_AA
    robot = sig.at("robot", t)[0]
    if np.isfinite(robot).all():
        u_tool, _ = cal.px(robot[[0, 2]])[0]
    else:
        u_tool = cal.px(np.array([np.nanmean(sig.robot[:, 0]), sig.s]))[0][0]
    # the table plane the model uses, around the tool
    a, b = cal.px(np.array([[np.nanmean(sig.robot[:, 0]) - 0.12, sig.s], [np.nanmean(sig.robot[:, 0]) + 0.12, sig.s]]))
    _dashed(out, a, b, (235, 235, 235), max(1, int(2 * sc)), int(14 * sc))
    gap, r = 0.075 * w, int(14 * sc)
    sides = {"hand": -1, "felt": +1, "command": +2}
    show_ref = reference and sig.guide is not None
    if show_ref:  # what the hand was asked to do, in the hand's lane: a ring now, and fading trails
        _trails(out, sig, cal, t, u_tool - gap, 0.2 * w, sc)
        z_ref = float(sig.reference(t)[0])
        if np.isfinite(z_ref):
            _, v_ref = cal.px(np.array([0.0, z_ref]))[0]
            ring = (int(round(u_tool - gap)), int(round(v_ref)))
            _blend_circle(out, ring, int(r * 1.7), _hex_bgr(REFERENCE), 0.35)
            cv2.circle(out, ring, int(r * 1.7), _hex_bgr(REFERENCE), max(2, int(3 * sc)), aa)
    for k in keys:
        if k == "robot":
            continue
        p = sig.at(k, t)[0]
        if not np.isfinite(p).all():
            continue
        side = sides[k]
        _, v = cal.px(p[[0, 2]])[0]
        u = u_tool + np.sign(side) * gap * abs(side)
        color = _hex_bgr(sig.colors[k])
        tip = u_tool - np.sign(side) * 0.012 * w  # stop the level line short of the tool
        cv2.line(out, (int(u), int(v)), (int(tip), int(v)), (255, 255, 255), int(5 * sc), aa)
        cv2.line(out, (int(u), int(v)), (int(tip), int(v)), color, int(3 * sc), aa)
        _marker(out, (int(round(u)), int(round(v))), sig.markers[k], color, r)
        if show_ref and side < 0:  # the trails run there; the card's legend names it
            continue
        label = sig.names[k]
        fs = 0.8 * sc
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)
        x0 = int(u - r - 12 * sc - tw) if side < 0 else int(u + r + 12 * sc)
        _text(out, label, (x0, int(v + th / 2)), fs)
    _card(out, sig, t, f_max, sc, keys, show_ref)
    return out


def _trails(img, sig: Signals, cal: Calibration, t: float, u0: float, length: float, sc: float) -> None:
    """The reference's and the hand's last ``TRAIL_S``, drifting left from ``u0`` over ``length``
    pixels and fading out: the reference thicker and fainter, the hand on top."""
    import cv2

    taus = np.linspace(0.0, TRAIL_S, 46)
    us = u0 - length * taus / TRAIL_S
    alpha = (1.0 - taus / TRAIL_S) ** 1.5
    z_ref = sig.reference(t - taus)
    z_hand = sig.at("hand", t - taus)[:, 2]
    for z, color, thick, peak in ((z_ref, REFERENCE, 7, 0.55), (z_hand, LEADER, 3, 0.8)):
        vs = np.array([cal.px(np.array([0.0, zz]))[0][1] if np.isfinite(zz) else np.nan for zz in z])
        layer, weight = np.zeros_like(img), np.zeros(img.shape[:2], np.float32)
        for k in range(len(taus) - 1, 0, -1):  # oldest first, the newest on top
            if not (np.isfinite(vs[k]) and np.isfinite(vs[k - 1])):
                continue
            p, q = (int(us[k]), int(vs[k])), (int(us[k - 1]), int(vs[k - 1]))
            cv2.line(layer, p, q, _hex_bgr(color), int(thick * sc), cv2.LINE_AA)
            cv2.line(weight, p, q, float(peak * alpha[k - 1]), int(thick * sc), cv2.LINE_AA)
        wgt = weight[..., None]
        np.copyto(img, (img * (1.0 - wgt) + layer * wgt).astype(np.uint8), where=wgt > 0)


def _blend_circle(img, center, r: int, color, alpha: float) -> None:
    import cv2

    x0, y0 = max(center[0] - r - 2, 0), max(center[1] - r - 2, 0)
    x1, y1 = min(center[0] + r + 3, img.shape[1]), min(center[1] + r + 3, img.shape[0])
    if x1 <= x0 or y1 <= y0:
        return
    roi = img[y0:y1, x0:x1]
    disk = roi.copy()
    cv2.circle(disk, (center[0] - x0, center[1] - y0), r, color, -1, cv2.LINE_AA)
    roi[:] = (roi * (1 - alpha) + disk * alpha).astype(np.uint8)


def _card(img, sig: Signals, t: float, f_max: float, sc: float, keys: list[str], show_ref: bool) -> None:
    """Top left: the method, the delay, the felt and robot forces as bars, and the legend."""
    import cv2

    x0, y0, wd, ht = int(30 * sc), int(30 * sc), int(560 * sc), int(300 * sc)
    patch = img[y0:y0 + ht, x0:x0 + wd]
    patch[:] = (0.35 * patch + 0.65 * np.array([25, 25, 25])).astype(np.uint8)
    _text(img, sig.talk_name, (x0 + int(20 * sc), y0 + int(50 * sc)), 1.15 * sc, thick=2)
    _text(img, sig.talk_delay, (x0 + int(20 * sc), y0 + int(95 * sc)), 0.9 * sc)
    bar_x, bar_w = x0 + int(150 * sc), int(300 * sc)
    for i, (name, key, color) in enumerate((("felt", "felt", LEADER), ("robot", "robot", ROBOT))):
        f = float(sig.force(key, t)[0])
        y = y0 + int((140 + 52 * i) * sc)
        _text(img, name, (x0 + int(20 * sc), y + int(12 * sc)), 0.8 * sc)
        cv2.rectangle(img, (bar_x, y - int(14 * sc)), (bar_x + bar_w, y + int(14 * sc)), (70, 70, 70), -1)
        if np.isfinite(f):
            n = int(bar_w * np.clip(abs(f) / f_max, 0.0, 1.0))
            cv2.rectangle(img, (bar_x, y - int(14 * sc)), (bar_x + n, y + int(14 * sc)), _hex_bgr(color), -1)
        _text(img, f"{f:4.1f} N" if np.isfinite(f) else "  -", (bar_x + bar_w + int(12 * sc), y + int(12 * sc)), 0.8 * sc)
    # legend: the reference (a ring), then the markers drawn
    y, x = y0 + int(262 * sc), x0 + int(20 * sc)
    items = ([("reference", None)] if show_ref else []) + [(k, k) for k in keys if k not in ("robot",)]
    for label, k in items:
        if k is None:
            cv2.circle(img, (x + int(10 * sc), y - int(7 * sc)), int(10 * sc), _hex_bgr(REFERENCE), max(2, int(3 * sc)),
                       cv2.LINE_AA)
            name = "reference"
        else:
            _marker(img, (x + int(10 * sc), y - int(7 * sc)), sig.markers[k], _hex_bgr(sig.colors[k]), int(9 * sc))
            name = sig.names[k]
        _text(img, name, (x + int(28 * sc), y), 0.65 * sc)
        x += int(28 * sc) + cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.65 * sc, 1)[0][0] + int(26 * sc)


def _dashed(img, a, b, color, thick: int, dash: int) -> None:
    import cv2

    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.linalg.norm(b - a) / dash))
    for i in range(0, n, 2):
        p, q = a + (b - a) * i / n, a + (b - a) * min(i + 1, n) / n
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, thick, cv2.LINE_AA)


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


def _text(img, text, org, fs, thick: int = 1):
    import cv2

    org = (int(org[0]), int(org[1]))
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fs, (20, 20, 20), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), thick, cv2.LINE_AA)


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


# ------------------------------------------------------------------ sync


def clip_start_wall(path: Path) -> float | None:
    """The clip's first frame, wall clock [s since the epoch]: its creation time (a phone stamps the
    end of the recording) minus its length; ``None`` without a creation time."""
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        return None
    info = json.loads(subprocess.run([ffprobe, "-v", "error", "-show_entries", "format=duration:format_tags=creation_time",
                                      "-of", "json", str(path)], capture_output=True, text=True).stdout or "{}")
    fmt = info.get("format", {})
    created = (fmt.get("tags") or {}).get("creation_time")
    if not created:
        return None
    end = datetime.datetime.fromisoformat(created.replace("Z", "+00:00")).timestamp()
    return end - float(fmt.get("duration", 0.0))


def impact_clicks(path: Path) -> np.ndarray:
    """Times of the clip's impact clicks [s]: peaks of its sound above 300 Hz (10 ms envelope),
    louder than 6 × the median, 0.25 s apart at least."""
    from scipy.io import wavfile
    from scipy.signal import butter, filtfilt, find_peaks

    ffmpeg = shutil.which("ffmpeg")
    wav = subprocess.run([ffmpeg, "-v", "error", "-i", str(path), "-vn", "-ac", "1", "-ar", "8000", "-f", "wav", "-"],
                         capture_output=True).stdout
    import io
    import warnings

    with warnings.catch_warnings():  # a piped WAV has no length in its header: harmless
        warnings.simplefilter("ignore")
        fs, a = wavfile.read(io.BytesIO(wav))
    a = a.astype(float) / max(np.abs(a).max(), 1.0)
    b, c = butter(4, 300 / (fs / 2), "high")
    env = np.convolve(np.abs(filtfilt(b, c, a)), np.ones(int(0.01 * fs)) / int(0.01 * fs), "same")
    peaks, _ = find_peaks(env, height=6 * np.median(env), distance=int(0.25 * fs))
    return peaks / fs


def audio_offset(contacts: np.ndarray, clicks: np.ndarray, guess: float, window: float, tol: float = 0.06):
    """The offset (video time - run time) that puts the most robot contacts on a click (within
    ``tol``), then the smallest gaps, searched ``guess ± window``; ``(offset, matched)``."""
    best = (-1, 0.0, guess)
    for off in np.arange(guess - window, guess + window, 0.005):
        gaps = np.array([np.min(np.abs(clicks - (c + off))) for c in contacts]) if len(clicks) else np.full(len(contacts), 1.0)
        score = (int(np.sum(gaps < tol)), -float(np.mean(np.minimum(gaps, tol))), off)
        if score[:2] > best[:2]:
            best = score
    return best[2], best[0]


# ------------------------------------------------------------------ main


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("runs", type=Path, nargs="+", help="the run(s); several: on one clip, one output each")
    p.add_argument("--mode", choices=("panel", "side", "overlay"), default="panel")
    p.add_argument("--video", type=Path, help="the camera clip (side, overlay, --pick)")
    p.add_argument("--offset", type=float, nargs="+", help="video time at run t = 0 [s], one per run")
    p.add_argument("--sync-audio", action="store_true", help="sync each run on the clip's impact clicks")
    p.add_argument("--audio-window", type=float, default=15.0, help="--sync-audio: search ± this around the clocks' guess [s]")
    p.add_argument("--audio-lead", type=float, default=0.16,
                   help="--sync-audio: how far the clip's sound runs ahead of its picture [s] (the phone: 0.14-0.19 s, "
                        "from the tool's motion against the robot's log on two clips)")
    p.add_argument("--sync-contact", type=float, help="one run: video time of the robot's first touch of the table [s]")
    p.add_argument("--calib", nargs="+", default=None, help="overlay: V,U,PX per point (video time, pixel column, row)")
    p.add_argument("--x-sign", type=float, default=1.0, help="overlay, two points: +1 if base +x points right in the image")
    p.add_argument("--pick", type=float, nargs="+", help="click the tool tip at these video times, print --calib")
    p.add_argument("--t0", type=float, help="one run: start [run time, s]")
    p.add_argument("--t1", type=float, help="one run: end [run time, s]")
    p.add_argument("--pad", type=float, default=2.0, help="video kept before and after each run [s]")
    p.add_argument("--window", type=float, default=5.0, help="history shown in the panel's traces [s]")
    p.add_argument("--command", action="store_true", help="also show the command as it reached the robot")
    p.add_argument("--no-reference", action="store_true", help="overlay: leave out the guide's reference and the trails")
    p.add_argument("--fps", type=float, default=30.0, help="panel mode frame rate")
    p.add_argument("--height", type=int, default=720, help="panel mode height [px]")
    p.add_argument("--out", type=Path, help="one run: the output file; several: the output directory")
    a = p.parse_args(argv)

    sigs = []
    for path in a.runs:
        run = load_run(path.expanduser())
        if run is None or run.get("haptic") is None or run.get("robot") is None:
            raise SystemExit(f"{path}: no complete run")
        sigs.append(Signals(run))
    for sig in sigs:
        print(f"[contact_video] {sig.title}: robot table contacts at run t = {np.round(sig.robot_contacts(), 2).tolist()} s")
    single = len(sigs) == 1

    # sync: one offset per run (video time = run time + offset)
    offsets: list[float | None] = [None] * len(sigs)
    if a.offset is not None:
        if len(a.offset) != len(sigs):
            raise SystemExit(f"--offset: one per run ({len(sigs)}), got {len(a.offset)}")
        offsets = list(a.offset)
    elif a.sync_contact is not None:
        if not single:
            raise SystemExit("--sync-contact takes one run; use --offset or --sync-audio for several")
        touches = sigs[0].robot_contacts()
        if not len(touches):
            raise SystemExit("--sync-contact: the robot never touched the table")
        offsets = [a.sync_contact - float(touches[0])]
    elif a.sync_audio:
        if a.video is None:
            raise SystemExit("--sync-audio needs --video")
        clicks, start = impact_clicks(a.video), clip_start_wall(a.video)
        print(f"[contact_video] {len(clicks)} impact clicks in the clip")
        for i, sig in enumerate(sigs):
            if start is None or sig.t0_wall is None:
                raise SystemExit("--sync-audio: no clock to start from (clip creation time, run start); use --offset")
            guess = sig.t0_wall - start
            touches = sig.robot_contacts()
            offsets[i], matched = audio_offset(touches, clicks, guess, a.audio_window)
            offsets[i] -= a.audio_lead  # on the clicks, the picture is this much later
            print(f"[contact_video] {sig.label}: offset {offsets[i]:.3f} s (clocks said {guess:.1f}), "
                  f"{matched}/{len(touches)} contacts on a click")
            if matched < max(2, (len(touches) + 1) // 2):  # quiet first impacts: more than half, two at least
                raise SystemExit(f"--sync-audio: {sig.label} matched too few contacts; give --offset")
    needs_video = a.mode != "panel" or a.pick
    if needs_video and (a.video is None or any(o is None for o in offsets)):
        raise SystemExit("--video and a sync (--offset, --sync-audio or --sync-contact) are needed for side, overlay "
                         "and --pick")
    for sig, off in zip(sigs, offsets):
        if off is not None:
            print(f"[contact_video] {sig.label}: video time = run time + {off:.3f} s "
                  f"(video {sig.span[0] + off:.1f}-{sig.span[1] + off:.1f} s)")

    def covering(tv: float) -> int | None:
        """The run whose span (padded) holds video time ``tv``."""
        for i, (sig, off) in enumerate(zip(sigs, offsets)):
            if sig.span[0] + off - a.pad <= tv <= sig.span[1] + off + a.pad:
                return i
        return None

    if a.pick:
        pts = pick(a.video, a.pick)
        print("--calib " + " ".join(f"{tv:g},{u},{v}" for tv, u, v in pts))
        return

    if a.mode == "panel":
        if not single:
            raise SystemExit("--mode panel takes one run")
        sig = sigs[0]
        t0 = a.t0 if a.t0 is not None else sig.span[0]
        t1 = a.t1 if a.t1 is not None else sig.span[1]
        out = a.out or a.runs[0].expanduser() / "analysis" / "contact_panel.mp4"
        out.parent.mkdir(parents=True, exist_ok=True)
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

    cal = None
    if a.mode == "overlay":
        if not a.calib:
            raise SystemExit("--mode overlay needs --calib (get it with --pick)")
        pts = np.array([[float(x) for x in c.split(",")] for c in a.calib])
        rob = []
        for tv in pts[:, 0]:
            i = covering(tv)
            rob.append(sigs[i].at("robot", tv - offsets[i])[0][[0, 2]] if i is not None else [np.nan, np.nan])
        rob = np.array(rob)
        if not np.isfinite(rob).all():
            raise SystemExit(f"--calib: video times {pts[~np.isfinite(rob).all(1), 0].tolist()} are outside every robot log")
        cal = Calibration(rob, pts[:, 1:], a.x_sign)

    _, fps, n = open_video(a.video)
    for i, (sig, off) in enumerate(zip(sigs, offsets)):
        t0 = a.t0 if (single and a.t0 is not None) else sig.span[0] - a.pad
        t1 = a.t1 if (single and a.t1 is not None) else sig.span[1] + a.pad
        v0, v1 = max(0.0, t0 + off), min(n / fps, t1 + off)
        if v1 <= v0:
            print(f"[contact_video] {sig.label}: not in the clip ({n / fps:.1f} s) with this sync, skipped")
            continue
        if single:
            out = a.out or a.runs[0].expanduser() / "analysis" / f"contact_{a.mode}.mp4"
        else:
            out = (a.out or a.video.parent) / f"{a.video.stem}_{sig.label}_{a.mode}.mp4"
        out.parent.mkdir(parents=True, exist_ok=True)
        ts = np.arange(sig.span[0], sig.span[1], 0.01)
        f_max = float(np.nanpercentile(np.r_[sig.force("felt", ts), sig.force("robot", ts)], 99.5)) * 1.1
        cap, _, _ = open_video(a.video)
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(v0 * fps)))
        panel, w = None, None
        keys = sig.keys(a.command)
        for k in range(int(round(v0 * fps)), int(v1 * fps)):
            ok, img = cap.read()
            if not ok:
                break
            t = k / fps - off
            if a.mode == "overlay":
                frame = cv2.cvtColor(draw_overlay(img, sig, cal, t, keys, max(f_max, 5.0), not a.no_reference),
                                     cv2.COLOR_BGR2RGB)
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
        print(f"[contact_video] {sig.label}: {a.mode} -> {out}")


if __name__ == "__main__":
    main()
