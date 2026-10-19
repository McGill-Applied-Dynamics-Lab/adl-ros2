"""Which joint drives a low-frequency orientation wobble? (friction / stick-slip diagnostic)

Band-passes (1-6 Hz) the controller's pitch error and every joint velocity over the moving part of
a tracking run, and prints their correlation, plus for each joint how long it sits near zero
velocity and how often its velocity changes sign (stick-slip signature). See FINDINGS.md.

Usage:
    python wobble.py results/tracking/<run>/data.npz
"""

import sys

import numpy as np
from scipy.signal import butter, sosfiltfilt, welch

d = dict(np.load(sys.argv[1]))
t0, t1 = d["ref_t"][0] + 1.5, d["ref_t"][-1] - 1.5  # moving part

# Align controller error and robot state on shared stamps
me = (d["err_t"] >= t0) & (d["err_t"] < t1)
ms = (d["state_t"] >= t0) & (d["state_t"] < t1)
es, err = d["err_stamp"][me], d["err"][me]
ss, st = d["state_stamp"][ms], d["state"][ms]
idx = np.clip(np.searchsorted(ss, es), 0, len(ss) - 1)
ok = np.abs(ss[idx] - es) < 6e-4
pitch, ex, dq, tau_j = err[ok, 4], err[ok, 0], st[idx[ok], 7:14], st[idx[ok], 14:21]

sos = butter(2, [1.0, 6.0], "bandpass", fs=1000.0, output="sos")
bp = lambda x: sosfiltfilt(sos, x, axis=0)  # noqa: E731
p_bp = bp(pitch)
f, P = welch(pitch - pitch.mean(), fs=1000.0, nperseg=4096)
band = (f > 1.0) & (f < 6.0)
print(f"pitch error: wobble peak {f[band][np.argmax(P[band])]:.2f} Hz, band RMS {p_bp.std() * 1e3:.2f} mrad")
print(f"corr(pitch_bp, x_err_bp) = {np.corrcoef(p_bp, bp(ex))[0, 1]:+.2f}")
dq_bp = bp(dq)
for j in range(7):
    c = np.corrcoef(p_bp, dq_bp[:, j])[0, 1]
    v = dq[:, j]
    near_zero = (np.abs(v) < 0.01).mean() * 100
    print(
        f"j{j + 1}: corr(pitch_bp, dq_bp) {c:+.2f}  dq band RMS {dq_bp[:, j].std() * 1e3:6.2f} mrad/s  "
        f"|mean dq| {abs(v.mean()) * 1e3:6.1f} mrad/s  time with |dq|<0.01 {near_zero:5.1f}%  sign changes {np.sum(np.diff(np.sign(v)) != 0)}"
    )
