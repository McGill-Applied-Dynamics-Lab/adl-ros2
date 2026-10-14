"""Calibration and real-time hole estimation for the acoustic waveguide sensor.

Same processing as paper_analysis/00_basic_touch.ipynb: subtract a no-hole background, rectify, smooth with an
exponential moving average (EMA), find the echo in the search window (from SEARCH_START to one echo width before
the end reflection), then either time the echo (envelope peak, squared-amplitude centroid or 50% leading edge) and
convert it to a distance with a linear calibration l = c0 + c1 t, or match the whole waveform against one template
per hole (template matching).

Two steps:

1. Calibrate once from a single-touch calibration recording (SAVE_TO_SQLITE.py; keys [0] = no hole, [1]..[N] = one
   hole covered; each pass starts with a [0] press):

       cal = calibrate("calibration.db", first_hole_cm=25.0, hole_spacing_cm=1.0, save_path="calibration.npz")

   or later, without reprocessing:

       cal = load_calibration("calibration.npz")

2. In the live loop (e.g. LIVE_PLOT.py's update_plot), with a background recorded with nothing covered:

       background = make_background(frames_with_nothing_covered)     # or cal["background"] from the same session
       result = estimate(latest_frame, cal, background, method="leading_edge")
       if result["detected"]:
           print(result["hole"], result["distance_cm"])

   method is "peak", "centroid", "leading_edge" or "template". estimate() takes about 20 us per 1500-sample frame
   on a laptop (every method), far below the ~16 ms between plot updates, so it can run on every frame.

A single frame is noisier than the five-frame average used for calibration; averaging a few frames
(make_background works for presses too) before estimate() trades speed for accuracy.
"""

import json
import sqlite3

import numpy as np
from scipy.signal import hilbert, lfilter

SAMPLING_RATE_HZ = 183150.2
ALPHA = 0.10              # EMA smoothing factor
SEARCH_START = 200        # echo search starts here (samples); skips the transmit ring-down
END_SEARCH_START = 400    # the end reflection is searched for from here (samples)
END_GUARD = 60            # the echo search stops this many samples before the end reflection
CENTROID_HALF_WIDTH = 30  # samples either side of the peak for the squared-amplitude centroid
CYCLES_PER_PRESS = 5      # frames per press in the calibration recording
METHODS = ("peak", "centroid", "leading_edge", "template")


# ----------------------------------------------------------------------------------------------- processing
def ema(rectified):
    """Exponential moving average along the last axis (y[0] = x[0]); a C-level IIR filter, so it is fast."""
    rectified = np.asarray(rectified, dtype=float)
    zi = (1 - ALPHA) * rectified[..., :1]
    return lfilter([ALPHA], [1.0, -(1 - ALPHA)], rectified, axis=-1, zi=zi)[0]


def make_background(frames):
    """Mean of several frames (rows): the background from frames with nothing covered, or a press average."""
    return np.asarray(frames, dtype=float).reshape(-1, np.shape(frames)[-1]).mean(axis=0)


def leading_edge(envelope, peak, floor=0):
    """Sample (fractional) where the envelope rises through half its peak value, walking back from the peak."""
    half = envelope[peak] / 2
    below = np.flatnonzero(envelope[floor:peak + 1] <= half)
    if len(below) == 0:
        return float(floor)
    b = floor + below[-1]
    return b + (half - envelope[b]) / (envelope[b + 1] - envelope[b])


def centroid(envelope, peak, start, stop):
    """Squared-amplitude centroid (fractional sample) within CENTROID_HALF_WIDTH of the peak, inside [start, stop)."""
    lo, hi = max(start, peak - CENTROID_HALF_WIDTH), min(stop, peak + CENTROID_HALF_WIDTH + 1)
    weights = envelope[lo:hi] ** 2
    return float(np.dot(np.arange(lo, hi), weights) / weights.sum())


def echo_samples(envelope, start, stop):
    """Peak sample and amplitude, and the echo sample for each timing method, in the window [start, stop)."""
    peak = start + int(np.argmax(envelope[start:stop]))
    return envelope[peak], {"peak": float(peak), "centroid": centroid(envelope, peak, start, stop),
                            "leading_edge": leading_edge(envelope, peak, start)}


# ----------------------------------------------------------------------------------------------- calibration
def _load_presses(db_path):
    """Five-frame-averaged presses, their hole keys and pass numbers (n-th recording of each key), in order."""
    with sqlite3.connect(db_path) as conn:
        rows = conn.execute("SELECT key_vector, signal_blob FROM signals ORDER BY id").fetchall()
    keys = np.array([json.loads(r[0])[0] for r in rows])
    frames = np.stack([np.frombuffer(r[1], dtype=np.uint16).astype(float) for r in rows])
    assert len(frames) % CYCLES_PER_PRESS == 0, "Incomplete press (frames not a multiple of CYCLES_PER_PRESS)"
    batch_keys = keys.reshape(-1, CYCLES_PER_PRESS)
    assert np.all(batch_keys == batch_keys[:, :1]), "Mixed keys in a press"
    keys = batch_keys[:, 0]
    presses = frames.reshape(-1, CYCLES_PER_PRESS, frames.shape[1]).mean(axis=1)
    passes = np.array([np.sum(keys[:i] == k) for i, k in enumerate(keys)])
    return keys, passes, presses


def calibrate(db_path, first_hole_cm, hole_spacing_cm, save_path=None, excluded_passes=(), detection_margin=1.10,
              end_search_start=END_SEARCH_START):
    """Calibration from a single-touch recording: echo-time lines, templates and detection settings.

    db_path: SAVE_TO_SQLITE.py database (keys [0] and [1]..[N], CYCLES_PER_PRESS frames per press, each pass
        starting with a [0] press). excluded_passes: pass numbers (0 = first recording of every key) to leave out.
    first_hole_cm, hole_spacing_cm: hole k is at first_hole_cm + (k - 1) * hole_spacing_cm.
    save_path: if given, the calibration is also saved there (.npz) for load_calibration().
    end_search_start: start of the end-reflection search (samples); later for a coiled tube with a large
        reflection before its end.

    Returns a dict: the mean background, the search window, the detection threshold, the hole ids and distances,
    for each timing method the line l = c0 + c1 t (cm, ms) fitted to every press, and the template-matching
    projection matrix. Accuracy against the presses themselves is printed.
    """
    keys, passes, presses = _load_presses(db_path)
    keep = ~np.isin(passes, list(excluded_passes))
    keys, passes, presses = keys[keep], passes[keep], presses[keep]
    assert keys[0] == 0, "The recording must start with a background press (key 0)"

    # Each press minus the background press at the start of its pass
    is_background = keys == 0
    backgrounds = presses[is_background]
    signal = presses - backgrounds[np.cumsum(is_background) - 1]
    background = backgrounds.mean(axis=0)

    # End reflection on the mean background (first difference drops the slow gain ramp); search window before it
    background_envelope = ema(np.abs(np.diff(background)))
    end_peak = end_search_start + int(np.argmax(background_envelope[end_search_start:]))
    end_samples = np.arange(end_peak - 60, min(end_peak + 61, len(background_envelope)))
    end_centroid = np.average(end_samples, weights=background_envelope[end_samples] ** 2)
    start, stop = SEARCH_START, int(end_centroid) - END_GUARD

    # Detection threshold: margin x the largest difference between consecutive background presses
    if len(backgrounds) > 1:
        threshold = detection_margin * ema(np.abs(np.diff(backgrounds, axis=0)))[:, start:stop].max()
    else:
        threshold = 0.0
        print("Only one background press: detection threshold set to 0 (every frame counts as detected)")

    covered = ~is_background
    holes = keys[covered]
    hole_ids = np.unique(holes)
    hole_cm = first_hole_cm + hole_spacing_cm * (hole_ids - 1)
    envelope = ema(np.abs(signal[covered]))
    amplitude, times = [], {m: [] for m in ("peak", "centroid", "leading_edge")}
    for env in envelope:
        a, samples = echo_samples(env, start, stop)
        amplitude.append(a)
        for m, s in samples.items():
            times[m].append(1000 * s / SAMPLING_RATE_HZ)
    detected = np.array(amplitude) > threshold
    distance = first_hole_cm + hole_spacing_cm * (holes - 1)

    cal = dict(sampling_rate_hz=SAMPLING_RATE_HZ, alpha=ALPHA, search_start=start, search_stop=stop,
               threshold=float(threshold), background=background, hole_ids=hole_ids, hole_cm=hole_cm,
               first_hole_cm=first_hole_cm, hole_spacing_cm=hole_spacing_cm, end_sample=int(end_peak))
    print(f"{covered.sum()} presses on {len(hole_ids)} holes, {is_background.sum()} background; window samples "
          f"{start}-{stop}; threshold {threshold:.1f}; detected {detected.sum()}/{covered.sum()}")
    for m, t in times.items():
        t = np.array(t)
        c1, c0 = np.polyfit(t[detected], distance[detected], 1)
        cal[f"{m}_c0"], cal[f"{m}_c1"] = float(c0), float(c1)
        nearest = hole_ids[np.argmin(np.abs((c0 + c1 * t)[:, None] - hole_cm[None, :]), axis=1)]
        print(f"  {m:13s} l = {c0:+.3f} + {c1:.3f} t (cm, ms); right hole {np.mean(nearest[detected] == holes[detected]):.1%}")

    # Template matching: one template per hole (mean background-subtracted waveform over the window). A frame x is
    # fitted as Re(a z) with z the template's analytic signal; the residual is |x|^2 - |Q x|^2 with Q an
    # orthonormal basis of {Re z, Im z}, so all holes cost one matrix-vector product.
    window = signal[covered][:, start:stop]
    bases = []
    for h in hole_ids:
        z = hilbert(window[detected & (holes == h)].mean(axis=0) if np.any(detected & (holes == h))
                    else window[holes == h].mean(axis=0))
        q, _ = np.linalg.qr(np.stack([z.real, z.imag], axis=1))   # window x 2, orthonormal columns
        bases.append(q.T)
    cal["template_projection"] = np.concatenate(bases)               # (2 holes) x window
    projected = (window @ cal["template_projection"].T) ** 2
    chosen = hole_ids[np.argmax(projected[:, 0::2] + projected[:, 1::2], axis=1)]
    print(f"  {'template':13s} right hole {np.mean(chosen[detected] == holes[detected]):.1%} (templates from these presses)")

    if save_path is not None:
        np.savez_compressed(save_path, **{k: np.asarray(v) for k, v in cal.items()})
        print(f"Saved calibration to {save_path}")
    return cal


def load_calibration(path):
    """Calibration saved by calibrate(..., save_path=path)."""
    with np.load(path) as data:
        cal = {k: data[k] for k in data.files}
    for k, v in cal.items():
        if v.ndim == 0:
            cal[k] = v.item()
    return cal


# ----------------------------------------------------------------------------------------------- real time
def estimate(frame, cal, background, method="leading_edge"):
    """Hole and distance for one frame (or a press average): fast enough to run on every frame.

    frame: one trace (e.g. the uint16 array from LIVE_PLOT.py's packet parser).
    cal: from calibrate() or load_calibration(). background: no-hole trace (make_background()), same length.
    method: "peak", "centroid", "leading_edge" or "template".

    Returns a dict: detected (echo above the calibration threshold), hole, distance_cm (from the line, or the
    chosen hole's distance for template matching), echo_time_ms (None for template matching), amplitude (EMA
    peak, ADC counts). hole and distance_cm are given even when not detected, so check detected first.
    """
    start, stop = cal["search_start"], cal["search_stop"]
    signal = np.asarray(frame[:stop], dtype=float) - background[:stop]
    envelope = ema(np.abs(signal))           # causal, so only the part up to the window end is needed
    peak = start + int(np.argmax(envelope[start:stop]))
    amplitude = float(envelope[peak])
    result = dict(method=method, detected=amplitude > cal["threshold"], amplitude=amplitude, echo_time_ms=None)

    if method == "template":
        projected = (cal["template_projection"] @ signal[start:stop]) ** 2
        k = int(np.argmax(projected[0::2] + projected[1::2]))   # least residual = most energy in the template's span
        result.update(hole=int(cal["hole_ids"][k]), distance_cm=float(cal["hole_cm"][k]))
        return result

    if method == "peak":
        sample = float(peak)
    elif method == "centroid":
        sample = centroid(envelope, peak, start, stop)
    elif method == "leading_edge":
        sample = leading_edge(envelope, peak, start)
    else:
        raise ValueError(f"method must be one of {METHODS}")
    time_ms = 1000 * sample / cal["sampling_rate_hz"]
    distance = cal[f"{method}_c0"] + cal[f"{method}_c1"] * time_ms
    k = int(np.argmin(np.abs(cal["hole_cm"] - distance)))
    result.update(echo_time_ms=time_ms, distance_cm=float(distance), hole=int(cal["hole_ids"][k]))
    return result
