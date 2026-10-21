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

   method is "peak", "centroid", "leading_edge", "template" or "multi" (one or two holes touched: result["hole2"]
   is the second hole, or None). estimate() takes about 20 us per 1500-sample frame
   on a laptop (every method), far below the ~16 ms between plot updates, so it can run on every frame.

A single frame is noisier than the five-frame average used for calibration; averaging a few frames
(make_background works for presses too) before estimate() trades speed for accuracy.

3. Or run estimate() on the Teensy itself (firmware/TEENSY_BLUETOOTH, over an HC-08 Bluetooth module): upload the
   calibration with TeensyController, record the background on command, and receive one small result packet per
   cycle (see the Bluetooth section at the end of this file).
"""

import json
import queue
import sqlite3
import struct
import threading
import time

import numpy as np
from scipy.signal import hilbert, lfilter

SAMPLING_RATE_HZ = 183150.2
ALPHA = 0.10              # EMA smoothing factor
SEARCH_START = 200        # echo search starts here (samples); skips the transmit ring-down
END_SEARCH_START = 400    # the end reflection is searched for from here (samples)
END_GUARD = 60            # the echo search stops this many samples before the end reflection
CENTROID_HALF_WIDTH = 30  # samples either side of the peak for the squared-amplitude centroid
CYCLES_PER_PRESS = 5      # frames per press in the calibration recording
METHODS = ("peak", "centroid", "leading_edge", "template", "multi")
PAIR_RATIO = 5.0          # multi: two holes are reported when the pair's residual is this many times smaller


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
def _load_presses(db_path, frames_per_press=CYCLES_PER_PRESS):
    """Presses (frames_per_press consecutive frames averaged), their hole keys and pass numbers (n-th recording of
    each key), in order."""
    with sqlite3.connect(db_path) as conn:
        rows = conn.execute("SELECT key_vector, signal_blob FROM signals ORDER BY id").fetchall()
    keys = np.array([json.loads(r[0])[0] for r in rows])
    frames = np.stack([np.frombuffer(r[1], dtype=np.uint16).astype(float) for r in rows])
    assert len(frames) % frames_per_press == 0, "Incomplete press (frames not a multiple of frames_per_press)"
    batch_keys = keys.reshape(-1, frames_per_press)
    assert np.all(batch_keys == batch_keys[:, :1]), "Mixed keys in a press"
    keys = batch_keys[:, 0]
    presses = frames.reshape(-1, frames_per_press, frames.shape[1]).mean(axis=1)
    passes = np.array([np.sum(keys[:i] == k) for i, k in enumerate(keys)])
    return keys, passes, presses


def calibrate(db_path, first_hole_cm, hole_spacing_cm, save_path=None, excluded_passes=(), detection_margin=1.10,
              end_search_start=END_SEARCH_START, frames_per_press=CYCLES_PER_PRESS):
    """Calibration from a single-touch recording: echo-time lines, templates and detection settings.

    db_path: SAVE_TO_SQLITE.py database (keys [0] and [1]..[N], CYCLES_PER_PRESS frames per press, each pass
        starting with a [0] press). excluded_passes: pass numbers (0 = first recording of every key) to leave out.
    first_hole_cm, hole_spacing_cm: hole k is at first_hole_cm + (k - 1) * hole_spacing_cm.
    save_path: if given, the calibration is also saved there (.npz) for load_calibration().
    end_search_start: start of the end-reflection search (samples); later for a coiled tube with a large
        reflection before its end.
    frames_per_press: database rows averaged into one press: CYCLES_PER_PRESS (5) for SAVE_TO_SQLITE.py over USB,
        1 for a database recorded over Bluetooth (TEENSY_BLUETOOTH already averages 5 cycles per packet).

    Returns a dict: the mean background, the search window, the detection threshold, the hole ids and distances,
    for each timing method the line l = c0 + c1 t (cm, ms) fitted to every press, and the template-matching
    projection matrix. Accuracy against the presses themselves is printed.
    """
    keys, passes, presses = _load_presses(db_path, frames_per_press)
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
def _pair_overlaps(cal):
    """Hole pairs (i < j) and the 2 x 2 overlaps of their template rows, cached in cal (computed once)."""
    if "_pairs" not in cal:
        q = np.asarray(cal["template_projection"], dtype=float)
        gram = (q @ q.T).reshape(len(q) // 2, 2, len(q) // 2, 2).transpose(0, 2, 1, 3)
        i, j = np.triu_indices(len(q) // 2, k=1)
        cal["_pairs"] = (i, j, gram[i, j])
    return cal["_pairs"]


def _best_pair(b, total, single_energy, cal):
    """Best pair of holes for one window: (i, j, single/pair residual ratio). As the firmware: the least-squares fit
    with the four rows of holes i and j explains b' M^-1 b, M = [[I, G], [G', I]], via the Schur complement."""
    i, j, g = _pair_overlaps(cal)
    bi, bj = b[i], b[j]                                         # (pairs, 2)
    s = np.eye(2) - np.einsum("pki,pkj->pij", g, g)              # I - G'G
    y = bj - np.einsum("pki,pk->pi", g, bi)                      # bj - G' bi
    det = s[:, 0, 0] * s[:, 1, 1] - s[:, 0, 1] ** 2
    with np.errstate(divide="ignore", invalid="ignore"):
        energy = (bi ** 2).sum(1) + (s[:, 1, 1] * y[:, 0] ** 2 - 2 * s[:, 0, 1] * y[:, 0] * y[:, 1]
                                     + s[:, 0, 0] * y[:, 1] ** 2) / det
    energy[det <= 1e-12] = -np.inf
    p = int(np.argmax(energy))
    pair_residual = total - energy[p]
    ratio = (total - single_energy) / pair_residual if pair_residual > 0 else np.inf
    return int(i[p]), int(j[p]), float(ratio)


def estimate(frame, cal, background, method="leading_edge", pair_ratio=PAIR_RATIO):
    """Hole and distance for one frame (or a press average): fast enough to run on every frame.

    frame: one trace (e.g. the uint16 array from LIVE_PLOT.py's packet parser).
    cal: from calibrate() or load_calibration(). background: no-hole trace (make_background()), same length.
    method: "peak", "centroid", "leading_edge", "template" or "multi".
    pair_ratio: multi only; two holes are reported when the best pair's residual is more than pair_ratio times
    smaller than the best single hole's (5 in paper_analysis/01_multi_touch.ipynb).

    Returns a dict: detected (echo above the calibration threshold), hole, distance_cm (from the line, or the
    chosen hole's distance for template matching), echo_time_ms (None for template matching), amplitude (EMA
    peak, ADC counts), hole2 (multi: the second hole, or None) and ratio (multi: single / pair residual).
    hole and distance_cm are given even when not detected, so check detected first. multi takes ~1 ms (64 holes).
    """
    start, stop = cal["search_start"], cal["search_stop"]
    signal = np.asarray(frame[:stop], dtype=float) - background[:stop]
    envelope = ema(np.abs(signal))           # causal, so only the part up to the window end is needed
    peak = start + int(np.argmax(envelope[start:stop]))
    amplitude = float(envelope[peak])
    result = dict(method=method, detected=amplitude > cal["threshold"], amplitude=amplitude, echo_time_ms=None,
                  hole2=None, ratio=None)

    if method in ("template", "multi"):
        window = signal[start:stop]
        b = (cal["template_projection"] @ window).reshape(-1, 2)
        energies = (b ** 2).sum(1)
        k = int(np.argmax(energies))                            # least residual = most energy in the template's span
        result.update(hole=int(cal["hole_ids"][k]), distance_cm=float(cal["hole_cm"][k]))
        if method == "multi":
            i, j, ratio = _best_pair(b, float(window @ window), energies[k], cal)
            result["ratio"] = ratio
            if ratio > pair_ratio:
                result.update(hole=int(cal["hole_ids"][i]), hole2=int(cal["hole_ids"][j]),
                              distance_cm=float(cal["hole_cm"][i]))
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


# ----------------------------------------------------------------------------------------------- Bluetooth
# On-board localization with firmware/TEENSY_BLUETOOTH (HC-08 BLE module): the Teensy runs estimate() itself and
# sends a small RESULT packet per update; the computer uploads the calibration and templates and sends commands.
#
#   cal = load_calibration("cal.npz")                     # from calibrate(), same tube and set-up
#   link = BLEReceiver(name="WAVEGUIDE").start()          # HC-08 over BLE (pip install bleak)
#   # link = SerialReceiver("/dev/ttyUSB0").start()       # or the HC-08 bridged to a serial port
#   teensy = TeensyController(link)
#   teensy.upload_calibration(cal)                        # window, threshold, lines, holes, templates
#   teensy.record_background(cycles=10)                   # nothing covered
#   teensy.set_method("leading_edge"); teensy.set_average(1)
#   teensy.stream(True)
#   for result in teensy.results():                       # dicts: hole, distance_cm, detected, ...
#       ...
#   frames = teensy.request_raw(count=5)                  # raw averaged frames, e.g. for FrameRecorder
#
# The calibration, templates and background live in the Teensy's RAM: upload again after a power cycle.
#
# Packets, both directions (little endian): 0xAA 0x55, u8 type, u16 payload length, payload,
# u16 CRC-16/CCITT-FALSE over type, length and payload. Types and payloads: see firmware/TEENSY_BLUETOOTH/waveguide_core.h.

BT_SYNC = b"\xaa\x55"
BT_MAX_PAYLOAD = 4096     # sanity limit, so a corrupted length cannot stall the parser
HC08_CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"   # HC-08 UART: notify (from it) and write (to it)
BLE_CHUNK_BYTES = 20      # BLE write size the HC-08 accepts
CMD = dict(set_config=0x01, template_info=0x02, template_data=0x03, record_background=0x04, set_method=0x05,
           send_raw=0x06, stream=0x07, get_status=0x08, set_average=0x09, set_pair_threshold=0x0A)
PKT_RESULT, PKT_RAW, PKT_ACK, PKT_STATUS = 0x10, 0x11, 0x12, 0x13
METHOD_CODES = {"peak": 0, "centroid": 1, "leading_edge": 2, "template": 3, "multi": 4}
ACK_STATUS = {0: "ok", 1: "bad length", 2: "out of range", 3: "not ready", 4: "unknown command"}

_CRC_TABLE = []
for _byte in range(256):
    _crc = _byte << 8
    for _ in range(8):
        _crc = ((_crc << 1) ^ 0x1021) if _crc & 0x8000 else (_crc << 1)
    _CRC_TABLE.append(_crc & 0xFFFF)


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE (polynomial 0x1021, initial 0xFFFF), as in the firmware."""
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[(crc >> 8) ^ byte]
    return crc


def encode_packet(packet_type, payload=b""):
    """One packet of the Teensy protocol: sync, type, length, payload, CRC."""
    body = bytes([packet_type]) + len(payload).to_bytes(2, "little") + bytes(payload)
    return BT_SYNC + body + crc16_ccitt(body).to_bytes(2, "little")


def unpack12(payload, n_samples):
    """12-bit samples packed two per three bytes (a[7:0], a[11:8] | b[3:0] << 4, b[11:4]) -> uint16 array."""
    raw = np.frombuffer(bytes(payload) + b"\x00" * (-len(payload) % 3), dtype=np.uint8).astype(np.uint16)
    raw = raw.reshape(-1, 3)
    samples = np.empty(2 * len(raw), dtype=np.uint16)
    samples[0::2] = raw[:, 0] | ((raw[:, 1] & 0x0F) << 8)
    samples[1::2] = (raw[:, 1] >> 4) | (raw[:, 2] << 4)
    return samples[:n_samples]


def pack12(samples):
    """Inverse of unpack12: uint16 samples (< 4096) -> packed bytes, as the firmware sends them."""
    samples = np.asarray(samples, dtype=np.uint16)
    n = len(samples)
    padded = np.append(samples, np.uint16(0)) if n % 2 else samples
    a, b = padded[0::2], padded[1::2]
    out = np.stack([a & 0xFF, ((a >> 8) & 0x0F) | ((b & 0x0F) << 4), b >> 4], axis=1).astype(np.uint8).ravel()
    return out[:(3 * n + 1) // 2].tobytes()


def _decode(packet_type, payload):
    """A received packet's payload as a dict (type-specific fields), or None for an unknown type."""
    if packet_type == PKT_RESULT and len(payload) == 28:
        (sequence, time_ms, method, detected, hole, distance, echo_time, amplitude, hole2,
         ratio) = struct.unpack("<HIBBhfffhf", payload)
        method = {v: k for k, v in METHOD_CODES.items()}.get(method, method)
        return dict(type="result", sequence=sequence, time_ms=time_ms, method=method, detected=bool(detected),
                    hole=hole, distance_cm=distance, echo_time_ms=None if np.isnan(echo_time) else echo_time,
                    amplitude=amplitude, hole2=None if hole2 < 0 else hole2, ratio=None if np.isnan(ratio) else ratio)
    if packet_type == PKT_RAW and len(payload) >= 8:
        sequence, kind, first_sample, n_samples, n_averaged = struct.unpack("<HBHHB", payload[:8])
        samples = unpack12(payload[8:], n_samples)
        if first_sample:
            samples = np.concatenate([np.zeros(first_sample, dtype=np.uint16), samples])
        return dict(type="raw", sequence=sequence, kind="background" if kind == 1 else "frame", samples=samples,
                    n_averaged=n_averaged, first_sample=first_sample)
    if packet_type == PKT_ACK and len(payload) == 6:
        command, status, value = struct.unpack("<BBI", payload)
        return dict(type="ack", command=command, status=ACK_STATUS.get(status, status), value=value)
    if packet_type == PKT_STATUS and len(payload) == struct.calcsize("<BBBBBBHHBHHIH"):   # 21 bytes
        fields = struct.unpack("<BBBBBBHHBHHIH", payload)
        names = ("config_ok", "background_ok", "templates_ok", "method", "average", "streaming", "search_start",
                 "search_stop", "n_holes", "template_rows", "template_window", "template_values_received", "n_samples")
        status = dict(zip(names, fields))
        for key in ("config_ok", "background_ok", "templates_ok", "streaming"):
            status[key] = bool(status[key])
        status["method"] = {v: k for k, v in METHOD_CODES.items()}.get(status["method"], status["method"])
        return dict(type="status", **status)
    return None


class PacketParser:
    """Turns the byte stream from the Teensy into packets. Feed it bytes as they arrive (any chunking); it finds the
    sync bytes, checks the CRC and returns decoded packets (dicts with a "type": result, raw, ack or status).
    Corrupted packets are dropped and counted in stats."""

    def __init__(self):
        self.buffer = bytearray()
        self.stats = dict(packets=0, crc_errors=0, bytes=0)

    def feed(self, data):
        self.buffer.extend(data)
        self.stats["bytes"] += len(data)
        packets = []
        while True:
            start = self.buffer.find(BT_SYNC)
            if start < 0:
                del self.buffer[:-1]                     # keep a possible first sync byte
                return packets
            del self.buffer[:start]
            if len(self.buffer) < 5:
                return packets
            packet_type, length = self.buffer[2], int.from_bytes(self.buffer[3:5], "little")
            if length > BT_MAX_PAYLOAD:
                del self.buffer[:1]                      # not a real packet: look for the next sync
                continue
            total = 5 + length + 2
            if len(self.buffer) < total:
                return packets
            packet = bytes(self.buffer[:total])
            if crc16_ccitt(packet[2:-2]) != int.from_bytes(packet[-2:], "little"):
                self.stats["crc_errors"] += 1
                del self.buffer[:1]                      # resynchronise on the next sync
                continue
            del self.buffer[:total]
            decoded = _decode(packet_type, packet[5:-2])
            if decoded is not None:
                decoded["received"] = time.time()
                self.stats["packets"] += 1
                packets.append(decoded)


class _Link:
    """Common part of the links: a parser, one queue per packet type filled by a background thread, and write()."""

    def __init__(self, max_queued=1000):
        self.parser = PacketParser()
        self.queues = {kind: queue.Queue(maxsize=max_queued) for kind in ("result", "raw", "ack", "status")}
        self._stop = threading.Event()
        self._thread = None
        self.ready = threading.Event()                  # set once the link can send and receive

    def _deliver(self, data):
        for packet in self.parser.feed(data):
            q = self.queues[packet["type"]]
            if q.full():                                 # keep the newest packets if nobody reads them
                q.get_nowait()
            q.put_nowait(packet)

    def start(self, timeout=20.0):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not self.ready.wait(timeout):
            raise TimeoutError("The link did not open (device not found, or port busy)")
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def get(self, kind, timeout=None):
        """Next packet of this kind (oldest first), or None after timeout seconds."""
        try:
            return self.queues[kind].get(timeout=timeout)
        except queue.Empty:
            return None

    def clear(self, kind):
        while not self.queues[kind].empty():
            self.queues[kind].get_nowait()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


class SerialReceiver(_Link):
    """Link over a serial port: the HC-08 bridged to the computer through a USB-UART adapter (e.g. a second HC-08 as
    master), or the Teensy's UART wired directly for testing. Needs pyserial."""

    def __init__(self, port, baud=115200, **kwargs):
        super().__init__(**kwargs)
        self.port, self.baud = port, baud
        self._serial = None
        self._write_lock = threading.Lock()

    def _run(self):
        import serial
        with serial.Serial(self.port, self.baud, timeout=0.05) as link:
            self._serial = link
            self.ready.set()
            while not self._stop.is_set():
                data = link.read(link.in_waiting or 1)
                if data:
                    self._deliver(data)

    def write(self, data):
        with self._write_lock:
            self._serial.write(data)


class BLEReceiver(_Link):
    """Link over BLE to the HC-08 (its UART characteristic FFE1: notifications from it, writes to it). Needs bleak
    (pip install bleak). Connects to the device with this name, or to address if given."""

    def __init__(self, name="WAVEGUIDE", address=None, scan_timeout=10.0, **kwargs):
        super().__init__(**kwargs)
        self.name, self.address, self.scan_timeout = name, address, scan_timeout
        self._loop = self._client = None
        self.error = None

    def _run(self):
        import asyncio
        self._loop = asyncio.new_event_loop()
        try:
            self._loop.run_until_complete(self._main())
        except Exception as error:                       # reported by start() timing out; kept for inspection
            self.error = error
        finally:
            self._loop.close()

    async def _main(self):
        import asyncio
        from bleak import BleakClient, BleakScanner
        target = self.address
        if target is None:
            target = await BleakScanner.find_device_by_name(self.name, timeout=self.scan_timeout)
            if target is None:
                raise RuntimeError(f"No BLE device named {self.name!r} found (is the HC-08 powered and not connected?)")
        async with BleakClient(target) as client:
            self._client = client
            await client.start_notify(HC08_CHARACTERISTIC_UUID, lambda _, data: self._deliver(data))
            self.ready.set()
            while not self._stop.is_set():
                await asyncio.sleep(0.05)
            await client.stop_notify(HC08_CHARACTERISTIC_UUID)

    def write(self, data):
        import asyncio
        for start in range(0, len(data), BLE_CHUNK_BYTES):
            chunk = bytes(data[start:start + BLE_CHUNK_BYTES])
            asyncio.run_coroutine_threadsafe(
                self._client.write_gatt_char(HC08_CHARACTERISTIC_UUID, chunk, response=False), self._loop).result(5)


def device_calibration(cal):
    """The calibration from calibrate() as the Teensy stores it: the SET_CONFIG payload, the TEMPLATE_INFO payload,
    and the templates as int16 (each row scaled to the full int16 range; row scale = max |value| / 32767)."""
    holes = [int(h) for h in cal["hole_ids"]]
    if any(not 0 <= h <= 255 for h in holes):
        raise ValueError("Hole ids must fit in one byte")
    config = struct.pack("<ffHHf", cal["sampling_rate_hz"], cal["alpha"], cal["search_start"], cal["search_stop"],
                         cal["threshold"])
    config += struct.pack("<3f", *(cal[f"{m}_c0"] for m in ("peak", "centroid", "leading_edge")))
    config += struct.pack("<3f", *(cal[f"{m}_c1"] for m in ("peak", "centroid", "leading_edge")))
    config += bytes([len(holes)]) + b"".join(struct.pack("<Bf", h, cm) for h, cm in zip(holes, cal["hole_cm"]))
    projection = np.asarray(cal["template_projection"], dtype=float)
    scale = np.abs(projection).max(axis=1) / 32767
    scale[scale == 0] = 1.0
    templates = np.round(projection / scale[:, None]).astype(np.int16)
    info = struct.pack("<HH", *projection.shape) + struct.pack(f"<{len(scale)}f", *scale)
    return config, info, templates, scale


class TeensyController:
    """Commands to the TEENSY_BLUETOOTH firmware over a link (BLEReceiver or SerialReceiver), and its results."""

    def __init__(self, link, timeout=2.0, retries=3):
        self.link, self.timeout, self.retries = link, timeout, retries

    def command(self, name, payload=b"", timeout=None, retries=None):
        """Sends a command and waits for its ACK (resending on timeout); returns the ACK's value. Raises on an error
        status or when no ACK arrives."""
        code = CMD[name]
        for _ in range((retries if retries is not None else self.retries) + 1):
            self.link.write(encode_packet(code, payload))
            deadline = time.time() + (timeout if timeout is not None else self.timeout)
            while time.time() < deadline:
                ack = self.link.get("ack", timeout=max(0.0, deadline - time.time()))
                if ack is None:
                    break
                if ack["command"] != code:
                    continue                             # a late ACK of an earlier command
                if ack["status"] != "ok":
                    raise RuntimeError(f"Teensy refused {name}: {ack['status']} (value {ack['value']})")
                return ack["value"]
        raise TimeoutError(f"No acknowledgement for {name}")

    def status(self, timeout=None):
        """The Teensy's state: config/background/templates loaded, method, averaging, streaming, window, holes."""
        self.link.clear("status")
        for _ in range(self.retries + 1):
            self.link.write(encode_packet(CMD["get_status"]))
            packet = self.link.get("status", timeout=timeout or self.timeout)
            if packet is not None:
                return packet
        raise TimeoutError("No status from the Teensy")

    def upload_calibration(self, cal, chunk_values=120, progress=True):
        """Sends the calibration (window, threshold, echo-time lines, hole positions) and the templates. The
        templates are about 2 x holes x window int16 values (~160 kB for 51 holes), so over BLE this takes about
        a minute; each chunk is acknowledged and resent if lost."""
        config, info, templates, _ = device_calibration(cal)
        self.command("set_config", config)
        total = self.command("template_info", info)
        values = templates.ravel()
        assert total == len(values), f"Teensy expects {total} template values, have {len(values)}"
        t0 = time.time()
        for offset in range(0, len(values), chunk_values):
            chunk = values[offset:offset + chunk_values]
            self.command("template_data", struct.pack("<I", offset) + chunk.astype("<i2").tobytes())
            if progress and (offset // chunk_values) % 50 == 0:
                print(f"\rtemplates {offset + len(chunk)}/{len(values)} values ({time.time() - t0:.0f} s)", end="")
        status = self.status()
        if progress:
            print(f"\rtemplates {len(values)}/{len(values)} values ({time.time() - t0:.0f} s); "
                  f"config ok {status['config_ok']}, templates ok {status['templates_ok']}")
        if not (status["config_ok"] and status["templates_ok"]):
            raise RuntimeError(f"Upload incomplete: {status}")
        return status

    def record_background(self, cycles=10):
        """Records the background on the Teensy (nothing covered): the mean of the next `cycles` cycles."""
        return self.command("record_background", bytes([cycles]), timeout=self.timeout + 0.1 * cycles, retries=1)

    def set_method(self, method):
        """"peak", "centroid", "leading_edge", "template" or "multi" (one or two holes)."""
        return self.command("set_method", bytes([METHOD_CODES[method]]))

    def set_average(self, cycles):
        """Cycles averaged per result (1-32): 1 gives a result every cycle (~20/s), 5 matches a calibration press."""
        return self.command("set_average", bytes([cycles]))

    def set_pair_threshold(self, ratio=PAIR_RATIO):
        """multi: two holes are reported when the pair's residual is more than `ratio` times smaller (> 1)."""
        return self.command("set_pair_threshold", struct.pack("<f", ratio))

    def stream(self, on=True):
        """Starts (or stops) a RESULT packet for every averaged frame. Needs the config, the background and (for
        template matching) the templates."""
        if on:
            self.link.clear("result")
        return self.command("stream", bytes([int(on)]))

    def results(self, timeout=None):
        """Yields result dicts as they arrive (hole, hole2, distance_cm, detected, echo_time_ms, amplitude, ratio,
        sequence, time_ms); stops when none arrives within timeout seconds (None: waits forever)."""
        while True:
            result = self.link.get("result", timeout=timeout)
            if result is None:
                return
            yield result

    def latest_result(self):
        """The newest result (older ones are discarded), or None."""
        latest = None
        while (packet := self.link.get("result", timeout=0)) is not None:
            latest = packet
        return latest

    def request_raw(self, count=1, kind="frame", timeout=10.0):
        """Raw frames from the Teensy (12-bit, averaged like the results): the next `count` frames, or the recorded
        background (kind="background"). Each is a dict with samples (uint16) and n_averaged."""
        self.link.clear("raw")
        self.command("send_raw", bytes([count if kind == "frame" else 1, int(kind == "background")]))
        frames = []
        for _ in range(count if kind == "frame" else 1):
            frame = self.link.get("raw", timeout=timeout)
            if frame is None:
                raise TimeoutError(f"Only {len(frames)} of {count} raw frames received")
            frames.append(frame)
        return frames


class FrameRecorder:
    """Stores frames in an SQLite database with the same table as SAVE_TO_SQLITE.py (key_vector, run_number,
    timestamp, signal_blob as uint16), so the analysis notebooks and calibrate() can read it. Raw frames from the
    Teensy are already averaged (n_averaged cycles each): calibrate such a database with frames_per_press=1."""

    def __init__(self, db_path):
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.execute("""CREATE TABLE IF NOT EXISTS signals (
            id INTEGER PRIMARY KEY AUTOINCREMENT, key_vector TEXT NOT NULL, run_number INTEGER NOT NULL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP, signal_blob BLOB NOT NULL)""")
        self.conn.commit()
        (last,) = self.conn.execute("SELECT MAX(run_number) FROM signals").fetchone()
        self.run_number = 0 if last is None else last + 1

    def save(self, frame, key, new_run=True):
        """Saves one frame (a raw-frame dict or a sample array) under key (e.g. [0] for background, [3] for hole 3).
        new_run=True gives it its own run number, like one press in SAVE_TO_SQLITE.py."""
        samples = frame["samples"] if isinstance(frame, dict) else frame
        self.conn.execute("INSERT INTO signals (key_vector, run_number, signal_blob) VALUES (?, ?, ?)",
                          (json.dumps(list(key)), self.run_number, np.asarray(samples, dtype=np.uint16).tobytes()))
        self.conn.commit()
        if new_run:
            self.run_number += 1

    def close(self):
        self.conn.close()
