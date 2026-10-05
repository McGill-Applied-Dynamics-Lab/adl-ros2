"""MCAP data loader for RIM teleoperation experiments."""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING

import pandas as pd
from mcap.records import Channel, Message, Schema
from mcap.stream_reader import StreamReader

if TYPE_CHECKING:
    from .experiment import ExperimentRun


def _flatten(values: dict, prefix: str = "") -> dict:
    """Flatten nested dicts; unwrap single-element lists/arrays to scalars."""
    out: dict = {}
    for key, val in values.items():
        col = f"{prefix}.{key}" if prefix else key
        if isinstance(val, dict):
            out.update(_flatten(val, col))
        else:
            _flatten_value(val, col, out)
    return out


def _flatten_value(val: object, col: str, out: dict) -> None:
    """Recursively flatten a value into ``out`` under ``col``."""
    if isinstance(val, list):
        scalars = [v for v in val if not isinstance(v, (list, dict))]
        if len(scalars) == len(val):
            # All scalars
            if len(val) == 1:
                out[col] = val[0]
            else:
                for i, v in enumerate(val):
                    out[f"{col}_{i}"] = v
        else:
            # Nested lists (e.g. 2-D matrices) — flatten recursively
            for i, v in enumerate(val):
                _flatten_value(v, f"{col}_{i}" if len(val) > 1 else col, out)
    else:
        out[col] = val


def load_streams(
    source: ExperimentRun | str | Path,
    streams: list[str] | None = None,
) -> dict[str, pd.DataFrame]:
    """Load JSON-encoded streams from an experiment's MCAP file.

    Parameters
    ----------
    source:
        An ExperimentRun, a path to a run directory, or a path to an .mcap file.
    streams:
        Whitelist of stream names to load (e.g. ``["haptic", "tracking"]``).
        Loads all JSON streams when None.

    Returns
    -------
    dict mapping stream name → DataFrame with a ``ts`` column and one column
    per signal (1-element lists are unwrapped to scalars).
    """
    mcap_path = _resolve_mcap_path(source)

    channels: dict[int, Channel] = {}
    rows: dict[str, list[dict]] = {}

    with open(mcap_path, "rb") as f:
        sr = StreamReader(f, record_size_limit=None)
        try:
            for record in sr.records:
                if isinstance(record, Schema):
                    pass
                elif isinstance(record, Channel):
                    channels[record.id] = record
                elif isinstance(record, Message):
                    ch = channels.get(record.channel_id)
                    if ch is None or ch.message_encoding != "json":
                        continue
                    data = json.loads(record.data)
                    stream = str(data.get("stream", ch.topic.split("/")[-1]))
                    if streams is not None and stream not in streams:
                        continue
                    flat: dict = {"ts": float(data.get("ts", 0.0))}
                    values = data.get("values", {})
                    if isinstance(values, dict):
                        flat.update(_flatten(values))
                    rows.setdefault(stream, []).append(flat)
        except Exception:
            pass  # EOF: invalid footer size in Foxglove-written MCAPs is expected

    return {
        stream: pd.DataFrame(records).sort_values("ts").reset_index(drop=True)
        for stream, records in rows.items()
    }


def _resolve_mcap_path(source: ExperimentRun | str | Path) -> Path:
    # avoid importing ExperimentRun at runtime to keep this module lightweight
    if hasattr(source, "mcap_path"):
        return source.mcap_path  # type: ignore[union-attr]
    p = Path(source)
    if p.suffix == ".mcap":
        return p
    candidate = p / "samples.mcap"
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"No samples.mcap found in {p}")
