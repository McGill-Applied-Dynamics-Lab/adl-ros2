"""Experiment run metadata and registry."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


@dataclass
class ExperimentRun:
    """A single logged experiment run."""

    path: Path
    name: str
    created_at: datetime
    config: dict
    notes: str = ""
    git_sha: str | None = None
    git_dirty: bool | None = None

    @classmethod
    def from_dir(cls, run_dir: Path) -> ExperimentRun:
        with open(run_dir / "metadata.json", encoding="utf-8") as f:
            meta = json.load(f)
        return cls(
            path=run_dir,
            name=run_dir.name,
            created_at=datetime.fromisoformat(meta["created_at"]),
            config=meta.get("config", {}),
            notes=meta.get("notes", ""),
            git_sha=meta.get("git_sha"),
            git_dirty=meta.get("git_dirty"),
        )

    @property
    def diff_path(self) -> Path:
        """Path to the captured uncommitted diff (exists only for dirty runs)."""
        return self.path / "uncommitted.diff"

    @property
    def mcap_path(self) -> Path:
        return self.path / "samples.mcap"

    def summary(self) -> dict:
        """Flat dict of key parameters for tabular display."""
        iface = self.config.get("interface", {})
        rates = self.config.get("rates", {})
        model = self.config.get("model", {})
        sha_short = self.git_sha[:8] if self.git_sha else None
        return {
            "name": self.name,
            "created_at": self.created_at.strftime("%Y-%m-%d %H:%M:%S"),
            "notes": self.notes,
            "git_sha": f"{sha_short}{'*' if self.git_dirty else ''}" if sha_short else None,
            "rim_enabled": iface.get("rim_enabled"),
            "force_feedback": iface.get("force_feedback"),
            "stiffness": iface.get("stiffness"),
            "damping": iface.get("damping"),
            "contact_surface": iface.get("contact_surface"),
            "haptic_hz": rates.get("haptic_rate_hz"),
            "control_hz": rates.get("control_rate_hz"),
            "tool_tip_offset": model.get("tool_tip_offset"),
        }


class ExperimentRegistry:
    """Scans a data directory and provides access to experiment runs."""

    def __init__(self, data_dir: str | Path) -> None:
        self.data_dir = Path(data_dir)

    def list_runs(self) -> list[ExperimentRun]:
        """Return all valid runs sorted newest-first."""
        runs: list[ExperimentRun] = []
        for d in self.data_dir.iterdir():
            if d.is_dir() and (d / "metadata.json").exists():
                try:
                    runs.append(ExperimentRun.from_dir(d))
                except Exception:
                    pass
        return sorted(runs, key=lambda r: r.created_at, reverse=True)

    def latest(self, n: int = 1) -> ExperimentRun | list[ExperimentRun]:
        """Return the most recent run, or a list of the n most recent."""
        runs = self.list_runs()
        if n == 1:
            if not runs:
                raise FileNotFoundError(f"No runs found in {self.data_dir}")
            return runs[0]
        return runs[:n]

    def get(self, name: str) -> ExperimentRun:
        run_dir = self.data_dir / name
        if not run_dir.exists():
            raise FileNotFoundError(f"Run not found: {name}")
        return ExperimentRun.from_dir(run_dir)
