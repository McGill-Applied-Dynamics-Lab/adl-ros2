"""Core data models for RIM computation."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class DynModel:
    """Instantaneous robot model used to compute reduced dynamics."""

    n: int
    m: int
    q: np.ndarray
    q_dot: np.ndarray
    x_i: np.ndarray
    v_i: np.ndarray
    M: np.ndarray
    c: np.ndarray
    J_i: np.ndarray
    b_i: np.ndarray
    tau_ext: np.ndarray | None = None
    stamp_s: float = 0.0


@dataclass
class RIM:
    """Reduced interface model state and parameters."""

    m: int
    M_eff: np.ndarray
    z_i: np.ndarray
    f_eff: np.ndarray
    x: np.ndarray
    v: np.ndarray
