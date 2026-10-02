"""Geometry of the RIM interface subspace.

The RIM proxy lives along a low-dimensional subspace of 3D Cartesian space (the
contact direction(s)). ``InterfaceFrame`` is the pure-geometry object that maps
between that subspace and 3D:

  - ``project``    3D vector -> subspace coordinates (e.g. the scalar fed to the integrator)
  - ``lift``       subspace coordinates -> 3D (e.g. the RIM component, or the feedforward force)
  - ``complement`` the part of a 3D vector orthogonal to the subspace (the free space)
  - ``compose``    recombine a subspace coordinate with a free-space source into a 3D target

For the single-axis case (``from_direction([0, 0, 1])``) this reduces exactly to
"take/replace the z component"; a tilted contact normal or a multi-DoF basis are
just a different ``basis`` with no change to the callers.
"""

from __future__ import annotations

import numpy as np


class InterfaceFrame:
    """Orthonormal basis ``B`` (3 x k) defining the RIM subspace and its complement."""

    def __init__(self, basis: np.ndarray) -> None:
        B = np.asarray(basis, dtype=float)
        if B.ndim == 1:
            B = B.reshape(3, 1)
        if B.ndim != 2 or B.shape[0] != 3:
            raise ValueError(f"basis must be (3,) or (3, k); got shape {B.shape}")
        if B.shape[1] < 1 or B.shape[1] > 3:
            raise ValueError(f"subspace dim must be 1..3; got k={B.shape[1]}")

        norms = np.linalg.norm(B, axis=0)
        if np.any(norms < 1e-12):
            raise ValueError("basis contains a zero-length vector")
        B = B / norms

        # Require an orthonormal basis: project/lift coordinates are only
        # meaningful (e.g. "signed distance along the contact normal") when the
        # columns are unit and mutually orthogonal. Reject anything else rather
        # than silently re-orthonormalizing and changing what the coords mean.
        gram = B.T @ B
        if not np.allclose(gram, np.eye(B.shape[1]), atol=1e-6):
            raise ValueError("basis columns must be mutually orthogonal")

        self.basis = B

    @classmethod
    def from_direction(cls, direction: np.ndarray) -> "InterfaceFrame":
        """Build a 1-DoF frame from a (possibly non-unit) 3D direction vector."""
        return cls(np.asarray(direction, dtype=float).reshape(3, 1))

    @property
    def dim(self) -> int:
        """Subspace dimension k (number of RIM DoF)."""
        return int(self.basis.shape[1])

    def project(self, v3: np.ndarray) -> np.ndarray:
        """3D vector -> subspace coordinates, shape (k,)."""
        v = np.asarray(v3, dtype=float).reshape(3)
        return self.basis.T @ v

    def lift(self, s: np.ndarray | float) -> np.ndarray:
        """Subspace coordinates (scalar or (k,)) -> 3D vector, shape (3,)."""
        s_arr = np.asarray(s, dtype=float).reshape(self.dim)
        return self.basis @ s_arr

    def complement(self, v3: np.ndarray) -> np.ndarray:
        """Part of a 3D vector orthogonal to the subspace, shape (3,)."""
        v = np.asarray(v3, dtype=float).reshape(3)
        return v - self.basis @ (self.basis.T @ v)

    def compose(self, s: np.ndarray | float, free3: np.ndarray) -> np.ndarray:
        """Recombine subspace coordinate ``s`` with the free-space part of ``free3``.

        Result = lift(s) + complement(free3): the subspace follows ``s`` (the RIM
        proxy), the orthogonal complement follows ``free3`` (e.g. the leader).
        """
        return self.lift(s) + self.complement(free3)
