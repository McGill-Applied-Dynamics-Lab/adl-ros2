"""The RIM facade: one object tying together geometry, reduction, and integration.

A ``RIM`` owns the three collaborators and exposes a single API:

  - geometry  (``project`` / ``lift`` / ``complement`` / ``compose``) — delegates to
    the :class:`InterfaceFrame`; pure and lock-free.
  - reduction (``update``) — runs the :class:`RIMCalculator` on a dynamics snapshot
    and refreshes the integrator's model terms.
  - integration (``add_leader_state`` / ``step`` / ``get_*``) — delegates to the
    thread-safe :class:`RIMIntegrator`.

The facade adds no shared mutable state of its own (the frame and calculator are
stateless), so the integrator's lock remains the single synchronization point.
"""

from __future__ import annotations

import numpy as np

from .calculator import RIMCalculator
from .frame import InterfaceFrame
from .integrator import RIMIntegrator
from .models import DynModel, RIMModel


class RIM:
    """Coordinates an InterfaceFrame, a RIMIntegrator, and a RIMCalculator."""

    def __init__(
        self,
        frame: InterfaceFrame,
        integrator: RIMIntegrator,
        calculator: RIMCalculator | None = None,
    ) -> None:
        if integrator.interface_dim != frame.dim:
            raise ValueError(
                f"integrator interface_dim ({integrator.interface_dim}) must match frame dim ({frame.dim})"
            )
        self.frame = frame
        self.integrator = integrator
        self.calculator = calculator if calculator is not None else RIMCalculator()

    @classmethod
    def create(
        cls,
        *,
        rim_direction: np.ndarray,
        dt: float,
        stiffness: float,
        damping: float,
        contact_surface: float,
        vel_filter_alpha: float = 1.0,
    ) -> "RIM":
        """Build a 1-DoF RIM along ``rim_direction`` with a matching integrator."""
        frame = InterfaceFrame.from_direction(rim_direction)
        integrator = RIMIntegrator(
            interface_dim=frame.dim,
            dt=dt,
            stiffness=stiffness,
            damping=damping,
            contact_surface=contact_surface,
            vel_filter_alpha=vel_filter_alpha,
        )
        return cls(frame, integrator)

    # -- geometry (pure, lock-free) ----------------------------------------

    @property
    def dim(self) -> int:
        return self.frame.dim

    def project(self, v3: np.ndarray) -> np.ndarray:
        return self.frame.project(v3)

    def lift(self, s: np.ndarray | float) -> np.ndarray:
        return self.frame.lift(s)

    def complement(self, v3: np.ndarray) -> np.ndarray:
        return self.frame.complement(v3)

    def compose(self, s: np.ndarray | float, free3: np.ndarray) -> np.ndarray:
        return self.frame.compose(s, free3)

    # -- reduction + integration (thread-safe via the integrator) ----------

    def update(self, model: DynModel) -> RIMModel:
        """Reduce a dynamics snapshot, refresh the integrator's terms, and return the reduced model."""
        reduced = self.calculator.compute(model)
        self.integrator.update_rim(reduced)
        return reduced

    def add_leader_state(self, position: np.ndarray, velocity: np.ndarray) -> None:
        self.integrator.add_leader_state(position, velocity)

    def step(self) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
        return self.integrator.step()

    def get_rim_state(self) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
        return self.integrator.get_rim_state()

    def get_interface_force(self) -> np.ndarray:
        return self.integrator.get_interface_force()

    def get_leader_vel(self) -> np.ndarray:
        return self.integrator.get_leader_vel()

    # -- convenience -------------------------------------------------------

    def target_position(self, free3: np.ndarray) -> np.ndarray | None:
        """Full 3D target: proxy along the RIM subspace, ``free3`` in the complement.

        Returns None if the integrator has no state yet.
        """
        x, _ = self.integrator.get_rim_state()
        if x is None:
            return None
        return self.compose(x, free3)
