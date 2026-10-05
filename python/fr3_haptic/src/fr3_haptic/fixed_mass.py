"""Fixed-mass calculator kept from the former local ``pyrim`` fork.

Temporary: used by the legacy orchestrator until it is rebuilt on
``haptic_teleop.FixedMassProxyRendering`` (see PLAN.md, Phase 2). Note the semantics
differ: this calculator re-seeds the proxy from the plant (``x_i``, ``v_i``) at every
model update; ``FixedMassProxyRendering`` seeds it once from the leader.
"""

from __future__ import annotations

import numpy as np
from pyrim import DynModel, RIMModel

__all__ = ["FixedMassCalculator"]


class FixedMassCalculator:
    """Constant virtual-inertia proxy: a drop-in alternative to :class:`RIMCalculator`.

    Baseline / ablation condition for the RIM. It reuses the whole teleop pipeline
    (same spring/damper coupling, same LCP wall) but replaces the physically
    consistent effective mass with a constant ``mass * I``. The nonlinear and
    external feedthrough terms are zeroed (``z_i = f_eff = 0``), so the operator
    feels a pure passive virtual mass rather than the reduced robot dynamics.

    The proxy position/velocity are seeded from the live model (``x_i``/``v_i``) so
    the constant-mass proxy still tracks the real tool tip; the integrator
    preserves its own integrated state across updates.
    """

    def __init__(self, mass: float) -> None:
        if mass <= 0.0:
            raise ValueError(f"fixed proxy mass must be positive, got {mass}")
        self._mass = float(mass)

    def compute(self, model: DynModel) -> RIMModel:
        m = model.m
        return RIMModel(
            m=m,
            M_eff=self._mass * np.eye(m),
            z_i=np.zeros(m),
            f_eff=np.zeros(m),
            x=model.x_i.copy(),
            v=model.v_i.copy(),
        )
