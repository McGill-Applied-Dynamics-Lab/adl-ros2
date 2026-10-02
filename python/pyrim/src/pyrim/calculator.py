"""RIM reduction from full robot dynamics."""

from __future__ import annotations

import numpy as np

from .models import RIMModel, DynModel


class RIMCalculator:
    """Compute reduced interface model parameters from a full model snapshot."""

    def compute(self, model: DynModel) -> RIMModel:
        """Compute the Reduced Interface Model (RIM) of a Dynamic Model.
        Computes:
        - Effective mass
        - Effective force
        - Effective NL terms

        Args:
            model (DynModel): The dynamic model

        Returns:
            RIM: The reduced interface model
        """
        m_inv = np.linalg.inv(model.M)
        lambda_inv = model.J_i @ m_inv @ model.J_i.T
        m_eff = np.linalg.inv(lambda_inv)
        z_i = m_eff @ (model.J_i @ m_inv @ model.c - model.b_i)
        if model.tau_ext is None:
            f_eff = np.zeros(model.m)
        else:
            f_eff = m_eff @ model.J_i @ m_inv @ model.tau_ext
        return RIMModel(m=model.m, M_eff=m_eff, z_i=z_i, f_eff=f_eff, x=model.x_i.copy(), v=model.v_i.copy())


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
