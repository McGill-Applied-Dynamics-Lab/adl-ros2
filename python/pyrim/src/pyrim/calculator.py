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
