"""RIM reduction from full robot dynamics."""

from __future__ import annotations

import numpy as np

from .rim_types import DynModel, RIM


class RIMCalculator:
    """Compute reduced interface model parameters from a full model snapshot."""

    def compute(self, model: DynModel) -> RIM:
        m_inv = np.linalg.inv(model.M)
        lambda_inv = model.J_i @ m_inv @ model.J_i.T
        m_eff = np.linalg.inv(lambda_inv)
        z_i = m_eff @ ((model.J_i @ m_inv @ model.c) - model.b_i)
        if model.tau_ext is None:
            f_eff = np.zeros(model.m)
        else:
            f_eff = m_eff @ model.J_i @ m_inv @ model.tau_ext
        return RIM(m=model.m, M_eff=m_eff, z_i=z_i, f_eff=f_eff, x=model.x_i.copy(), v=model.v_i.copy())
