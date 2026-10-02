import numpy as np

from pyrim import DynModel, RIMCalculator


def test_rim_calculator_shapes():
    calc = RIMCalculator()
    n = 7
    m = 1

    model = DynModel(
        n=n,
        m=m,
        q=np.zeros(n),
        q_dot=np.zeros(n),
        x_i=np.zeros(m),
        v_i=np.zeros(m),
        M=np.eye(n),
        c=np.zeros(n),
        J_i=np.ones((m, n)),
        b_i=np.zeros(m),
        tau_ext=np.zeros(n),
    )

    rim = calc.compute(model)

    assert rim.M_eff.shape == (m, m)
    assert rim.z_i.shape == (m,)
    assert rim.f_eff.shape == (m,)
    assert rim.x.shape == (m,)
    assert rim.v.shape == (m,)
