"""Tests for RIMCalculator against known analytical results."""

import numpy as np
import numpy.testing as npt
import pytest
from pyrim import DynModel, RIMCalculator


def make_model(M, J_i, c=None, b_i=None, tau_ext=None, x_i=None, v_i=None):
    n = M.shape[0]
    m = J_i.shape[0]
    return DynModel(
        n=n,
        m=m,
        q=np.zeros(n),
        q_dot=np.zeros(n),
        x_i=np.zeros(m) if x_i is None else x_i,
        v_i=np.zeros(m) if v_i is None else v_i,
        M=M,
        c=np.zeros(n) if c is None else c,
        J_i=J_i,
        b_i=np.zeros(m) if b_i is None else b_i,
        tau_ext=tau_ext,
    )


class TestRIMCalculatorScalar:
    """1-DOF, 1D interface: all results are scalars that can be verified by hand."""

    def setup_method(self):
        self.calc = RIMCalculator()

    def test_identity_system(self):
        model = make_model(M=np.eye(1), J_i=np.ones((1, 1)))
        rim = self.calc.compute(model)
        npt.assert_allclose(rim.M_eff, [[1.0]])
        assert rim.z_i == pytest.approx([0.0])
        assert rim.f_eff == pytest.approx([0.0])

    def test_scaled_mass(self):
        # M = 2*I, J = I → M_eff = 2, z_i = 0
        model = make_model(M=2.0 * np.eye(1), J_i=np.ones((1, 1)))
        rim = self.calc.compute(model)
        npt.assert_allclose(rim.M_eff, [[2.0]])

    def test_coriolis_projection(self):
        # M = I, J = I, c = [3], b_i = [1] → z_i = M_eff*(J*M^-1*c - b_i) = 1*(3-1) = 2
        model = make_model(M=np.eye(1), J_i=np.ones((1, 1)), c=np.array([3.0]), b_i=np.array([1.0]))
        rim = self.calc.compute(model)
        assert rim.z_i == pytest.approx([2.0])

    def test_feff_with_tau_ext(self):
        # M = 2*I, J = I, tau_ext = [4] → f_eff = M_eff*J*M^-1*tau = 2*(0.5*4) = 4
        model = make_model(M=2.0 * np.eye(1), J_i=np.ones((1, 1)), tau_ext=np.array([4.0]))
        rim = self.calc.compute(model)
        assert rim.f_eff == pytest.approx([4.0])

    def test_feff_zero_when_tau_ext_none(self):
        model = make_model(M=np.eye(1), J_i=np.ones((1, 1)), tau_ext=None)
        rim = self.calc.compute(model)
        assert rim.f_eff == pytest.approx([0.0])

    def test_interface_state_copied(self):
        x_i = np.array([0.5])
        v_i = np.array([1.2])
        model = make_model(M=np.eye(1), J_i=np.ones((1, 1)), x_i=x_i, v_i=v_i)
        rim = self.calc.compute(model)
        assert rim.x == pytest.approx([0.5])
        assert rim.v == pytest.approx([1.2])
        # Mutation of source should not affect RIM
        x_i[0] = 99.0
        assert rim.x == pytest.approx([0.5])


class TestRIMCalculatorMultiDOF:
    """Multi-DOF robot, 1D interface."""

    def setup_method(self):
        self.calc = RIMCalculator()

    def test_two_dof_diagonal_mass(self):
        # n=2, m=1, M=diag(2,3), J=[1,0]
        # M_inv = diag(0.5, 1/3), lambda_inv = 0.5, M_eff = 2
        M = np.diag([2.0, 3.0])
        J_i = np.array([[1.0, 0.0]])
        model = make_model(M=M, J_i=J_i)
        rim = self.calc.compute(model)
        npt.assert_allclose(rim.M_eff, [[2.0]])

    def test_two_dof_feff_only_first_joint(self):
        # tau_ext = [6, 0], J=[1,0], M=diag(2,3)
        # f_eff = M_eff * J * M^-1 * tau = 2 * [1,0] * [3, 0] = 6
        M = np.diag([2.0, 3.0])
        J_i = np.array([[1.0, 0.0]])
        model = make_model(M=M, J_i=J_i, tau_ext=np.array([6.0, 0.0]))
        rim = self.calc.compute(model)
        assert rim.f_eff == pytest.approx([6.0])

    def test_two_dof_feff_second_joint(self):
        # tau_ext = [0, 9], J=[0,1], M=diag(2,3)
        # f_eff = M_eff * J * M^-1 * tau = 3 * [0,1] * [0, 3] = 9
        M = np.diag([2.0, 3.0])
        J_i = np.array([[0.0, 1.0]])
        model = make_model(M=M, J_i=J_i, tau_ext=np.array([0.0, 9.0]))
        rim = self.calc.compute(model)
        assert rim.f_eff == pytest.approx([9.0])
