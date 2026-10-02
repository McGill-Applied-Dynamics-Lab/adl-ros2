"""Tests for the RIM facade: construction, geometry delegation, and dynamics."""

import numpy as np
import pytest
from pyrim import RIM, DynModel, InterfaceFrame, RIMIntegrator


DT = 1e-3


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


def make_rim(direction=(0.0, 0.0, 1.0), **kwargs):
    defaults = dict(dt=DT, stiffness=100.0, damping=10.0, contact_surface=0.0)
    defaults.update(kwargs)
    return RIM.create(rim_direction=np.array(direction), **defaults)


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------

class TestRIMConstruction:
    def test_create_builds_matching_frame_and_integrator(self):
        rim = make_rim()
        assert rim.dim == 1
        assert rim.frame.dim == 1
        assert rim.integrator.interface_dim == 1

    def test_create_normalizes_direction(self):
        rim = make_rim(direction=(0.0, 0.0, 7.0))
        np.testing.assert_allclose(rim.frame.basis[:, 0], [0.0, 0.0, 1.0])

    def test_rejects_dim_mismatch(self):
        frame = InterfaceFrame(np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]]))  # k=2
        integ = RIMIntegrator(interface_dim=1, dt=DT, stiffness=1.0, damping=1.0, contact_surface=0.0)
        with pytest.raises(ValueError):
            RIM(frame, integ)


# ---------------------------------------------------------------------------
# Geometry delegation (must match the underlying frame exactly)
# ---------------------------------------------------------------------------

class TestRIMGeometryDelegation:
    def test_geometry_matches_frame(self):
        rim = make_rim(direction=(1.0, 0.0, 1.0))
        v = np.array([3.0, -2.0, 5.0])
        np.testing.assert_allclose(rim.project(v), rim.frame.project(v))
        np.testing.assert_allclose(rim.complement(v), rim.frame.complement(v))
        np.testing.assert_allclose(rim.lift(2.0), rim.frame.lift(2.0))
        np.testing.assert_allclose(rim.compose(2.0, v), rim.frame.compose(2.0, v))


# ---------------------------------------------------------------------------
# Reduction + integration through the facade
# ---------------------------------------------------------------------------

class TestRIMDynamics:
    def test_update_returns_reduced_model(self):
        # The orchestrator logs reduced.M_eff etc., so update() must return the model.
        rim = make_rim()
        reduced = rim.update(make_model(M=np.eye(1), J_i=np.ones((1, 1))))
        assert reduced is not None
        assert reduced.M_eff.shape == (1, 1)
        assert reduced.z_i.shape == (1,)

    def test_update_then_step_ready(self):
        rim = make_rim()
        # Not ready before update
        rim.add_leader_state(np.zeros(1), np.zeros(1))
        assert rim.step() == (None, None)
        # update() runs reduction + refreshes integrator
        rim.update(make_model(M=np.eye(1), J_i=np.ones((1, 1)), x_i=np.array([0.5])))
        rim.add_leader_state(np.zeros(1), np.zeros(1))
        x, v = rim.step()
        assert x is not None and v is not None

    def test_proxy_pulled_toward_leader(self):
        rim = make_rim()
        rim.update(make_model(M=np.eye(1), J_i=np.ones((1, 1)), x_i=np.array([1.0])))
        rim.add_leader_state(np.zeros(1), np.zeros(1))
        last = None
        for _ in range(500):
            last, _ = rim.step()
        assert abs(last[0]) < 0.1  # converged toward leader at 0

    def test_target_position_none_before_state(self):
        rim = make_rim()
        assert rim.target_position(np.array([1.0, 2.0, 3.0])) is None

    def test_target_position_composes_proxy_and_free(self):
        rim = make_rim()  # z direction
        rim.update(make_model(M=np.eye(1), J_i=np.ones((1, 1)), x_i=np.array([0.4])))
        rim.add_leader_state(np.zeros(1), np.zeros(1))
        rim.step()
        x, _ = rim.get_rim_state()
        target = rim.target_position(np.array([1.0, 2.0, 7.0]))
        # x/y from free source, z from proxy
        np.testing.assert_allclose(target, [1.0, 2.0, x[0]], atol=1e-12)
