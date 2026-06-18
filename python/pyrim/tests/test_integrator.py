"""Tests for RIMIntegrator integration, contact, and model-update semantics."""

import numpy as np
import pytest
from pyrim import RIMModel, RIMIntegrator


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

DT = 1e-3
K = 100.0
D = 10.0
SURFACE = 0.0


def make_rim(x=0.0, v=0.0, m_eff=1.0, f_eff=0.0, z_i=0.0, m=1):
    return RIMModel(
        m=m,
        M_eff=np.array([[m_eff]]),
        z_i=np.array([z_i]),
        f_eff=np.array([f_eff]),
        x=np.array([x]),
        v=np.array([v]),
    )


def make_integrator(**kwargs):
    defaults = dict(interface_dim=1, dt=DT, stiffness=K, damping=D, contact_surface=SURFACE)
    defaults.update(kwargs)
    return RIMIntegrator(**defaults)


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

class TestRIMIntegratorInit:
    def test_step_returns_none_before_update(self):
        drim = make_integrator()
        drim.add_leader_state(np.zeros(1), np.zeros(1))
        x, v = drim.step()
        assert x is None and v is None

    def test_step_ready_after_both(self):
        drim = make_integrator()
        drim.update_rim(make_rim())
        drim.add_leader_state(np.zeros(1), np.zeros(1))
        x, v = drim.step()
        assert x is not None and v is not None


# ---------------------------------------------------------------------------
# Free-space stepping
# ---------------------------------------------------------------------------

class TestRIMIntegratorFreeSpace:
    def test_spring_pulls_rim_toward_leader(self):
        """RIM displaced +1 from stationary leader should converge back."""
        drim = make_integrator()
        drim.update_rim(make_rim(x=1.0, v=0.0))
        drim.add_leader_state(np.zeros(1), np.zeros(1))

        positions = []
        for _ in range(500):
            x, _ = drim.step()
            positions.append(x[0])

        # Should monotonically decrease toward 0 (overdamped or underdamped, but converging)
        assert positions[-1] < positions[0]
        assert abs(positions[-1]) < 0.1  # well on its way toward 0 after 0.5 s

    def test_interface_force_sign(self):
        """When RIM is ahead of leader, coupling force should be positive (pushes leader forward)."""
        drim = make_integrator()
        drim.update_rim(make_rim(x=0.5, v=0.0))
        drim.add_leader_state(np.zeros(1), np.zeros(1))
        drim.step()
        f = drim.get_interface_force()
        # x_rim > x_leader → f = k*(x_rim - x_leader) > 0
        assert f[0] > 0.0

    def test_interface_force_formula(self):
        """Interface force should equal k*(x-xl) + d*(v-vl) using post-step state."""
        drim = make_integrator()
        drim.update_rim(make_rim(x=0.3, v=0.1))
        drim.add_leader_state(np.array([0.1]), np.array([0.0]))
        x, v = drim.step()
        f = drim.get_interface_force()
        expected = K * (x[0] - 0.1) + D * (v[0] - 0.0)
        assert f[0] == pytest.approx(expected, abs=1e-10)

    def test_zero_displacement_zero_force(self):
        """RIM starting at leader position with zero velocity → near-zero force."""
        drim = make_integrator()
        drim.update_rim(make_rim(x=0.5, v=0.0))
        drim.add_leader_state(np.array([0.5]), np.zeros(1))
        drim.step()
        f = drim.get_interface_force()
        assert abs(f[0]) < 1e-6

    def test_f_eff_applied(self):
        """Non-zero f_eff should produce a non-zero velocity from rest at leader position."""
        drim = make_integrator()
        drim.update_rim(make_rim(x=0.0, v=0.0, f_eff=10.0))
        drim.add_leader_state(np.zeros(1), np.zeros(1))
        _, v = drim.step()
        assert v[0] > 0.0

    def test_position_update_uses_old_velocity(self):
        """x_next = x + dt*v_old (forward Euler on position)."""
        drim = make_integrator(stiffness=0.0, damping=0.0)  # disable coupling to isolate kinematics
        drim.update_rim(make_rim(x=0.5, v=2.0))
        drim.add_leader_state(np.zeros(1), np.zeros(1))
        x, _ = drim.step()
        assert x[0] == pytest.approx(0.5 + DT * 2.0, abs=1e-10)


# ---------------------------------------------------------------------------
# Contact / unilateral constraint
# ---------------------------------------------------------------------------

class TestRIMIntegratorContact:
    def test_rim_cannot_go_below_surface(self):
        """RIM pushed hard below the contact surface should stay at the surface."""
        surface = 0.5
        drim = make_integrator(contact_surface=surface, stiffness=1000.0)
        drim.update_rim(make_rim(x=0.6, v=-5.0))  # moving fast into surface
        drim.add_leader_state(np.array([0.0]), np.zeros(1))  # leader far below

        for _ in range(200):
            x, _ = drim.step()
            assert x[0] >= surface - 1e-12

    def test_inward_velocity_killed_at_contact(self):
        """When contact fires and velocity is into the wall, it should be zeroed."""
        surface = 0.0
        drim = make_integrator(contact_surface=surface)
        # Place RIM just above surface with strong inward velocity
        drim.update_rim(make_rim(x=0.001, v=-10.0))
        drim.add_leader_state(np.array([-1.0]), np.zeros(1))  # leader below surface
        _, v = drim.step()
        # Contact fires → velocity zeroed
        assert v[0] >= 0.0

    def test_no_contact_when_above_surface(self):
        """RIM well above surface should evolve freely (no velocity clamp)."""
        drim = make_integrator(contact_surface=SURFACE)
        drim.update_rim(make_rim(x=1.0, v=-0.1))  # slightly negative v, won't reach surface
        drim.add_leader_state(np.array([1.0]), np.zeros(1))
        x, v = drim.step()
        # x_next = 1.0 + DT*(-0.1) = 0.9999 > SURFACE → no clamp
        assert x[0] > SURFACE
        assert v[0] < 0.0  # velocity not zeroed

    def test_contact_force_is_positive_at_surface(self):
        """When RIM is at the surface and leader is below, force should be positive."""
        surface = 0.5
        drim = make_integrator(contact_surface=surface)
        drim.update_rim(make_rim(x=surface, v=0.0))
        drim.add_leader_state(np.array([0.0]), np.zeros(1))  # leader below surface
        drim.step()
        f = drim.get_interface_force()
        assert f[0] > 0.0


# ---------------------------------------------------------------------------
# Model update semantics
# ---------------------------------------------------------------------------

class TestRIMIntegratorModelUpdate:
    def test_state_preserved_on_update(self):
        """update_rim must preserve x and v from the integrated state."""
        drim = make_integrator()
        drim.update_rim(make_rim(x=0.0, v=0.0))
        drim.add_leader_state(np.zeros(1), np.zeros(1))

        # Run for a bit to accumulate some state
        for _ in range(50):
            drim.step()

        x_before, v_before = drim.get_rim_state()

        # Push a new model update (different M_eff)
        drim.update_rim(make_rim(x=99.0, v=99.0, m_eff=2.0))  # x/v in the new RIM should be ignored

        x_after, v_after = drim.get_rim_state()
        assert x_after == pytest.approx(x_before, abs=1e-10)
        assert v_after == pytest.approx(v_before, abs=1e-10)

    def test_new_model_params_used_after_update(self):
        """After update_rim the new M_eff should affect the next step."""
        drim_light = make_integrator()
        drim_heavy = make_integrator()

        rim_light = make_rim(x=1.0, v=0.0, m_eff=0.1)
        rim_heavy = make_rim(x=1.0, v=0.0, m_eff=10.0)

        drim_light.update_rim(rim_light)
        drim_heavy.update_rim(rim_heavy)

        leader = np.zeros(1)
        drim_light.add_leader_state(leader, np.zeros(1))
        drim_heavy.add_leader_state(leader, np.zeros(1))

        _, v_light = drim_light.step()
        _, v_heavy = drim_heavy.step()

        # Lighter mass accelerates faster toward leader
        assert abs(v_light[0]) > abs(v_heavy[0])

    def test_get_rim_state_returns_none_before_init(self):
        drim = make_integrator()
        x, v = drim.get_rim_state()
        assert x is None and v is None
