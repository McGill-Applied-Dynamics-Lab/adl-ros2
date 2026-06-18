"""Tests for InterfaceFrame subspace geometry."""

import numpy as np
import pytest
from pyrim import InterfaceFrame

# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------


class TestInterfaceFrameInit:
    def test_from_direction_normalizes(self):
        frame = InterfaceFrame.from_direction([0.0, 0.0, 5.0])
        assert frame.dim == 1
        np.testing.assert_allclose(frame.basis[:, 0], [0.0, 0.0, 1.0])

    def test_accepts_flat_vector(self):
        frame = InterfaceFrame(np.array([1.0, 0.0, 0.0]))
        assert frame.basis.shape == (3, 1)

    def test_rejects_zero_vector(self):
        with pytest.raises(ValueError):
            InterfaceFrame.from_direction([0.0, 0.0, 0.0])

    def test_rejects_bad_shape(self):
        with pytest.raises(ValueError):
            InterfaceFrame(np.zeros((2, 1)))

    def test_rejects_non_orthogonal_multi_basis(self):
        B = np.array([[1.0, 1.0], [0.0, 1.0], [0.0, 0.0]])  # columns not orthogonal
        with pytest.raises(ValueError):
            InterfaceFrame(B)

    def test_accepts_orthonormal_multi_basis(self):
        B = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])  # x and y
        frame = InterfaceFrame(B)
        assert frame.dim == 2


# ---------------------------------------------------------------------------
# Z-axis reduces to "take/replace the z component" (today's behavior)
# ---------------------------------------------------------------------------


class TestInterfaceFrameZAxis:
    @pytest.fixture
    def frame(self):
        return InterfaceFrame.from_direction([0.0, 0.0, 1.0])

    def test_project_takes_z(self, frame):
        np.testing.assert_allclose(frame.project([1.0, 2.0, 3.0]), [3.0])

    def test_lift_places_on_z(self, frame):
        np.testing.assert_allclose(frame.lift(3.0), [0.0, 0.0, 3.0])

    def test_complement_drops_z(self, frame):
        np.testing.assert_allclose(frame.complement([1.0, 2.0, 3.0]), [1.0, 2.0, 0.0])

    def test_compose_matches_per_axis_assignment(self, frame):
        # subspace coord (proxy z) = 9; free source (leader) = (1, 2, 7)
        # -> [leader_x, leader_y, proxy_z]
        np.testing.assert_allclose(frame.compose(9.0, [1.0, 2.0, 7.0]), [1.0, 2.0, 9.0])


# ---------------------------------------------------------------------------
# Arbitrary (tilted) direction — the generalization
# ---------------------------------------------------------------------------


class TestInterfaceFrameTilted:
    @pytest.fixture
    def frame(self):
        return InterfaceFrame.from_direction([1.0, 0.0, 1.0])  # 45° in x-z plane

    def test_complement_is_orthogonal_to_basis(self, frame):
        v = np.array([3.0, -2.0, 5.0])
        comp = frame.complement(v)
        assert frame.basis[:, 0] @ comp == pytest.approx(0.0, abs=1e-12)

    def test_project_recovers_signed_distance_along_normal(self, frame):
        n = frame.basis[:, 0]
        v = 4.0 * n + np.array([0.0, 7.0, 0.0])  # 4 along normal + something orthogonal
        assert frame.project(v)[0] == pytest.approx(4.0, abs=1e-12)

    def test_compose_recovers_coord_in_subspace(self, frame):
        # project(compose(s, free)) == s, regardless of free
        s, free = 2.5, np.array([1.0, -3.0, 4.0])
        np.testing.assert_allclose(frame.project(frame.compose(s, free)), [s], atol=1e-12)

    def test_compose_keeps_free_in_complement(self, frame):
        # complement(compose(s, free)) == complement(free)
        s, free = 2.5, np.array([1.0, -3.0, 4.0])
        np.testing.assert_allclose(
            frame.complement(frame.compose(s, free)),
            frame.complement(free),
            atol=1e-12,
        )


# ---------------------------------------------------------------------------
# Multi-DoF roundtrip
# ---------------------------------------------------------------------------


class TestInterfaceFrameMultiDof:
    def test_project_lift_roundtrip_2dof(self):
        frame = InterfaceFrame(np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]]))  # x, y
        s = np.array([2.0, -3.0])
        np.testing.assert_allclose(frame.project(frame.lift(s)), s, atol=1e-12)

    def test_compose_2dof(self):
        frame = InterfaceFrame(np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]]))  # x, y in subspace; z free
        # subspace = (x, y) follows s; z follows free
        out = frame.compose(np.array([5.0, 6.0]), np.array([1.0, 1.0, 9.0]))
        np.testing.assert_allclose(out, [5.0, 6.0, 9.0], atol=1e-12)
