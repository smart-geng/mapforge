import numpy as np
import pytest
from pyclothoids import Clothoid

from mapforge.ops.shared_section import (
    solve_shared_section, cubic_bernstein, width_control_bounds,
)
from spikes.connector_cross_section import flat_join_spline, jet


LENGTHS = np.array([12., 25., 18., 12., 14.])
K = np.array([0., .015, .04, .02, -.01, 0.])
START = np.array([[1.8, .005, .0001], [-1.7, .01, -.0002]])
END = np.array([[2.1, -.01, .0003], [-1.5, -.005, .0001]])


def test_zero_shear_reproduces_existing_flat_model_without_new_records():
    cls = [Clothoid.StandardParams(0, 0, 0, k, (ke-k)/l, l)
           for l, k, ke in zip(LENGTHS, K[:-1], K[1:])]
    new = solve_shared_section(LENGTHS, K, START, END)
    assert len(new.knots)-1 == len(cls)+2
    assert new.rank == 4*(len(cls)+2)
    for i in range(2):
        knots, co = flat_join_spline(cls, START[i], END[i])
        assert new.knots == pytest.approx(knots)
        assert new.coefficients[i] == pytest.approx(co, abs=1e-10)


def curvature(j, k, dk):
    t, p, pp = j
    a = 1-k*t
    return (k*a*a+a*pp+p*(dk*t+2*k*p))/(a*a+p*p)**1.5


def test_nonzero_shared_shear_preserves_world_g2_across_entire_width():
    q = np.array([.03, -.07, .06, .02])
    result = solve_shared_section(LENGTHS, K, START, END, q)
    dk = np.diff(K)/LENGTHS
    assert result.residual < 1e-10
    for b in range(2):
        assert jet(result.coefficients[b, 0], 0) == pytest.approx(START[b], abs=1e-10)
        assert jet(result.coefficients[b, -1], np.diff(result.knots)[-1]) == pytest.approx(END[b], abs=1e-10)
    for j, s in enumerate(np.cumsum(LENGTHS)[:-1]):
        i = int(np.argmin(abs(result.knots-s)))
        jl = np.array([jet(c, result.knots[i]-result.knots[i-1]) for c in result.coefficients[:, i-1]])
        jr = np.array([jet(c, 0.) for c in result.coefficients[:, i]])
        assert jl[:, :2] == pytest.approx(jr[:, :2], abs=1e-10)
        assert jr[:, 1]/(1-K[j+1]*jr[:, 0]) == pytest.approx(np.repeat(q[j], 2))
        # Not just the two edges: every sampled affine fraction, including
        # the lane center, has the same limiting world curvature.
        for f in np.linspace(0, 1, 11):
            left, right = (1-f)*jl[0]+f*jl[1], (1-f)*jr[0]+f*jr[1]
            assert curvature(left, K[j+1], dk[j]) == pytest.approx(curvature(right, K[j+1], dk[j+1]), abs=1e-10)
    flat = solve_shared_section(LENGTHS, K, START, END)
    assert np.max(abs(result.coefficients-flat.coefficients)) > .01


def test_ordered_multi_lane_boundaries_and_exact_basis_conversion():
    start = np.array([[5., 0., 0.], [1.5, 0., 0.], [-2., 0., 0.]])
    result = solve_shared_section([20., 30., 20.], [0., 0., 0., 0.], start, start)
    assert width_control_bounds(result) == pytest.approx(np.full((2, 5, 4), 3.5))
    c = np.array([1., -.2, .01, .002])
    b = cubic_bernstein(c, 12.)
    for u in np.linspace(0, 1, 30):
        value = b@np.array([(1-u)**3, 3*u*(1-u)**2, 3*u*u*(1-u), u**3])
        assert value == pytest.approx(jet(c, u*12)[0])


def test_bernstein_is_sufficient_not_necessary_and_must_not_clip():
    # (s-.5)^2+.05 is positive throughout [0,1], but its cubic Bernstein
    # hull has a negative coefficient. A failed hull is NOT infeasibility.
    c = np.array([.30, -1., 1., 0.])
    assert min(cubic_bernstein(c, 1.)) < 0
    assert jet(c, .5)[0] == pytest.approx(.05)
    negative = np.array([-2., 0., 0., 0.])
    assert cubic_bernstein(negative, 20.) == pytest.approx(np.full(4, -2.))


@pytest.mark.parametrize('lengths,k,start,end,q', [
    ([0, 2], [0, 0, 0], START, END, None),
    ([2, 2], [0, 0], START, END, None),
    ([2, 2], [0, 0, 0], START, END, [0, 0]),
    ([2, 2], [0, 0, 0], START[:1], END[:1], None),
    ([2, 2], [0, np.nan, 0], START, END, None),
])
def test_invalid_inputs_rejected(lengths, k, start, end, q):
    with pytest.raises(ValueError):
        solve_shared_section(lengths, k, start, end, q)
