import numpy as np
import pytest
from pyclothoids import Clothoid
from mapforge.ops.joint_connector_fit import basis_for
from mapforge.ops.endpoint_jet_coordinates import complete_endpoint_coefficients


@pytest.mark.parametrize('lengths',[[6,12,8],[6,12,8,6],[6.000001,12.2,8.3,6.5]])
def test_six_end_jets_exact_without_changing_free_coefficients_or_basis(lengths):
    refs=[Clothoid.StandardParams(0,0,0,0,0,L) for L in lengths]
    knots,B,_=basis_for(refs);old=B.t.copy();n=len(B.c)
    free=np.linspace(-2,3,n-6);target=np.array([1.5,.03,.002,-.7,-.08,.001])
    c=complete_endpoint_coefficients(B,knots[-1],free,target)
    actual=np.array([B(t,d)@c/knots[-1]**d for t in (0.,1.) for d in range(3)])
    assert actual==pytest.approx(target,abs=1e-12)
    assert np.array_equal(c[3:-3],free) and np.array_equal(old,B.t)


def test_corrupt_endpoint_input_rejected():
    _,B,_=basis_for([Clothoid.StandardParams(0,0,0,0,0,6)]*3)
    with pytest.raises(ValueError):complete_endpoint_coefficients(B,18,[1],np.zeros(6))
