import math
import numpy as np
import pytest
from scripts.review_ordinary_source_state import endpoint_jet


def test_direct_stored_boundary_jet_line_and_arc_truth():
    d=dict(families=[dict(features=['b'],knots=[0.]*4+[20.]*4,column_range=[0,4])],
           chart=dict(origin=[0.,0.],tangent=[1.,0.]),reference_curvature=0.)
    q=endpoint_jet(d,3.+np.arange(4)*.2*20/3,'b',5.)
    assert q['xy']==pytest.approx([5.,4.])
    assert q['heading_rad']==pytest.approx(math.atan(.2))
    assert q['curvature_per_m']==pytest.approx(0.,abs=1e-14)
    d['reference_curvature']=.01
    q=endpoint_jet(d,np.full(4,3.),'b',0.)
    assert q['xy']==pytest.approx([0.,3.])
    assert q['curvature_per_m']==pytest.approx(.01/.97)
    with pytest.raises(ValueError,match='extrapolation'):endpoint_jet(d,np.full(4,3.),'b',20.1)
