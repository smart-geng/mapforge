import numpy as np
import pytest
from spikes.map_coordinate_family import source_mouth_heading,coordinate_axis


def test_chart_heading_uses_terminal_source_not_long_chord():
    a=np.array([[0.,0.],[70.,0.],[100.,3.]])
    b=a+[0.,4.];before=a.copy()
    h,evidence=source_mouth_heading([('a',a),('b',b)])
    assert h==pytest.approx(np.arctan2(3.,30.),abs=1e-12)
    assert np.array_equal(a,before)
    assert all(r['raw_vertex_count']==3 for r in evidence['lanes'])
    p,_,_=coordinate_axis(a,a,heading=h)
    assert len(p.segs)==1 and p.segs[0].kind=='line'
    end=np.array([p.x0,p.y0])+p.segs[0].length*np.array([np.cos(p.hdg),np.sin(p.hdg)])
    assert np.allclose(end,a[-1])


def test_chart_does_not_average_opposing_lanes_or_delete_duplicate_points():
    a=np.array([[0.,0.],[50.,0.]])
    with pytest.raises(ValueError,match='inconsistent'):source_mouth_heading([('a',a),('b',a[::-1])])
    with pytest.raises(ValueError,match='duplicate'):source_mouth_heading([('a',np.r_[a,a[-1:]])])


def test_terminal_chart_keeps_complete_far_domain_and_shorter_lane_evidence():
    a=np.array([[0.,0.],[100.,0.]])
    h,e=source_mouth_heading([('a',a),('b',np.array([[95.,3.],[100.,3.]]))])
    assert h==0 and [r['used_arc_span_m'] for r in e['lanes']]==[30.,5.]
    pv,_,_=coordinate_axis(a,a,heading=h)
    assert pv.segs[0].length==100.


def test_conflicting_chart_direction_is_rejected_without_fragmented_fallback():
    a=np.array([[0.,0.],[100.,0.]])
    assert coordinate_axis(a,a,heading=np.pi/2) is None
