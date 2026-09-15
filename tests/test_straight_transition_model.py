import numpy as np
import pytest
from pyclothoids import Clothoid, SolveG2

from spikes.sparse_path_model import Limits, sample, straight_candidate
from spikes.straight_transition_model import fit_transition, heading_extrema, straight_caps


def test_caps_reject_overlap_and_do_not_turn_an_entire_line_into_an_s_bend():
    raw=np.array([[0.,0.],[25.,0.],[60.,0.],[100.,0.]])
    with pytest.raises(ValueError,match='overlap'):straight_caps(raw)
    cc,r=fit_transition(raw,Limits(10.))
    assert cc is None and r['status']=='REJECTED'


def test_caps_keep_all_source_points_and_label_two_point_support_as_hypothesis():
    raw=np.array([[0.,0.],[40.,0.],[60.,3.],[80.,3.],[100.,3.]])
    copy=raw.copy();caps=straight_caps(raw)
    assert caps[0][0]==1 and caps[0][3]==2
    assert caps[1][0]==2
    cc,r=fit_transition(raw,Limits(60/3.6),speed_frontier=True)
    t=r['trials'][0]
    assert t['cap_hypotheses']['raw_vertices']==[2,3]
    assert t['model_kind']=='line-3spirals-line'
    assert t['primitive_count']==5 and min(t['lengths_m'])>5
    assert t['g2_connected'] and t['heading_envelope_pass']
    assert t['tube_certified']
    assert t['status']=='REJECTED' and cc is None # still tested at 60, not diagnostic speed
    np.testing.assert_array_equal(raw,copy)


def test_protected_shape_model_can_accept_a_gentle_realizable_transition():
    middle=SolveG2(40.,0.,0.,0.,100.,3.,0.,0.)
    cc=[Clothoid.StandardParams(0.,0.,0.,0.,0.,40.),*middle,
        Clothoid.StandardParams(100.,3.,0.,0.,0.,40.)]
    raw=sample(cc,1.)
    got,result=fit_transition(raw,Limits(5.),speed_frontier=True)
    assert result['status']=='PATH_CANDIDATE' and len(got)==5
    assert result['trials'][0]['tube_certified']
    assert got[0].dk==0 and got[-1].dk==0
    assert got[0].KappaStart==0 and got[-1].KappaEnd==0


def test_analytic_heading_envelope_finds_interior_extrema():
    c=Clothoid.StandardParams(0.,0.,.1,-.04,.002,40.)
    values=heading_extrema([c])
    assert len(values)==3
    assert min(values)==pytest.approx(-.3)


def test_tls_does_not_accept_backtracking_source_as_straight():
    assert straight_candidate(np.array([[0.,0.],[40.,0.],[20.,0.],[100.,0.]])) is None
