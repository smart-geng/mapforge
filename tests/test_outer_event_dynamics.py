"""Analytic dynamic constraints and a non-vacuous real-data feasibility refusal."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree as ET
import numpy as np
import pytest
import yaml

from mapforge.repair_web.outer_event import OuterEvent
from mapforge.repair_web.outer_event_dynamics import DynamicEvent,normalized_dynamics
from scripts.check_outer_event_written import check_dynamics

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def real():
    packet=json.loads((ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json').read_text(encoding='utf8'))
    data=(ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr').read_bytes()
    roles=yaml.safe_load((ROOT/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml').read_text(encoding='utf8'))['decisions']
    e=OuterEvent(data,packet,approved_roles={(r['source_lane_id'],r['contact']) for r in roles})
    return e,packet,DynamicEvent(e,packet)


@pytest.mark.parametrize('jets',[(.17,.018,-.002),(0.,0.,0.),(-.25,-.009,.0007)])
def test_analytic_dynamic_derivatives(jets):
    x=np.asarray(jets);_,jac=normalized_dynamics(x,60)
    eps=1e-7
    numeric=np.column_stack([(normalized_dynamics(x+np.eye(3)[i]*eps,60)[0]-
                              normalized_dynamics(x-np.eye(3)[i]*eps,60)[0])/(2*eps) for i in range(3)])
    np.testing.assert_allclose(jac,numeric,atol=2e-7,rtol=1e-7)


def test_model_intervals_match_final_xml_no_newborn_exemption(real):
    e,packet,d=real;x,fit=e.solve();assert fit['accepted']
    computed=d.interval_audit(x)
    written=check_dynamics(e.compile(x),packet,e.scope.__dict__)
    assert computed['counts']==written['counts']=={'FAIL':37,'UNKNOWN':0,'BOUNDED':13}
    assert len(d.spans)==50
    assert any(s['lane']==-4 and s['start']==151.6089 for s in d.spans)
    for expected,actual in zip(computed['rows'],written['rows']):
        assert expected['lane']==actual['lane']
        np.testing.assert_allclose([m['value'] for m in expected['check']['observed_maxima']],
                                   [m['value'] for m in actual['check']['observed_maxima']],rtol=1e-8,atol=1e-8)


def test_actual_coefficient_jacobian_and_one_sided_third_derivative(real):
    e,_,d=real;x,_=e.solve();value,jac=d.sampled(x)
    direction=np.linspace(-.3,.2,e.nvar);eps=1e-6
    numeric=(d.sampled(x+eps*direction)[0]-d.sampled(x-eps*direction)[0])/(2*eps)
    np.testing.assert_allclose(jac@direction,numeric,atol=1e-7,rtol=1e-7)
    # Third derivatives at a knot must use both polynomial limits, not only RHS.
    pair=[w for w in d.witnesses if w['lane']==-3 and abs(w['s']-157.6089)<1e-8]
    assert len(pair)==2
    assert not np.allclose(pair[0]['jet_rows'][2],pair[1]['jet_rows'][2])
    assert np.allclose(pair[0]['jet_rows'][1]@x,pair[1]['jet_rows'][1]@x)


def test_necessary_conflict_stops_blind_optimization_preserves_source(real):
    e,packet,d=real;before=e.data
    state,report=d.solve()
    assert state is None and report['status']=='REJECTED_BEFORE_NONLINEAR_SEARCH'
    necessary=report['necessary']
    assert necessary['source_relaxation_feasible']
    assert necessary['status']=='FIXED_REPRESENTATION_NUMERICALLY_INFEASIBLE'
    assert necessary['minimum_extra_normalized_allowance']>2
    assert report['nonlinear_iterations']==0
    assert report['map_accepted'] is False and necessary['formal_proof'] is False
    assert e.data==before


def test_no_silent_speed_fallback(real):
    e,packet,_=real
    packet=copy.deepcopy(packet)
    packet['observations']['2023041111104132424']['source_max_speed_kmh']=15
    with pytest.raises(ValueError,match='speed'):DynamicEvent(e,packet)


class StraightEvent:
    """Feasible full state for solver plumbing; not a map quality fixture."""
    def __init__(self):
        self.nvar=2;self.origin=np.array([0.,3.5]);self.Z=np.array([[1.],[0.]])
        self.start=0.;self.end=100.;self.count=1;self.births={}
        self.scope=SimpleNamespace(knots=(0.,100.),source_tolerance_m=.1)
        self.road=ET.fromstring('''<road length="100"><lanes><laneSection s="0"><right>
          <lane id="-1"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          <speed sOffset="0" max="16.666666666666668"/><userData code="mapforge.source_lane" value="one"/>
          </lane></right></laneSection></lanes></road>''')
    def row(self,edge,s,derivative=0):return np.zeros(2) if derivative else np.array([1.,-float(edge)])
    def linear_model(self,displacement):
        A=np.eye(2);y=np.array([0.,3.5]);C=np.array([[1.,0.]])
        return A,y,C,np.array([-.1]),np.array([.1]),1,np.array([1.,-1.]),50.
    def solve(self,displacement):return self.origin.copy(),{'accepted':True}
    def source_error(self,x):return {'max_m':abs(x[0]),'rows':[]}


def test_feasible_straight_model_reaches_joint_interval_acceptance():
    event=StraightEvent();d=DynamicEvent(event,{'observations':{'one':{'source_max_speed_kmh':60}}})
    x,report=d.solve()
    np.testing.assert_allclose(x,event.origin)
    assert report['status']=='JOINT_EVENT_CANDIDATE_NOT_MAP'
    assert report['dynamic_audit']['status']=='BOUNDED'
    assert report['map_accepted'] is False
