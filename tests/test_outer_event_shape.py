"""Actual XML nonregression, exact convex reversal and frozen source contracts."""
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
import yaml

from mapforge.repair_web.outer_event import OuterEvent
from mapforge.repair_web.outer_event_shape import wrong_way_distance, reversal_support, solve_shape_no_regression
from mapforge.repair_web.model import digest, complexity
from scripts.check_outer_event_shape import written_shape
from scripts.check_outer_event_written import check

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def event_inputs():
    base=(ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr').read_bytes()
    prior=(ROOT/'out/node4-outer-event-l01-20260915-r3/candidate.xodr').read_bytes()
    packet=json.loads((ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json').read_text(encoding='utf8'))
    roles={(d['source_lane_id'],d['contact']) for d in yaml.safe_load(
        (ROOT/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml').read_text(encoding='utf8'))['decisions']}
    event=OuterEvent(base,packet,approved_roles=roles)
    budgets=[written_shape(prior,event)['by_edge'][str(i)] for i in range(5)]
    return event,base,prior,packet,budgets


def test_exact_integral_finds_reversal_hidden_by_equal_endpoints():
    co=[0.,1.,-3.,2.]
    assert wrong_way_distance(co,1.,1.)==pytest.approx(np.sqrt(3)/9)
    assert wrong_way_distance(co,1.,-1.)==pytest.approx(np.sqrt(3)/9)
    assert wrong_way_distance(co,1.,0.)==pytest.approx(2*np.sqrt(3)/9)
    assert wrong_way_distance([2.,.1,0.,0.],20.,1.)==0


@pytest.mark.parametrize('c,L,sgn', [([0.,0.,0.,float('nan')],1.,1.),([0.,1.],1.,1.),
                                    ([0.,0.,0.,0.],0.,1.),([0.,0.,0.,0.],1.,2.)])
def test_bad_integral_inputs_rejected(c,L,sgn):
    with pytest.raises(ValueError): wrong_way_distance(c,L,sgn)


def test_convex_support_is_lower_bound_not_its_negative(event_inputs):
    event=event_inputs[0]
    rng=np.random.default_rng(519)
    x=event.origin+event.Z@rng.normal(size=event.Z.shape[1])
    values,support=reversal_support(event,x)
    assert values==pytest.approx(support@x)
    for _ in range(5):
        y=event.origin+event.Z@rng.normal(size=event.Z.shape[1])
        actual,_=reversal_support(event,y)
        assert np.all(support@y<=actual+1e-8)


@pytest.fixture(scope='module')
def result(event_inputs):
    event,_,_,_,budgets=event_inputs
    x,report=solve_shape_no_regression(event,budgets)
    assert x is not None
    return x,report,event.compile(x)


def test_real_shape_improves_every_edge_without_new_refline_segments(event_inputs,result):
    event,base,prior,packet,budgets=event_inputs
    x,report,data=result
    final=written_shape(data,event)
    values=np.array([final['by_edge'][str(i)] for i in range(5)])
    assert np.all(values<=np.asarray(budgets)+1e-7)
    assert sum(values)<=sum(budgets)*.5+1e-7
    assert values==pytest.approx(reversal_support(event,x)[0],abs=1e-8)
    assert complexity(ET.fromstring(data))['geometry']==complexity(ET.fromstring(base))['geometry']==94
    assert np.min(np.diff(event.scope.knots))>=6
    readback=check(data,base,packet,event.scope.__dict__,event.inventory)
    assert readback['status']=='PASS_LOCAL_EVENT_NOT_MAP'
    assert readback['source_event_max_m']<=.7500001
    assert readback['outside_event_max_change_m']==0
    assert readback['other_roads_unchanged'] and readback['ids_links_speeds_marks_unchanged']
    assert not report['static_shape_pass'] and not report['map_accepted']
    assert final['monotonicity']=='RESIDUAL_REVERSALS'


def test_replay_and_unmodified_originals(event_inputs,result):
    event,base,_,packet,_=event_inputs
    x,report,data=result
    assert event.data==base
    assert event.compile(x)==data
    assert digest(data)!=digest(base)
    for path in event.paths:
        assert path['points']==packet['observations'][path['source_lane_id']]['lane_path']


def test_solver_failure_cannot_export_a_previous_trial(event_inputs,monkeypatch):
    import mapforge.repair_web.outer_event_shape as module
    monkeypatch.setattr(module,'_convex',lambda *a,**k:(None,dict(status='test-time-budget',iterations=0)))
    x,report=solve_shape_no_regression(event_inputs[0],event_inputs[4])
    assert x is None and report['status']=='GEOMETRY_REJECTED'
    assert not report['map_accepted']


@pytest.mark.parametrize('budgets', [[1.],[-1.]*5,[float('nan')]*5])
def test_unbound_or_invalid_shape_budgets_rejected(event_inputs,budgets):
    with pytest.raises(ValueError):solve_shape_no_regression(event_inputs[0],budgets)
