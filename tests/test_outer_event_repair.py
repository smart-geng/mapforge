"""Real L01 event: source/G2 repair remains rejected for driving dynamics."""
import copy
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
import yaml

from mapforge.repair_web.outer_event import OuterEvent, WEST_SPLIT, EventScope
from mapforge.repair_web.model import digest,complexity
from scripts.check_outer_event_written import check,check_dynamics

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def inputs():
    data=(ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr').read_bytes()
    packet=json.loads((ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json').read_text(encoding='utf8'))
    decisions=yaml.safe_load((ROOT/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml').read_text(encoding='utf8'))
    roles={(d['source_lane_id'],d['contact']) for d in decisions['decisions']}
    return data,packet,roles


@pytest.fixture(scope='module')
def candidate(inputs):
    data,packet,roles=inputs
    event=OuterEvent(data,packet,approved_roles=roles)
    state,report=event.solve()
    assert report['accepted']
    return event,state,report,event.compile(state)


def test_written_real_source_and_event_c2_without_new_refline_segments(inputs,candidate):
    data,packet,_=inputs
    event,x,report,result=candidate
    before=complexity(ET.fromstring(data));after=complexity(ET.fromstring(result))
    assert before['geometry']==after['geometry']==94
    assert min(np.diff(event.scope.knots))>=6
    assert report['free_variables']==24
    assert report['exact_extremum_constraint_rounds']>0  # sampled envelope was insufficient
    independent=check(result,data,packet,event.scope.__dict__,event.inventory)
    assert independent['status']=='PASS_LOCAL_EVENT_NOT_MAP'
    assert independent['other_roads_unchanged']
    assert independent['ids_links_speeds_marks_unchanged']
    assert independent['outside_event_max_change_m']==0
    assert independent['source_event_max_m']<=.7500001
    assert len(independent['birth_jets'])==2
    assert {r['road'] for r in independent['whole_map_internal_failures']}=={'12'}
    assert not independent['map_accepted']


def test_source_role_approval_not_expanded_or_guessed(inputs):
    data,packet,roles=inputs
    with pytest.raises(ValueError,match='approval'):
        OuterEvent(data,packet,approved_roles=set())
    bad=copy.deepcopy(packet)
    bad['observations']['2023041111104060474']['start_width_mm']=3100
    with pytest.raises(ValueError,match='approval'):
        OuterEvent(data,bad,approved_roles=roles)


def test_every_original_path_is_retained_and_not_replaced_with_midpoint(inputs,candidate):
    _,packet,_=inputs
    event=candidate[0]
    ids={o['source_lane_id'] for o in packet['occurrences'] if o['road']=='11'}
    assert {p['source_lane_id'] for p in event.paths}==ids
    for p in event.paths:assert p['points']==packet['observations'][p['source_lane_id']]['lane_path']


@pytest.mark.parametrize('value',[True,float('nan'),float('inf'),2.01,None])
def test_bad_intent_does_not_mutate_baseline(candidate,value):
    event=candidate[0];before=digest(event.data)
    with pytest.raises(ValueError):event.solve(value)
    assert digest(event.data)==before


def test_invalid_coefficients_rejected_and_replay_is_identical(candidate):
    event,x,report,data=candidate
    with pytest.raises(ValueError):event.compile(x+1.)
    assert event.compile(x)==data


def test_short_shape_span_rejected(inputs):
    data,packet,roles=inputs
    scope=EventScope('11',(100.,101.,*WEST_SPLIT.knots[1:]),WEST_SPLIT.births)
    with pytest.raises(ValueError,match='Short'):
        OuterEvent(data,packet,scope,approved_roles=roles)


def test_independent_check_detects_nonwidth_and_outside_changes(inputs,candidate):
    baseline,packet,_=inputs
    event,_,_,data=candidate
    root=ET.fromstring(data)
    road=next(r for r in root.findall('road') if r.get('id')=='11')
    road.find('lanes/laneOffset').set('a','9')
    out=check(ET.tostring(root),baseline,packet,event.scope.__dict__,event.inventory)
    assert out['status']=='FAIL' and out['outside_event_max_change_m']>1
    root=ET.fromstring(data);root.find('header').set('name','tampered')
    assert check(ET.tostring(root),baseline,packet,event.scope.__dict__,event.inventory)['status']=='FAIL'


def test_g2_is_not_dynamic_acceptance_and_source_speed_cannot_be_lowered(inputs,candidate):
    _,packet,_=inputs
    event,_,_,data=candidate
    d=check_dynamics(data,packet,event.scope.__dict__)
    assert d['status']=='FAIL'
    assert d['counts']['FAIL']==37 and d['counts']['BOUNDED']==13
    assert d['max_observed'][0]>2.5 and d['max_observed'][1]>1
    assert all(r['check']['speed_kmh']==60 for r in d['rows'])
    root=ET.fromstring(data)
    road=next(r for r in root.findall('road') if r.get('id')=='11')
    for speed in road.findall('.//speed'):speed.set('max','2')
    with pytest.raises(ValueError,match='speed differs'):
        check_dynamics(ET.tostring(root),packet,event.scope.__dict__)
