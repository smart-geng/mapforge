import copy
import math
import numpy as np
import pytest
from pyclothoids import Clothoid

from scripts.prepare_coupled_event_contract import load_contract
from mapforge.repair_web.coupled_event_sources import EventSources
from mapforge.repair_web.coupled_event_guards import reference_turn
from mapforge.repair_web.coupled_event_intervals import (
    LongEdge, maximum_enclosure, owned_domains, check_turn_domains, target_to_source, source_to_target,
)
from mapforge.repair_web.coupled_event_compiler import SevenRoadCompiler
from mapforge.repair_web.model import parse, digest
from xml.etree import ElementTree as ET


def turn(ref,left=(0.,0.,0.,0.),right=(-3.,0.,0.,0.)):
    return dict(refs=[ref],stations=np.array([0.,ref.length]),
                coefficients=dict(left=np.array([left]),right=np.array([right])))


def test_monotone_join_plane_and_no_nearest_crop():
    e=LongEdge(turn(Clothoid.StandardParams(0,0,0,0,0,10)),'left')
    r=e.intersections([3,0],[1,0]); assert not r['unresolved_intervals']
    assert len(r['roots'])==1 and r['roots'][0]['station_m']==pytest.approx(3.,abs=1e-8)
    assert r['roots'][0]['direction']==1 and not r['nearest_point_selection']


def test_loop_does_not_select_a_convenient_crossing():
    e=LongEdge(turn(Clothoid.StandardParams(0,0,0,1,0,2*math.pi)),'left')
    r=e.intersections([.5,0],[1,0])
    assert not r['unresolved_intervals'] and len(r['roots'])==2
    rows=[dict(role='predecessor',feature='a',oriented_full_xy=np.array([[-1.,0],[.5,0]])),
          dict(role='successor',feature='b',oriented_full_xy=np.array([[.7,0],[2.,0]]))]
    assert owned_domains(e,rows)['status']=='UNRESOLVED_OWNERSHIP'


def test_grazing_plane_is_unresolved_not_false_unique_root():
    e=LongEdge(turn(Clothoid.StandardParams(0,0,0,1,0,math.pi)),'left')
    r=e.intersections([1.,0],[1,0],budget=150)
    assert r['unresolved_intervals']


def test_discontinuous_primitives_not_bridged_by_evaluation_mesh():
    t=turn(Clothoid.StandardParams(0,0,0,0,0,10))
    t['refs'].append(Clothoid.StandardParams(12,0,0,0,0,10))
    t['stations']=np.array([0.,10.,20.]); t['coefficients']={k:np.vstack([v,v]) for k,v in t['coefficients'].items()}
    with pytest.raises(ValueError,match='Discontinuous'): LongEdge(t,'left')


def test_reference_domain_cannot_be_extrapolated_for_a_smaller_error():
    t=turn(Clothoid.StandardParams(0,0,0,0,0,10)); t['stations'][-1]=12.
    with pytest.raises(ValueError,match='interval domain'): LongEdge(t,'left')


def test_continuous_bound_catches_between_sparse_samples():
    # y=4s(1-s), endpoints y=0, midpoint y=1; endpoint-only sampling lies.
    e=LongEdge(turn(Clothoid.StandardParams(0,0,0,0,0,1),(0,4,-4,0)),'left')
    r=target_to_source(e,[0.,1.],np.array([[0.,0],[1.,0]]),budget=600)
    assert r['lower_m']<=1.<=r['upper_m'] and r['lower_m']>.99


def test_budget_exhaustion_retains_complete_upper_bound():
    r=maximum_enclosure([(None,0.,10.)],lambda _,a,b: (1.,1.),budget=1)
    assert not r['resolved'] and r['lower_m']==1. and r['upper_m']==6.
    with pytest.raises(ValueError,match='initial complete domain'):
        maximum_enclosure([(None,0.,1.),(None,1.,2.)],lambda *a:(0.,1.),budget=1)


def test_source_to_curve_bounds_include_chord_error():
    e=LongEdge(turn(Clothoid.StandardParams(0,0,0,0,0,10)),'left')
    r=source_to_target(e,[0.,10.],[np.array([[0.,.2],[10.,.2]])],budget=1200)
    assert r['lower_m']<=.2<=r['upper_m'] and r['upper_m']<.22
    assert r['target_chord_bound_m']>0


@pytest.fixture(scope='module')
def contract(): return load_contract()


def test_real_111_tail_still_fails_continuously(contract):
    result=check_turn_domains(reference_turn(contract.graph.roads['111']),EventSources(contract).by_turn['111'],budget=1200)
    side=result['sides']['right']; assert side['ownership']['status']=='NUMERIC_UNIQUE_JOIN_PLANES'
    r=next(r for r in side['checks'] if r['role']=='predecessor')
    assert r['status']=='FAIL_BOUND' and r['target_to_original']['lower_m']>.573
    assert r['target_to_original']['upper_m']<.575
    assert not result['map_accepted'] and result['original_vertices_removed']==0


def test_empty_owned_source_does_not_skip_target_tail(contract):
    result=check_turn_domains(reference_turn(contract.graph.roads['106']),EventSources(contract).by_turn['106'],budget=1200)
    row=result['sides']['right']['checks'][-1]
    assert row['role']=='successor' and row['owned_original_to_target'] is None
    assert row['status']=='FAIL_BOUND' and row['target_to_original']['lower_m']>.352
    assert row['domain_m'][1]-row['domain_m'][0]==pytest.approx(.04508233,abs=1e-6)
    assert 'FULL_PART_RETAINED' in row['empty_owned_source_status']


@pytest.fixture(scope='module')
def compiler(contract): return SevenRoadCompiler(contract)


@pytest.fixture(scope='module')
def staged(compiler):
    s=compiler.model.evaluate(compiler.model.diagnostic)
    return s,compiler._encode(s)


def test_seven_road_actual_document_readback_does_not_accept_bad_map(compiler,staged):
    snapshot,data=staged; r=compiler.audit_bytes(data,snapshot)
    assert r['xsd_1_5M_valid'] and r['maximum_world_jet_readback_error']<1e-9
    assert r['frozen_upstream_error_m']==0 and r['geometry_joint_or_contact_failure']
    assert not r['export_allowed'] and not r['map_accepted']
    assert r['written_map_files']==0 and not r['esmini_checked']
    assert all(t['geometry_records']==t['width_records']==t['lane_offset_records']==5 for t in r['turns'].values())
    with pytest.raises(ValueError,match='No admitted'): compiler.export(data)


@pytest.mark.parametrize('mode',['speed','other-road','topology','geometry','width','partial-turn'])
def test_tampered_or_partial_transaction_rejected(compiler,staged,mode):
    snapshot,data=staged; root=parse(data); road=next(r for r in root.findall('road') if r.get('id')=='111')
    if mode=='speed': road.find('lanes/laneSection/right/lane/speed').set('max','1.0')
    if mode=='other-road': next(r for r in root.findall('road') if r.get('id')=='12').set('name','changed')
    if mode=='topology': road.find('link/successor').set('elementId','13')
    if mode=='geometry': road.find('planView/geometry').set('x','999')
    if mode=='width': road.find('lanes/laneSection/right/lane/width').set('a','50')
    if mode=='partial-turn':
        before=compiler.contract.graph.roads['111']; index=list(root).index(road)
        root.remove(road); root.insert(index,copy.deepcopy(before))
    with pytest.raises(ValueError): compiler.audit_bytes(ET.tostring(root),snapshot)


def test_failed_assembly_does_not_mutate_reference_or_publish(compiler,staged):
    snapshot,_=staged; missing=dict(snapshot,turns=dict(snapshot['turns'])); del missing['turns']['107']
    original=compiler.contract.reference
    with pytest.raises(ValueError,match='All six'): compiler._encode(missing)
    assert compiler.contract.reference is original
    assert digest(original)==compiler.reference_sha


def test_full_primitive_readback_is_not_only_sampling(compiler,staged):
    snapshot,data=staged; root=parse(data); road=next(r for r in root.findall('road') if r.get('id')=='111')
    road.find('planView/geometry/spiral').set('curvEnd','0.2')
    with pytest.raises(ValueError,match='analytic primitive'): compiler.audit_bytes(ET.tostring(root),snapshot)
