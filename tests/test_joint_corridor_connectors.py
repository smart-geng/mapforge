import copy
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from lxml import etree

from tests.test_joint_mouth_candidate import turn
from tests.test_source_boundary_joint import fixture
from tests.test_map_joint_source_certificate import centers
from spikes.joint_corridor_connectors import solve,recover_parameters,center_row
from spikes.joint_mouth_candidate import connectors
from spikes.map_lane_family import ManifestCenters
from spikes.road_boundary_family import solve_road
from scripts.rebuild_map_joint import topology_speed_signature
from mapforge.validate.junction_edges import audit
from scripts.internal_edge_jets import audit as inside


def setup_network():
    root=turn();manifest={'lanes':[]}
    for rid,role,xy in [('10','successor',[[0.,0.],[20.,0.]]),
                        ('11','predecessor',[[55.,35.],[55.,55.]])]:
        road=root.find(f"road[@id='{rid}']")
        group=ET.Element('link');ET.SubElement(group,role,elementType='junction',elementId='1');road.insert(0,group)
        lane=road.find('lanes/laneSection/right/lane')
        ET.SubElement(lane,'userData',code='mapforge.source_lane',value=rid)
        manifest['lanes'].append({'source_lane_id':rid,'geometry':{'coordinates':xy}})
    connectors(root)
    return root,manifest


def test_joint_solver_is_one_state_and_preserves_source_topology_and_edges():
    root,manifest=setup_network();before=ET.tostring(root);sig=topology_speed_signature(root)
    result,report=solve(root,manifest,target_ratio=.999)
    assert report['status']=='CANDIDATE',report
    assert report['connections']==1 and report['variables']==report['ordinary_variables']+5
    assert ET.tostring(root)==before
    new,curves=result;connectors(new,frozen_curves=curves)
    assert topology_speed_signature(new)==sig
    assert audit(new)['status']=='PASS'
    assert inside(new)['status']=='PASS'
    assert len(new.find("road[@id='100']/planView"))==3
    assert all(c.length>=5.-1e-7 for c in curves['100'])


def test_cubic_state_recovery_and_center_map_are_exact(tmp_path):
    _,road=fixture(tmp_path)
    # Start with a source-constrained long family (not the fixture's short impulse).
    from spikes.clarabel_joint_candidate import interior_qp
    from unittest.mock import patch
    with patch('spikes.road_boundary_family._convex_qp',interior_qp):
        r=solve_road(road,ManifestCenters(centers()),np.asarray,source_mode='centers',
            source_error_budget='absolute',all_source_vertices=True,source_certificate=True)
    assert r['status']=='CANDIDATE'
    m=solve_road(road,ManifestCenters(centers()),np.asarray,source_mode='centers',
        source_error_budget='absolute',all_source_vertices=True,source_certificate=True,model_only=True)
    x=recover_parameters(m)
    assert np.max(abs(m.E@x-m.er),initial=0.)<1e-7
    assert np.max(m.lower-m.C@x,initial=0.)<1e-7
    from mapforge.validate.smoothness import lane_edges_at
    e=lane_edges_at(road,0.,'right')
    assert center_row(m,-1,'start')@x==pytest.approx((e[0]+e[1])/2,abs=1e-9)
    assert abs(center_row(m,-1,'start')@x+1.75)<.35


def test_frozen_curve_set_must_be_complete():
    root,_=setup_network();before=ET.tostring(root)
    with pytest.raises(ValueError,match='complete dependency'):connectors(root,frozen_curves={})
    assert ET.tostring(root)==before


def test_frozen_curve_rejects_stale_parent_state():
    from pyclothoids import Clothoid
    root,_=setup_network();r=root.find("road[@id='100']");curves=[]
    for g in r.findall('planView/geometry'):
        s=g.find('spiral');length=float(g.get('length'));a=float(s.get('curvStart'));b=float(s.get('curvEnd'))
        curves.append(Clothoid.StandardParams(float(g.get('x')),float(g.get('y')),float(g.get('hdg')),a,(b-a)/length,length))
    root.find("road[@id='10']/lanes/laneOffset").set('a','2.5')
    before=ET.tostring(root)
    with pytest.raises(ValueError,match='no longer matches'):connectors(root,frozen_curves={'100':curves})
    assert ET.tostring(root)==before


def test_joint_rejection_cannot_return_partially_repaired_root(monkeypatch):
    root,manifest=setup_network();before=ET.tostring(root)
    monkeypatch.setattr('spikes.joint_corridor_connectors.interior_qp',lambda *a:(None,{'status':'synthetic failure'}))
    result,report=solve(root,manifest)
    assert result is None and report['status']=='REJECTED'
    assert ET.tostring(root)==before


def test_source_connector_speed_cannot_be_silently_overridden():
    root,manifest=setup_network()
    ET.SubElement(root.find("road[@id='100']/lanes/laneSection/right/lane"),'speed',sOffset='0',max='20',unit='m/s')
    with pytest.raises(ValueError,match='source connector speed'):solve(root,manifest)


def test_near_straight_cross_lane_transition_is_not_misclassified_as_turn_wiggle():
    from pyclothoids import SolveG2
    from spikes.connector_shape import heading_values,heading_envelope
    curves=SolveG2(0.,0.,0.,0.,40.,6.,0.,0.)
    lengths=[c.length for c in curves];ks=[curves[0].KappaStart]+[c.KappaEnd for c in curves]
    values=heading_values(lengths,ks,0.)
    assert max(abs(values))<1.
    assert max(abs(values))*math.pi/4>math.radians(5)
    assert heading_envelope(0.)[2]=='forward-single-S'
    assert ks[1]*ks[2]<0  # One necessary S, not an extra turn/loop.


def test_heading_extremum_detects_interior_backtracking():
    from spikes.connector_shape import heading_values
    assert max(abs(heading_values([10.,10.,10.],[0.,.4,-.4,0.],0.)))>1.


@pytest.mark.parametrize('seed',[math.pi,-math.pi])
def test_half_turn_preserves_initial_side_at_angle_branch(seed):
    from spikes.connector_shape import turn_branch
    assert turn_branch(-seed,seed)==pytest.approx(seed)
    with pytest.raises(ValueError,match='extra revolution'):turn_branch(0.,2*math.pi)


@pytest.mark.parametrize('delta',[-5e-7,5e-7])
def test_near_half_turn_keeps_exact_modulo_heading(delta):
    from spikes.connector_shape import turn_branch
    seed=math.pi+delta
    result=turn_branch(seed-2*math.pi,seed)
    assert result==pytest.approx(seed,abs=1e-12)
    with pytest.raises(ValueError,match='finite'):turn_branch(float('nan'),seed)


def test_frozen_internal_seam_is_checked_before_atomic_write():
    from pyclothoids import Clothoid
    root,_=setup_network();curves=[]
    for g in root.findall("road[@id='100']/planView/geometry"):
        s=g.find('spiral');length=float(g.get('length'));a=float(s.get('curvStart'));b=float(s.get('curvEnd'))
        curves.append(Clothoid.StandardParams(float(g.get('x')),float(g.get('y')),float(g.get('hdg')),a,(b-a)/length,length))
    c=curves[1];curves[1]=Clothoid.StandardParams(c.XStart+1.,c.YStart,c.ThetaStart,c.KappaStart,c.dk,c.length)
    before=ET.tostring(root)
    with pytest.raises(ValueError,match='internal G2 seam'):connectors(root,frozen_curves={'100':curves})
    assert ET.tostring(root)==before


def test_joint_failed_restoration_keeps_original_map(monkeypatch):
    root,manifest=setup_network();before=ET.tostring(root)
    monkeypatch.setattr('spikes.joint_corridor_connectors.interior_qp',lambda *a:(None,{'status':'test failure'}))
    monkeypatch.setattr('spikes.nonlinear_feasibility.restore_joint',lambda *a,**kw:(None,{'status':'REJECTED','reason':'test failure'}))
    result,report=solve(root,manifest,restore=True)
    assert result is None and report['restoration']['status']=='REJECTED'
    assert ET.tostring(root)==before


def test_rotating_axes_are_shared_with_all_ports_and_edges():
    root,manifest=setup_network();before=ET.tostring(root);signature=topology_speed_signature(root)
    solved,report=solve(root,manifest,target_ratio=.999,rotating_axes=True,restore=True)
    assert solved is not None,report
    out,curves=solved;connectors(out,frozen_curves=curves)
    assert report['fixed_reference_axes'] is False
    assert set(report['axis_angle_changes_deg'])=={'10','11'}
    assert max(abs(v) for v in report['axis_angle_changes_deg'].values())>1e-5
    assert audit(out)['status']=='PASS' and inside(out)['status']=='PASS'
    assert topology_speed_signature(out)==signature
    assert all(len(r.findall('planView/geometry'))==1 for r in out.findall('road') if r.get('junction')=='-1')
    assert ET.tostring(root)==before


def test_joint_initialization_keeps_source_width_and_all_ports_in_one_state():
    root,manifest=setup_network();before=ET.tostring(root);signature=topology_speed_signature(root)
    solved,report=solve(root,manifest,target_ratio=.999,rotating_axes=True,restore=True,
                        joint_initialization=True,source_widths={'10':3.5,'11':3.5})
    assert solved is not None,report
    out,curves=solved;connectors(out,frozen_curves=curves)
    assert report['joint_initialization'] and report['restoration']['source_slack_search_only']
    assert report['restoration']['exact_violation']<5e-8
    assert max(report['final_source_observation_max_m'].values())<=.35
    assert topology_speed_signature(out)==signature
    assert audit(out)['status']=='PASS' and inside(out)['status']=='PASS'
    from mapforge.validate.map_width import width_extrema
    for road in out.findall('road'):
        if road.get('junction')!='-1':continue
        lane=road.find('lanes/laneSection/right/lane')
        assert all(abs(w-3.5)<1e-7 for s,w in width_extrema(lane,0.,0.,float(road.get('length'))))
    assert all(c.length>=5.-1e-7 for c in curves['100'])
    assert ET.tostring(root)==before


def test_joint_initialization_never_runs_the_old_independent_admission(monkeypatch):
    from scripts.rebuild_map_joint import geometry_stage
    root,manifest=setup_network();before=ET.tostring(root)
    def forbidden(*a,**kw):raise AssertionError('independent fixed-axis admission must not run')
    monkeypatch.setattr('scripts.rebuild_map_joint.family.solve_road',forbidden)
    result,report=geometry_stage(root,manifest,coupled=True,restore=True,rotating_axes=True,
                                 joint_initialization=True,source_widths={'10':3.5,'11':3.5})
    assert result is not None,report
    assert all(r['status']=='JOINT_SEED_ONLY' for r in report['corridors'])
    assert ET.tostring(root)==before


def test_joint_initialization_failure_keeps_original_and_cannot_write_seed(monkeypatch):
    from scripts.rebuild_map_joint import geometry_stage
    root,manifest=setup_network();before=ET.tostring(root)
    monkeypatch.setattr('spikes.nonlinear_feasibility.restore_joint',lambda *a,**kw:(None,{
        'status':'REJECTED','reason':'nonzero complete-source violation','worst_positive_rows':[]}))
    result,report=geometry_stage(root,manifest,coupled=True,restore=True,rotating_axes=True,joint_initialization=True)
    assert result is None and report['status']=='REJECTED'
    assert ET.tostring(root)==before


def test_joint_initialization_requires_all_dependent_modes():
    from scripts.rebuild_map_joint import geometry_stage
    root,manifest=setup_network()
    with pytest.raises(ValueError,match='requires'):solve(root,manifest,joint_initialization=True)
    with pytest.raises(ValueError,match='requires'):
        geometry_stage(root,manifest,coupled=True,rotating_axes=True,joint_initialization=True)


def test_inexact_family_recovery_is_allowed_only_for_unqualified_seeds(tmp_path):
    _,road=fixture(tmp_path)
    model=solve_road(road,ManifestCenters(centers()),np.asarray,source_mode='centers',
                    source_error_budget='absolute',all_source_vertices=True,source_certificate=True,model_only=True)
    with pytest.raises(ValueError,match='not representable'):recover_parameters(model)
    state=recover_parameters(model,seed_only=True)
    assert np.isfinite(state).all()
    assert np.max(abs(model.E@state-model.er),initial=0.)<1e-7
    mask=np.array([l['kind'] not in ('source-center','whole-source-center') for l in model.labels])
    assert np.max(model.lower[mask]-model.C[mask]@state,initial=0.)<1e-7


def test_seed_structural_projection_does_not_solve_or_drop_source_constraints(tmp_path):
    _,road=fixture(tmp_path)
    model=solve_road(road,ManifestCenters(centers()),np.asarray,source_mode='centers',
                    source_error_budget='absolute',all_source_vertices=True,source_certificate=True,model_only=True)
    # Add a structural lower bound inconsistent with the old LS seed but
    # compatible with equalities. Keep the source rows unchanged.
    row=np.zeros(model.nvar);row[model.families[0].columns]=model.families[0].basis(60.)
    model.C=np.vstack([model.C,row]);model.lower=np.r_[model.lower,1.]
    model.labels.append({'kind':'width'})
    state=recover_parameters(model,seed_only=True)
    assert row@state>=1.-1e-7
    assert any(l['kind']=='whole-source-center' for l in model.labels)
