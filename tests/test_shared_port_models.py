"""Coupled-road building blocks: no relaxed/partial state is auto-published."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest
from lxml import etree

from spikes.road_boundary_family import solve_road
from spikes.shared_boundary_window import SharedBoundaryWindow
from mapforge.validate.smoothness import lane_edges_kinematics_at
from tests.test_source_boundary_joint import fixture, StraightBoundaries


def with_port(road):
    link = etree.Element('link')
    etree.SubElement(link,'successor',elementType='junction',elementId='2')
    road.insert(0,link)


def test_free_ports_cannot_write_without_complete_dependency_solver(tmp_path):
    _,road=fixture(tmp_path); before=etree.tostring(road)
    r=solve_road(road,StraightBoundaries(),np.asarray,junction_endpoint_mode='free')
    assert r['status']=='REJECTED' and 'dependency' in r['reason']
    assert etree.tostring(road)==before


def test_model_only_has_source_even_when_parent_ports_are_free(tmp_path):
    _,road=fixture(tmp_path); with_port(road); before=etree.tostring(road)
    model=solve_road(road,StraightBoundaries(),np.asarray,min_span=20.,
        junction_endpoint_mode='free',model_only=True,source_error_budget='absolute',
        boundary_association='source-order',all_source_vertices=True)
    x,report=model.preflight()
    assert report['status']=='FEASIBLE'
    result=model.materialize(x)
    assert etree.tostring(road)==before
    for name in ('planView','link'):
        assert etree.tostring(result.find(name))==etree.tostring(road.find(name))
    assert [etree.tostring(e) for e in result.findall('.//lane/link')]==[etree.tostring(e) for e in road.findall('.//lane/link')]
    assert np.max(model.lower-model.C@x)<=1e-7
    for s in np.linspace(.01,119.99,201):
        edges=np.asarray(lane_edges_kinematics_at(result,s,'right'))[:,0]
        assert np.all(abs(edges-[0,-3.5,-7])<=.350001)
    assert etree.tostring(road)==before


def test_model_only_cannot_succeed_without_source_observations(tmp_path):
    _,road=fixture(tmp_path)
    src=SimpleNamespace(lane_boundary_geometries=lambda _:[])
    r=solve_road(road,src,np.asarray,model_only=True,junction_endpoint_mode='free')
    assert r['status']=='REJECTED' and r['reason']=='no source observations'


@pytest.mark.parametrize('side',['right','left'])
def test_one_shared_edge_changes_both_widths_without_moving_outer_edges(tmp_path,side):
    _,road=fixture(tmp_path)
    if side=='left':
        for group in road.findall('lanes/laneSection/right'):
            group.tag='left'
            for lane in group.findall('lane'):
                lane.set('id',str(-int(lane.get('id'))))
    before=etree.tostring(road)
    window=SharedBoundaryWindow(road,side,1,span=40,min_span=20)
    # Find a state with +0.3m at the mouth; cut position/tangent/curvature fixed.
    base,row=window.expression(window.length)
    old=window.old_jets(window.length)[1,0]
    z=np.linalg.lstsq(row[None,:],np.array([old+.3-base]),rcond=None)[0]
    result=window.write(z)
    for s in np.linspace(.01,119.99,171):
        expected=window.jets(s,z)
        read=np.asarray(lane_edges_kinematics_at(result,s,side))
        np.testing.assert_allclose(read,expected[:,:3],atol=1e-8)
        original=np.asarray(lane_edges_kinematics_at(road,s,side))
        np.testing.assert_allclose(read[[0,2]],original[[0,2]],atol=1e-8)
        if s<window.lo:np.testing.assert_allclose(read,original,atol=1e-8)
    np.testing.assert_allclose(window.jets(window.lo,z)[1,:3],window.old_jets(window.lo)[1,:3],atol=1e-9)
    assert window.jets(window.length,z)[1,0]==pytest.approx(old+.3)
    assert etree.tostring(road)==before


def test_window_source_preflight_does_not_disguise_bad_fixed_anchor(tmp_path):
    _,road=fixture(tmp_path)
    group=road.find('lanes'); group.insert(0,etree.Element('laneOffset',s='0',a='2',b='0',c='0',d='0'))
    class Source(StraightBoundaries):
        def lane(self,sid):
            return SimpleNamespace(geometry=np.array([[0,-3.5*(int(sid)-.5)],[120,-3.5*(int(sid)-.5)]]))
    before=etree.tostring(road)
    window=SharedBoundaryWindow(road,'right',1,span=40,min_span=20)
    window.gather_source(Source(),np.asarray)
    _,r=window.source_preflight()
    assert not r['source_feasible'] and r['minimum_additional_source_slack_m']>=1.64
    assert any(not v['mutable'] for v in r['conflicts'])
    assert etree.tostring(road)==before


def test_branch_diagnosis_never_changes_strict_model_or_exposes_relaxed_state():
    from spikes.road_constraint_model import RoadConstraintModel
    # Test exact labels, not row-number modulo assumptions. x must equal 0
    # under branch-C2 and >=1 under source. Position alone is unconstrained.
    model=RoadConstraintModel(None,[],{}, {},np.array([]),np.array([]),np.eye(1),np.zeros(1),
        np.array([[1.]]),np.array([0.]),np.array([[1.]]),np.array([1.]),
        [{'kind':'source'}],[],np.array([]),np.array([]),np.array([]),[],[],
        [{'kind':'branch-tie','derivative':2}])
    old=copy.deepcopy(model.E)
    rows=model.diagnose_branch_ties()
    assert [r['status'] for r in rows]==['FEASIBLE','FEASIBLE','INFEASIBLE']
    assert all(r['diagnostic_only'] and 'x' not in r for r in rows)
    assert model.preflight()[1]['status']=='INFEASIBLE'
    np.testing.assert_array_equal(old,model.E)
