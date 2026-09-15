import copy
import json
import xml.etree.ElementTree as ET
from unittest.mock import patch

import numpy as np
import pytest
from lxml import etree

from tests.test_source_boundary_joint import fixture
from spikes.road_boundary_family import solve_road
from spikes.clarabel_joint_candidate import interior_qp
from spikes.map_lane_family import ManifestCenters
from mapforge.validate.smoothness import lane_edges_kinematics_at
from scripts.rebuild_map_joint import geometry_stage,topology_speed_signature,cover_source_prefix


def centers(spike=False):
    lanes=[]
    for sid in ('1','2'):
        y=-3.5*(int(sid)-.5)
        points=[[0.,y],[120.,y]]
        if spike and sid=='1':points=[[0.,y],[40.48,y],[40.49,y+1.],[40.50,y],[120.,y]]
        lanes.append({'source_lane_id':sid,'geometry':{'coordinates':points}})
    return {'lanes':lanes}


def solve(road,source,**kw):
    args=dict(source_mode='centers',source_error_budget='absolute',all_source_vertices=True,
              source_certificate=True,source_tol=.35,junction_endpoint_mode='source-parallel')
    args.update(kw)
    with patch('spikes.road_boundary_family._convex_qp',interior_qp):
        return solve_road(road,ManifestCenters(source),np.asarray,**args)


def test_sparse_bracketing_source_constrains_short_section_without_new_geometry(tmp_path):
    tree,road=fixture(tmp_path)
    before=topology_speed_signature(tree.getroot());pv=etree.tostring(road.find('planView'))
    result=solve(road,centers())
    assert result['status']=='CANDIDATE',result
    assert result['whole_source_certificate_requested']
    assert result['source_error_budget']=='absolute'
    intervals=result['certified_source_intervals']
    for sid in ('1','2'):
        rows=[r for r in intervals if r['source']==sid]
        assert sum(r['end_s_m']-r['start_s_m'] for r in rows)==pytest.approx(120.)
        assert any(r['start_s_m']==38. and r['end_s_m']==40. for r in rows)
    assert result['minimum_independent_span_m']>=15.
    assert topology_speed_signature(tree.getroot())==before
    assert etree.tostring(road.find('planView'))==pv
    for s in np.linspace(.01,119.99,400):
        edges=np.asarray(lane_edges_kinematics_at(road,s,'right'))
        assert np.allclose((edges[:-1,0]+edges[1:,0])/2,[-1.75,-5.25],atol=.001)


def test_raw_vertex_between_sample_stations_cannot_disappear(tmp_path):
    _,road=fixture(tmp_path);before=etree.tostring(road)
    result=solve(road,centers(spike=True))
    assert result['status']=='REJECTED'
    assert etree.tostring(road)==before


@pytest.mark.parametrize('options',[
    {'source_error_budget':'legacy-envelope'}, {'all_source_vertices':False}, {'source_mode':'boundaries'}])
def test_certificate_never_inherits_legacy_envelope(options,tmp_path):
    _,road=fixture(tmp_path);before=etree.tostring(road)
    result=solve(road,centers(),**options)
    assert result['status']=='REJECTED'
    assert etree.tostring(road)==before


def test_non_line_chart_cannot_claim_exact_polyline_certificate(tmp_path):
    _,road=fixture(tmp_path);g=road.find('planView/geometry');g.remove(g.find('line'))
    etree.SubElement(g,'arc',curvature='.001')
    result=solve(road,centers())
    assert result['status']=='REJECTED'
    assert 'Line chart' in result['reason']


def test_a_rejected_corridor_cannot_emit_partially_repaired_junction(tmp_path,monkeypatch):
    tree,_=fixture(tmp_path);root=ET.fromstring(etree.tostring(tree));before=ET.tostring(root)
    def reject(road,*a,**kw):
        road.find('.//width').set('a','99')
        return {'status':'REJECTED','reason':'synthetic unsatisfied source'}
    monkeypatch.setattr('spikes.road_boundary_family.solve_road',reject)
    candidate,report=geometry_stage(root,centers())
    assert candidate is None and report['status']=='REJECTED'
    assert ET.tostring(root)==before


def test_speed_and_source_identity_cannot_change_during_geometry_stage(tmp_path):
    tree,_=fixture(tmp_path);root=tree.getroot();before=topology_speed_signature(root)
    root.find('.//speed').set('max','1')
    assert topology_speed_signature(root)!=before


def test_prefix_uses_raw_lane_points_not_reference_chord_without_new_primitive():
    from mapforge.ops.refline_fit import PlanView,PlanSeg
    pv=PlanView(0.,0.,0.,[PlanSeg('line',100.)])
    raw=np.array([[-.72,-1.75],[50.,-1.75],[99.,-1.75]])
    unchanged=raw.copy()
    assert cover_source_prefix(pv,[('source-1',raw)])==pytest.approx(.72)
    assert pv.x0==-0.72 and pv.segs[0].length==pytest.approx(100.72)
    assert pv.x0+pv.segs[0].length==pytest.approx(100.)
    assert len(pv.segs)==1 and np.array_equal(raw,unchanged)


def test_overlay_samples_both_exact_section_endpoints(tmp_path):
    from scripts.basemap_overlay import _lane_lines
    tree,road=fixture(tmp_path)
    _,lines=_lane_lines(road,ds=7.)
    sections=road.findall('lanes/laneSection')
    for pts,side,si,index in lines:
        start=float(sections[si].get('s'))
        end=float(sections[si+1].get('s')) if si+1<len(sections) else float(road.get('length'))
        assert pts[0,0]==pytest.approx(start)
        assert pts[-1,0]==pytest.approx(end)
