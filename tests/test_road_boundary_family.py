"""Whole-road candidate: numerical formula, geometry and atomic rejection."""
import numpy as np
import copy
import pytest
from lxml import etree

from tests.test_source_boundary_joint import fixture, StraightBoundaries
from spikes.road_boundary_family import solve_road, world_kinematics
from spikes.road_boundary_family import ordered_source_boundary
from spikes.road_boundary_family import phase_certificate
from mapforge.validate.smoothness import lane_edges_kinematics_at, _edge_world_curvature
from scripts.source_lane_consistency import compare_lane


def test_source_order_is_independent_of_list_order_and_wrong_target_location():
    low = np.array([[0., -8.], [30., -7.]])
    high = np.array([[0., -4.], [30., -3.]])
    for pair in ([low, high], [high, low]):
        np.testing.assert_array_equal(ordered_source_boundary(pair, select_high=True), high)
        np.testing.assert_array_equal(ordered_source_boundary(pair, select_high=False), low)


def test_phase_certificate_reports_dual_support_not_all_relaxed_constraints():
    from scipy.optimize import linprog
    # x >= 2, x <= 0 conflict; the irrelevant third row must not be blamed.
    result=linprog([0.,1.],A_ub=[[-1.,-1.],[1.,-1.],[-1.,-1.]],
                   b_ub=[-2.,0.,100.],bounds=[(None,None),(0,None)])
    rows=phase_certificate(result,[{'row':i} for i in range(3)])
    assert {r['row'] for r in rows}=={0,1}
    assert sum(r['dual_weight'] for r in rows)==pytest.approx(1.)


def test_crossing_or_incomplete_boundaries_are_not_rebound_to_nearest_target():
    a = np.array([[0., 0.], [10., 2.]])
    b = np.array([[0., 2.], [10., 0.]])
    with pytest.raises(ValueError, match='cross'):
        ordered_source_boundary([a,b], select_high=True)
    with pytest.raises(ValueError, match='two'):
        ordered_source_boundary([a], select_high=False)


def test_world_kinematics_circle_and_spiral_parallel_curve():
    t=3.0; k=.01; kp=.00005; factor=1-k*t
    result=world_kinematics(np.array([t,0.,0.,0.]),k,kp)
    assert result[0]==pytest.approx(k/factor,abs=1e-14)
    assert result[1]==pytest.approx(kp/factor**3,abs=1e-14)


def test_world_kinematics_matches_finite_difference_in_lane_arclength():
    t,d1,d2,d3=2.,.025,.0008,.0001
    k,kp=.002,.00003
    jets=np.array([t,d1,d2,d3])
    got=world_kinematics(jets,k,kp)
    def curvature(s):
        return _edge_world_curvature(t+d1*s+d2*s*s/2+d3*s**3/6,
            d1+d2*s+d3*s*s/2,d2+d3*s,k+kp*s,kp)
    step=1e-4; speed_factor=np.hypot(1-k*t,d1)
    finite=(curvature(step)-curvature(-step))/(2*step*speed_factor)
    assert got[0]==pytest.approx(curvature(0),abs=1e-14)
    assert got[1]==pytest.approx(finite,rel=1e-8,abs=1e-12)


def test_whole_road_qp_preserves_geometry_topology_and_source(tmp_path):
    pytest.importorskip('osqp')
    tree,road=fixture(tmp_path)
    geometry=etree.tostring(road.find('planView'))
    links=[etree.tostring(x) for x in road.findall('.//lane/link')]
    speeds=[dict(x.attrib) for x in road.findall('.//lane/speed')]
    report=solve_road(road,StraightBoundaries(),np.asarray,min_span=20.)
    assert report['status']=='CANDIDATE',report
    assert report['minimum_independent_span_m']>=20.
    assert report['source_max_m']<1e-4
    assert etree.tostring(road.find('planView'))==geometry
    assert [etree.tostring(x) for x in road.findall('.//lane/link')]==links
    assert [dict(x.attrib) for x in road.findall('.//lane/speed')]==speeds
    for s in np.linspace(.001,119.999,151):
        edges=np.array(lane_edges_kinematics_at(road,float(s),'right'))
        assert np.allclose(edges[:,0],[0.,-3.5,-7.],atol=1e-4)
        assert np.max(abs(edges[:,1:]))<1e-5
    schema=etree.XMLSchema(etree.parse('OpenDRIVE_1.5M.xsd'))
    assert schema.validate(tree),schema.error_log


def test_whole_road_rejects_missing_source_without_mutation(tmp_path):
    class Missing:
        def lane_boundary_geometries(self,_sid): return []
    _,road=fixture(tmp_path); before=etree.tostring(road)
    report=solve_road(road,Missing(),np.asarray)
    assert report['status']=='REJECTED'
    assert report['reason']=='no source observations'
    assert etree.tostring(road)==before


def test_absolute_error_budget_does_not_grandfather_wrong_fixed_mouth(tmp_path):
    _,road=fixture(tmp_path)
    lanes=road.find('lanes')
    lanes.insert(0,etree.Element('laneOffset',s='0',a='2',b='0',c='0',d='0'))
    links=etree.Element('link');etree.SubElement(links,'successor',elementType='junction',elementId='2');road.insert(0,links)
    before=etree.tostring(road)
    result=solve_road(road,StraightBoundaries(),np.asarray,boundary_association='source-order',source_error_budget='absolute')
    assert result['status']=='REJECTED'
    assert result['reason']=='infeasible whole-road constraints'
    assert result['source_error_budget']=='absolute'
    assert etree.tostring(road)==before


def test_whole_road_does_not_create_short_independent_controls(tmp_path):
    _,road=fixture(tmp_path,length=14.)
    before=etree.tostring(road)
    report=solve_road(road,StraightBoundaries(),np.asarray,min_span=15.)
    assert report['status']=='REJECTED'
    assert 'short physical boundary' in report['reason']
    assert etree.tostring(road)==before


def test_map_centers_are_not_treated_as_edges_and_mirror_is_exact(tmp_path):
    pytest.importorskip('osqp')
    _,road=fixture(tmp_path)
    for sec in road.findall('lanes/laneSection'):
        left=etree.Element('left'); sec.insert(0,left)
        for lane in sec.findall('right/lane'):
            clone=copy.deepcopy(lane); clone.set('id',str(-int(lane.get('id'))))
            for link in clone.findall('link/*'): link.set('id',str(-int(link.get('id'))))
            for ud in clone.findall("userData[@code='mapforge.source_lane']"): clone.remove(ud)
            left.append(clone)
    class Centers:
        def lane_center_geometry(self,sid):
            t=-3.5*(int(sid)-.5)
            return np.array([[0.,t],[120.,t]])
        def lane_boundary_geometries(self,sid):
            raise AssertionError('MAP supplies centers, not physical boundaries')
    result=solve_road(road,Centers(),np.asarray,source_mode='centers',mirror=True,min_span=20.)
    assert result['status']=='CANDIDATE',result
    assert result['source_geometry_semantics']=='centers'
    assert result['strict_same_leg_mirror']
    assert result['minimum_independent_span_m']>=20.
    for s in np.linspace(.001,119.999,97):
        right=np.asarray(lane_edges_kinematics_at(road,s,'right'))
        left=np.asarray(lane_edges_kinematics_at(road,s,'left'))
        np.testing.assert_allclose(left,2*right[0]-right,atol=1e-8)
        np.testing.assert_allclose((right[:-1,0]+right[1:,0])/2,[-1.75,-5.25],atol=.01)


def test_map_center_missing_observations_rejects_without_mutation(tmp_path):
    class Missing:
        def lane_center_geometry(self,sid): return None
    _,road=fixture(tmp_path); before=etree.tostring(road)
    result=solve_road(road,Missing(),np.asarray,source_mode='centers')
    assert result['status']=='REJECTED'
    assert result['reason']=='no source observations'
    assert etree.tostring(road)==before


def test_source_merge_path_not_assumed_to_be_boundary_midpoint():
    center=np.array([[0.,-1.8],[20.,1.],[40.,2.5]])
    bounds=[np.array([[0.,0.],[40.,0.]]),np.array([[0.,0.],[40.,5.]])]
    before=center.copy()
    result=compare_lane(center,bounds)
    assert result['status']=='OBSERVED'
    assert result['samples'][0]['boundary_width_m']==0.
    assert result['center_outside_max_m']==pytest.approx(1.8)
    assert np.array_equal(center,before)


def test_source_backtracking_boundary_is_not_sorted_into_a_fake_match():
    center=np.array([[0.,0.],[20.,0.]])
    bounds=[np.array([[0.,-1.],[20.,-1.],[10.,-2.]]),np.array([[0.,1.],[20.,1.]])]
    assert compare_lane(center,bounds)['status']=='UNRESOLVED'
