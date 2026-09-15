"""Candidate geometry tests, independent of existing G11 provenance exclusions."""
from pathlib import Path

import numpy as np
from lxml import etree

from mapforge.adapters.opendrive.writer import XodrDoc, Road, Lane, LaneSection
from mapforge.ops.lane_family_border import _materialize_widths
from mapforge.validate.smoothness import lane_edges_kinematics_at
from spikes.source_boundary_joint import solve_side

ROOT=Path(__file__).resolve().parents[1]


class StraightBoundaries:
    def lane_boundary_geometries(self, source_id):
        i=int(source_id)
        return [np.array([[0.,-3.5*j],[120.,-3.5*j]]) for j in (i-1,i)]


def fixture(tmp_path, length=120.):
    doc=XodrDoc('boundary-solve-test',geo_reference='+proj=eqc +units=m')
    rd=Road(1); rd.add_geometry('line',0,0,0,length)
    stations=[0.,38.,40.] if length>40 else [0.]
    for si,s in enumerate(stations):
        sec=LaneSection(s)
        for i in (1,2):
            ln=Lane(-i,source_id=str(i),speed_ms=60/3.6,
                    provenance={'eligibility':'comparable','status':'TRANSFORMED'})
            if si: ln.pred=-i
            if si+1<len(stations): ln.succ=-i
            # Short width impulse, while the full road outer edge stays -7m.
            sign=1 if i==1 else -1
            if s==38.: ln.add_width(3.5,.5*sign,-.25*sign)
            else: ln.add_width(3.5)
            sec.right.append(ln)
        rd.sections.append(sec)
    doc.add_road(rd); path=tmp_path/'source.xodr'; doc.write(path)
    tree=etree.parse(str(path))
    return tree,tree.find('road')


def test_joint_source_fit_removes_width_impulse_without_extra_geometry(tmp_path):
    tree,road=fixture(tmp_path)
    topology=etree.tostring(road.find('planView'))
    speeds=[dict(x.attrib) for x in road.findall('.//speed')]
    links=[etree.tostring(x) for x in road.findall('.//lane/link')]
    before=np.array(lane_edges_kinematics_at(road,120.-1e-7,'right'))
    report=solve_side(road,'right',StraightBoundaries(),np.asarray,
                      knot_span=20.,kinematic_constraints=True)
    assert report['status']=='CANDIDATE',report
    assert report['minimum_control_span_m']>=20.
    assert report['source_max_m']<1e-4
    secs=road.findall('lanes/laneSection')
    assert _materialize_widths(road,secs,[0,38,40],[38,40,120])
    assert len(secs)==3
    assert etree.tostring(road.find('planView'))==topology
    assert [dict(x.attrib) for x in road.findall('.//speed')]==speeds
    assert [etree.tostring(x) for x in road.findall('.//lane/link')]==links
    after=np.array(lane_edges_kinematics_at(road,120.-1e-7,'right'))
    assert np.max(abs(before-after))<1e-7
    for s in np.linspace(.001,119.999,500):
        edges=np.array(lane_edges_kinematics_at(road,float(s),'right'))
        assert np.allclose(edges[:,0],[0,-3.5,-7],atol=1e-4)
        assert np.max(abs(edges[:,1:]))<1e-4
    schema=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    assert schema.validate(tree),schema.error_log


def test_short_source_branch_refuses_tiny_spline_controls(tmp_path):
    _tree,road=fixture(tmp_path,length=30.)
    before=etree.tostring(road)
    report=solve_side(road,'right',StraightBoundaries(),np.asarray,knot_span=15.)
    assert report['status']=='REJECTED'
    assert report['reason']=='short-boundary-chain'
    assert etree.tostring(road)==before


def test_inconsistent_source_tube_does_not_mutate_road(tmp_path):
    _tree,road=fixture(tmp_path)
    # The connected mouth remains an exact boundary condition. Contradictory
    # narrow lanes cannot be silently written as negative widths.
    lane=road.findall('lanes/laneSection')[-1].find("right/lane[@id='-1']")
    lane.find('width').set('a','-2')
    before=etree.tostring(road)
    report=solve_side(road,'right',StraightBoundaries(),np.asarray,knot_span=20.)
    assert report['status']=='REJECTED'
    assert etree.tostring(road)==before
