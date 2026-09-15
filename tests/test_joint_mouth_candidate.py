import copy
import ctypes
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from lxml import etree
from tests.test_junction_edges import network
from tests.test_source_boundary_joint import fixture, StraightBoundaries
from spikes.joint_mouth_candidate import connectors
from spikes.road_boundary_family import solve_road
from mapforge.validate.junction_edges import audit as contacts
from mapforge.validate.smoothness import lane_edges_kinematics_at
from scripts.internal_edge_jets import audit as internal
from scripts.esmini_lane_interfaces import endpoint
from scripts.esmini_rm_check import DLL, _bind


def turn():
    root=network(vary=True)
    g=root.find("road[@id='11']/planView/geometry")
    g.set('x','55');g.set('y','35');g.set('hdg',str(math.pi/2))
    return root


def test_joint_parallel_mouth_has_three_primitives_and_all_edge_continuity():
    root=turn()
    topology=ET.tostring(root.find('junction'))
    rows=connectors(root)
    assert len(rows[0]['primitive_lengths'])==3
    assert 3<=rows[0]['width_records']<=5
    assert ET.tostring(root.find('junction'))==topology
    assert contacts(root)['status']=='PASS'
    assert internal(root)['status']=='PASS'


def test_do_not_center_a_connector_until_parent_frame_is_solved():
    root=turn()
    root.find("road[@id='10']/lanes/laneOffset").set('b','.01')
    with pytest.raises(ValueError,match='not jointly solved'):
        connectors(root)


def test_rejected_current_fit_does_not_validate_or_overwrite_stale_target(tmp_path, monkeypatch):
    import json
    from spikes import joint_mouth_candidate as candidate
    tree, _ = fixture(tmp_path)
    source = tmp_path/'source.xodr'; tree.write(str(source))
    source.with_suffix('.source-lanes.json').write_text(json.dumps({'lanes': []}), encoding='utf-8')
    target = tmp_path/'old-candidate.xodr'; target.write_bytes(b'old candidate retained')
    monkeypatch.setattr(candidate, 'solve_road', lambda *a, **k:
                        {'status': 'REJECTED', 'reason': 'test current-fit rejection'})
    assert candidate.run(source, target) is False
    assert target.read_bytes() == b'old candidate retained'
    assert json.loads(target.with_suffix('.joint.json').read_text())['status'] == 'REJECTED'


def test_reference_g2_and_c2_width_alone_are_not_edge_g2():
    from spikes.connector_cross_section import endpoint_frame
    from mapforge.ops.refline_fit import solve_g2_balanced
    from mapforge.ops.connector_width import centred_width_records
    from mapforge.adapters.opendrive import writer as W
    source=turn()
    a=endpoint_frame(source.find("road[@id='10']"),-1,'end',True)
    b=endpoint_frame(source.find("road[@id='11']"),-1,'start',True)
    cls,_=solve_g2_balanced(a['pose'],b['pose'],a['k'],b['k'])
    road=W.Road(100,junction=1)
    for c in cls:road.add_geometry('spiral',c.XStart,c.YStart,c.ThetaStart,c.length,c.KappaStart,c.KappaEnd)
    lane=W.Lane(-1)
    for s,*co in centred_width_records(3,5,road.length):
        lane.add_width(*co,s_offset=s);road.add_offset(s,*[v/2 for v in co])
    road.sections=[W.LaneSection(0,right=[lane])]
    root=ET.Element('OpenDRIVE');W.XodrDoc('test')._road_el(root,road)
    assert internal(root)['status']=='FAIL'
    assert internal(root)['maxima']['curvature_per_m']>1e-7


def test_parallel_end_mode_retains_source_width_and_long_span(tmp_path):
    pytest.importorskip('osqp')
    tree,road=fixture(tmp_path)
    link=road.find('link')
    if link is None:link=etree.Element('link');road.insert(0,link)
    etree.SubElement(link,'successor',elementType='junction',elementId='9')
    original=etree.tostring(road.find('planView'))
    result=solve_road(road,StraightBoundaries(),np.asarray,min_span=20.,junction_endpoint_mode='parallel')
    assert result['status']=='CANDIDATE'
    assert result['junction_endpoint_mode']=='parallel'
    assert result['minimum_independent_span_m']>=20.
    assert result['source_max_m']<1e-4
    assert etree.tostring(road.find('planView'))==original
    assert np.max(abs(np.array(lane_edges_kinematics_at(road,120.,'right'))[:,1:]))<1e-8


@pytest.mark.skipif(not DLL.exists(),reason='independent esmini DLL not installed')
def test_esmini_edge_offset_sign_in_forward_and_reverse_travel(tmp_path):
    from mapforge.adapters.opendrive import writer as W
    road=W.Road(1).add_geometry('line',0,0,0,80).add_offset(0,1.5)
    road.sections=[W.LaneSection(0,right=[W.Lane(-1).add_width(3)],left=[W.Lane(1).add_width(3)])]
    doc=W.XodrDoc('test');doc.add_road(road);path=tmp_path/'independent.xodr';doc.write(path)
    rm=ctypes.CDLL(str(DLL));_bind(rm)
    assert rm.RM_Init(str(path).encode())==0
    try:
        handle=rm.RM_CreatePosition()
        for lid,center in ((-1,0.),(1,3.)):
            for forward in (True,False):
                for side,sign in (('left',1),('right',-1)):
                    state=endpoint(rm,handle,1,lid,'end',forward,.02,side)
                    assert state['y']==pytest.approx(center+1.5*sign*(1 if forward else -1),abs=1e-10)
                    assert abs(state['curvature_per_m'])<1e-8
    finally:rm.RM_Close()
