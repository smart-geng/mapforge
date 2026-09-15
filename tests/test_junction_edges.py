import xml.etree.ElementTree as ET
import pytest
from mapforge.adapters.opendrive import writer as W
from mapforge.ops.connector_width import centred_width_records
from mapforge.validate.junction_edges import audit
from mapforge.validate.smoothness import junction_lane_interfaces


def network(end_width=5.,vary=False):
    incoming=W.Road(10).add_geometry('line',0,0,0,20).add_offset(0,1.5)
    incoming.sections=[W.LaneSection(0,right=[W.Lane(-1).add_width(3)])]
    outgoing=W.Road(11).add_geometry('line',60,0,0,20).add_offset(0,end_width/2)
    outgoing.sections=[W.LaneSection(0,right=[W.Lane(-1).add_width(end_width)])]
    connector=W.Road(100,junction=1).add_geometry('line',20,0,0,40)
    records=centred_width_records(3,end_width if vary else 3,40)
    lane=W.Lane(-1);lane.pred=-1;lane.succ=-1
    for station,*co in records:
        connector.add_offset(station,*[x/2 for x in co]);lane.add_width(*co,s_offset=station)
    connector.sections=[W.LaneSection(0,right=[lane])]
    connector.add_link('predecessor','road',10,'end');connector.add_link('successor','road',11,'start')
    root=ET.Element('OpenDRIVE')
    for road in (incoming,outgoing,connector):W.XodrDoc('test')._road_el(root,road)
    j=ET.SubElement(root,'junction',id='1')
    c=ET.SubElement(j,'connection',id='0',incomingRoad='10',connectingRoad='100',contactPoint='start')
    ET.SubElement(c,'laneLink',**{'from':'-1','to':'-1'})
    return root


def test_center_pass_does_not_hide_one_meter_boundary_gap():
    root=network()
    assert max(r['position_m'] for r in junction_lane_interfaces(root))<1e-10
    result=audit(root)
    assert result['status']=='FAIL' and result['count']==4
    assert result['maxima']['position_m']==pytest.approx(1.)


def test_c2_size_transition_joins_constant_width_straight_edges():
    result=audit(network(vary=True))
    assert result['maxima']['position_m']<1e-8
    assert result['maxima']['heading_deg']<1e-8
    assert result['maxima']['curvature_per_m']<1e-8
    assert result['status']=='PASS'
    assert audit(network(end_width=3))['status']=='PASS'


def test_matching_width_alone_does_not_match_varying_parent_edge_jets():
    root=network(vary=True)
    incoming=root.find("road[@id='10']")
    incoming.find('lanes/laneOffset').set('a','1.4')
    incoming.find('lanes/laneOffset').set('b','.005')
    incoming.find('lanes/laneSection/right/lane/width').set('a','2.8')
    incoming.find('lanes/laneSection/right/lane/width').set('b','.01')
    assert max(x['position_m'] for x in junction_lane_interfaces(root))<1e-8
    result=audit(root)
    assert result['maxima']['position_m']<1e-8
    assert result['maxima']['heading_deg']>.1 and result['status']=='FAIL'


def test_missing_boundary_link_must_not_be_skipped():
    root=network(end_width=3)
    root.find("road[@id='100']/lanes/laneSection/right/lane/link/successor").set('id','-20')
    result=audit(root)
    assert result['status']=='FAIL' and result['unresolved']
