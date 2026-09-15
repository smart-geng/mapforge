import math
import xml.etree.ElementTree as ET

import pytest
from mapforge.adapters.opendrive import writer as W
from mapforge.ops.map_to_xodr import _rounded_mouth_apron
from spikes.map_corner_surface import build


def synthetic():
    doc=W.XodrDoc('surface')
    mouths=[]
    for i,(x,y,h) in enumerate(((-20,0,0),(0,-20,math.pi/2),(20,0,math.pi),(0,20,-math.pi/2))):
        r=W.Road(10+i).add_geometry('line',x-40*math.cos(h),y-40*math.sin(h),h,40)
        r.sections=[W.LaneSection(0,left=[W.Lane(1).add_width(7)],right=[W.Lane(-1).add_width(7)])]
        r.add_link('successor','junction',1);doc.add_road(r)
        mouths.append({'pose':(x,y,h),'left_t':7,'right_t':-7})
    W.add_paving_road(doc,_rounded_mouth_apron(mouths),1,smooth_profile=True,overlap_m=.75)
    root=ET.Element('OpenDRIVE')
    for road in doc.roads:doc._road_el(root,road)
    ET.SubElement(root,'junction',id='1')
    return root


def test_boundary_native_surface_has_analytic_edges_and_no_sampling_width_chain():
    root=synthetic();stats=build(root)
    assert stats['surface']['paving_components']==1
    assert stats['surface']['paving_holes_gt1cm2']==0
    assert stats['surface']['paving_leg_overlap_min']>=.2
    assert stats['excess_m2']<.001
    assert stats['paving_reference_segments']==[3]*4+[1]*6
    assert stats['paving_width_records']==[1]*4+[2]*6
    for road in root.findall("road[@name='junction_paving']"):
        assert not road.findall('link') and not road.findall('.//lane/link')
        assert len(road.findall('lanes/laneSection'))==1
        assert all(l.get('type')=='restricted' for l in road.findall('lanes/laneSection/left/lane'))


def test_disconnected_inner_cover_is_rejected_atomically():
    root=synthetic();before=ET.tostring(root)
    with pytest.raises(ValueError,match='surface candidate rejected'):
        build(root,width_factor=.05)
    assert ET.tostring(root)==before
