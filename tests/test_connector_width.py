import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from mapforge.adapters.opendrive import writer as W
from mapforge.ops.connector_width import written_lane_width, centred_width_records
from mapforge.ops.map_to_xodr import _written_jet
from mapforge.validate.smoothness import lane_endpoint_state


def test_width_uses_final_section_and_lane_identity():
    road=W.Road(10).add_geometry('line',0,0,0,50)
    road.add_offset(0,.4,.01)
    road.sections=[W.LaneSection(0,right=[W.Lane(-1).add_width(3.5)]),
                   W.LaneSection(30,right=[W.Lane(-1).add_width(3.),
                                          W.Lane(-3).add_width(2.2,.02)],
                                 left=[W.Lane(2).add_width(3.1,.01)])]
    assert written_lane_width(road,-1,'start')==pytest.approx(3.5)
    assert written_lane_width(road,-3,'end')==pytest.approx(2.6)
    assert written_lane_width(road,2,'end')==pytest.approx(3.3)
    with pytest.raises(ValueError,match='absent'):written_lane_width(road,-2,'end')


def test_absolute_borders_are_not_misread_as_widths():
    road=W.Road(10).add_geometry('line',0,0,0,20)
    road.sections=[W.LaneSection(0,left=[W.Lane(1).add_border(1.1),W.Lane(4).add_border(4.1,.02)])]
    assert written_lane_width(road,4,'end')==pytest.approx(3.4)


@pytest.mark.parametrize('widths',[(2.26,3.91),(4.8,2.4),(3.5,3.5)])
def test_c2_transition_is_monotone_and_does_not_move_driving_center(widths):
    length=45.;records=centred_width_records(*widths,length)
    s=np.linspace(0,length,501)
    w=np.array([_written_jet(records,u)[0] for u in s])
    assert w[0]==pytest.approx(widths[0]) and w[-1]==pytest.approx(widths[1])
    assert min(w)>=min(widths)-1e-10 and max(w)<=max(widths)+1e-10
    assert _written_jet(records,0)[1:]==pytest.approx([0,0],abs=1e-12)
    assert _written_jet(records,length)[1:]==pytest.approx([0,0],abs=1e-12)
    for i,record in enumerate(records[1:],1):
        assert _written_jet(records[:i],record[0])==pytest.approx(_written_jet(records,record[0]),abs=1e-12)
    r=W.Road(100,junction=1).add_geometry('spiral',0,0,.2,length,.01,.02)
    lane=W.Lane(-1)
    for station,*co in records:r.add_offset(station,*[c/2 for c in co]);lane.add_width(*co,s_offset=station)
    r.sections=[W.LaneSection(0,right=[lane])]
    xml=ET.Element('OpenDRIVE');W.XodrDoc('test')._road_el(xml,r)
    for contact in ('start','end'):
        actual=lane_endpoint_state(xml.find('road'),-1,contact)
        pose=(0.,0.,.2) if contact=='start' else r.end_pose()
        assert [actual[k] for k in ('x','y','heading')]==pytest.approx(pose,abs=1e-7)
        assert actual['curvature']==pytest.approx(.01 if contact=='start' else .02,abs=1e-10)


@pytest.mark.parametrize('args',[(0,3,20),(-1,3,20),(3,math.nan,20),(3,4,0)])
def test_bad_cross_section_does_not_fall_back_to_nominal_width(args):
    with pytest.raises(ValueError):centred_width_records(*args)
