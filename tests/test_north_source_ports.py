import copy
import xml.etree.ElementTree as ET
import numpy as np
import pytest

from scripts.rebuild_north_source_ports import install_same_identity_parent,fresh_seed
from mapforge.ops.port_dependencies import PortDependencies
from tests.test_connector_cross_section import turning_network


def parent_fixture():
    root=turning_network()
    road=next(r for r in root.findall('road') if r.get('id')=='10')
    for sec in road.findall('lanes/laneSection'):
        for lane in sec.findall('*/lane'):
            if lane.get('id')=='0':continue
            ET.SubElement(lane,'userData',code='mapforge.source_lane',value='source:'+lane.get('id'))
    new=copy.deepcopy(road)
    for link in new.findall('link'):new.remove(link)
    return root,new


def test_component_keeps_explicit_graph_without_second_legacy_remap():
    root,new=parent_fixture();graph=PortDependencies(root)
    new.find('planView/geometry').set('y','1')
    install_same_identity_parent(root,new)
    current=PortDependencies(root)
    assert current.connections==graph.connections
    assert current.validate_junction_table()


def test_changed_source_identity_or_rank_refuses_entire_installation():
    for change in ('source','rank'):
        root,new=parent_fixture();before=ET.tostring(root)
        lane=new.find('lanes/laneSection/right/lane')
        if change=='source':lane.find("userData[@code='mapforge.source_lane']").set('value','different')
        else:lane.set('id','-99')
        with pytest.raises(ValueError,match='identities changed'):
            install_same_identity_parent(root,new)
        assert ET.tostring(root)==before


def frame(slope=False):
    return dict(pose=(0.,0.,0.),k=0.,dk=0.,edges={side:dict(x=0.,y=y,heading=.05 if slope else 0.,curvature=0.)
        for side,y in [('left',2.),('right',-2.)]})


def test_release_flat_port_adds_one_long_cap_not_source_point_segments():
    old=[frame(True),frame(False)];new=[frame(True),frame(True)]
    previous=dict(reference_core_count=3,shape_parameters=[7.,9.,12.,6.,.1,.2])
    q,count,info=fresh_seed(*new,previous,old)
    assert count==3 and len(q)==7
    np.testing.assert_allclose(q,[7,9,12,6,6,.1,.2])
    assert info['new_caps']==(True,True) and min(q[:5])>=6


def test_unchanged_cap_family_retains_all_long_parameters():
    f=[frame(True),frame(False)]
    previous=dict(reference_core_count=4,shape_parameters=[7.,9.,12.,6.,8.,.1,.2,.3])
    q,count,_=fresh_seed(*f,previous,f)
    assert count==4
    np.testing.assert_allclose(q,previous['shape_parameters'])


def test_wrong_historical_model_is_not_silently_reinterpreted():
    f=[frame(True),frame(False)]
    with pytest.raises(ValueError,match='primitive count disagrees'):
        fresh_seed(*f,dict(shape_parameters=[6.,6.,6.,.1,.2]),f)
