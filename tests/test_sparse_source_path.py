from types import SimpleNamespace
import json
import numpy as np
import pytest
from lxml import etree

from scripts.sparse_source_path_trial import source_path


def fixture_source(against=False):
    road=etree.Element('road');lanes=etree.SubElement(road,'lanes')
    for i,sid in enumerate(('a','b')):
        sec=etree.SubElement(lanes,'laneSection',s=str(20*i))
        side=etree.SubElement(sec,'right');lane=etree.SubElement(side,'lane',id='-1')
        if i==0:etree.SubElement(etree.SubElement(lane,'link'),'successor',id='-1')
        etree.SubElement(lane,'speed',max='60',unit='km/h')
        etree.SubElement(lane,'userData',code='mapforge.source_lane',value=sid)
        etree.SubElement(lane,'userData',code='mapforge.provenance/v1',
                         value=json.dumps({'travel_direction':'against_s' if against else 'with_s'}))
    records={s:SimpleNamespace(geometry=np.array([[x,0.],[x+20,0.]])[::-1 if against else 1],
                              max_speed_kmh=60) for s,x in (('a',0.),('b',20.))}
    src=SimpleNamespace(lane=lambda sid:records[sid],topo_out={'a':['b']},topo_in={'a':['b']})
    return road,src,records


@pytest.mark.parametrize('against',[False,True])
def test_full_raw_path_uses_original_topology_and_explicit_direction(against):
    road,src,_=fixture_source(against)
    raw,ids,speed=source_path(road,src,'right',-1,np.asarray)
    assert ids==['a','b'] and speed==pytest.approx(60/3.6)
    np.testing.assert_array_equal(raw,[[0.,0.],[20.,0.],[40.,0.]])


@pytest.mark.parametrize('failure',['speed','topology','gap','missing_speed'])
def test_source_mismatch_cannot_be_hidden_by_written_lane_metadata(failure):
    road,src,recs=fixture_source()
    if failure=='speed':recs['b'].max_speed_kmh=80
    elif failure=='topology':src.topo_out={}
    elif failure=='gap':recs['b'].geometry[0,1]=.000001
    else:
        lane=road.findall('.//right/lane')[1];lane.remove(lane.find('speed'))
    with pytest.raises(ValueError):source_path(road,src,'right',-1,np.asarray)
