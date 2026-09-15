import copy
import pytest
from mapforge.ops.source_speed_events import speed_event,speed_records
from mapforge.ops.source_transition_domains import source_transition_domains
from tests.test_source_transition_domains import oblique_fixture


def fixture():
    m=oblique_fixture(degree=3)
    m.source_speed_boundary_policy='original-lane-node'
    m.source_speed_node_fields={'L':{'start':'SN','end':'EN'}}
    m.scope['observations']['b']['source_max_speed_kmh']=40
    for o in m.scope['occurrences']:o['travel_direction']='with_s'
    return m


def test_speed_change_uses_original_node_not_geometry_cut_and_keeps_source_intact():
    m=fixture();before=copy.deepcopy(m.raw);count=m.nvar
    event=speed_event(m,m.contacts['transition_events'][0])
    assert event['station_m']==pytest.approx(30)
    assert event['before_s_kmh']==60 and event['after_s_kmh']==40
    ds=source_transition_domains(m)['intervals']
    import numpy as np
    assert np.allclose([(d['a'],d['b'],d['source_speed_kmh']) for d in ds],[(29.,30.,60.),(30.,31.,40.)],rtol=0,atol=1e-12)
    assert m.nvar==count and all((before[k]==v).all() for k,v in m.raw.items())
    chain={'speed_events':[event]}
    assert np.allclose(speed_records(chain,20,40),[(0.,60.),(10.,40.)],rtol=0,atol=1e-12)
    assert speed_records(chain,30,40)==[(0.,40.)]
    assert speed_records(chain,0,30)==[(0.,60.)]


def test_reverse_travel_writes_increasing_station_not_reversed_speed_values():
    m=fixture()
    for o in m.scope['occurrences']:o['travel_direction']='against_s'
    event=speed_event(m,m.contacts['transition_events'][0])
    import numpy as np
    assert np.allclose(speed_records({'speed_events':[event]},20,40),[(0.,40.),(10.,60.)],rtol=0,atol=1e-12)


@pytest.mark.parametrize('fault',['field','node','point','lateral-point','direction','disabled'])
def test_speed_event_requires_actual_source_authority(fault):
    m=fixture();event=m.contacts['transition_events'][0]
    if fault=='field':m.source_speed_node_fields={}
    elif fault=='node':m.scope['observations']['b']['raw_records'][0]['attributes']['SN']='not-same'
    elif fault=='point':m.raw['lane:b'][0,0]+=.1
    elif fault=='lateral-point':m.raw['lane:b'][0,1]+=.1
    elif fault=='direction':m.scope['occurrences'][0]['travel_direction']='unknown'
    else:m.source_speed_boundary_policy=None
    with pytest.raises(ValueError):speed_event(m,event)


def test_speed_only_records_do_not_add_geometry_intervals(monkeypatch):
    from tests.test_source_cubic_export import fixture as writer_fixture
    from mapforge.ops import source_cubic_export as compiler
    m,x,original=writer_fixture(monkeypatch)
    base=compiler.source_chains(m)
    event=dict(station_m=25.,before_s_kmh=60.,after_s_kmh=40.)
    base[0]['speed_events']=[event];base[0]['speed_kmh']=None
    monkeypatch.setattr(compiler,'source_chains',lambda _:base)
    road,report,_=compiler.compile_road(m,x,original)
    assert len(road.findall('planView/geometry'))==1
    assert len(road.findall('lanes/laneSection'))==3
    section=road.findall('lanes/laneSection')[1]
    changed=[l for l in section.findall('*/lane') if len(l.findall('speed'))==2]
    assert len(changed)==1
    assert [float(s.get('sOffset')) for s in changed[0].findall('speed')]==[0.,5.]
    assert [float(s.get('max'))*3.6 for s in changed[0].findall('speed')]==pytest.approx([60.,40.])
    assert report['minimum_width_span_m']==20.
