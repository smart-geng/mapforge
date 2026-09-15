import xml.etree.ElementTree as ET
import numpy as np
import pytest
from mapforge.validate.map_width import width_extrema,raw_widths


def test_width_extrema_include_an_interior_bulge():
    lane=ET.Element('lane');ET.SubElement(lane,'width',sOffset='0',a='3.5',b='1',c='-1',d='0')
    result=width_extrema(lane,100.,100.,101.)
    assert max(v for s,v in result)==pytest.approx(3.75)
    assert any(s==pytest.approx(100.5) for s,v in result)


def test_missing_width_coverage_is_not_a_pass():
    with pytest.raises(ValueError,match='coverage'):width_extrema(ET.Element('lane'),0.,0.,10.)


def test_raw_width_is_not_replaced_by_default_and_conflicts_reject(monkeypatch):
    from mapforge.adapters.v2xmap.xml_reader import MapNode,MapLink,MapLane
    lane=MapLane(1,None,None);link=MapLink('west',(1,2),None,lanes=[lane]);node=MapNode('n',1,3,links=[link])
    monkeypatch.setattr('mapforge.validate.map_width.parse_map_xml_all',lambda _: [node])
    assert next(iter(raw_widths(['x']).values())) is None
    lane.width_cm=350
    assert next(iter(raw_widths(['x']).values()))==3.5
    other=MapNode('n',1,3,links=[MapLink('west',(1,2),None,lanes=[MapLane(1,400,None)])])
    monkeypatch.setattr('mapforge.validate.map_width.parse_map_xml_all',lambda path: [node if path=='x' else other])
    with pytest.raises(ValueError,match='ambiguous'):raw_widths(['x','y'])


def test_duplicate_width_stations_and_nonfinite_intervals_reject():
    lane=ET.Element('lane')
    for width in ('3.5','4'):
        ET.SubElement(lane,'width',sOffset='0',a=width,b='0',c='0',d='0')
    with pytest.raises(ValueError,match='duplicate'):width_extrema(lane,0.,0.,10.)
    with pytest.raises(ValueError,match='interval'):width_extrema(lane,0.,0.,np.nan)


def test_explicit_width_is_a_model_constraint_not_a_weak_prior(tmp_path):
    from tests.test_source_boundary_joint import fixture
    from tests.test_map_joint_source_certificate import solve,centers
    _,road=fixture(tmp_path)
    result=solve(road,centers(),source_widths={'1':3.5,'2':3.5})
    assert result['status']=='CANDIDATE',result
    sections=road.findall('lanes/laneSection');ends=[38.,40.,120.]
    for section,end in zip(sections,ends):
        start=float(section.get('s'))
        for lane in section.findall('right/lane'):
            assert all(abs(v-3.5)<1e-7 for s,v in width_extrema(lane,start,start,end))


def test_rotated_source_support_keeps_width_constraint(tmp_path):
    from tests.test_rotating_source_tube import setup
    from spikes.rotating_source_tube import RotatingSourceTube
    from spikes.map_lane_family import ManifestCenters
    from tests.test_map_joint_source_certificate import centers
    t,x=setup(tmp_path);tube=RotatingSourceTube(t.model,ManifestCenters(centers()),source_widths={'1':3.5})
    assert max(tube.evaluate(x,0.))<=0.
    # Preserve both centers exactly while shrinking one lane: the free
    # alternating-boundary gauge must no longer be accepted.
    for i,f in enumerate(t.model.families):x[f.columns]+=(.5 if i%2 else -.5)
    assert max(tube.evaluate(x,0.))>1.


def test_width_failure_blocks_delivery_even_when_geometry_and_crs_pass(tmp_path,monkeypatch):
    from mapforge.report.decision import finalize_opendrive_g8
    path=tmp_path/'minimal.xodr';path.write_text('<OpenDRIVE/>',encoding='utf-8')
    monkeypatch.setattr('mapforge.report.decision.evaluate_g8',lambda *a:{'gate_id':'G8','status':'PASS'})
    monkeypatch.setattr('mapforge.validate.g11.audit_file',lambda *a:{'gate_id':'G11','status':'PASS'})
    monkeypatch.setattr('mapforge.validate.junction_edges.audit',lambda *a:{'status':'PASS'})
    monkeypatch.setattr('mapforge.validate.map_source.audit_map_source_manifest',lambda *a:{'status':'PASS'})
    monkeypatch.setattr('mapforge.validate.map_width.audit',lambda *a:{'gate_id':'MAP-explicit-width','status':'FAIL'})
    result=finalize_opendrive_g8(path,{'source_format':'map','comparison_crs':{'integrity':'verified'}},{})
    assert result['decision']['status']=='BLOCKED'
    assert 'MAP-explicit-width' in result['decision']['required_gates']
    assert path.with_suffix('.source-width.json').exists()
