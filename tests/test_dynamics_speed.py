import copy
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mapforge.ops.port_dependencies import revision
from mapforge.validate.dynamics_speed import lane_limits, limit_samples, movement_bindings
from mapforge.validate.g11 import _audit_d, load_policy


def scene(junction='1'):
    return ET.fromstring('''<OpenDRIVE><road id="100" length="30" junction="%s">
      <planView><geometry s="0" x="0" y="0" hdg="0" length="30"><arc curvature="0.1"/></geometry></planView>
      <lanes><laneSection s="0"><center><lane id="0" type="none"/></center><right>
      <lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>
      <speed sOffset="0" max="60" unit="km/h"/></lane></right></laneSection></lanes>
      </road></OpenDRIVE>''' % junction)


def contract(root):
    return dict(schema='mapforge/movement-design-speed/v1', artifact_revision=revision(root),
                approval_state='proposed', bindings=[dict(road_id='100', section_s_m=0., lane_id='-1',
                movement_id='synthetic-turn', target_speed_kmh=15., basis='test-independent-design-condition')])


@pytest.mark.parametrize('unit,value', [('m/s', 10.), ('km/h', 36.), ('mph', 22.369362920544)] )
def test_units_zero_and_interval_semantics(unit,value):
    lane=ET.fromstring(f'<lane><speed sOffset="2" max="{value}" unit="{unit}"/>'
                      '<speed sOffset="5" max="0"/></lane>')
    speeds,missing=limit_samples(lane_limits(lane),[0.,2.,4.,5.,9.],60.)
    assert speeds == pytest.approx([60.,36.,36.,0.,0.])
    assert missing.tolist()==[True,False,False,False,False]


@pytest.mark.parametrize('records',[
    '<speed sOffset="0" max="NaN"/>', '<speed sOffset="-1" max="10"/>',
    '<speed sOffset="0" max="-1"/>', '<speed sOffset="0" max="10" unit="bad"/>',
    '<speed sOffset="1" max="10"/><speed sOffset="0" max="10"/>',
    '<speed sOffset="0" max="10"/><speed sOffset="0" max="20"/>'])
def test_invalid_limit_fails_closed(records):
    with pytest.raises(ValueError): lane_limits(ET.fromstring('<lane>'+records+'</lane>'))


def test_design_proposal_keeps_limit_stress_and_requires_approval():
    root=scene(); before=ET.tostring(root)
    policy=load_policy('profiles/validation/g11-opendrive-v1.draft.yaml')
    baseline=_audit_d(root,policy)
    policy['dynamics']['movement_design']=contract(root)
    report=_audit_d(root,policy)
    row=report['speed_evidence'][0]
    assert row['target_speed_kmh']==15.
    assert row['written_limits'][0]['max_kmh']==60.
    assert row['design_ay_mps2']<2.5 < row['written_limit_or_fallback_stress_ay_mps2']
    assert baseline['metrics']['lateral_acceleration_max_mps2'] == pytest.approx(
        row['written_limit_or_fallback_stress_ay_mps2'])
    assert report['status']=='FAIL'
    assert any(x['code']=='movement_design_speed_requires_approval' for x in report['issues'])
    assert ET.tostring(root)==before


@pytest.mark.parametrize('mutation',['stale','missing','duplicate','unknown','negative','basis','ordinary'])
def test_movement_binding_rejects_invalid_or_incomplete(mutation):
    root=scene(); doc=contract(root)
    if mutation=='stale': doc['artifact_revision']='wrong'
    if mutation=='missing': doc['bindings']=[]
    if mutation=='duplicate': doc['bindings']*=2
    if mutation=='unknown': doc['bindings'][0]['road_id']='999'
    if mutation=='negative': doc['bindings'][0]['target_speed_kmh']=-1.
    if mutation=='basis': doc['bindings'][0]['basis']=''
    if mutation=='ordinary':
        root.find('road').set('junction','-1'); doc['artifact_revision']=revision(root)
    with pytest.raises(ValueError): movement_bindings(root,doc)


def test_movement_contract_cannot_change_ordinary_condition():
    root=scene('-1'); p=load_policy('profiles/validation/g11-opendrive-v1.draft.yaml')
    p['dynamics']['movement_design']=dict(schema='mapforge/movement-design-speed/v1',
        artifact_revision=revision(root),approval_state='proposed',bindings=[])
    row=_audit_d(root,p)['speed_evidence'][0]
    assert row['target_speed_kmh']==60. and row['design_ay_mps2']>2.5


def test_speed_step_does_not_become_whole_lane_maximum():
    root=scene(); lane=root.find('.//right/lane')
    lane.find('speed').set('max','15')
    ET.SubElement(lane,'speed',sOffset='20',max='30',unit='km/h')
    p=load_policy('profiles/validation/g11-opendrive-v1.draft.yaml')
    row=_audit_d(root,p)['speed_evidence'][0]
    assert row['target_speed_kmh']==30.
    assert len(row['written_limits'])==2
    ET.SubElement(lane,'speed',sOffset='30',max='60',unit='km/h')
    with pytest.raises(ValueError,match='outside'): _audit_d(root,p)
