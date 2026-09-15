import math
import copy
import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest

from tests.test_measured_connector_caps import ordinary
from tests.test_connector_cross_section import turning_network
from spikes.joint_source_mouth import make_window,splice_window
from spikes.measured_connector_caps import frames,needs_cap,seed_chain,chain,ribbon,write_ribbon,polish_endpoint
from spikes.road_boundary_family import linear_feasible
from mapforge.validate.smoothness import lane_edges_kinematics_at,junction_lane_interfaces
from mapforge.validate.junction_edges import audit as edge_audit


@pytest.mark.parametrize('contact',['start','end'])
def test_window_splice_keeps_full_reference_and_all_unmodified_geometry(contact):
    road=ordinary('10',12,-4,.2,contact);raw=ET.tostring(road)
    window,lo,hi=make_window(road,30.)
    result=splice_window(road,window,lo,hi)
    assert ET.tostring(road)==raw
    assert ET.tostring(result.find('planView'))==ET.tostring(road.find('planView'))
    assert ET.tostring(result.find('link'))==ET.tostring(road.find('link'))
    assert result.get('length')==road.get('length')
    for s in np.linspace(0,80,101):
        np.testing.assert_allclose(lane_edges_kinematics_at(result,float(s),'right'),
                                   lane_edges_kinematics_at(road,float(s),'right'),atol=1e-9)
    assert b'mouth-window-fixed-cut' not in ET.tostring(result)
    assert all(e.get('max')==str(60/3.6) for e in result.findall('.//lane/speed'))


def test_splice_refuses_new_lane_at_fixed_cut():
    road=ordinary('10',0,0,0,'end');window,lo,hi=make_window(road,30.)
    right=window.find('lanes/laneSection/right')
    new=copy.deepcopy(right.find('lane'));new.set('id','-2');right.append(new)
    with pytest.raises(ValueError,match='birth/death'):splice_window(road,window,lo,hi)


def test_fully_fixed_source_family_is_checked_without_empty_lp():
    fixed=np.zeros((2,0))
    assert linear_feasible(fixed,np.array([-1.,0.])).success
    assert not linear_feasible(fixed,np.array([-1.,.1])).success


def _parallel(road):
    for e in road.findall('lanes/laneOffset')+road.findall('.//lane/width'):
        for k in 'bcd':e.set(k,'0')


@pytest.mark.parametrize('flatten',[(True,True),(True,False),(False,True)])
def test_unneeded_reference_caps_removed_only_when_edge_jets_allow(flatten):
    root=turning_network();roads=[root.find("road[@id='10']"),root.find("road[@id='11']")]
    for r,do in zip(roads,flatten):
        if do:_parallel(r)
    cr=next(r for r in root.findall('road') if r.get('junction')!='-1')
    lane=cr.find('lanes/laneSection/right/lane')
    ET.SubElement(lane,'userData',code='mapforge.provenance/v1',value='{}')
    a,b=frames(root,cr);caps=(needs_cap(a),needs_cap(b))
    for flag,cap in zip(flatten,caps):
        if flag:assert not cap
    cls=seed_chain(a,b,6. if caps[0] else 0.,6. if caps[1] else 0.)
    i=int(caps[0]);z=np.r_[[c.length for c in cls],cls[i].KappaEnd*20,cls[i+1].KappaEnd*20]
    cs=chain(z,a,b,caps);kn,co=ribbon(cs,a,b);result=write_ribbon(cr,cs,kn,co)
    assert len(result.findall('planView/geometry'))==3+sum(caps)
    metadata=json.loads(result.find(".//userData[@code='mapforge.provenance/v1']").get('value'))
    assert metadata['reference_primitive_count']==3+sum(caps)
    assert metadata['width_record_count']==5+sum(caps)
    assert 'G3' not in metadata['cross_section_fit']
    root.remove(cr);root.append(result)
    assert edge_audit(root)['status']=='PASS'
    assert all(r['position_m']<1e-6 and r['heading_deg']<1e-6 and r['curvature_per_m']<1e-7 for r in junction_lane_interfaces(root))


def test_endpoint_polishing_solves_parameters_without_translating_chain():
    root=turning_network()
    for r in root.findall('road'):
        if r.get('junction')=='-1':_parallel(r)
    cr=next(r for r in root.findall('road') if r.get('junction')!='-1');a,b=frames(root,cr)
    cls=seed_chain(a,b,0.,0.);z=np.r_[[c.length for c in cls],cls[0].KappaEnd*20,cls[1].KappaEnd*20]
    z[1]+=1e-5;result,info=polish_endpoint(z,a,b,(False,False))
    assert max(abs(v) for v in info['after'])<1e-8
    assert info['shape_shift_max']<1e-3
    actual=chain(result,a,b,(False,False))
    assert (actual[0].XStart,actual[0].YStart)==pytest.approx(a['pose'][:2])
    for left,right in zip(actual,actual[1:]):
        assert (left.XEnd,left.YEnd)==pytest.approx((right.XStart,right.YStart))
    assert actual[-1].KappaEnd==pytest.approx(b['k'])


def test_source_anchor_projection_does_not_sort_backtracking_points():
    from spikes.mouth_anchor_review import project_curve
    road=ordinary('10',0,0,0,'end')
    with pytest.raises(ValueError,match='backtracking'):
        project_curve([[0,0],[5,0],[3,0],[10,0]],road,np.asarray)
    np.testing.assert_allclose(project_curve([[10,0],[5,0],[0,0]],road,np.asarray),[[0,0],[5,0],[10,0]])


def test_source_construction_margin_is_stricter_and_raw_turn_not_diluted():
    from spikes.measured_connector_caps import source_slacks
    error={'composite':{'source_to_target':np.zeros(100)},
           'raw_via:left':{'source_to_target':np.array([.36,.37,.38])}}
    original=source_slacks(error);construction=source_slacks(error,.001)
    assert min(original)<0  # many perfect straight points cannot hide bad via
    np.testing.assert_allclose(construction,original-.001)
    with pytest.raises(ValueError):source_slacks(error,-.001)


def test_shape_selection_does_not_discard_margin_safe_optimized_shape():
    from spikes.measured_connector_caps import choose_shape
    cheap=np.array([.1]);safe=np.array([.6])
    objective=lambda p:p[0]
    margins=lambda p:np.array([p[0]-.5])
    selected,count=choose_shape([cheap,safe],objective,margins,True)
    np.testing.assert_array_equal(selected,safe);assert count==1
    # Readback must not claim it reoptimized a cached shape.
    selected,count=choose_shape([cheap,safe],objective,margins,False)
    np.testing.assert_array_equal(selected,cheap)
    selected,count=choose_shape([],objective,margins,True)
    assert selected is None and count==0


def test_headless_review_has_no_conflicting_vehicle_or_ground_options(tmp_path,monkeypatch):
    from scripts import visual_sweep
    called=[]
    class Process:
        def poll(self):return 0
    def popen(cmd,**kwargs):called.append(cmd);return Process()
    monkeypatch.setattr(visual_sweep.subprocess,'Popen',popen)
    visual_sweep.capture(tmp_path/'test.xodr',['--density','0','--ground_plane','off'],tmp_path)
    cmd=called[0]
    assert cmd.count('--density')==cmd.count('--ground_plane')==1
    assert cmd[cmd.index('--density')+1]=='0'
    assert cmd[cmd.index('--ground_plane')+1]=='off'


def test_trial_road_replacement_preserves_schema_order_and_identity():
    from spikes.trial_xml import replace_road,order_root
    root=ET.Element('OpenDRIVE');ET.SubElement(root,'header')
    a=ET.SubElement(root,'road',id='10');ET.SubElement(root,'road',id='11')
    junction=ET.SubElement(root,'junction',id='1')
    new=copy.deepcopy(a);new.set('name','replacement')
    replace_road(root,a,new)
    assert [r.tag for r in root]==['header','road','road','junction']
    with pytest.raises(ValueError,match='identity'):replace_road(root,new,ET.Element('road',id='12'))
    # Repair an older trial without changing subtree contents or unknown data.
    root.remove(new);root.append(new);extra=ET.SubElement(root,'unknown-data',value='keep')
    payloads={id(e):ET.tostring(e) for e in root}
    assert order_root(root)
    assert list(root).index(new)<list(root).index(junction)
    assert extra in list(root)
    assert payloads=={id(e):ET.tostring(e) for e in root}
    assert not order_root(root)


def test_trial_xsd_is_not_reported_pass_when_required_content_missing(tmp_path):
    from spikes.trial_xml import write_trial
    tree=ET.ElementTree(ET.Element('OpenDRIVE'))
    result=write_trial(tree,tmp_path/'bad.xodr')
    assert result['status']=='FAIL' and result['errors']
