import copy
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from spikes.measured_connector_caps import (distances,unique_source_path,crop_poly_records,
    trim_ordinary,frames,ribbon,sample,write_ribbon,chain)
from spikes.connector_cross_section import endpoint_g3_chain
from mapforge.validate.smoothness import lane_edges_kinematics_at,junction_lane_interfaces
from mapforge.validate.junction_edges import audit as edge_audit


def test_point_segment_error_keeps_outside_endpoints():
    np.testing.assert_allclose(distances([[-2,0],[5,2],[12,0]],[[0,0],[10,0]]),[2,2,2])


def test_source_path_is_unique_original_topology_only():
    assert unique_source_path({'a':['b','b'],'b':['c']},'a','c')==['a','b','c']
    with pytest.raises(ValueError,match='no original'):unique_source_path({},'a','c')
    with pytest.raises(ValueError,match='ambiguous'):
        unique_source_path({'a':['b','d'],'b':['c'],'d':['c']},'a','c')


def ordinary(rid,x,y,h,contact):
    r=ET.Element('road',id=rid,length='80',junction='-1')
    ET.SubElement(ET.SubElement(r,'link'),'successor' if contact=='end' else 'predecessor',elementType='junction',elementId='1')
    g=ET.SubElement(ET.SubElement(r,'planView'),'geometry',s='0',x=str(x),y=str(y),hdg=str(h),length='80');ET.SubElement(g,'line')
    group=ET.SubElement(r,'lanes');ET.SubElement(group,'laneOffset',s='0',a='1.8',b='0.008',c='0',d='0')
    for s in [0.,73.]:
        sec=ET.SubElement(group,'laneSection',s=str(s));lane=ET.SubElement(ET.SubElement(sec,'right'),'lane',id='-1',type='driving')
        ET.SubElement(lane,'width',sOffset='0',a=str(3.5+.002*s),b='.002',c='0',d='0')
        ET.SubElement(lane,'speed',sOffset='0',max=str(60/3.6),unit='m/s')
    return r


@pytest.mark.parametrize('contact',['start','end'])
def test_mouth_restriction_preserves_polynomial_geometry_and_source(contact):
    r=ordinary('1',0,0,0,contact);before=ET.tostring(r);out=trim_ordinary(r,9.)
    assert ET.tostring(r)==before
    assert float(out.get('length'))==71.
    shift=9. if contact=='start' else 0.
    for s in [0.,10.,40.,70.9]:
        np.testing.assert_allclose(lane_edges_kinematics_at(out,s,'right'),lane_edges_kinematics_at(r,s+shift,'right'),atol=1e-10)
    assert float(out.find('planView/geometry').get('x'))==shift
    assert not any(float(s.get('s'))>=71. for s in out.findall('lanes/laneSection'))
    assert all(e.get('max')==str(60/3.6) for e in out.findall('.//lane/speed'))


def test_crop_cubic_rebases_exactly_and_does_not_mutate():
    e=ET.Element('width',sOffset='2',a='3',b='.1',c='.02',d='.001')
    out=crop_poly_records([e],'sOffset',5.,20.)[0]
    for s in [0,1,8]:
        old=sum(float(e.get(k))*(s+3)**i for i,k in enumerate('abcd'))
        new=sum(float(out.get(k))*s**i for i,k in enumerate('abcd'))
        assert new==pytest.approx(old)
    assert e.get('sOffset')=='2'


def test_trim_removes_deleted_section_link_not_junction_identity():
    road=ordinary('1',0,0,0,'end')
    lane=road.find('lanes/laneSection/right/lane');link=ET.SubElement(lane,'link')
    ET.SubElement(link,'successor',id='-1')
    result=trim_ordinary(road,9.)
    assert result.find('lanes/laneSection/right/lane/link/successor') is None
    assert result.find('link/successor').get('elementId')=='1'


@pytest.mark.parametrize('distance',[-1.,float('nan'),float('inf')])
def test_trim_rejects_invalid_distances(distance):
    with pytest.raises(ValueError,match='invalid mouth'):
        trim_ordinary(ordinary('1',0,0,0,'end'),distance)


def test_review_rejects_stale_geometry_report(tmp_path):
    import json
    import hashlib
    from scripts.review_measured_ribbon import bound_trial
    path=tmp_path/'trial.xodr';path.write_bytes(b'<OpenDRIVE/>')
    data={'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    path.with_suffix('.ribbon.json').write_text(json.dumps(data),encoding='utf-8')
    assert bound_trial(path)==data
    path.write_bytes(b'<OpenDRIVE><road/></OpenDRIVE>')
    with pytest.raises(ValueError,match='hash does not match'):bound_trial(path)


def test_fairness_family_preserves_constant_and_end_jets():
    from spikes.ribbon_fairness import fair_join_spline
    from spikes.connector_cross_section import jet
    from pyclothoids import Clothoid
    cls=[Clothoid.StandardParams(0,0,0,0,0,length) for length in [6,12,14,12,6]]
    kn,co=fair_join_spline(cls,np.array([1.75,0.,0.]),np.array([1.75,0.,0.]))
    assert len(co)==8
    np.testing.assert_allclose(co[:,0],1.75,atol=1e-8)
    np.testing.assert_allclose(co[:,1:],0.,atol=1e-8)
    kn,co=fair_join_spline(cls,np.array([1.75,.01,.002]),np.array([1.85,-.01,.001]))
    np.testing.assert_allclose(jet(co[0],0),[1.75,.01,.002],atol=1e-8)
    np.testing.assert_allclose(jet(co[-1],kn[-1]-kn[-2]),[1.85,-.01,.001],atol=1e-8)
    for i in range(len(co)-1):
        np.testing.assert_allclose(jet(co[i],kn[i+1]-kn[i]),jet(co[i+1],0),atol=1e-8)


def test_strip_review_rejects_crossing_even_when_each_edge_is_simple():
    from scripts.review_measured_ribbon import sampled_strip_validity
    curves={'left':np.array([[0,1],[10,-1]]),'right':np.array([[0,-1],[10,1]]),
            'center':np.array([[0,0],[10,0]])}
    result=sampled_strip_validity(curves)
    assert all(result['curves_simple'].values())
    assert result['status']=='FAIL'


def test_exact_width_review_catches_negative_dip_between_samples():
    from scripts.review_measured_ribbon import minimum_width
    road=ordinary('1',0,0,0,'end');road.set('length','1')
    lanes=road.find('lanes');lanes.remove(lanes.findall('laneSection')[1])
    width=road.find('.//width')
    # (s-.257)^2-.000001: endpoints and a 0.05m grid look positive.
    for k,v in zip('abcd',[.257**2-1e-6,-.514,1.,0.]):width.set(k,str(v))
    assert minimum_width(road)==pytest.approx(-1e-6)


def test_shared_frame_preserves_center_and_both_edges_with_nonparallel_parent_widths():
    root=ET.Element('OpenDRIVE')
    root.append(ordinary('10',-80,0,0,'end'));root.append(ordinary('11',40,40,math.pi/2,'start'))
    cr=ET.SubElement(root,'road',id='100',length='60',junction='1')
    link=ET.SubElement(cr,'link')
    ET.SubElement(link,'predecessor',elementType='road',elementId='10',contactPoint='end')
    ET.SubElement(link,'successor',elementType='road',elementId='11',contactPoint='start')
    ET.SubElement(cr,'planView');group=ET.SubElement(cr,'lanes');sec=ET.SubElement(group,'laneSection',s='0')
    lane=ET.SubElement(ET.SubElement(sec,'right'),'lane',id='-1',type='driving');ll=ET.SubElement(lane,'link')
    ET.SubElement(ll,'predecessor',id='-1');ET.SubElement(ll,'successor',id='-1');ET.SubElement(lane,'speed',max=str(15/3.6),sOffset='0',unit='m/s')
    connection=ET.SubElement(ET.SubElement(root,'junction',id='1'),'connection',id='0',incomingRoad='10',connectingRoad='100',contactPoint='start')
    ET.SubElement(connection,'laneLink',attrib={'from':'-1','to':'-1'})
    original=ET.tostring(root);a,b=frames(root,cr);cls,_=endpoint_g3_chain(a,b,6.)
    z=np.r_[[c.length for c in cls],cls[1].KappaEnd*20,cls[2].KappaEnd*20]
    cls=chain(z,a,b);kn,co=ribbon(cls,a,b);written=write_ribbon(cr,cls,kn,co)
    assert ET.tostring(root)==original
    root.remove(cr);root.append(written)
    assert edge_audit(root)['status']=='PASS'
    assert all(x['position_m']<1e-6 and x['heading_deg']<1e-6 and x['curvature_per_m']<1e-7 for x in junction_lane_interfaces(root))
    assert len(written.findall('planView/geometry'))==5
    assert written.find('.//lane/speed').get('max')==str(15/3.6)
    points,jets,dyn=sample(cls,kn,co)
    assert np.isfinite(dyn).all()
    assert min(jets['left'][:,0]-jets['right'][:,0])>0
    np.testing.assert_allclose(points['center'],(points['left']+points['right'])/2)
