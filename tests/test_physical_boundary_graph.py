"""The outer collapsed border can continue while its lane divider is born."""
import copy
import json
from types import SimpleNamespace
import numpy as np

import pytest
from lxml import etree
from spikes.physical_boundary_graph import build_graph,source_tip_evidence


def fixture(side='right', merge=False):
    r=etree.Element('road',id='1',length='160',junction='-1')
    pv=etree.SubElement(r,'planView');g=etree.SubElement(pv,'geometry',s='0',x='0',y='0',hdg='0',length='160');etree.SubElement(g,'line')
    lanes=etree.SubElement(r,'lanes');sign='-' if side=='right' else ''
    groups=[[(1,3.5,0.,1),(2,3.5,0.,3)],[(1,3.5,0.,None),(2,0.,3.5/80,None),(3,3.5,0.,None)]]
    if merge:
        groups=[[(1,3.5,0.,1),(2,3.5,-3.5/80,None),(3,3.5,0.,2)],[(1,3.5,0.,None),(2,3.5,0.,None)]]
    for si,rows in enumerate(groups):
        sec=etree.SubElement(lanes,'laneSection',s=str(si*80));group=etree.SubElement(sec,side)
        for k,a,b,nxt in rows:
            lane=etree.SubElement(group,'lane',id=sign+str(k),type='driving')
            if nxt is not None: etree.SubElement(etree.SubElement(lane,'link'),'successor',id=sign+str(nxt))
            etree.SubElement(lane,'width',sOffset='0',a=str(a),b=str(b),c='0',d='0')
    return r


@pytest.mark.parametrize('side',['right','left'])
def test_split_boundary_graph_differs_from_travel_link_and_retains_every_occurrence(side):
    road=fixture(side);before=etree.tostring(road);s='-' if side=='right' else ''
    occ,succ,chains,ties,events=build_graph(road,side,lambda *a:True)
    assert succ[0,s+'1']==(1,s+'2')
    assert ties[1,s+'1','predecessor']==(1,s+'2')
    assert len(events)==1 and events[0]['kind']=='birth'
    assert {x for c in chains for x in c}==set(occ)
    assert etree.tostring(road)==before


@pytest.mark.parametrize('side',['right','left'])
def test_merge_outer_edge_continues_and_inner_divider_ends_on_it(side):
    road=fixture(side,True);s='-' if side=='right' else ''
    _,succ,_,ties,events=build_graph(road,side,lambda *a:True)
    assert (0,s+'1') not in succ
    assert succ[0,s+'2']==(1,s+'1')
    assert ties[0,s+'1','successor']==(0,s+'2')
    assert events[0]['kind']=='death'


def test_written_zero_without_source_tip_evidence_does_not_rewire():
    _,succ,_,ties,events=build_graph(fixture(),'right',lambda *a:False)
    assert succ[0,'-1']==(1,'-1')
    assert not ties and not events


def test_missing_link_is_not_inferred_from_reused_lane_number():
    road=fixture();lane=road.find("lanes/laneSection/right/lane[@id='-1']")
    lane.remove(lane.find('link'))
    _,succ,_,_,_=build_graph(road,'right',lambda *a:False)
    assert (0,'-1') not in succ


def test_ambiguous_many_to_one_cannot_silently_merge_boundary_families():
    road=fixture(); road.find("lanes/laneSection/right/lane[@id='-2']/link/successor").set('id','-1')
    with pytest.raises(ValueError,match='many-to-one'):build_graph(road,'right',lambda *a:True)


@pytest.mark.parametrize('direction,role',[('with_s','predecessor'),('against_s','successor')])
def test_tip_evidence_uses_source_end_not_driving_side(direction,role):
    lane=etree.Element('lane');etree.SubElement(lane,'userData',code='mapforge.provenance/v1',
        value=json.dumps({'travel_direction':direction}))
    rec=SimpleNamespace(s_width_known=True,e_width_known=True,s_width_mm=0,e_width_mm=3500)
    assert source_tip_evidence(lane,rec,role)
    assert not source_tip_evidence(lane,rec,'successor' if role=='predecessor' else 'predecessor')
    rec.s_width_known=False
    assert not source_tip_evidence(lane,rec,role)


def test_long_synthetic_split_solves_c2_boundary_graph_without_changing_travel_links(monkeypatch):
    pytest.importorskip('clarabel')
    from spikes import road_boundary_family as family
    from spikes.clarabel_joint_candidate import interior_qp
    from mapforge.validate.smoothness import lane_edges_kinematics_at
    road=fixture();road.set('length','240');road.find('planView/geometry').set('length','240')
    for i,lane in enumerate(road.findall('lanes/laneSection/right/lane'),1):
        etree.SubElement(lane,'userData',code='mapforge.source_lane',value=str(i))
        etree.SubElement(lane,'userData',code='mapforge.provenance/v1',value=json.dumps({'travel_direction':'with_s'}))
    before=[etree.tostring(e) for e in road.findall('.//lane/link')]
    geometry=etree.tostring(road.find('planView'))
    class Source:
        def lane(self,sid):
            return SimpleNamespace(s_width_known=True,e_width_known=True,
                s_width_mm=0 if sid=='4' else 3500,e_width_mm=3500)
        def lane_boundary_geometries(self,sid):
            s=np.linspace(0,80,81) if int(sid)<=2 else np.linspace(80,240,161)
            u=(s-80)/160;w=3.5*(10*u**3-15*u**4+6*u**5)
            a,b={'1':(0.,-3.5),'2':(-3.5,-7.),'3':(w,w-3.5),'4':(w-3.5,-3.5),'5':(-3.5,-7.)}[sid]
            return [np.c_[s,np.broadcast_to(v,s.shape)] for v in (a,b)]
    monkeypatch.setattr(family,'_convex_qp',interior_qp)
    result=family.solve_road(road,Source(),np.asarray,physical_graph=True,
        boundary_association='source-order',source_error_budget='absolute')
    assert result['status']=='CANDIDATE',result
    assert result['source_max_m']<=.350001 and result['minimum_independent_span_m']>=15.
    assert result['exact_construction_ratio']<=1.000001
    assert result['physical_graph_events'][0]['edge_continuation']==['-1','-2']
    assert [etree.tostring(e) for e in road.findall('.//lane/link')]==before
    assert etree.tostring(road.find('planView'))==geometry
    prev=np.array(lane_edges_kinematics_at(road,80.-1e-7,'right'))
    post=np.array(lane_edges_kinematics_at(road,80.+1e-7,'right'))
    np.testing.assert_allclose(prev[[0,1,2]],post[[0,2,3]],atol=1e-7)
    np.testing.assert_allclose(post[1],post[2],atol=1e-7)
