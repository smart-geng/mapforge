"""Actual side support, continuous laneOffset, no fictitious zero-width road."""
import copy

import numpy as np
import pytest
from lxml import etree as ET

from test_source_cubic_export import fixture
from mapforge.ops import source_cubic_export as compiler
from mapforge.ops.source_export_domain import direction_starts,plan_source_domain
from mapforge.validate.smoothness import lane_edges_at


def asymmetric(monkeypatch):
    m,x,old=fixture(monkeypatch)
    chains=compiler.source_chains(m)
    monkeypatch.setattr('mapforge.ops.source_export_domain.source_chains',lambda _:chains)
    chains[0]['a']=20.;chains[0]['domains'][0]['a']=20.
    ET.SubElement(ET.SubElement(old,'link'),'successor',elementType='junction',elementId='1')
    domain=plan_source_domain(m,old)
    return m,x,old,domain


def test_asymmetric_start_uses_real_one_sided_lanes_and_continuous_offset(monkeypatch):
    m,x,old,domain=asymmetric(monkeypatch)
    assert domain['direction_starts_m']=={'with_s':20.,'against_s':0.}
    assert domain['end_m']==40. and len(domain['retained_junction_tails'])==2
    assert not domain['aggregate_source_coverage_accepted']
    before=ET.tostring(old)
    road,report,ports=compiler.compile_road(m,x,old,source_domain=domain)
    assert ET.tostring(old)==before
    sections=road.findall('lanes/laneSection')
    assert sections[0].find('right') is None  # absent, not 20m fake zero-width lanes
    assert [e.get('id') for e in sections[0].findall('left/lane')]==['1']
    assert [e.get('id') for e in sections[1].findall('right/lane')]==['-1','-2']
    for s in np.linspace(0,40,81):
        assert lane_edges_at(road,float(s),'left')==pytest.approx([m.expression(k,s)@x for k in ('br','bl')])
        if s>=20:
            assert lane_edges_at(road,float(s),'right')==pytest.approx([m.expression(k,s)@x for k in ('br','al','ar')])
    assert len(road.findall('planView/geometry'))==1
    assert report['minimum_width_span_m']==20
    assert ('start',-1) not in ports and ports['end',-1]==-2
    assert ports['start',2]==1
    assert sections[0].find('left/lane/link/successor').get('id')=='1'
    assert sections[1].find("right/lane[@id='-2']/link/predecessor") is None
    assert not report['production_accepted']


def test_linked_external_port_may_not_disappear(monkeypatch):
    m,x,old,domain=asymmetric(monkeypatch)
    ET.SubElement(old.find('link'),'predecessor',elementType='road',elementId='9',contactPoint='end')
    with pytest.raises(ValueError,match='upstream dependency rebuild'):
        compiler.compile_road(m,x,old,source_domain=domain)


def test_domain_requires_original_junction_and_structural_knot(monkeypatch):
    m,x,old,domain=asymmetric(monkeypatch)
    old.find('link').remove(old.find('link/successor'))
    with pytest.raises(ValueError,match='original junction'):plan_source_domain(m,old)
    ET.SubElement(old.find('link'),'successor',elementType='junction',elementId='1')
    m.global_breaks=np.array([0.,19.,40.,60.])
    with pytest.raises(ValueError,match='rebuild shared cubic basis'):plan_source_domain(m,old)


def test_direction_start_does_not_move_later_interior_birth(monkeypatch):
    m,x,old,domain=asymmetric(monkeypatch)
    chains=compiler.source_chains(m)
    birth=copy.deepcopy(chains[0]);birth['a']=35.;birth['key']=3;chains.append(birth)
    assert direction_starts(m)['with_s']==20.


def observations(m,x):
    from mapforge.ops.arc_source_chart import ArcChart
    m.axis=ArcChart((0.,0.),0.,0.)
    m.contact_station=lambda xy:float(m.axis.project(np.asarray(xy))[0])
    for k,fi in m.owner.items():m.families[fi].features=[k]
    m.raw={}
    for key in m.owner:
        ss=np.linspace(20 if key.startswith('a') else 0,60,81)
        m.raw[key]=np.c_[ss,[m.expression(key,s)@x for s in ss]]
    for sid,l,r in [('a','al','ar'),('b','bl','br')]:m.raw['lane:'+sid]=.5*(m.raw[l]+m.raw[r])
    m.source_xy=copy.deepcopy(m.raw)
    m.roles={'feature_roles':{'lane:'+sid:{'role':'physical_lane_center_observation'} for sid in ('a','b')}}
    m.contacts={'source_endpoint_inventory':[],'boundary_endpoints':{}}
    m.C=np.zeros((0,m.nvar));m.lower=np.zeros(0);m.labels=[]


def test_missing_tail_ownership_cannot_turn_cropped_component_into_pass(monkeypatch):
    from scripts.review_asymmetric_dependents import aggregate_original
    m,x,old,domain=asymmetric(monkeypatch);observations(m,x)
    road,compiled,_=compiler.compile_road(m,x,old,source_domain=domain)
    root=ET.Element('OpenDRIVE');root.append(road)
    report,_=aggregate_original(root,m,compiled,[],None,None,step=.5)
    assert report['status']=='FAIL' and len(report['unassigned_tails'])==2
    assert report['full_source_to_written_max_m']>19.9
    assert report['source_vertices_removed']==0
    assert sum(r['raw_vertices'] for r in report['rows'])==sum(len(xy) for xy in m.source_xy.values())


def test_source_domain_cannot_be_changed_just_to_export(monkeypatch):
    m,x,old,domain=asymmetric(monkeypatch);domain['end_m']=39.
    with pytest.raises(ValueError,match='domain differs'):compiler.compile_road(m,x,old,source_domain=domain)


def test_asymmetric_cap_uses_full_euclidean_budget_and_retains_zero_tip(monkeypatch):
    from mapforge.ops.source_export_domain import constrain_external_cuts
    m,x,old,domain=asymmetric(monkeypatch);observations(m,x)
    t=m.expression('al',20)@x
    m.raw['al']=np.vstack([[19.8,t-.3],m.raw['al']]);m.source_xy['al']=m.raw['al'].copy()
    constrain_external_cuts(m,domain)
    assert max(m.lower-m.C@x)>0  # sqrt(.2**2+.3**2) > .35
    chains=compiler.source_chains(m);chains[0]['a']=19.9;chains[0]['domains'][0]['a']=19.9
    m.contacts={'source_endpoint_inventory':[dict(source_lane_id='a',width_mm=0,boundary_endpoints={'left':'tip'})],
        'boundary_endpoints':{'tip':{'xy':[19.9,0.]}}}
    with pytest.raises(ValueError,match='move an original zero-width tip'):constrain_external_cuts(m,domain)


def test_new_section_station_near_cap_does_not_create_tiny_cubic_family():
    from tests.test_source_contact_fit import straight_fixture
    from spikes.source_contact_fit import SourceBoundaryBlock
    from spikes.arc_source_boundary import ArcSourceBoundaryBlock
    m=SourceBoundaryBlock(*straight_fixture(),degree=3)
    # Structural start at 20cm is not a reference primitive or a freedom to
    # bend a boundary inside its tiny cap; same cubic must cross it.
    # Required global span is itself too short, so it must be rejected here.
    with pytest.raises(ValueError,match='sub-budget'):ArcSourceBoundaryBlock(m,structural_stations=[.2])
