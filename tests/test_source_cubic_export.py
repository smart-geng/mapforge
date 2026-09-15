import copy
from types import SimpleNamespace

import numpy as np
import pytest
from lxml import etree as ET

from mapforge.ops import source_cubic_export as compiler
from mapforge.validate.smoothness import lane_edges_at


def fixture(monkeypatch):
    model=SimpleNamespace(degree=3,reference_curvature=0.,nvar=16,road='10',min_span=6.,source_tol=.35,
        chart={'origin':[0.,0.],'tangent':[1.,0.]},global_breaks=np.array([0.,20.,40.,60.]),
        owner={'al':0,'ar':1,'bl':2,'br':3},families=[SimpleNamespace(knots=np.array([0.,60.])) for _ in range(4)])
    def expression(feature,s,derivative=0):
        from math import factorial
        row=np.zeros(16);i=model.owner[feature]
        for power in range(derivative,4):row[i*4+power]=factorial(power)/factorial(power-derivative)*s**(power-derivative)
        return row
    model.expression=expression
    model.audit=lambda x:dict(inequality_violation=0.,scaled_C2_residual=0.,boundary_same_chart_max_m=0.,exact_width_min_m=3.,
        full_source_center_support={'physical_source_to_center_certified':True})
    x=np.array([[1.,.02,-.0002,.000001],[-2.,.02,-.0002,.000001],[5.,.02,-.0002,.000001],[2.,.02,-.0002,.000001]]).ravel()
    chains=[]
    for k,sid,l,r,d in ((0,'a','al','ar','with_s'),(1,'b','bl','br','against_s')):
        chains.append(dict(key=k,source_ids=[sid],left=l,right=r,a=0.,b=60.,direction=d,speed_kmh=60.,
                           domains=[dict(left=l,right=r,a=0.,b=60.,source_lanes=[sid])]))
    monkeypatch.setattr(compiler,'source_chains',lambda _:chains)
    road=ET.fromstring(b'''<road id="10" length="60" junction="-1" name="test">
      <planView><geometry s="0" x="0" y="0" hdg="0" length="60"><line/></geometry></planView>
      <lanes><laneSection s="0"><left><lane id="2" type="driving"><userData code="mapforge.source_lane" value="b"/></lane></left>
      <center><lane id="0" type="none"/></center><right><lane id="-1" type="driving"><userData code="mapforge.source_lane" value="a"/></lane></right>
      </laneSection></lanes></road>''')
    return model,x,road


def test_cubic_widths_are_exact_shared_differences_and_readback_keeps_source_speed(monkeypatch):
    m,x,old=fixture(monkeypatch);before=ET.tostring(old)
    road,report,ports=compiler.compile_road(m,x,old)
    assert ET.tostring(old)==before
    assert len(road.findall('planView/geometry'))==1 and len(road.findall('lanes/laneSection'))==3
    assert report['minimum_width_span_m']==20 and not report['production_accepted']
    assert ports=={('start',2):2,('end',2):2,('start',-1):-1,('end',-1):-1}
    for s in np.linspace(0,60,121):
        assert lane_edges_at(road,float(s),'right')==pytest.approx([m.expression(k,s)@x for k in ('al','ar')])
        assert lane_edges_at(road,float(s),'left')==pytest.approx([m.expression(k,s)@x for k in ('al','br','bl')])
    assert {float(e.get('max')) for e in road.findall('.//speed')}=={60/3.6}
    assert all(len(l.findall('width'))==1 for l in road.findall('lanes/laneSection/left/lane')+road.findall('lanes/laneSection/right/lane'))


def test_quintic_and_failed_geometry_cannot_be_exported(monkeypatch):
    m,x,old=fixture(monkeypatch);m.degree=5
    with pytest.raises(ValueError,match='direct cubic'):compiler.compile_road(m,x,old)
    with pytest.raises(ValueError,match='quintic'):compiler.power(m,x,'al',0)
    m.degree=3;m.audit=lambda _:dict(inequality_violation=.001)
    with pytest.raises(ValueError,match='state failed'):compiler.compile_road(m,x,old)


def test_cubic_minimum_catches_interior_negative_width():
    assert compiler.coefficients_minimum([.1,-1.,1.,0.],1.)==pytest.approx(-.15)


def test_unrepresented_objects_and_short_written_spans_reject(monkeypatch):
    m,x,old=fixture(monkeypatch);ET.SubElement(ET.SubElement(old,'objects'),'object',id='12')
    with pytest.raises(ValueError,match='object'):compiler.compile_road(m,x,old)
    old.remove(old.find('objects'));m.global_breaks=np.array([0.,1.,20.,40.,60.])
    with pytest.raises(ValueError,match='short output'):compiler.compile_road(m,x,old)


def test_non_shared_adjacent_boundaries_do_not_get_filled(monkeypatch):
    m,x,old=fixture(monkeypatch)
    # Cross the two innermost boundaries; no abs(width) or synthetic paving.
    x[12]=-2.
    with pytest.raises(ValueError,match='median edges cross'):compiler.compile_road(m,x,old)


def test_geometry_only_connector_mode_never_returns_accepted_status():
    import xml.etree.ElementTree as stdET
    from spikes.measured_connector_caps import fit
    road=stdET.fromstring('''<road id="100" length="40"><planView/>
      <lanes><laneSection s="0"><right><lane id="-1" type="driving"><link/><width sOffset="0" a="4" b="0" c="0" d="0"/>
      <speed sOffset="0" max="16.666666666666668" unit="m/s"/></lane></right></laneSection></lanes></road>''')
    def frame(x):
        return dict(pose=(x,0.,0.),k=0.,dk=0.,edges={side:dict(x=x,y=y,heading=0.,curvature=0.) for side,y in [('left',2.),('right',-2.)]})
    raw={side:np.array([[0.,y],[40.,y]]) for side,y in [('left',2.),('right',-2.),('center',0.)]}
    original=stdET.tostring(road)
    _,r=fit(road,frame(0.),frame(40.),raw,optimize=False,require_dynamics=False)
    assert r['status'] in ('GEOMETRY_REVIEW_CANDIDATE','REJECTED')
    assert not r['dynamics_required_for_this_research_selection'] and not r['production_accepted']
    assert r['dynamics_status']=='PASS' and stdET.tostring(road)==original


def test_written_source_audit_detects_end_loss_and_uses_physical_side_order(monkeypatch):
    from mapforge.validate.source_cubic_readback import audit_written_source
    m,x,old=fixture(monkeypatch)
    for feature,fi in m.owner.items():m.families[fi].features=[feature]
    ss=np.linspace(0,60,61)
    m.raw={k:np.c_[ss,[m.expression(k,s)@x for s in ss]] for k in m.owner}
    for sid,l,r in (('a','al','ar'),('b','bl','br')):
        m.raw['lane:'+sid]=.5*(m.raw[l]+m.raw[r])
    m.source_xy=copy.deepcopy(m.raw)
    m.roles={'feature_roles':{'lane:'+sid:{'role':'physical_lane_center_observation'} for sid in ('a','b')}}
    road,report,_=compiler.compile_road(m,x,old)
    a=audit_written_source(road,report,m,step=.2)
    assert a['status']=='SAMPLED_PASS_NOT_CERTIFICATE' and a['source_to_written_max_m']<.01
    assert not a['complete_continuous_certificate']
    # Preserve an original source endpoint outside the written transverse cut.
    # It must fail even when the solver's own source audit still says pass.
    m.source_xy['al']=np.vstack([[-1.,m.raw['al'][0,1]],m.source_xy['al']])
    a=audit_written_source(road,report,m,step=.2)
    assert a['status']=='FAIL' and a['source_to_written_max_m']>=1.-1e-7
    assert a['source_vertices_removed']==0


def test_single_arc_compilation_uses_exact_frame_and_written_world_edges(monkeypatch):
    from mapforge.ops.arc_source_chart import ArcChart
    from mapforge.validate.source_cubic_readback import audit_written_source
    import xml.etree.ElementTree as stdET
    m,x,old=fixture(monkeypatch);m.reference_curvature=-.001
    m.axis=ArcChart((3.,-4.),.4,m.reference_curvature)
    m.chart={'origin':[3.,-4.],'tangent':[np.cos(.4),np.sin(.4)]}
    ss=np.linspace(0,60,301)
    m.raw={k:np.c_[ss,[m.expression(k,s)@x for s in ss]] for k in m.owner}
    for k,i in m.owner.items():m.families[i].features=[k]
    for sid,l,r in (('a','al','ar'),('b','bl','br')):m.raw['lane:'+sid]=.5*(m.raw[l]+m.raw[r])
    m.source_xy={k:m.axis.world(st) for k,st in m.raw.items()}
    m.roles={'feature_roles':{'lane:'+sid:{'role':'physical_lane_center_observation'} for sid in ('a','b')}}
    old.set('name','真实道路')
    road,report,_=compiler.compile_road(m,x,old)
    assert len(road.findall('planView/geometry'))==1
    assert float(road.find('planView/geometry/arc').get('curvature'))==-.001
    actual=stdET.fromstring(ET.tostring(road,encoding='utf-8'))
    assert actual.get('name')=='真实道路'
    result=audit_written_source(actual,report,m,step=.1)
    assert result['status']=='SAMPLED_PASS_NOT_CERTIFICATE'
    assert result['source_to_written_max_m']<1e-4


def test_written_endpoint_constraints_spend_one_euclidean_budget(monkeypatch):
    from mapforge.ops.arc_source_chart import ArcChart
    m,x,_=fixture(monkeypatch);m.axis=ArcChart((0.,0.),0.,0.)
    m.C=np.zeros((0,16));m.lower=np.zeros(0);m.labels=[]
    m.raw={k:np.c_[[0.,60.],[m.expression(k,s)@x for s in (0.,60.)]] for k in m.owner}
    for sid,l,r in (('a','al','ar'),('b','bl','br')):m.raw['lane:'+sid]=.5*(m.raw[l]+m.raw[r])
    m.roles={'feature_roles':{'lane:'+sid:{'role':'physical_lane_center_observation'} for sid in ('a','b')}}
    m.raw['al']=np.vstack([[-.2,x[0]-.3],m.raw['al']]);m.source_xy=copy.deepcopy(m.raw)
    compiler.constrain_written_endpoints(m)
    # .3 m lateral alone passes; sqrt(.3^2+.2^2) must fail the .35 m budget.
    assert len(m.C)==2 and max(m.lower-m.C@x)>0
    x[0]-=.03
    assert max(m.lower-m.C@x)<=0
    m.raw['al'][0,0]=-.36;m.source_xy['al'][0,0]=-.36
    with pytest.raises(ValueError,match='endpoint budget impossible'):compiler.constrain_written_endpoints(m)


def test_moved_port_uses_supporting_source_record_without_changing_topology():
    chain=dict(source_ids=['first','second'],domains=[dict(a=0.,b=40.,source_lanes=['first']),
                                                  dict(a=40.,b=60.,source_lanes=['second'])])
    before=copy.deepcopy(chain)
    assert compiler.source_at_port(chain,10.)=='first'
    assert compiler.source_at_port(chain,55.)=='second'
    assert chain==before
    with pytest.raises(ValueError,match='unique original'):compiler.source_at_port(chain,65.)
    with pytest.raises(ValueError,match='unique original'):compiler.source_at_port(chain,40.)


def test_source_speed_restoration_does_not_inherit_old_curve_tuned_limits():
    import xml.etree.ElementTree as stdET
    from scripts.build_source_geometry_candidate import restore_source_speeds
    road=stdET.fromstring('''<road id="100"><lanes><laneSection s="0"><right><lane id="-1" type="driving">
      <width sOffset="0" a="3.5" b="0" c="0" d="0"/><speed sOffset="0" max="3.333333333"/>
      <userData code="mapforge.source_lane" value="original"/></lane></right></laneSection></lanes></road>''')
    source=SimpleNamespace(lane=lambda _:SimpleNamespace(max_speed_kmh=60))
    width=stdET.tostring(road.find('.//width'));rows=restore_source_speeds([road],source)
    assert float(road.find('.//speed').get('max'))==60/3.6
    assert rows[0]['source_changed'] is False and rows[0]['original_source_kmh']==60
    assert stdET.tostring(road.find('.//width'))==width
    source.lane=lambda _:SimpleNamespace(max_speed_kmh=0)
    with pytest.raises(ValueError,match='original connector speed'):restore_source_speeds([road],source)


def test_c2_linear_system_keeps_physical_jets_at_unequal_long_spans():
    from spikes.connector_cross_section import flat_join_spline,jet
    cls=[SimpleNamespace(length=v) for v in (6.,100.,6.,100.,6.)]
    start=np.array([3.,.1,-.01]);end=np.array([-1.,-.1,.01])
    knots,c=flat_join_spline(cls,start,end)
    assert jet(c[0],0)==pytest.approx(start,abs=1e-9)
    assert jet(c[-1],knots[-1]-knots[-2])==pytest.approx(end,abs=1e-9)
    for i,L in enumerate(np.diff(knots)[:-1]):assert jet(c[i],L)==pytest.approx(jet(c[i+1],0),abs=1e-9)


def test_shared_port_hypothesis_adds_constraints_not_geometry_variables(monkeypatch):
    m,x,_=fixture(monkeypatch);m.E=np.zeros((0,m.nvar));m.equality_labels=[]
    for k,fi in m.owner.items():m.families[fi].features=[k]
    before=m.global_breaks.copy();compiler.constrain_flat_connector_port(m)
    assert m.nvar==16 and np.array_equal(m.global_breaks,before)
    assert len(m.E)==4 and m.connector_port_hypothesis['source_changed'] is False
    for row,k in zip(m.E,m.owner):assert row@x==pytest.approx(m.expression(k,60.,1)@x)


@pytest.mark.parametrize('direction,side,removed', [('with_s','right','left'),('against_s','left','right')])
def test_explicit_one_direction_road_preserves_source_edges_without_fake_opposite(monkeypatch,direction,side,removed):
    m,x,old=fixture(monkeypatch);chains=compiler.source_chains(m)
    with pytest.raises(ValueError,match='exactly that original'):compiler.compile_road(m,x,old,source_direction=direction)
    monkeypatch.setattr(compiler,'source_chains',lambda _: [c for c in chains if c['direction']==direction])
    sec=old.find('lanes/laneSection');sec.remove(sec.find(removed))
    if direction=='against_s':sec.find('left/lane').set('id','1')
    original=ET.tostring(old)
    with pytest.raises(ValueError,match='bidirectional'):compiler.compile_road(m,x,old)
    road,report,ports=compiler.compile_road(m,x,old,source_direction=direction)
    assert not road.findall(f'lanes/laneSection/{removed}/lane') and not road.findall('.//lane[@type="median"]')
    assert len(road.findall('planView/geometry'))==1
    keys=('al','ar') if direction=='with_s' else ('br','bl')
    for s in np.linspace(0,60,121):
        assert lane_edges_at(road,float(s),side)==pytest.approx([m.expression(k,s)@x for k in keys])
    assert report['source_direction']==direction and len(ports)==2
    assert ET.tostring(old)==original
