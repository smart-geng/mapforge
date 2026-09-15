import copy

import numpy as np
import pytest

from mapforge.ops.source_transition_domains import source_transition_domains,transition_source_profile
from spikes.source_contact_fit import SourceBoundaryBlock
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from spikes.shared_boundary_dynamics import midpoint_rows,audit_transition_bands
from spikes.source_jet_relaxation import SourceJetRelaxation,graph_derivative_bounds
from tests.test_source_contact_fit import straight_fixture


def oblique_fixture(degree=5):
    # Synthetic two-source road only: left contact x=29, right contact x=31.
    # The source-center contact remains x=30. No real source file is edited.
    m=SourceBoundaryBlock(*straight_fixture(),degree=degree)
    e=np.asarray(m.chart['tangent']);n=np.array([-e[1],e[0]]);changed={}
    for event in m.contacts['transition_events']:
        for pair in event['contacts']:
            s=29. if pair['from_side']=='left' else 31.
            for name in ('from_endpoint','to_endpoint'):
                tip=m.contacts['boundary_endpoints'][pair[name]];key=tip['feature']
                i=m.source_vertex_indices[key].index(tip['vertex_index'])
                changed[(key,float(m.raw[key][i,0]))]=s
                m.raw[key][i,0]=s
                tip['xy']=(m.chart['origin']+m.raw[key][i]@np.stack([e,n])).tolist()
    m.endpoint_groups=[[(key,changed.get((key,s),s)) for key,s in group] for group in m.endpoint_groups]
    m._build_constraints()
    return m


def test_oblique_source_contact_closes_gap_without_new_geometry_or_lane_identity():
    m=oblique_fixture();before=copy.deepcopy(m.raw);count=m.nvar
    c=source_transition_domains(m)
    assert len(c['intervals'])==1 and sum(e['added_length_m'] for e in c['events'])==pytest.approx(2.)
    d=c['intervals'][0];assert [d['a'],d['b']]==[29.,31.]
    assert d['source_lanes']==['a','b'] and d['source_speed_kmh']==60.
    assert not d['geometry_added'] and not d['lane_link_inferred'] and not c['export_allowed']
    rows,scale,labels=midpoint_rows(m)
    gap=[v for v in labels if v['domain_kind']=='source_transition_band']
    assert any(v['s']==30 for v in gap)
    assert all(v['source_lanes']==['a','b'] for v in gap)
    assert m.nvar==count and all(np.array_equal(m.raw[k],v) for k,v in before.items())
    x,phase=m.solve();assert x is not None,phase
    a=audit_transition_bands(m,x)
    assert a['exact_width_min_m']==pytest.approx(4.,abs=1e-6)
    assert max(r['value'] for r in a['rows'])<1e-5


def test_gap_heading_bound_uses_proven_adjacent_source_context_not_only_gap():
    m=oblique_fixture();d=source_transition_domains(m)['intervals'][0]
    source=transition_source_profile(m,d,np.unique(np.concatenate([r[:,0] for r in m.raw.values()])))
    assert source[0,0]==pytest.approx(0.) and source[-1,0]==pytest.approx(60.)
    assert max(abs(source[:,1]))<1e-7
    full=graph_derivative_bounds(source,[29,31],.35,.009,.000216)
    tiny=graph_derivative_bounds(np.array([[29.,0.],[31.,0.]]),[29,31],.35,.009,.000216)
    assert full['slope']<tiny['slope']
    check=SourceJetRelaxation(m,2.)
    added=[r for r in check.dynamic_intervals if r.get('domain_kind')=='source_transition_band']
    assert added and all(r['source_lanes']==['a','b'] for r in added)
    _,report=check.solve();assert report['status']=='RELAXATION_FEASIBLE_ONLY'
    assert not report['export_allowed']


@pytest.mark.parametrize('fault',['speed','side','relation','station'])
def test_gap_closure_requires_source_authority_and_never_guesses(fault):
    m=oblique_fixture();event=m.contacts['transition_events'][0];pair=event['contacts'][0]
    if fault=='speed':m.scope['observations']['b']['source_max_speed_kmh']=40
    elif fault=='side':pair['to_side']='right'
    elif fault=='relation':m.physical_graph['relations']=[]
    else:
        tip=m.contacts['boundary_endpoints'][pair['to_endpoint']]
        tip['xy'][0]+=1
    with pytest.raises(ValueError):source_transition_domains(m)


def test_arc_chart_reprojects_source_band_instead_of_reusing_old_stations():
    original=oblique_fixture(degree=3);m=ArcSourceBoundaryBlock(original,curvature=.001)
    c=source_transition_domains(m);assert c['intervals']
    a,b=c['intervals'][0]['a'],c['intervals'][0]['b']
    assert abs(a-29)>1e-3 or abs(b-31)>1e-3
    x,phase=m.solve();assert x is not None,phase
    audit=audit_transition_bands(m,x)
    assert audit['rows'] and audit['exact_width_min_m']>3
    with pytest.raises(ValueError):transition_source_profile(m,c['intervals'][0],[a,b])
