import copy

import pytest

from mapforge.ops.physical_continuations import compile_physical_continuations
from mapforge.ops.source_contacts import compile_source_contacts
from mapforge.ops.source_roles import resolve_source_roles
from tests.test_source_contacts import source_fixture,seal
from tests.test_source_roles import decision_for


def packets():
    root,source,scope,domain=source_fixture()
    contacts=compile_source_contacts(root,source,scope,domain)
    roles=resolve_source_roles(scope,domain,contacts,decision_for(contacts))
    return contacts,roles


def test_taper_contact_is_not_an_ordinary_lane_link_but_keeps_g2():
    c,r=packets();before=copy.deepcopy((c,r));result=compile_physical_continuations(c,r)
    assert len(result['forks'])==1
    assert result['forks'][0]['continuing_edge_pairs']==1
    assert result['forks'][0]['taper_only_pairs']==1
    taper=[e for e in result['relations'] if e['role']=='zero_width_taper_contact']
    assert len(taper)==1 and not taper[0]['ordinary_lane_link_supported']
    assert all(e['required_geometric_continuity']=='G2' for e in result['relations'])
    assert any(len(e['witnesses'])==2 for e in result['relations'])  # same contact, two source roles
    assert not result['xodr_lane_links_generated'] and not result['export_allowed']
    assert before==(c,r)


def test_unpaired_zero_endpoint_gets_g2_but_no_fabricated_topology_or_family_union():
    root,source,scope,domain=source_fixture()
    source.rows=[r for r in source.rows if r['attributes']['T']!='c']
    contacts=compile_source_contacts(root,source,scope,domain)
    roles=resolve_source_roles(scope,domain,contacts,decision_for(contacts))
    before=copy.deepcopy((contacts,roles))
    graph=compile_physical_continuations(contacts,roles)
    assert len(contacts['transition_events'])==1
    assert len(graph['external_zero_width_tapers'])==1
    row=graph['external_zero_width_tapers'][0]
    assert row['required_geometric_continuity']=='G2' and not row['ordinary_lane_link_supported']
    assert row['source_group_index'] is None and row['witnesses'][0]['topology_record'] is None
    assert row['witnesses'][0]['source_lane_id']=='c'
    assert before==(contacts,roles)
    bad=copy.deepcopy(contacts)
    bad['boundary_endpoints'][row['endpoints'][1]]['xy'][0]+=.001
    seal(bad);roles['decision']['source_contact_sha256']=bad['content_sha256'];seal(roles)
    with pytest.raises(ValueError,match='noncoincident'):compile_physical_continuations(bad,roles)


@pytest.mark.parametrize('fault',['body','revision','unresolved','status','kind','side','endpoint','group'])
def test_typing_fails_closed(fault):
    c,r=packets()
    if fault=='body':c['export_allowed']=True
    elif fault=='revision':r['decision']['source_contact_sha256']='stale';seal(r)
    elif fault=='unresolved':r['role_binding_complete']=False;seal(r)
    else:
        e=c['transition_events'][0]
        if fault=='status':e['status']='GUESS'
        if fault=='kind':e['kind']='geometric_nearest'
        if fault=='side':e['contacts'][0]['to_side']='opposite'
        if fault=='endpoint':e['contacts'][0]['to_endpoint']='unknown'
        if fault=='group':c['contact_groups'].pop()
        seal(c);r['decision']['source_contact_sha256']=c['content_sha256'];seal(r)
    with pytest.raises(ValueError):compile_physical_continuations(c,r)
