"""Read-only integration against the retained (NOT accepted) node4 research map."""
import hashlib
from pathlib import Path

from lxml import etree
import pytest

from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops.reconstruction_scope import whole_junction_scope, prepare_scope, require_solver_input
from mapforge.ops.source_domain import prepare_source_domain, require_domain_replay

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr'


@pytest.fixture(scope='module')
def current():
    if not BASE.exists() or not (ROOT/'shp_0222-0326/IBD_LANE_LINK.shp').exists():
        pytest.skip('local retained node4 and original SHP required')
    before=hashlib.sha256(BASE.read_bytes()).hexdigest()
    root=etree.parse(str(BASE)).getroot(); source=ProfileSource(str(ROOT/'shp_0222-0326'),'ibd-smarteditor-v1')
    whole=whole_junction_scope(root,'1')
    packet=prepare_scope(root,source,whole['ordinary_roads'],
                         {'source_junction_id':'2023041113170861627','xodr_junction_id':'1'},'source-chain-v1')
    domain=prepare_source_domain(root,source,packet,'2023041113170861627','1','exact-line-arc-v1')
    yield root,source,whole,packet,domain
    assert hashlib.sha256(BASE.read_bytes()).hexdigest()==before


def test_current_whole_input_contains_all_chains_and_all_source_movements(current):
    root,source,whole,packet,domain=current
    assert whole['ordinary_roads']==['10','11','12','13','30']
    assert len(whole['connectors'])==24 and not packet['fixed_ports']
    assert len(packet['observations'])==120 and len(packet['boundaries'])==171
    assert packet['source_admission']=='ADMITTED_FOR_RESEARCH' and not packet['issues']
    assert domain['comparison']['status']=='MATCH'
    assert domain['comparison']['expected_count']==domain['comparison']['written_count']==24
    assert require_solver_input(packet,root,source)


def test_current_unassigned_tails_are_not_hidden_by_chain_or_arc_support(current):
    _,_,_,packet,domain=current
    partition=domain['partition']
    assert partition['length_by_status_m']['UNASSIGNED_REQUIRES_SCOPE_DECISION']==pytest.approx(7.5263480261,abs=1e-7)
    assert partition['status']=='UNRESOLVED'
    assert set(partition['features'])==({'lane:'+s for s in packet['observations']} |
                                       {'boundary:'+b for b in packet['boundaries']})
    for f in partition['features'].values():
        for p in f['parts']:
            assert len(p['raw_vertices'])==len(p['vertices'])
            assert sum(a['source_s_m'][1]-a['source_s_m'][0] for a in p['atoms'])==pytest.approx(p['length_m'])
    assert not domain['export_allowed'] and not domain['geometry_solver_ran']


def test_current_speed_comparison_and_fresh_readback_do_not_approve_map(current):
    root,source,_,packet,domain=current
    counts=domain['source_speed_intervals']['counts']
    assert counts=={'NO_ORIGINAL_OVERLAP':976,'MATCH':453}
    assert require_domain_replay(domain,root,source,packet)
    assert not domain['partition']['geometry_fit_validated']


def test_current_full_source_contacts_use_arc_station_and_keep_all_role_conflicts(current):
    from mapforge.ops.source_contacts import compile_source_contacts
    from mapforge.ops.source_domain import _arc_axis
    root,source,_,scope,domain=current
    before=scope['content_sha256']
    contacts=compile_source_contacts(root,source,scope,domain,chart_mode='exact-line-arc-v1')
    assert contacts['schema']=='mapforge.source-contact-model/v2'
    assert contacts['source_support_compiled'] and not contacts['issues']
    assert len(contacts['full_source_support'])==291
    assert len(contacts['transition_events'])==77 and len(contacts['role_conflicts'])==9
    assert len(contacts['source_endpoint_inventory'])==240
    for event in contacts['transition_events']:
        assert event['status']=='SOURCE_C0_CONTACT_PROVEN'
        points=[contacts['boundary_endpoints'][c[key]]['xy']
                for c in event['contacts'] for key in ('from_endpoint','to_endpoint')]
        s=_arc_axis(contacts['exact_parent_charts'][event['road']]).project(points)[:,0]
        assert event['source_station_band_m']==pytest.approx([min(s),max(s)],abs=1e-10)
    assert scope['content_sha256']==before
    assert not contacts['export_allowed'] and not contacts['geometry_solver_ran']
    assert not contacts['policy']['source_roles_automatically_changed']
