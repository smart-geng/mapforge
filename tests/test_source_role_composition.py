"""Prior approval replay is source-specific, not a new policy for all tapers."""
import copy

import pytest

from mapforge.ops.source_roles import (compose_reviewed_source_roles,
    replay_source_role_packet, resolve_source_roles)
from tests.test_source_roles import fixture
from tests.test_source_contacts import seal


def inputs():
    root,source,scope,domain,contacts,decision=fixture()
    old=dict(scope=copy.deepcopy(scope),domain=copy.deepcopy(domain),
             contacts=copy.deepcopy(contacts),decision=copy.deepcopy(decision))
    # Added provenance changes the target packet revision, not original facts.
    scope['identity_mode']='source-chain-v1';seal(scope)
    domain['scope_sha256']=scope['content_sha256'];seal(domain)
    contacts['scope_sha256']=scope['content_sha256']
    contacts['source_domain_sha256']=domain['content_sha256'];seal(contacts)
    return root,scope,domain,contacts,old


def reseal_target(scope,domain,contacts):
    seal(scope);domain['scope_sha256']=scope['content_sha256'];seal(domain)
    contacts['scope_sha256']=scope['content_sha256']
    contacts['source_domain_sha256']=domain['content_sha256'];seal(contacts)


def test_review_replays_only_identical_facts_without_a_new_human_decision():
    _,scope,domain,contacts,old=inputs();before=copy.deepcopy((scope,domain,contacts,old))
    roles=compose_reviewed_source_roles(scope,domain,contacts,[old])
    assert roles['schema']=='mapforge.resolved-source-roles/v2'
    assert roles['role_binding_complete'] and len(roles['resolved'])==1
    assert roles['decision']['authorization']==dict(basis='replay-of-identical-reviewed-source-facts',new_authorizations=0)
    assert roles['decision']['decisions']==old['decision']['decisions']
    assert roles['new_source_role_authorizations']==0
    assert not roles['geometry_solver_ran'] and not roles['export_allowed']
    assert not roles['movement_paths_validated']
    assert replay_source_role_packet(scope,domain,contacts,roles)==roles
    assert (scope,domain,contacts,old)==before
    # A replay record cannot masquerade as a new direct approval.
    with pytest.raises(ValueError):resolve_source_roles(scope,domain,contacts,roles['decision'])


@pytest.mark.parametrize('fault',['lane','side','speed','width','boundary','part','support',
                                 'projection','budget','conflict','road','inventory','old_authority'])
def test_unchanged_id_does_not_allow_changed_facts_to_reuse_approval(fault):
    _,scope,domain,contacts,old=inputs()
    if fault=='lane':scope['observations']['c']['raw_records'][0]['parts'][0][0][0]+=.001
    elif fault=='side':scope['observations']['c']['boundary_relations'][0]['declared_side']='right'
    elif fault=='speed':scope['observations']['c']['source_max_speed_kmh']=99
    elif fault=='width':scope['observations']['c']['start_width_mm']=1
    elif fault=='boundary':scope['boundaries']['B:cl']['records'][0]['attributes']['SN']='OTHER'
    elif fault=='part':domain['partition']['features']['lane:c']['parts'][0]['raw_vertices'].reverse()
    elif fault=='support':contacts['full_source_support']['lane:c']['parts'][0]['vertex_indices'].pop()
    elif fault=='projection':domain['partition']['projection']['lat_0']=1.
    elif fault=='budget':contacts['policy']['source_error_budget_m']=.75
    elif fault=='conflict':contacts['role_conflicts'][0]['gap_m']+=.01
    elif fault=='road':contacts['source_bindings']['c']=['road:other']
    elif fault=='inventory':contacts['source_endpoint_inventory'].pop()
    else:old['decision']['authorization']['basis']='auto-inferred'
    reseal_target(scope,domain,contacts)
    with pytest.raises(ValueError):compose_reviewed_source_roles(scope,domain,contacts,[old])


def test_overlapping_or_missing_review_is_not_silently_deduplicated():
    _,scope,domain,contacts,old=inputs()
    with pytest.raises(ValueError,match='overlapping'):compose_reviewed_source_roles(scope,domain,contacts,[old,old])
    with pytest.raises(ValueError,match='nonempty'):compose_reviewed_source_roles(scope,domain,contacts,[])


def test_new_conflict_remains_blocked_and_tampering_self_rehash_does_not_resolve_it():
    _,scope,domain,contacts,old=inputs()
    other=copy.deepcopy(contacts['role_conflicts'][0]);other['source_lane_id']='a'
    contacts['role_conflicts'].append(other);seal(contacts)
    roles=compose_reviewed_source_roles(scope,domain,contacts,[old])
    assert not roles['role_binding_complete']
    assert [r['source_lane_id'] for r in roles['unresolved']]==['a']
    assert roles['feature_roles']['lane:a']['physical_geometry_constraint']
    roles['unresolved']=[];roles['role_binding_complete']=True;seal(roles)
    with pytest.raises(ValueError,match='fresh verified'):replay_source_role_packet(scope,domain,contacts,roles)


def test_dispatcher_preserves_direct_v1_result_and_rejects_altered_roles():
    _,_,scope,domain,contacts,decision=fixture()
    roles=resolve_source_roles(scope,domain,contacts,decision)
    assert replay_source_role_packet(scope,domain,contacts,roles)==roles
    roles['resolved'][0]['source_lane_id']='other';seal(roles)
    with pytest.raises(ValueError,match='fresh verified'):replay_source_role_packet(scope,domain,contacts,roles)
