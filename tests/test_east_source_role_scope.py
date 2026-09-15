"""The three explicitly approved east cases do not generalize any other role."""
import copy
import json
from pathlib import Path
import pytest
import yaml
from mapforge.ops.source_roles import resolve_source_roles,rebind_unchanged_source_decision
from mapforge.ops.reconstruction_scope import digest


def test_east_decision_is_exactly_three_source_revision_bound_exceptions():
    root=Path(__file__).resolve().parents[1]
    p=root/'out/node4-east-source-v166'
    if not p.is_dir():pytest.skip('readonly node4 east preparation is not available')
    def read(name):return json.loads((p/name).read_text(encoding='utf8'))
    scope=read('input/reconstruction-input.json');domain=read('input/source-domain.json');contacts=read('contact-model.json')
    decision=yaml.safe_load((root/'profiles/repair/node4-east-zero-width-source-roles-v1.yaml').read_text(encoding='utf8'))
    before=copy.deepcopy((scope,domain,contacts,decision))
    roles=resolve_source_roles(scope,domain,contacts,decision)
    assert roles['role_binding_complete'] and not roles['unresolved']
    expected={'2023041111113051690','2023041111104112823','2023041810460339677'}
    assert {r['source_lane_id'] for r in roles['resolved']}==expected
    assert len(roles['resolved'])==3
    assert {key[5:] for key,r in roles['feature_roles'].items() if r['role']=='movement_path_observation'}==expected
    assert all(r['physical_geometry_constraint'] for key,r in roles['feature_roles'].items() if key[5:] not in expected)
    assert (scope,domain,contacts,decision)==before
    assert not roles['movement_paths_validated'] and not roles['export_allowed']
    corrupt=copy.deepcopy(decision);corrupt['decisions'][0]['source_lane_id']='2023081117223317273'
    with pytest.raises(ValueError):resolve_source_roles(scope,domain,contacts,corrupt)


def east_revisions():
    root=Path(__file__).resolve().parents[1]
    old=root/'out/node4-east-source-v166';new=root/'out/node4-east-source-v167'
    if not (old.is_dir() and new.is_dir()):pytest.skip('readonly east preparations unavailable')
    def read(path):return json.loads(path.read_text(encoding='utf8'))
    scope=read(new/'input/reconstruction-input.json');domain=read(new/'input/source-domain.json')
    previous=read(old/'contact-model.json');current=read(new/'contact-model.json')
    policy=yaml.safe_load((root/'profiles/repair/node4-east-zero-width-source-roles-v1.yaml').read_text(encoding='utf8'))
    return root,scope,domain,previous,current,policy


def test_inventory_migration_keeps_three_approvals_but_blocks_the_fourth_before_fit():
    root,scope,domain,old,new,policy=east_revisions()
    before=copy.deepcopy((scope,domain,old,new,policy))
    migrated=rebind_unchanged_source_decision(scope,domain,old,new,policy)
    roles=resolve_source_roles(scope,domain,new,migrated)
    assert migrated['decisions']==policy['decisions'] and migrated['authorization']==policy['authorization']
    assert migrated['binding_migration']['new_source_role_authorizations']==0
    assert not roles['role_binding_complete'] and len(roles['resolved'])==3
    assert [(c['source_lane_id'],c['contact']) for c in roles['unresolved']]==[('2023061915261945676','start')]
    assert roles['feature_roles']['lane:2023061915261945676']['physical_geometry_constraint']
    assert (scope,domain,old,new,policy)==before


@pytest.mark.parametrize('fault',['original','conflict','coverage','new-approval'])
def test_inventory_migration_cannot_silently_broaden_or_change_source(fault):
    _,scope,domain,old,new,policy=east_revisions()
    if fault=='original':new['full_source_support']['lane:2023061915261945676']['parts'][0]['vertex_indices'].pop()
    elif fault=='conflict':new['role_conflicts'][0]['gap_m']+=.01
    elif fault=='coverage':new['source_endpoint_inventory'].pop()
    else:policy['decisions'][0]['source_lane_id']='2023061915261945676'
    new['content_sha256']=digest({k:v for k,v in new.items() if k!='content_sha256'})
    with pytest.raises(ValueError):rebind_unchanged_source_decision(scope,domain,old,new,policy)


def test_fourth_user_approval_is_exactly_one_more_and_is_still_not_map_acceptance():
    root,scope,domain,old,new,policy=east_revisions()
    fresh=yaml.safe_load((root/'profiles/repair/node4-east-zero-width-source-roles-v2.yaml').read_text(encoding='utf8'))
    roles=resolve_source_roles(scope,domain,new,fresh)
    old_ids={(e['source_lane_id'],e['contact']) for e in policy['decisions']}
    new_ids={(e['source_lane_id'],e['contact']) for e in fresh['decisions']}
    assert new_ids-old_ids=={('2023061915261945676','start')} and old_ids<=new_ids
    assert roles['role_binding_complete'] and len(roles['resolved'])==4
    assert not roles['movement_paths_validated'] and not roles['export_allowed']
    assert sum(r['movement_path_check_required'] for r in roles['feature_roles'].values())==4
    assert len(new['source_endpoint_inventory'])==114
