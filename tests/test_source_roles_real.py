import copy
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from mapforge.ops.source_roles import resolve_source_roles,rebind_unchanged_source_decision
from spikes.source_contact_fit import SourceBoundaryBlock
from tests.test_source_contacts_real import contacts
from tests.test_source_domain_real import original


@pytest.fixture(scope='module')
def block(contacts):
    root,source,scope,domain,model=contacts
    path=Path(__file__).resolve().parents[1]/'profiles/repair/node4-zero-width-source-roles-v1.yaml'
    policy=yaml.safe_load(path.read_text(encoding='utf8'))
    if policy['source_contact_sha256']!=model['content_sha256']:
        previous_path=path.parents[2]/'out/source-role-input-v151/contact-model.json'
        if not previous_path.exists():pytest.skip('historical approved source packet unavailable for exact migration')
        previous=json.loads(previous_path.read_text(encoding='utf8'))
        policy=rebind_unchanged_source_decision(scope,domain,previous,model,policy)
    roles=resolve_source_roles(scope,domain,model,policy)
    return SourceBoundaryBlock(root,scope,domain,model,roles),roles


def test_user_decision_only_changes_two_source_roles_and_never_original_geometry(block,contacts):
    model,roles=block
    *_,scope,domain,contacts=contacts
    assert roles['role_binding_complete'] and not roles['unresolved']
    changed={sid for sid,r in roles['feature_roles'].items() if r['movement_path_check_required']}
    assert changed=={'lane:2023081117223317273','lane:2023081117251327605'}
    assert len(roles['feature_roles'])==118
    assert sum(len(p['vertex_indices']) for r in roles['feature_roles'].values() for p in r['parts'])==3009
    assert not roles['movement_paths_validated'] and not roles['export_allowed']
    assert len(model.source_ids)==25 and len(model.owner)==33 and len(model.families)==11
    assert model.describe()['minimum_independent_span_m']>16
    assert model.nvar==79
    assert all(k in model.raw for f in model.families for k in f.features)


def test_real_fixed_basis_failure_is_not_only_a_conservative_hull_or_a_center_role_error(block):
    model,roles=block
    params,necessary=model.necessary_source_preflight()
    assert necessary['status']=='INFEASIBLE_CHOSEN_BASIS'
    assert necessary['minimum_additional_source_slack_m']==pytest.approx(.8252669004,abs=1e-6)
    assert not necessary['includes_width_center_or_dynamics_constraints']
    assert not necessary['all_point_acceptance']
    candidate,phase=model.solve()
    assert candidate is None and phase['status']=='INFEASIBLE'
    assert phase['minimum_uniform_constraint_slack_m']>=necessary['minimum_additional_source_slack_m']-1e-7
    assert len(model.legacy_uncovered_centers)==14
    assert not model.uncovered_centers  # full intervals or same-budget endpoint witnesses, no extrapolation
    assert sum(d['kind']=='euclidean_endpoint_cap' for d in model.center_support['endpoint_caps'])==12
    assert sum(d['kind']=='numerical_contact_cap' for d in model.center_support['endpoint_caps'])==1
    assert np.isfinite(params).all()


def test_real_quintic_shared_geometry_becomes_feasible_without_shorter_spans_or_a_map_pass(block,contacts):
    _,roles=block
    root,source,scope,domain,c=contacts
    model=SourceBoundaryBlock(root,scope,domain,c,roles,degree=5)
    assert model.nvar==171 and len(model.families)==11
    assert model.describe()['minimum_independent_span_m']>16
    assert [(f['continuing_edge_pairs'],f['taper_only_pairs']) for f in model.physical_graph['forks']]==[(1,1),(2,0)]
    x,phase=model.solve()
    assert x is not None and phase['status']=='FEASIBLE'
    audit=model.audit(x)
    assert audit['boundary_same_chart_max_m']<=.35+1e-8
    assert audit['exact_width_min_m']>=-1e-8 and audit['scaled_C2_residual']<1e-8
    assert not audit['uncovered_center_support']
    coverage=audit['full_source_center_support']
    assert coverage['physical_source_to_center_certified']
    assert coverage['maximum_conservative_error_m']<=.35+1e-7
    assert not coverage['reverse_fidelity_certified'] and not coverage['dynamics_certified']
    for sid in model.source_ids:
        if sid in model.movement_observations:continue
        spans=sorted((d['a'],d['b']) for d in model.center_support['pieces']+model.center_support['endpoint_caps']
                     if d['source_lane']==sid)
        raw=model.raw['lane:'+sid]
        assert spans[0][0]==pytest.approx(raw[0,0]) and spans[-1][-1]==pytest.approx(raw[-1,0])
        assert all(abs(a[1]-b[0])<1e-8 for a,b in zip(spans[:-1],spans[1:]))
    assert max(c['value'] for c in audit['physical_midpoint_dynamics_worst'])>2.5
    assert audit['status']=='BLOCKED' and not audit['independent_xodr_validation_ran']
