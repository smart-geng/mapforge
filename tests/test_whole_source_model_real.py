"""Read-only verification of the whole-node4 registration, not a map gate."""
import json
from pathlib import Path

import numpy as np
import pytest

from scripts.fit_source_boundary_block import unchanged
from scripts.research_code_revision import bind_current_code
from mapforge.ops.source_roles import replay_source_role_packet

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=ROOT/'out/node4-whole-source-model-r4-20260914'
PREPARATION=ROOT/'out/node4-global-model-preflight-20260914/final-input'


def read(path): return json.loads(path.read_text(encoding='utf8'))


@pytest.fixture(scope='module')
def real():
    if not (OUTPUT/'report.json').exists():pytest.skip('local full-node4 registration not installed')
    report=read(OUTPUT/'report.json')
    # Historical measurements stay immutable; later implementation edits do
    # not masquerade as a rerun of the old compiler. Only Python revisions may
    # migrate explicitly; raw inputs/config/decisions still match exactly.
    current,changes=bind_current_code(report['input_files_sha256'])
    unchanged(current)
    report['_current_input_binding']=current
    report['_code_revision_changes']=changes
    return report


def test_exact_nine_approved_roles_replay_without_editing_originals(real):
    scope=read(PREPARATION/'reconstruction-input.json');domain=read(PREPARATION/'source-domain.json')
    contacts=read(ROOT/'out/node4-whole-source-role-binding-20260914/whole-contact-model.json')
    roles=read(OUTPUT/'source-roles.json')
    assert replay_source_role_packet(scope,domain,contacts,roles)==roles
    assert roles['role_binding_complete'] and len(roles['resolved'])==9 and not roles['unresolved']
    assert real['explicit_decisions_added']==3
    assert len(scope['observations'])==120 and real['original_endpoints']==240
    assert len(roles['feature_roles'])==291
    assert sum(v['movement_path_check_required'] for v in roles['feature_roles'].values())==9
    assert not roles['movement_paths_validated'] and not roles['export_allowed']


def test_five_parents_all24_shapes_share_one_actual_state_but_initial_curve_is_rejected(real):
    assert {r['road'] for r in real['port_registration']}=={'10','11','12','13','30'}
    assert all(r['status']=='SOURCE_PORTS_REGISTERED_NOT_FEASIBLE' for r in real['port_registration'])
    shape=real['whole_geometry_state']
    assert shape['status']=='WHOLE_SOURCE_AND_SHAPE_EVALUATED_NOT_ACCEPTED'
    assert shape['source_variables']==1081 and shape['turn_variables']==600 and shape['variables']==1681
    assert {r['road'] for r in shape['turns']}==set(map(str,range(100,124)))
    assert all(r['reference_primitives']==5 and r['minimum_reference_span_m']>=6-1e-9 for r in shape['turns'])
    assert all(r['minimum_width_span_m']>=5.5-1e-9 for r in real['parents'])
    assert shape['equality_count']==507 and shape['inequality_count']==51948
    assert shape['maximum_scaled_equality']>1. and shape['minimum_inequality_slack']< -1.
    assert not shape['optimization_ran'] and not shape['export_allowed']
    assert not shape['final_domain_dynamics_and_surface_constraints_complete']
    assert not shape['separate_movement_paths_validated']
    saved=read(OUTPUT/'whole-coordinate-state.json')
    assert len(saved['initial'])==1681 and np.isfinite(saved['initial']).all()
    spans=sorted(saved['turn_slices'].values())
    assert spans[0][0]==1081 and spans[-1][1]==1681
    assert all(a[1]==b[0] for a,b in zip(spans[:-1],spans[1:]))
    assert not list(OUTPUT.glob('*.xodr')) and not real['production_accepted']


def test_real_west_parent_probe_updates_all_six_dependents_and_nothing_else(real):
    p=real['whole_source_state']['finite_incidence_probe']
    expected=['106','107','108','109','110','111']
    assert p['parent']=='11' and p['origin_y_step_m']==1e-5
    assert p['expected_movements']==p['changed_movements']==expected
    assert not p['numerical_optimization_ran']
    unchanged(real['_current_input_binding'])
