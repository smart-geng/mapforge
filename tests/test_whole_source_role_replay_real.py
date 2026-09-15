"""Local original-data replay. These checks never fit or accept a map."""
import copy
import json
from pathlib import Path

import pytest
from lxml import etree as ET

from mapforge.ops.source_roles import replay_source_role_packet
from scripts.fit_source_boundary_block import unchanged
from scripts.prepare_whole_source_roles import independent_pending
from spikes.source_contact_fit import SourceBoundaryBlock


ROOT = Path(__file__).resolve().parents[1]
INPUT = ROOT / 'out/node4-global-model-preflight-20260914/final-input'
OUTPUT = ROOT / 'out/node4-whole-source-role-binding-20260914'
APPROVED = {
    ('2023081117223317273', 'start'), ('2023081117251327605', 'start'),
    ('2023041111113051690', 'start'), ('2023041111104112823', 'start'),
    ('2023041810460339677', 'end'), ('2023061915261945676', 'start'),
}
PENDING = {
    '2023041111104060474': (['road:11'], 1.5662110920907693),
    '2023041111104128071': (['road:11'], 1.7671845769635228),
    '2023041810470164021': (['road:12'], 1.8287687004077389),
}


def read(path):
    return json.loads(path.read_text(encoding='utf8'))


@pytest.fixture(scope='module')
def packet():
    if not (OUTPUT / 'report.json').exists():
        pytest.skip('local whole-source original review is not installed')
    report = read(OUTPUT / 'report.json')
    unchanged(report['input_files_sha256'])
    config = read(INPUT / 'run.json')
    return (ET.parse(config['input']).getroot(), read(INPUT / 'reconstruction-input.json'),
            read(INPUT / 'source-domain.json'), read(OUTPUT / 'whole-contact-model.json'),
            read(OUTPUT / 'source-roles.json'), config, report)


def test_full_original_inventory_replays_exactly_six_not_all_nine_conflicts(packet):
    _, scope, domain, contacts, roles, _, report = packet
    before = copy.deepcopy((scope, domain, contacts, roles))
    assert replay_source_role_packet(scope, domain, contacts, roles) == roles
    assert len(scope['observations']) == 120
    assert len(contacts['source_endpoint_inventory']) == 240
    assert len(roles['feature_roles']) == 291
    assert len(contacts['role_conflicts']) == 9
    assert {(r['source_lane_id'], r['contact']) for r in roles['resolved']} == APPROVED
    assert {r['source_lane_id'] for r in roles['unresolved']} == set(PENDING)
    assert report['scope']['status'] == 'SCOPE_COMPLETE_NOT_MODEL_FEASIBILITY'
    assert report['scope']['expected']['ordinary_roads'] == ['10', '11', '12', '13', '30']
    assert report['scope']['expected']['connectors'] == list(map(str, range(100, 124)))
    assert report['status'] == 'SOURCE_ROLE_DECISION_REQUIRED'
    assert not roles['role_binding_complete'] and not roles['movement_paths_validated']
    assert not report['new_source_role_authorizations']
    assert not report['geometry_solver_ran'] and not report['xodr_generated']
    assert not report['production_accepted'] and not report['source_modified']
    for sid in PENDING:
        assert roles['feature_roles']['lane:' + sid]['physical_geometry_constraint']
    assert (scope, domain, contacts, roles) == before


def test_three_pending_tips_reproduce_from_raw_shp_and_block_before_geometry(packet):
    root, scope, domain, contacts, roles, config, report = packet
    rows = independent_pending(scope, domain, contacts, roles, config['source_dir'])
    assert rows == report['pending']
    for row in rows:
        owners, gap = PENDING[row['source_lane_id']]
        assert row['logical_consumers'] == owners
        assert row['contact'] == 'start' and row['raw_zero_width_field'] == 'S_WIDTH'
        assert float(row['raw_zero_width_value']) == 0
        assert row['boundary_tip_separation_m'] < 1e-7
        assert row['gap_m'] == pytest.approx(gap, abs=1e-10)
        assert row['conditional_minimum_common_radius_m'] > row['source_budget_m']
        assert not row['general_geometry_impossibility_proven']
        assert not row['role_override_applied'] and row['proposed_decision'] is None
    with pytest.raises(ValueError, match='unresolved or altered bindings'):
        SourceBoundaryBlock(root, scope, domain, contacts, roles)
    unchanged(report['input_files_sha256'])
