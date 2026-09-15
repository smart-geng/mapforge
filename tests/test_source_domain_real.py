"""Readonly node4 tests: expected graph never comes from the exported roads."""
import copy
import hashlib
from pathlib import Path

import pytest
from lxml import etree

from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops.port_dependencies import PortDependencies
from mapforge.ops.reconstruction_scope import prepare_scope
from mapforge.ops.source_domain import original_movements, compare_movements, prepare_source_domain

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'out/shp-shared-section-final-v145/node4.xodr'
SOURCE_JID = '2023041113170861627'


@pytest.fixture(scope='module')
def original():
    if not BASE.exists() or not (ROOT/'shp_0222-0326/IBD_LANE_LINK.shp').exists():
        pytest.skip('local readonly node4 assets unavailable')
    before = hashlib.sha256(BASE.read_bytes()).hexdigest()
    source = ProfileSource(str(ROOT/'shp_0222-0326'), 'ibd-smarteditor-v1')
    root = etree.parse(str(BASE)).getroot()
    yield root, source, original_movements(source, SOURCE_JID)
    assert hashlib.sha256(BASE.read_bytes()).hexdigest() == before


def test_node4_independent_inventory_counts_and_relocated_exit_support(original):
    root, source, inv = original
    assert not inv['issues'] and len(inv['paths']) == 24
    assert len(inv['ingress_lanes']) == 16 and len(inv['egress_lanes']) == 8
    compared = compare_movements(root, source, inv, '1')
    assert compared['status'] == 'MATCH' and compared['written_count'] == 24
    rows = {a['connector']: a for a in compared['actual']}
    assert rows['106']['source_path'][-2:] == ['2023081515293628015', '2023081117193451282']
    assert rows['106']['movement'][-1] == '2023081515293628015'
    assert all(len(p) == 3 for p in inv['paths'])


def test_node4_removing_104_and_junction_record_is_still_a_missing_original_turn(original):
    root, source, inv = original; candidate = copy.deepcopy(root)
    candidate.remove(candidate.find("road[@id='104']"))
    j = candidate.find("junction[@id='1']")
    j.remove(j.find("connection[@connectingRoad='104']"))
    assert PortDependencies(candidate).validate_junction_table()
    result = compare_movements(candidate, source, inv, '1')
    assert result['written_count'] == 23 and result['expected_count'] == 24
    assert result['missing'] == [('2023081515293376893', '2024010619462695252', '2023070414510748078')]


def test_node4_existing_cuts_do_not_cover_all_original_intervals(original):
    root, source, _ = original
    packet = prepare_scope(root, source, ['10'])
    domain = prepare_source_domain(root, source, packet, SOURCE_JID, '1')
    partition = domain['partition']
    assert partition['status'] == 'UNRESOLVED' and not partition['geometry_fit_validated']
    assert not partition['issues']
    assert partition['length_by_status_m']['UNASSIGNED_REQUIRES_SCOPE_DECISION'] == pytest.approx(15.7846729088, abs=1e-6)
    assert len(partition['features']) == 118
    assert sum(len(p['vertices']) for f in partition['features'].values() for p in f['parts']) == 3009
    largest = max(a['source_s_m'][1]-a['source_s_m'][0] for f in partition['features'].values()
                  for p in f['parts'] for a in p['atoms'] if a['status'] == 'UNASSIGNED_REQUIRES_SCOPE_DECISION')
    assert largest == pytest.approx(1.2655565143, abs=1e-6)
