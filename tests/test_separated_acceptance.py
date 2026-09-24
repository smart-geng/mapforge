"""Separation is not approval of cases, relaxed geometry, or a map release."""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from mapforge.validate.separated_acceptance import (
    POLICY, STATIC_GROUPS, admit_cases, audit_file, driving_domains, load_policy,
    separate_static, sha256,
)


def scene():
    return ET.fromstring('''<OpenDRIVE><road id="11" length="60" junction="-1">
    <lanes><laneSection s="0"><right>
      <lane id="-1" type="driving"><width sOffset="0" a="3.5"/></lane>
      <lane id="-2" type="driving"><width sOffset="0" a="0" b="0.1"/></lane>
    </right></laneSection><laneSection s="30"><right>
      <lane id="-1" type="driving"><width sOffset="0" a="3.5"/></lane>
      <lane id="-2" type="shoulder"><width sOffset="0" a="0"/></lane>
    </right></laneSection></lanes></road></OpenDRIVE>''')


def evidence(tmp_path, name, payload):
    data = json.dumps(payload).encode()
    (tmp_path/name).write_bytes(data)
    return dict(path=name, sha256=sha256(data), basis='synthetic test input, not real operating evidence')


def casebook(tmp_path, approval='approved'):
    domains = driving_domains(scene())
    case = dict(id='synthetic-only', covered_domains=[d['id'] for d in domains],
                path=evidence(tmp_path, 'path.json', {'path': [[0, 0], [60, 0]]}),
                speed_profile=evidence(tmp_path, 'speed.json', {'speed_kmh': 20}),
                approval=dict(state=approval, evidence=evidence(tmp_path, 'approval.json', {'synthetic': True})))
    doc = dict(schema='mapforge/operating-cases/v1', xodr_sha256='a'*64,
               source_manifest_sha256='b'*64, cases=[case])
    path = tmp_path/'cases.json'
    path.write_text(json.dumps(doc), encoding='utf8')
    return path, doc, domains


def save(path, doc):
    path.write_text(json.dumps(doc), encoding='utf8')


def test_policy_pins_old_thresholds_and_is_separate_file():
    doc, _, legacy = load_policy()
    assert legacy['_sha256'] == doc['legacy_policy']['sha256']
    assert legacy['dynamics']['fail']['lateral_acceleration_mps2'] == 2.5
    assert legacy['dynamics']['excluded_provenance_codes'] == []
    assert doc['delivery']['static_only_release'] is False


@pytest.mark.parametrize('change', ['speed', 'threshold', 'legacy', 'missing', 'static', 'release', 'coverage'])
def test_no_policy_shortcuts(tmp_path, change):
    doc = yaml.safe_load(POLICY.read_text(encoding='utf8'))
    if change == 'speed': doc['operating_dynamics']['infer_speed_from_geometry'] = True
    if change == 'threshold': doc['operating_dynamics']['lateral_acceleration_mps2'] = 20
    if change == 'legacy': doc['legacy_policy']['sha256'] = '0'*64
    if change == 'missing': doc['operating_dynamics']['missing_conditions'] = 'PASS'
    if change == 'static': doc['static']['require_full_source_coverage'] = False
    if change == 'release': doc['delivery']['static_only_release'] = True
    if change == 'coverage': doc['operating_dynamics']['require_complete_coverage'] = False
    p = tmp_path/'policy.yaml'
    p.write_text(yaml.safe_dump(doc), encoding='utf8')
    with pytest.raises(ValueError): load_policy(p)


def test_missing_conditions_block_even_if_empty_road():
    result = admit_cases(None, 'a'*64, 'b'*64, [])
    assert result['status'] == 'BLOCKED'
    assert result['reason'] == 'OPERATING_CASES_MISSING'
    assert not result['evaluated'] and not result['map_accepted']


def test_zero_width_domain_and_whole_section_not_removed():
    domains = driving_domains(scene())
    assert len(domains) == 3
    assert domains[1]['id'] == '11/0/-2'
    assert domains[1]['interval_m'] == [0., 30.]
    root = scene()
    root.find('road/lanes/laneSection/right/lane').set('id', '-2')
    with pytest.raises(ValueError, match='duplicate'): driving_domains(root)


def test_registered_approved_case_is_not_dynamics_pass(tmp_path):
    path, _, domains = casebook(tmp_path)
    result = admit_cases(path, 'a'*64, 'b'*64, domains)
    assert result['input_status'] == 'READY_FOR_EVALUATION'
    assert result['status'] == 'BLOCKED'
    assert not result['map_accepted'] and not result['evaluated']
    assert result['reason'] == 'OPERATING_CASE_EVALUATOR_NOT_IMPLEMENTED'


def test_separation_approval_is_not_operating_case_approval(tmp_path):
    path, _, domains = casebook(tmp_path, 'proposed')
    result = admit_cases(path, 'a'*64, 'b'*64, domains)
    assert result['input_status'] == 'UNAPPROVED'
    assert result['unapproved_cases'] == ['synthetic-only']


def test_partial_claim_cannot_hide_newborn_or_uncovered_domain(tmp_path):
    path, doc, domains = casebook(tmp_path)
    doc['cases'][0]['covered_domains'] = [domains[0]['id']]
    save(path, doc)
    result = admit_cases(path, 'a'*64, 'b'*64, domains)
    assert result['input_status'] == 'INCOMPLETE'
    assert result['missing_domains'] == ['11/0/-2', '11/30/-1']


@pytest.mark.parametrize('change', ['xodr', 'source', 'empty', 'speed', 'basis', 'hash', 'file',
                                    'duplicate', 'unknown', 'escape', 'approval', 'nan'])
def test_stale_incomplete_or_tampered_cases_rejected(tmp_path, change):
    path, doc, domains = casebook(tmp_path)
    case = doc['cases'][0]
    if change == 'xodr': doc['xodr_sha256'] = 'c'*64
    if change == 'source': doc['source_manifest_sha256'] = 'c'*64
    if change == 'empty': doc['cases'] = []
    if change == 'speed': del case['speed_profile']
    if change == 'basis': case['speed_profile']['basis'] = ''
    if change == 'hash': case['path']['sha256'] = '0'*64
    if change == 'file': (tmp_path/'speed.json').write_text('changed')
    if change == 'duplicate': doc['cases'] *= 2
    if change == 'unknown': case['covered_domains'] = ['not-real']
    if change == 'escape': case['path']['path'] = '../other.json'
    if change == 'approval': del case['approval']
    if change == 'nan': case['unexpected_speed'] = float('nan')
    save(path, doc)
    with pytest.raises(ValueError): admit_cases(path, 'a'*64, 'b'*64, domains)


def legacy_result():
    groups = {name: dict(status='PASS', issues=[]) for name in STATIC_GROUPS}
    groups['G11-D'] = dict(status='FAIL', issues=[dict(severity='FAIL', code='lateral_jerk_exceeded')])
    return dict(status='FAIL', groups=groups)


def test_speed_failure_remains_old_fail_and_static_incomplete_not_pass():
    old = legacy_result(); before = copy.deepcopy(old)
    report = separate_static(old, dict(status='PASS', failures=[]))
    assert report['implemented_checks'] == 'PASS'
    assert report['status'] == 'BLOCKED' and report['missing_checks']
    assert old == before and old['status'] == 'FAIL'


def test_intrinsic_curvature_failure_is_not_discarded_with_speed_checks():
    old = legacy_result()
    old['groups']['G11-D']['issues'].append(dict(severity='FAIL', code='driving_lane_curvature_jump'))
    report = separate_static(old, dict(status='PASS', failures=[]))
    assert report['status'] == 'FAIL'
    assert report['groups']['lane_center_curvature']['issues'][0]['code'] == 'driving_lane_curvature_jump'


def test_exact_edge_failure_stays_static_failure():
    report = separate_static(legacy_result(), dict(status='FAIL', failures=[{'road':'12'}]))
    assert report['status'] == 'FAIL'


def test_real_written_candidate_keeps_old_failure_and_blocks_missing_case():
    root = Path(__file__).resolve().parents[1]
    folder = root/'out/node4-outer-event-l01-20260915-r3'
    xodr = folder/'candidate.xodr'
    packet = root/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json'
    stress = folder/'dynamics.json'
    before = {p:sha256(p.read_bytes()) for p in (xodr, packet, stress)}
    report, legacy, domains = audit_file(xodr, packet, historical_stress=stress)
    assert report['status'] == 'BLOCKED_RESEARCH_CANDIDATE'
    assert report['static_map']['status'] == 'FAIL'
    assert report['operating_dynamics']['reason'] == 'OPERATING_CASES_MISSING'
    assert report['legacy_limit_stress']['historical']['counts']['FAIL'] == 37
    assert report['legacy_limit_stress']['historical']['status'] == 'FAIL'
    assert legacy['status'] == 'FAIL' and len(domains) > 100
    assert not report['map_accepted'] and not report['export_allowed']
    assert before == {p:sha256(p.read_bytes()) for p in before}
