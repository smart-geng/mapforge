"""User-approved static / operating-case separation, without changing old G11.

This first adapter runs inherited static checks and admits immutable case inputs.
It does NOT evaluate arbitrary driving paths or release maps. Registration of a
case, an old limit-stress PASS, and a local edge PASS cannot substitute for that.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

from mapforge.validate.g11 import audit_file as audit_g11, load_policy as load_g11
from scripts.internal_edge_jets import audit as audit_edges


ROOT = Path(__file__).resolve().parents[2]
POLICY = ROOT / 'profiles/validation/separated-acceptance-v1.yaml'
STATIC_GROUPS = ('G11-A', 'G11-B', 'G11-C', 'G11-E')
STATIC_D_CODES = {'non_finite_lane_curvature', 'driving_lane_curvature_jump'}
SHA_RE = re.compile(r'[0-9a-f]{64}')


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_json(path: Path) -> dict:
    def bad_constant(value):
        raise ValueError('nonfinite JSON value: ' + value)
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate JSON key: ' + key)
            result[key] = value
        return result
    result = json.loads(path.read_text(encoding='utf8'), parse_constant=bad_constant,
                        object_pairs_hook=unique)
    if not isinstance(result, dict):
        raise ValueError('JSON object required')
    return result


def load_policy(path=POLICY):
    path = Path(path)
    data = path.read_bytes()
    doc = yaml.safe_load(data)
    if (doc.get('schema') != 'mapforge/separated-acceptance-policy/v1'
            or doc.get('version') != 1):
        raise ValueError('unsupported separated acceptance policy')
    legacy = (ROOT / doc['legacy_policy']['path']).resolve()
    if not legacy.is_relative_to(ROOT) or sha256(legacy.read_bytes()) != doc['legacy_policy']['sha256']:
        raise ValueError('legacy policy drift; do not silently migrate thresholds')
    if (tuple(doc['static']['inherited_groups']) != STATIC_GROUPS
            or set(doc['static']['retained_speed_independent_d_codes']) != STATIC_D_CODES):
        raise ValueError('static scope changed without a versioned implementation')
    if any(value is not True for key, value in doc['static'].items() if key.startswith('require_')):
        raise ValueError('cannot waive required static checks')
    if (any(value is not False for value in doc['delivery'].values())
            or doc['legacy_stress']['preserve_original_files'] is not True
            or doc['legacy_stress']['preserve_failures'] is not True):
        raise ValueError('cannot release incomplete evidence or erase old failures')
    # This adapter has deliberately no configurable shortcut for unknown cases.
    dynamics = doc['operating_dynamics']
    if any(dynamics.get(key) is not True for key in ('require_explicit_path', 'require_explicit_speed_profile',
            'bind_final_xodr_sha256', 'bind_source_manifest_sha256', 'require_complete_coverage',
            'variable_speed_requires_longitudinal_term')):
        raise ValueError('explicit bound operating evidence required')
    for key in ('missing_conditions', 'unapproved_conditions', 'unevaluated_conditions'):
        if dynamics[key] != 'BLOCKED':
            raise ValueError('conditions must fail closed')
    if any(dynamics[key] is not False for key in
           ('infer_speed_from_geometry', 'infer_operating_speed_from_limit', 'modify_written_speed_limits')):
        raise ValueError('cannot infer operating speed or change source limits')
    if (dynamics['lateral_acceleration_mps2'], dynamics['lateral_demand_rate_mps3']) != (2.5, 1.0):
        raise ValueError('unchanged research dynamics limits required')
    return doc, sha256(data), load_g11(legacy)


def driving_domains(root):
    """Coverage inventory from FINAL XML, not from a submitted list of easy cases.

The inventory does not force a vehicle to follow each geometric midpoint. A
declared path can cover several domains, but its spatial/topological coverage
still needs independent evaluation. Zero-width and born lanes are not omitted.
"""
    domains = []
    ids = set()
    for road in root.findall('road'):
        sections = road.findall('lanes/laneSection')
        ends = [float(s.get('s')) for s in sections[1:]] + [float(road.get('length'))]
        for section, end in zip(sections, ends):
            start = float(section.get('s'))
            if not all(map(math.isfinite, (start, end))) or start < 0 or end <= start:
                raise ValueError('invalid section interval')
            for lane in section.findall('left/lane') + section.findall('right/lane'):
                if lane.get('type') != 'driving':
                    continue
                rid, lid = road.get('id'), lane.get('id')
                if not rid or not lid or int(lid) == 0:
                    raise ValueError('invalid driving lane identity')
                key = f'{rid}/{start:.17g}/{lid}'
                if key in ids:
                    raise ValueError('duplicate driving domain')
                ids.add(key)
                domains.append(dict(id=key, road_id=rid, section_s_m=start,
                                    lane_id=lid, interval_m=[start, end]))
    return domains


def _evidence(ref, folder):
    if not isinstance(ref, dict) or not isinstance(ref.get('path'), str):
        raise ValueError('explicit relative evidence path required')
    relative = Path(ref['path'])
    if relative.is_absolute():
        raise ValueError('evidence path must be relative to the casebook')
    path = (folder / relative).resolve()
    if not path.is_relative_to(folder) or not path.is_file():
        raise ValueError('missing or out-of-bundle evidence')
    digest = ref.get('sha256')
    if not isinstance(digest, str) or not SHA_RE.fullmatch(digest) or sha256(path.read_bytes()) != digest:
        raise ValueError('case evidence hash mismatch')
    return path


def admit_cases(casebook, xodr_sha, source_sha, domains):
    """Check immutable input bindings only. Never infer a speed or approve a case.

Even READY_FOR_EVALUATION is BLOCKED for delivery. Path containment, topology,
actual coverage, speed limits, variable-speed dynamics and case approval
evidence must be independently checked by the future operating-case evaluator.
"""
    expected = {d['id'] for d in domains}
    base = dict(status='BLOCKED', evaluated=False, map_accepted=False,
                required_domains=len(expected), claimed_domains=0, cases=0)
    if casebook is None:
        return dict(base, input_status='MISSING', reason='OPERATING_CASES_MISSING')
    casebook = Path(casebook).resolve()
    case_sha = sha256(casebook.read_bytes())
    doc = read_json(casebook)
    if (doc.get('schema') != 'mapforge/operating-cases/v1'
            or doc.get('xodr_sha256') != xodr_sha
            or doc.get('source_manifest_sha256') != source_sha):
        raise ValueError('stale or unsupported operating casebook')
    cases = doc.get('cases')
    if not isinstance(cases, list) or not cases:
        raise ValueError('nonempty explicit operating cases required')
    seen, claimed, unapproved = set(), set(), []
    evidence_bindings = {}
    def bind(ref):
        path = _evidence(ref, casebook.parent)
        evidence_bindings[path] = ref['sha256']
    for case in cases:
        if not isinstance(case, dict):
            raise ValueError('case object required')
        ident = case.get('id')
        if not isinstance(ident, str) or not ident.strip() or ident in seen:
            raise ValueError('unique nonempty case id required')
        seen.add(ident)
        coverage = case.get('covered_domains')
        if (not isinstance(coverage, list) or not coverage
                or not all(isinstance(d, str) for d in coverage)
                or len(set(coverage)) != len(coverage) or not set(coverage) <= expected):
            raise ValueError('invalid or duplicate declared domain coverage')
        claimed.update(coverage)
        for field in ('path', 'speed_profile'):
            ref = case.get(field)
            if not isinstance(ref, dict) or not isinstance(ref.get('basis'), str) or not ref['basis'].strip():
                raise ValueError('independent path and speed basis required')
            bind(ref)
        approval = case.get('approval', {})
        if not isinstance(approval, dict) or approval.get('state') not in ('proposed', 'approved'):
            raise ValueError('explicit case approval state required')
        if approval['state'] == 'approved':
            bind(approval.get('evidence'))
        else:
            unapproved.append(ident)
    evidence_bindings[casebook] = case_sha
    if any(sha256(path.read_bytes()) != digest for path, digest in evidence_bindings.items()):
        raise ValueError('case input changed during admission')
    base.update(claimed_domains=len(claimed), cases=len(seen), casebook_sha256=case_sha,
                missing_domains=sorted(expected-claimed), unapproved_cases=unapproved)
    if not expected or expected != claimed:
        return dict(base, input_status='INCOMPLETE', reason='OPERATING_COVERAGE_INCOMPLETE')
    if unapproved:
        return dict(base, input_status='UNAPPROVED', reason='OPERATING_CASES_UNAPPROVED')
    return dict(base, input_status='READY_FOR_EVALUATION', reason='OPERATING_CASE_EVALUATOR_NOT_IMPLEMENTED',
                notice='Registration is not path/speed validity, approval verification, or dynamic PASS')


def separate_static(legacy, edges):
    """Preserve all non-speed checks, including intrinsic checks formerly in D."""
    groups = {key: copy.deepcopy(legacy['groups'][key]) for key in STATIC_GROUPS}
    intrinsic = [copy.deepcopy(issue) for issue in legacy['groups']['G11-D']['issues']
                 if issue['code'] in STATIC_D_CODES]
    groups['lane_center_curvature'] = dict(status='FAIL' if any(i['severity']=='FAIL' for i in intrinsic) else 'PASS',
                                           issues=intrinsic)
    groups['world_internal_edges'] = copy.deepcopy(edges)
    passed = all(group['status'] == 'PASS' for group in groups.values())
    return dict(status='FAIL' if not passed else 'BLOCKED',
                implemented_checks='PASS' if passed else 'FAIL', groups=groups,
                scope='inherited G11 static groups and linked internal edges; not whole static-map acceptance',
                missing_checks=['complete_source_ownership_and_fidelity', 'shape_overshoot_and_variation',
                                'birth_death_and_all_junction_interfaces', 'surface_holes_and_full_topology',
                                'absolute_crs', 'whole_map_consumer_and_visual_review'])


def audit_file(xodr, source_manifest, *, casebook=None, historical_stress=None, policy=POLICY):
    """Read-only check: only report a BLOCKED candidate until all gates exist."""
    xodr, source_manifest = Path(xodr), Path(source_manifest)
    data = xodr.read_bytes()
    xsha, source_sha = sha256(data), sha256(source_manifest.read_bytes())
    config, policy_sha, g11_policy = load_policy(policy)
    root = ET.fromstring(data)
    domains = driving_domains(root)
    legacy = audit_g11(xodr, g11_policy)
    legacy['xodr_sha256'] = xsha
    edges = audit_edges(root)
    static = separate_static(legacy, edges)
    dynamic = admit_cases(casebook, xsha, source_sha, domains)
    history = None
    if historical_stress is not None:
        path = Path(historical_stress)
        report = read_json(path)
        if report.get('xodr_sha256') != xsha:
            raise ValueError('historical stress belongs to a different final XODR')
        if report.get('status') not in ('PASS', 'FAIL', 'UNKNOWN', 'BOUNDED'):
            raise ValueError('unsupported historical stress result')
        history = dict(path=str(path.resolve()), sha256=sha256(path.read_bytes()),
                       status=report['status'], counts=report.get('counts'), scope=report.get('scope'))
    if sha256(xodr.read_bytes()) != xsha or sha256(source_manifest.read_bytes()) != source_sha:
        raise ValueError('artifact or source manifest changed during evaluation')
    result = dict(schema='mapforge/separated-acceptance-result/v1', status='BLOCKED_RESEARCH_CANDIDATE',
                  xodr_sha256=xsha, source_manifest_sha256=source_sha,
                  policy=dict(id=config['id'], version=config['version'], sha256=policy_sha),
                  source_manifest_binding='hash binding only; full source integrity/coverage still required',
                  static_map=static, operating_dynamics=dynamic,
                  legacy_limit_stress=dict(status=legacy['groups']['G11-D']['status'],
                                           legacy_g11_status=legacy['status'], role=config['legacy_stress']['role'],
                                           historical=history),
                  written_speed_limits_modified=False, map_accepted=False, export_allowed=False)
    return result, legacy, domains
