"""Bind legacy-cut source debt to proven physical/path continuations.

Source observation ownership is distinct from the presence of a written
laneSection. Ordinary continuations may cross a legacy cut; a birth may NOT
be unioned into an ordinary lane. Out-of-domain observations are retained as
finite endpoint constraints, never evaluated on an extrapolated road.
"""
from collections import Counter, deque

import numpy as np

from mapforge.ops.physical_continuations import compile_physical_continuations
from mapforge.ops.source_roles import replay_source_role_packet
from mapforge.ops.reconstruction_scope import digest as object_digest
from mapforge.repair_web.model import digest, parse
from mapforge.repair_web.split_event_admission import (
    checked_payload, collect_sources, partition_rows, REFERENCE_SHA, BIRTHS, END,
)
from scripts.check_outer_event_written import boundary_poly, max_abs


def original_slice(st, intrinsic, interval):
    """Retain exact source arclength, every interior vertex, and both cut points."""
    ss = np.asarray(intrinsic, float); a, b = interval
    if not (len(ss) == len(st) and 0 <= a < b <= ss[-1]+1e-8) or np.any(np.diff(ss) <= 0):
        raise ValueError('Invalid original interval or degenerate arclength')
    query = np.unique(np.r_[a, ss[(ss > a) & (ss < b)], b])
    return np.c_[np.interp(query, ss, st[:, 0]), np.interp(query, ss, st[:, 1])]


def proven_path(graph, source, target):
    """Exact identity graph, with no coordinate-nearest fallback."""
    if source == target: return []
    queue = deque([(source, [])]); visited = {source}
    while queue:
        key, trail = queue.popleft()
        for neighbor, witness in graph.get(key, []):
            if neighbor in visited: continue
            if neighbor == target: return trail+[witness]
            visited.add(neighbor); queue.append((neighbor, trail+[witness]))
    raise ValueError('No ordinary source-proven continuation to actual written identity')


def continuation_graphs(contacts, physical):
    boundaries, paths = {}, {}
    def pair(graph, a, b, witness):
        graph.setdefault(a, []).append((b, witness)); graph.setdefault(b, []).append((a, witness))
    endpoints = contacts['boundary_endpoints']
    for i, relation in enumerate(physical['relations']):
        if not relation['ordinary_lane_link_supported'] or not any(
                w.get('road') == '11' and w.get('kind') == 'ordinary_continuation' for w in relation['witnesses']):
            continue
        a, b = (endpoints[key]['feature'] for key in relation['endpoints'])
        pair(boundaries, a, b, dict(physical_relation=i, endpoints=relation['endpoints']))
    for i, event in enumerate(contacts['transition_events']):
        if event['road'] == '11' and event['kind'] == 'ordinary_continuation':
            if event['status'] != 'SOURCE_C0_CONTACT_PROVEN': raise ValueError('Unproven path transition')
            pair(paths, 'lane:'+event['from_source'], 'lane:'+event['to_source'], dict(event_index=i))
    return boundaries, paths


def contact_witnesses(row, part, contacts):
    a, b = row['source_arclength_m']
    index = 0 if abs(a) < 1e-8 else len(part['raw_vertices'])-1 if abs(b-part['length_m']) < 1e-8 else None
    if index is None: raise ValueError('Interior gap is not an endpoint-contact debt')
    key = row['feature']; matches = []
    for i, event in enumerate(contacts['transition_events']):
        if event['road'] != '11' or event['status'] != 'SOURCE_C0_CONTACT_PROVEN': continue
        if row['kind'] == 'physical_boundary':
            address = f"{key}/part{row['part']}/vertex{index}"
            if any(address in (c['from_endpoint'], c['to_endpoint']) for c in event['contacts']):
                matches.append(i)
        else:
            cp = 'start' if index == 0 else 'end'
            if any(event[role+'_source'] == key[5:] and event[role+'_source_contact'] == cp for role in ('from', 'to')):
                matches.append(i)
    return matches


def written_feature(sec, row, packet):
    """Identity of the actual boundary/path in this XML section, not geometry proximity."""
    edge = row.get('edge'); lid = row.get('lane', -max(1, edge or 0))
    lane = sec.find(f"right/lane[@id='{lid}']")
    if lane is None: return None
    metadata = lane.findall("userData[@code='mapforge.source_lane']")
    if len(metadata) != 1: raise ValueError('Actual lane has ambiguous source identity')
    sid = metadata[0].get('value')
    if row['kind'] == 'movement_path': return 'lane:'+sid
    declared = 'right' if edge == 0 else 'left'
    rels = [r for r in packet['observations'][sid]['boundary_relations'] if r['declared_side'] == declared]
    if len(rels) != 1: raise ValueError('Actual edge lacks explicit original side relation')
    return 'boundary:'+rels[0]['boundary_key']


def target_power(road, row, station):
    if row['kind'] == 'physical_boundary': return boundary_poly(road, row['edge'], station)
    edge = -row['lane']
    return .5*(boundary_poly(road, edge-1, station)+boundary_poly(road, edge, station))


def bind_interval(road, row, st, packet, graphs, contacts, provenance):
    if st[-1, 0] < st[0, 0]: st = st[::-1]
    if np.any(np.diff(st[:, 0]) <= 0): raise ValueError('Nonmonotone source needs explicit chart model')
    lo, hi = map(float, (st[0, 0], st[-1, 0])); sections = road.findall('lanes/laneSection')
    starts = [float(s.get('s')) for s in sections]
    result = dict(source_st=st.tolist(), part_identity_retained=True, source_interval_clipped=False,
                  observation_role='physical_boundary' if row['kind'] == 'physical_boundary' else 'independent_movement_path',
                  movement_center_equality_required=False, binding_kind=None, targets=[])
    if lo < -1e-8:
        if hi > 1e-7 or provenance['classification'] != 'PACKAGE_EXTERNAL_SOURCE_CONTACT':
            raise ValueError('Unexpected external source interval')
        result.update(binding_kind='EXTERNAL_ENDPOINT_OBSERVATION', target_station=0.,
                      external_contact_evidence=provenance['external_contact_evidence'])
        target_s = 0.
    else:
        if lo < 0 or hi > END+1e-7: raise ValueError('Cannot extrapolate the written parent')
        cuts = sorted({lo, hi} | {s for s in starts if lo < s < hi})
        missing = []
        for a, b in zip(cuts, cuts[1:]):
            si = max(i for i, s in enumerate(starts) if s <= (a+b)/2)
            feature = written_feature(sections[si], row, packet)
            if feature is None: missing.append((a, b)); continue
            graph = graphs[0 if row['kind'] == 'physical_boundary' else 1]
            evidence = proven_path(graph, row['feature'], feature)
            result['targets'].append(dict(road='11', section=si, domain=[a, b], actual_source_feature=feature,
                continuation_evidence=evidence, edge=row.get('edge'), lane=row.get('lane')))
        if not missing:
            result['binding_kind'] = 'WRITTEN_INTERVAL_VIA_PROVEN_CONTINUATION'
            # The full residual consumer below re-splits at actual coefficient cuts.
            result['residual'] = interval_residual(road, row, st)
            return result
        edge = row.get('edge', -row.get('lane', 0))
        if edge not in BIRTHS or any(t['domain'][1]-t['domain'][0] > 1e-7 for t in result['targets']):
            raise ValueError('Unwritten branch cannot be assigned to another lane')
        birth, sid = BIRTHS[edge]
        if lo < 0 or abs(hi-birth) > 1e-7 or lo >= birth:
            raise ValueError('Source interval not adjacent to the registered birth')
        ev = [i for i in provenance['event_indices'] if contacts['transition_events'][i]['kind'] == 'zero_width_birth'
              and contacts['transition_events'][i]['to_source'] == sid]
        if not ev: raise ValueError('Missing original birth contact evidence')
        # Do NOT alias an unborn branch to its mother edge/center. A finite
        # endpoint observation has different semantics from interval coverage.
        result.update(binding_kind='PREBIRTH_ENDPOINT_OBSERVATION', birth_source_lane=sid,
                      birth_event_indices=ev, target_station=birth, birth_station_changed=False)
        target_s = birth
    target_t = float(target_power(road, row, target_s)[0])
    errors = np.linalg.norm(st-np.array([target_s, target_t]), axis=1)
    result.update(target_st=[target_s, target_t], residual=dict(max_distance_m=float(max(errors)),
        method='complete original linear fragment to actual finite endpoint; convex maximum at original vertices',
        geometric_coverage_proven=False, acceptance='NOT_ADMITTED',
        meaning='physical endpoint residual' if row['kind'] == 'physical_boundary' else 'lane-center diagnostic only, not a movement path equality'))
    return result


def interval_residual(road, row, st):
    cuts = {float(o.get('s')) for o in road.findall('lanes/laneOffset')}
    for sec in road.findall('lanes/laneSection'):
        s = float(sec.get('s')); cuts.add(s)
        cuts.update(s+float(w.get('sOffset')) for w in sec.findall('.//width'))
    errors = []
    for p, q in zip(st[:-1], st[1:]):
        slope = (q[1]-p[1])/(q[0]-p[0]); ss = sorted({float(p[0]), float(q[0])} | {v for v in cuts if p[0] < v < q[0]})
        for a, b in zip(ss, ss[1:]):
            source = np.array([p[1]+slope*(a-p[0]), slope, 0., 0.])
            errors.append(max_abs(target_power(road, row, a)-source, b-a))
    return dict(max_lateral_m=float(max(errors)), method='all original segments vs actual XML cubic extrema',
                acceptance='NOT_ADMITTED', meaning='physical edge residual' if row['kind'] == 'physical_boundary' else
                'lane-center diagnostic only; full original movement path remains separate')


def bind_sources(reference, packet, domain, contacts, roles):
    for payload in (packet, domain, contacts): checked_payload(payload)
    if digest(reference) != REFERENCE_SHA: raise ValueError('Unreviewed reference')
    # Replays the nine exact prior source-role decisions; grants no new one.
    replay_source_role_packet(packet, domain, contacts, roles)
    physical = compile_physical_continuations(contacts, roles)
    root = parse(reference); road, features = collect_sources(root, packet)
    rows = partition_rows(features, domain['partition'])
    if any(r['status'] not in ('ASSIGNED', 'SHARED_CONNECTOR_SUPPORT', 'UNASSIGNED_REQUIRES_SCOPE_DECISION') for r in rows):
        raise ValueError('Conflicting/unknown source ownership cannot be reclassified as a cut debt')
    debts = [r for r in rows if r['status'] == 'UNASSIGNED_REQUIRES_SCOPE_DECISION']
    graphs = continuation_graphs(contacts, physical)
    receipts = []
    for row in debts:
        matches = [r for r in contacts['previous_unassigned_intervals'] if r['feature'] == row['feature']
            and r['part_index'] == row['part'] and np.allclose(r['source_s_m'], row['source_arclength_m'], atol=1e-9, rtol=0)]
        if len(matches) != 1: raise ValueError('Missing or duplicate exact source debt receipt')
        old = matches[0]; feature = features[row['feature']]
        if old['classification'] not in ('SOURCE_CONTACT_BEYOND_LEGACY_CUT', 'PACKAGE_EXTERNAL_SOURCE_CONTACT'):
            raise ValueError('Source extent has no supported contact classification')
        part = domain['partition']['features'][row['feature']]['parts'][row['part']]
        if old['logical_consumers'] != ['road:11'] or old['source_observation_retained'] is not True:
            raise ValueError('Source observation not uniquely assigned to this parent')
        provenance = dict(classification=old['classification'], event_indices=contact_witnesses(row, part, contacts),
                          external_contact_evidence=old['external_contact_evidence'])
        if old['classification'] == 'SOURCE_CONTACT_BEYOND_LEGACY_CUT' and not provenance['event_indices']:
            raise ValueError('Logical label without actual source endpoint witness')
        if old['classification'] == 'PACKAGE_EXTERNAL_SOURCE_CONTACT' and not old['external_contact_evidence']:
            raise ValueError('External head has no original package contact')
        st = original_slice(feature['parts_st'][row['part']], part['source_vertex_s_m'], row['source_arclength_m'])
        try:
            bound = bind_interval(road, row, st, packet, graphs, contacts, provenance)
        except ValueError as exc:
            raise ValueError(f"{row['feature']} {row['source_arclength_m']}: {exc}") from exc
        receipts.append(dict(original_debt=row, proof=provenance, binding=bound))
    counts = Counter(r['binding']['binding_kind'] for r in receipts)
    endpoint_observations = [r for r in receipts if r['binding']['binding_kind'] != 'WRITTEN_INTERVAL_VIA_PROVEN_CONTINUATION']
    # Use the ALREADY declared endpoint budget, never invent a distance that
    # makes these four observations pass. This admits constraint bookkeeping,
    # not longitudinal coverage, visual shape, or an actual movement trajectory.
    endpoint_budget = contacts['policy']['source_error_budget_m']
    endpoint_checks = []
    for receipt in endpoint_observations:
        row, binding = receipt['original_debt'], receipt['binding']
        if row['kind'] == 'physical_boundary':
            status = ('WITHIN_EXISTING_ENDPOINT_BUDGET' if binding['residual']['max_distance_m'] <= endpoint_budget+1e-8
                      else 'ENDPOINT_BUDGET_EXCEEDED')
            check = dict(status=status, tolerance_m=endpoint_budget, tolerance_origin='frozen source contact policy',
                         max_distance_m=binding['residual']['max_distance_m'])
        else:
            sid = row['feature'][5:]; obs = packet['observations'][sid]
            if obs.get('geometry_role') != 'lane_path_observation_not_assumed_boundary_midpoint':
                raise ValueError('Movement observation role not explicit')
            decisions = [r for r in roles['resolved'] if r['source_lane_id'] == sid and r['contact'] == 'start']
            if binding['binding_kind'] == 'PREBIRTH_ENDPOINT_OBSERVATION' and len(decisions) != 1:
                raise ValueError('Prebirth independent path needs its exact existing role decision')
            check = dict(status='INDEPENDENT_PATH_RETAINED_NOT_GEOMETRY_ACCEPTED',
                         original_path_sha256=object_digest(obs['lane_path']),
                         existing_role_decision_replayed=bool(decisions), physical_midpoint_test_applied=False)
        endpoint_checks.append(dict(feature=row['feature'], source_arclength_m=row['source_arclength_m'], **check))
    constraint_ready = not any(c['status'] == 'ENDPOINT_BUDGET_EXCEEDED' for c in endpoint_checks)
    report = dict(schema='mapforge/event-source-binding/v1', status='SOURCE_RELATIONS_BOUND_NOT_GEOMETRY_ACCEPTED',
        reference_sha256=digest(reference), packet_sha256=packet['content_sha256'], domain_sha256=domain['content_sha256'],
        contacts_sha256=contacts['content_sha256'], roles_sha256=roles['content_sha256'], physical_graph=physical,
        receipts=receipts, binding_counts=dict(counts), source_relation_debt_count=len(debts),
        original_source_rows=len(rows), endpoint_observation_count=len(endpoint_observations),
        endpoint_checks=endpoint_checks,
        old_structural_ledger_rewritten=False, source_role_authorizations_added=0,
        geometry_solver_ran=False, new_xodr=False, map_accepted=False, export_allowed=False,
        source_relation_status='BOUND', source_constraint_status='READY_FOR_SHAPE_DESIGN' if constraint_ready else 'BLOCKED_ENDPOINT_BUDGET',
        written_extent_status='INTERVAL_TARGETS_AND_FINITE_ENDPOINT_OBSERVATIONS_NOT_FULL_INTERVAL_COVERAGE',
        next_step='retain finite endpoint observations and independent movement roles in E2 full shape design; no solve or silent birth/mouth movement')
    report['content_sha256'] = object_digest(report)
    return report
