"""Compile a dependency-closed, identity-preserving SHP reconstruction input.

Preparation only. Neither source admission nor a scoped constraint plan is a
geometry PASS. The accepted source and old XODR are never mutated here.
"""
from dataclasses import asdict
import hashlib
import json
import math

from mapforge.ops.port_dependencies import LanePort, PortDependencies, revision


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    allow_nan=False, separators=(',', ':')).encode()).hexdigest()


def _source_id(lane):
    rows = lane.findall("userData[@code='mapforge.source_lane']")
    if len(rows) != 1 or not rows[0].get('value'):
        raise ValueError('requires one explicit original lane identity')
    return rows[0].get('value')


def _source_ids(lane):
    """The port ID is compatibility metadata, not the full long-lane lineage."""
    rows = lane.findall("userData[@code='mapforge.source_chain/v1']")
    if not rows:
        return [_source_id(lane)]
    if len(rows) != 1:
        raise ValueError('ambiguous source chain records')
    try:
        ids = json.loads(rows[0].get('value', ''))
    except (ValueError, TypeError) as exc:
        raise ValueError('invalid source chain JSON') from exc
    if (not isinstance(ids, list) or not ids or any(not isinstance(s, str) or not s for s in ids)
            or len(set(ids)) != len(ids)):
        raise ValueError('source chain requires ordered unique original IDs')
    ports = lane.findall("userData[@code='mapforge.source_lane']")
    if len(ports) > 1 or ports and ports[0].get('value') not in ids:
        raise ValueError('port source identity is not in its complete source chain')
    return ids


def _chain_direction(root, road, ids, source):
    """Research inference from original point order in an exact parent chart.

    This never changes point order, role approvals, or traffic topology. It is
    not a default lane-sign rule. Nonmonotone/conflicting sources are rejected.
    New compiled lanes carry explicit direction and need no such inference.
    """
    import numpy as np
    from mapforge.ops.source_domain import exact_parent_chart, _arc_axis, source_projection
    from mapforge.validate.shp_boundary_fidelity import _project
    lat, lon, _ = source_projection(root)
    axis = _arc_axis(exact_parent_chart(road))
    directions = set(); evidence = []
    for sid in ids:
        rec = source.lane(sid)
        if rec is None:
            raise ValueError('missing original path for direction')
        st = axis.project(_project(np.asarray(rec.geometry), lat, lon))
        diffs = np.diff(st[:, 0])
        if len(diffs) and min(diffs) >= -1e-8 and st[-1, 0]-st[0, 0] > 1e-6:
            direction = 'with_s'
        elif len(diffs) and max(diffs) <= 1e-8 and st[0, 0]-st[-1, 0] > 1e-6:
            direction = 'against_s'
        else:
            raise ValueError('original ordered path is nonmonotone in parent chart')
        directions.add(direction)
        evidence.append(dict(source_lane_id=sid, first_s_m=float(st[0, 0]), last_s_m=float(st[-1, 0]),
                             complete_vertex_count=len(st)))
    if len(directions) != 1:
        raise ValueError('source chain point orders disagree')
    return next(iter(directions)), dict(status='INFERRED',
        basis='original_ordered_paths_vs_exact_parent_chart_not_lane_sign', rows=evidence)


def _lane_at(graph, port):
    sections = graph.roads[port.road].findall('lanes/laneSection')
    section = sections[0 if port.contact == 'start' else -1]
    matches = [l for l in section.findall('left/lane') + section.findall('right/lane')
               if int(l.get('id')) == port.lane]
    if len(matches) != 1:
        raise ValueError('ambiguous or missing port lane')
    return matches[0]


def _geometry_ok(parts):
    return bool(parts) and all(len(p) >= 2 and all(
        len(q) >= 2 and all(math.isfinite(float(v)) for v in q) for q in p) for p in parts)


def source_chain(topo, start, end, budget=10000):
    """Use an explicit edge, or a non-branching original chain to a cut mouth.

    This is not a bounded search pretending to prove global uniqueness. If
    there is a branch, cycle or traversal truncation, the corridor is unresolved.
    """
    if start == end:
        return [start]
    if end in topo.get(start, ()):
        return [start, end]
    path, seen = [start], {start}
    while len(path) <= budget:
        choices = sorted(set(topo.get(path[-1], ())))
        if len(choices) != 1:
            raise ValueError('source chain branches or ends before target')
        nxt = choices[0]
        if nxt in seen:
            raise ValueError('source chain cycle')
        path.append(nxt)
        if nxt == end:
            return path
        seen.add(nxt)
    raise ValueError('source chain traversal budget exceeded; not proof of uniqueness')


def whole_junction_scope(root, junction_id):
    """All explicit participants, including attached roads without a turn.

    This collects existing structure, NOT all missing source roads. The original
    movement inventory/domain comparison remains mandatory downstream. Do not
    silently cross into another junction when selecting a connected component.
    """
    jid = str(junction_id)
    graph = PortDependencies(root)
    graph.validate_junction_table()
    if sum(j.get('id') == jid for j in root.findall('junction')) != 1:
        raise ValueError('one explicitly bound junction required')
    turns = {rid for rid in graph.connections if graph.roads[rid].get('junction') == jid}
    if not turns:
        raise ValueError('junction has no explicit movements')
    parents = {p.road for rid in turns for p in graph.connections[rid]}
    parents.update(rid for rid, r in graph.roads.items() if r.get('junction', '-1') == '-1'
                   and any(l.get('elementType') == 'junction' and l.get('elementId') == jid
                           for l in r.findall('link/*')))
    crossing = {rid for rid, ports in graph.connections.items()
                if any(p.road in parents for p in ports)} - turns
    external_links = [(rid, l.get('elementId')) for rid in parents
                      for l in graph.roads[rid].findall('link/*')
                      if l.get('elementType') == 'road'
                      or l.get('elementType') == 'junction' and l.get('elementId') != jid]
    if crossing or external_links:
        raise ValueError('whole junction needs explicit outside-boundary scope; no silent expansion')
    return dict(xodr_junction_id=jid, ordinary_roads=sorted(parents), connectors=sorted(turns),
                source_inventory_still_required=True, geometry_validated=False)


def model_scope_report(root, junction_id, parent_roads, connectors):
    """A cheap, complete-scope prerequisite before any numerical initialization."""
    whole = whole_junction_scope(root, junction_id)
    parent_roads, connectors = list(parent_roads), list(connectors)
    missing_parents = sorted(set(whole['ordinary_roads'])-set(parent_roads))
    extra_parents = sorted(set(parent_roads)-set(whole['ordinary_roads']))
    missing_turns = sorted(set(whole['connectors'])-set(connectors))
    extra_turns = sorted(set(connectors)-set(whole['connectors']))
    duplicates = len(set(parent_roads)) != len(parent_roads) or len(set(connectors)) != len(connectors)
    complete = not (missing_parents or extra_parents or missing_turns or extra_turns or duplicates)
    return dict(status='SCOPE_COMPLETE_NOT_MODEL_FEASIBILITY' if complete else 'INCOMPLETE_WHOLE_JUNCTION_MODEL',
                expected=whole, missing_parent_models=missing_parents, extra_parent_models=extra_parents,
                missing_connector_models=missing_turns, extra_connector_models=extra_turns,
                duplicate_models=duplicates, geometry_solver_ran=False, export_allowed=False)


def prepare_scope(root, source, mutable_roads, junction_binding=None, identity_mode='single-id-v1'):
    """Whole ordinary roads, all incident connectors, explicit fixed opposite ends.

    Boundary identity comes solely from original feature references. Similar
    coordinates never merge identities. Source speed is recorded independently
    of old written speed; neither is rewritten or taken as acceptance authority.
    """
    if identity_mode not in ('single-id-v1', 'source-chain-v1'):
        raise ValueError('unsupported source identity mode')
    chain_mode = identity_mode == 'source-chain-v1'
    requested = tuple(sorted(set(str(r) for r in mutable_roads)))
    if not requested:
        raise ValueError('select at least one whole ordinary road')
    graph = PortDependencies(root)
    graph.validate_junction_table()
    if junction_binding is not None and (set(junction_binding) != {'source_junction_id', 'xodr_junction_id'}
            or not all(isinstance(v, str) and v for v in junction_binding.values())):
        raise ValueError('explicit source/XODR junction binding required')
    mutable = set()
    for rid in requested:
        road = graph.roads.get(rid)
        if road is None or road.get('junction', '-1') != '-1':
            raise ValueError('selection must resolve to ordinary roads')
        sections = road.findall('lanes/laneSection')
        if not sections:
            raise ValueError('ordinary road has no lane sections')
        for cp, sec in (('start', sections[0]), ('end', sections[-1])):
            for lane in sec.findall('left/lane') + sec.findall('right/lane'):
                mutable.add(LanePort(rid, cp, int(lane.get('id'))))
    closure = graph.closure(mutable)
    required_roads = set(requested) | set(closure['connectors'])
    issues, observations, boundaries, usage, derived = [], {}, {}, [], []

    def issue(code, **detail):
        issues.append({'code': code, **detail})

    for rid in requested:
        for link in graph.roads[rid].findall('link/*'):
            if link.get('elementType') == 'road':
                issue('ORDINARY_ROAD_CONTACT_NEEDS_SCOPE_MODEL', road=rid,
                      role=link.tag, other_road=link.get('elementId'))

    def observe(sid):
        if sid in observations:
            return
        rec = source.lane(sid)
        if rec is None:
            issue('SOURCE_LANE_MISSING', source_lane_id=sid)
            observations[sid] = {'source_lane_id': sid, 'missing': True}
            return
        raw = source.lane_raw_records(sid)
        rows = source.lane_boundary_records(sid)
        obs = {'source_lane_id': sid, 'raw_records': raw,
               'lane_path': rec.geometry.tolist(),
               'geometry_role': 'lane_path_observation_not_assumed_boundary_midpoint',
               'source_max_speed_kmh': rec.max_speed_kmh,
               'start_width_mm': rec.s_width_mm, 'end_width_mm': rec.e_width_mm,
               'start_width_known': rec.s_width_known, 'end_width_known': rec.e_width_known,
               'boundary_relations': []}
        observations[sid] = obs
        selected = raw
        policy = getattr(source, 'p', {}).get('lane_identity', 'reject-cross-layer-duplicates')
        if policy == 'primary-with-supplement-records':
            primary = [r for r in raw if r['layer'] == source.L['lane']['file']]
            secondary = [r for r in raw if r['layer'] == source.L.get('lane_merge', {}).get('file')]
            selected = primary or secondary
            if len(primary) > 1 or len(secondary) > 1:
                issue('SOURCE_LANE_DUPLICATE_WITHIN_LAYER', source_lane_id=sid)
            obs['identity_resolution'] = {
                'policy': policy, 'selected_layer': selected[0]['layer'] if len(selected) == 1 else None,
                'supplement_records_retained': len(secondary) if primary else 0,
                'supplement_geometry_used': not bool(primary)}
        if len(selected) != 1:
            issue('SOURCE_LANE_IDENTITY_AMBIGUOUS', source_lane_id=sid, record_count=len(raw))
        for rr in selected:
            if rr['geometry_origin'] == 'field' and not _geometry_ok(rr['parts']):
                issue('SOURCE_PATH_GEOMETRY_INVALID', source_lane_id=sid)
            if len(rr['parts']) > 1:
                issue('SOURCE_PATH_MULTIPART_NEEDS_MODEL', source_lane_id=sid)
        if not math.isfinite(rec.max_speed_kmh) or rec.max_speed_kmh <= 0:
            issue('SOURCE_SPEED_UNRESOLVED', source_lane_id=sid)
        if not rows:
            issue('SOURCE_BOUNDARY_RELATIONS_MISSING', source_lane_id=sid)
        if {r['declared_side'] for r in rows} != {'left', 'right'}:
            issue('SOURCE_BOUNDARY_SIDES_INCOMPLETE', source_lane_id=sid)
        refs = [(r['declared_side'], r['boundary_id']) for r in rows]
        if len(set(refs)) != len(refs):
            issue('SOURCE_BOUNDARY_RELATION_DUPLICATE', source_lane_id=sid)
        for row in rows:
            bid = row['boundary_id']
            key = str(source.L['boundary']['file']) + ':' + bid
            relation = {k: v for k, v in row.items() if k != 'records'}
            relation['boundary_key'] = key
            obs['boundary_relations'].append(relation)
            item = {'boundary_key': key, 'source_boundary_id': bid, 'records': row['records']}
            if key in boundaries and digest(boundaries[key]) != digest(item):
                issue('SHARED_BOUNDARY_PAYLOAD_CONFLICT', boundary_key=key)
            else:
                boundaries[key] = item
            if row['declared_side'] not in ('left', 'right'):
                issue('SOURCE_BOUNDARY_SIDE_UNKNOWN', source_lane_id=sid,
                      relation_index=row['relation_index'], raw_side=row['raw_side'])
            if len(row['records']) != 1 or not bid:
                issue('SOURCE_BOUNDARY_IDENTITY_AMBIGUOUS', boundary_key=key,
                      record_count=len(row['records']))
            for rr in row['records']:
                if not _geometry_ok(rr['parts']):
                    issue('SOURCE_BOUNDARY_GEOMETRY_INVALID', boundary_key=key)

    def use(lane, road_id, section_index, scope_role):
        try:
            ids = _source_ids(lane) if chain_mode else [_source_id(lane)]
        except ValueError as exc:
            if chain_mode and lane.findall("userData[@code='mapforge.source_chain/v1']"):
                issue('SOURCE_CHAIN_IDENTITY_INVALID', road=road_id, section=section_index,
                      lane=lane.get('id'), reason=str(exc))
            # A median can be an explicit derived gap, but it must remain in
            # the reconstruction state with BOTH supporting lane identities.
            sec = graph.roads[road_id].findall('lanes/laneSection')[section_index]
            lid = int(lane.get('id'))
            side = 'left' if lid > 0 else 'right'
            ordered = sorted(sec.findall(side+'/lane'), key=lambda l: abs(int(l.get('id'))))
            index = next(i for i, l in enumerate(ordered) if int(l.get('id')) == lid)
            neighbors = ordered[max(0, index-1):index] + ordered[index+1:index+2]
            if index == 0:
                opposite = sorted(sec.findall(('right' if side == 'left' else 'left')+'/lane'),
                                  key=lambda l: abs(int(l.get('id'))))
                neighbors += opposite[:1]
            support = []; groups = []
            for other in neighbors:
                try:
                    group = _source_ids(other) if chain_mode else [_source_id(other)]
                    groups.append(group); support.extend(group)
                except ValueError:
                    pass
            if (lane.get('type') != 'median' or len(groups) != 2
                    or chain_mode and set(groups[0]) & set(groups[1])):
                issue('DERIVED_LANE_SUPPORT_UNRESOLVED', road=road_id, section=section_index,
                      lane=lid, lane_type=lane.get('type'))
            for s in support:
                observe(s)
            derived.append({'road': road_id, 'section': section_index, 'lane': lid,
                            'lane_type': lane.get('type'), 'source_lane_ids': support,
                            'role': 'derived_gap_not_an_original_lane',
                            'geometry_validation': 'NOT_RUN'})
            return None
        for sid in ids:
            observe(sid)
        is_chain = chain_mode and bool(lane.findall("userData[@code='mapforge.source_chain/v1']"))
        if is_chain:
            for a, b in zip(ids[:-1], ids[1:]):
                if b not in source.topo_out.get(a, ()):
                    issue('SOURCE_CHAIN_TOPOLOGY_MISMATCH', road=road_id, section=section_index,
                          lane=lane.get('id'), from_source=a, to_source=b)
        meta = lane.findall("userData[@code='mapforge.provenance/v1']")
        direction = None; direction_evidence = None
        if len(meta) == 1:
            try:
                direction = json.loads(meta[0].get('value', '{}')).get('travel_direction')
            except (ValueError, TypeError):
                pass
        if is_chain and direction not in ('with_s', 'against_s'):
            try:
                direction, direction_evidence = _chain_direction(root, graph.roads[road_id], ids, source)
            except ValueError as exc:
                direction_evidence = dict(status='UNRESOLVED', reason=str(exc))
        if scope_role != 'connector' and direction not in ('with_s', 'against_s'):
            for sid in ids:
                issue('SOURCE_DIRECTION_UNRESOLVED', source_lane_id=sid, road=road_id)
        speeds = []
        for s in lane.findall('speed'):
            unit = s.get('unit', 'm/s')
            try:
                value = float(s.get('max')) * {'m/s': 3.6, 'km/h': 1.}[unit]
                if not math.isfinite(value) or value <= 0:
                    raise ValueError()
                speeds.append({'sOffset': s.get('sOffset'), 'kmh': value})
            except (KeyError, TypeError, ValueError):
                issue('WRITTEN_SPEED_INVALID', road=road_id, lane=lane.get('id'))
        if not speeds:
            issue('WRITTEN_SPEED_MISSING', road=road_id, lane=lane.get('id'))
        for sid in ids:
            source_speed = observations[sid].get('source_max_speed_kmh')
            item = {'road': road_id, 'section': section_index, 'lane': int(lane.get('id')),
                      'source_lane_id': sid, 'scope_role': scope_role,
                      'travel_direction': direction, 'written_speeds': speeds,
                      'written_source_speed_match': source_speed is not None and bool(speeds)
                      and all(abs(s['kmh'] - source_speed) <= 1e-6 for s in speeds)}
            if is_chain:
                item.update(source_identity_chain=ids, source_consumption='intersect_full_original_part_with_section',
                            written_source_speed_match=None,
                            written_source_speed_validation='REQUIRES_ORIGINAL_INTERVAL_READBACK')
                if direction_evidence is not None:
                    item['direction_evidence'] = direction_evidence
            usage.append(item)
        return ids[0] if len(ids) == 1 else None

    for rid in sorted(required_roads):
        role = 'ordinary' if rid in requested else 'connector'
        for si, sec in enumerate(graph.roads[rid].findall('lanes/laneSection')):
            for lane in sec.findall('left/lane') + sec.findall('right/lane'):
                use(lane, rid, si, role)
    for p in sorted(closure['fixed_ports']):
        si = 0 if p.contact == 'start' else len(graph.roads[p.road].findall('lanes/laneSection')) - 1
        use(_lane_at(graph, p), p.road, si, 'fixed_support')

    links = []
    certified_paths = None
    if junction_binding:
        # Complete original movements, not a nonbranching legacy walk.
        from mapforge.ops.source_domain import original_movements, compare_movements
        inventory = original_movements(source, junction_binding['source_junction_id'])
        compared = compare_movements(root, source, inventory, junction_binding['xodr_junction_id'])
        issues.extend(inventory['issues'] + compared['issues'])
        certified_paths = {a['connector']: a['source_path'] for a in compared['actual']}
    for cid in sorted(closure['connectors']):
        incoming, outgoing = graph.connections[cid]
        vid = _source_id(graph.roads[cid].find('lanes/laneSection/right/lane'))
        a, b = _source_id(_lane_at(graph, incoming)), _source_id(_lane_at(graph, outgoing))
        for half, (start, end) in enumerate(((a, vid), (vid, b))):
            try:
                if certified_paths is not None:
                    full = certified_paths.get(cid, [])
                    if full.count(vid) != 1:
                        raise ValueError('no unique source-enumerated movement in bound junction')
                    split = full.index(vid)
                    path = full[:split+1] if half == 0 else full[split:]
                    if path[0] != start or path[-1] != end:
                        raise ValueError('source-enumerated path disagrees with connector contacts')
                else:
                    path = source_chain(source.topo_out, start, end)
                valid, reason = True, None
                for sid in path:
                    observe(sid)
            except ValueError as exc:
                path, valid, reason = [], False, str(exc)
            links.append({'kind': 'connector_two_hop', 'connector': cid,
                          'from_source': start, 'to_source': end, 'source_topology_match': valid,
                          'source_path': path, 'reason': reason,
                          'coverage_partition': 'NOT_ALLOCATED'})
            if not valid:
                issue('SOURCE_CONNECTOR_TOPOLOGY_MISMATCH', connector=cid,
                      from_source=start, to_source=end)
    for rid in requested:
        sections = graph.roads[rid].findall('lanes/laneSection')
        for si, sec in enumerate(sections[:-1]):
            for side in ('left', 'right'):
                for lane in sec.findall(side + '/lane'):
                    successors = lane.findall('link/successor')
                    if len(successors) > 1:
                        issue('AMBIGUOUS_SECTION_SUCCESSOR', road=rid, section=si, lane=lane.get('id'))
                        continue
                    if not successors:
                        continue  # Missing/birth/death is not same-ID continuation.
                    other = sections[si+1].find(side + "/lane[@id='" + successors[0].get('id') + "']")
                    if other is None:
                        issue('SECTION_SUCCESSOR_MISSING', road=rid, section=si, lane=lane.get('id'))
                        continue
                    if chain_mode and (lane.findall("userData[@code='mapforge.source_chain/v1']")
                                       or other.findall("userData[@code='mapforge.source_chain/v1']")):
                        try:
                            a_ids, b_ids = _source_ids(lane), _source_ids(other)
                        except ValueError:
                            continue  # Invalid identity already recorded by use().
                        if a_ids == b_ids:
                            continue  # Same COMPLETE chain across a representation cut.
                        issue('SOURCE_SECTION_CHAIN_TRANSITION_UNRESOLVED', road=rid, section=si,
                              from_sources=a_ids, to_sources=b_ids)
                        continue
                    try:
                        a, b = _source_id(lane), _source_id(other)
                    except ValueError:
                        continue  # Recorded derived gaps above, not fake travel links.
                    if a == b:
                        continue  # A section cut is not a new source feature.
                    ua = next(u for u in usage if u['road'] == rid and u['section'] == si
                              and u['lane'] == int(lane.get('id')))
                    if ua['travel_direction'] == 'against_s':
                        a, b = b, a
                    valid = b in source.topo_out.get(a, ())
                    links.append({'kind': 'ordinary_section', 'road': rid, 'section': si,
                                  'from_source': a, 'to_source': b, 'source_topology_match': valid})
                    if not valid:
                        issue('SOURCE_SECTION_TOPOLOGY_MISMATCH', road=rid, section=si,
                              from_source=a, to_source=b)
    owners = {key: sorted(sid for sid, obs in observations.items()
                         if any(r['boundary_key'] == key for r in obs.get('boundary_relations', ())))
              for key in boundaries}
    result = {'schema': 'mapforge.reconstruction-input/v2' if junction_binding else 'mapforge.reconstruction-input/v1',
              'base_revision': revision(root),
              'status': 'BLOCKED', 'source_admission': 'REJECTED' if issues else 'ADMITTED_FOR_RESEARCH',
              'mutable_roads': list(requested), 'replacement_roads': sorted(required_roads),
              'mutable_ports': [asdict(p) for p in sorted(mutable)],
              'fixed_ports': [asdict(p) for p in sorted(closure['fixed_ports'])],
              'connectors': sorted(closure['connectors']),
              'observations': observations, 'boundaries': boundaries, 'boundary_owners': owners,
              'occurrences': usage, 'derived_lanes': derived, 'source_links': links, 'issues': issues,
              'not_validated': ['full_source_interval_partition', 'source_expected_missing_connectors',
                                'world_geometry', 'dynamics', 'paving', 'absolute_crs', 'xodr_readback'],
              'export_allowed': False}
    if junction_binding:
        result['source_domain_binding'] = dict(junction_binding)
    if chain_mode:
        result.update(schema='mapforge.reconstruction-input/v3', identity_mode=identity_mode)
    result['content_sha256'] = digest(result)
    return result


def require_solver_input(packet, root, source=None):
    """Refuse incomplete, edited or stale input before allocating solver variables."""
    unsigned = {k: v for k, v in packet.items() if k != 'content_sha256'}
    if digest(unsigned) != packet.get('content_sha256'):
        raise ValueError('reconstruction input digest mismatch')
    binding = packet.get('source_domain_binding')
    identity_mode = packet.get('identity_mode', 'single-id-v1')
    expected_schema = ('mapforge.reconstruction-input/v3' if identity_mode == 'source-chain-v1'
                       else 'mapforge.reconstruction-input/v2' if binding else 'mapforge.reconstruction-input/v1')
    if packet.get('schema') != expected_schema:
        raise ValueError('reconstruction version/binding mismatch')
    if packet['base_revision'] != revision(root):
        raise ValueError('stale reconstruction input')
    if packet['source_admission'] != 'ADMITTED_FOR_RESEARCH' or packet['issues']:
        raise ValueError('source admission rejected; cannot solve an ambiguous source model')
    if not packet['mutable_roads']:
        raise ValueError('empty reconstruction scope')
    graph = PortDependencies(root)
    graph.validate_junction_table()
    expected_ports = set()
    for rid in packet['mutable_roads']:
        road = graph.roads.get(rid)
        if road is None or road.get('junction', '-1') != '-1':
            raise ValueError('invalid whole-road reconstruction selection')
        secs = road.findall('lanes/laneSection')
        for cp, sec in (('start', secs[0]), ('end', secs[-1])):
            expected_ports.update(LanePort(rid, cp, int(l.get('id')))
                                  for l in sec.findall('left/lane') + sec.findall('right/lane'))
    if expected_ports != {LanePort(**p) for p in packet['mutable_ports']}:
        raise ValueError('incomplete whole-road port declaration')
    closure = graph.closure(LanePort(**p) for p in packet['mutable_ports'])
    if set(packet['connectors']) != set(closure['connectors']):
        raise ValueError('incomplete connector scope')
    if set(packet['replacement_roads']) != set(packet['mutable_roads']) | set(closure['connectors']):
        raise ValueError('incomplete replacement scope')
    if {LanePort(**p) for p in packet['fixed_ports']} != set(closure['fixed_ports']):
        raise ValueError('incomplete fixed port support')
    if source is None:
        raise ValueError('fresh source reader required; self-hashed admission is not authority')
    if prepare_scope(root, source, packet['mutable_roads'], binding, identity_mode)['content_sha256'] != packet['content_sha256']:
        raise ValueError('prepared model differs from fresh original inputs')
    return True  # Research admission only; no geometry or production approval.
