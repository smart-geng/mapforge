"""Compile full original supports and source-proven C0 boundary contacts.

Source features are observations, NOT independent curve primitives or new
laneSections. No geometry is cropped, moved, fitted or written by this module.
Traffic links and physical boundary contacts are separate graphs.
"""
from collections import defaultdict
import math
import numpy as np

from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.port_dependencies import revision
from mapforge.ops.source_domain import linear_chart, exact_parent_chart, _arc_axis
from mapforge.validate.shp_boundary_fidelity import _project


def _node_id(attributes, name):
    value=attributes.get(name) if name else None
    return '' if value is None else str(value).strip()


def _bindings(scope, domain):
    owners = defaultdict(set)
    for u in scope['occurrences']:
        if u['scope_role'] != 'connector':
            owners[u['source_lane_id']].add('road:'+u['road'])
    for a in domain['comparison']['actual']:
        if a['connector'] not in scope['connectors']: continue
        path = a['source_path']; movement = a['movement']
        i, j = path.index(movement[0]), path.index(movement[-1])
        for sid in path[:i+1]: owners[sid].add('road:'+a['incoming_port']['road'])
        for sid in path[j:]: owners[sid].add('road:'+a['outgoing_port']['road'])
        for sid in path[i+1:j]: owners[sid].add('connector:'+a['connector'])
    return {s: sorted(v) for s, v in sorted(owners.items())}


def contact_groups(nodes, pairs, tolerance_m):
    """Union ONLY source-TOPO-supported endpoint pairs; bound cluster diameter.

    Coordinates never rename/merge original IDs. Chained proximity alone may
    exceed the tolerance, so every proposed group is independently bounded.
    """
    parent = {k: k for k in nodes}
    def find(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]; k = parent[k]
        return k
    for a, b in pairs:
        if a not in nodes or b not in nodes: raise ValueError('unknown physical endpoint')
        if not nodes[a].get('original_node_id') or nodes[a].get('original_node_id') != nodes[b].get('original_node_id'):
            raise ValueError('different original endpoint node identities')
        if np.linalg.norm(np.asarray(nodes[a]['xy'])-nodes[b]['xy']) > tolerance_m + 1e-10:
            raise ValueError('unsupported distant endpoint pair')
        aa, bb = find(a), find(b)
        if aa != bb: parent[max(aa, bb)] = min(aa, bb)
    buckets = defaultdict(list)
    for k in sorted(nodes): buckets[find(k)].append(k)
    result = []
    for members in sorted(buckets.values()):
        xy = np.array([nodes[k]['xy'] for k in members])
        diameter = float(np.max(np.linalg.norm(xy[:, None, :]-xy[None, :, :], axis=2)))
        if diameter > tolerance_m + 1e-10:
            raise ValueError('transitive endpoint group exceeds source contact tolerance')
        result.append({'members': members, 'diameter_m': diameter,
                       'kind': 'source_supported_C0_contact' if len(members) > 1 else 'unpaired_endpoint',
                       'derivatives_validated': False})
    return result


def compile_source_contacts(root, source, scope, domain, contact_tolerance_m=.02,
                            source_error_budget_m=.35, *, chart_mode='line-only-v1'):
    if chart_mode not in ('line-only-v1','exact-line-arc-v1'):
        raise ValueError('unsupported original contact chart mode')
    if not (math.isfinite(contact_tolerance_m) and 0 < contact_tolerance_m <= .02
            and math.isfinite(source_error_budget_m) and 0 < source_error_budget_m <= .35):
        raise ValueError('contact/source tolerances must not silently relax the research baseline')
    for packet in (scope, domain):
        if digest({k: v for k, v in packet.items() if k != 'content_sha256'}) != packet['content_sha256']:
            raise ValueError('prepared input content mismatch')
    if scope['base_revision'] != revision(root):
        raise ValueError('source contacts cannot use a stale baseline revision')
    if domain['scope_sha256'] != scope['content_sha256'] or domain['comparison']['status'] != 'MATCH':
        raise ValueError('matched and correctly bound source domain required')
    if scope.get('issues') or domain['partition'].get('issues'):
        raise ValueError('upstream source issues prohibit a complete contact model')
    projection = domain['partition']['projection']; lat, lon = projection['lat_0'], projection['lon_0']
    observations, features = scope['observations'], domain['partition']['features']
    expected = {'lane:'+s for s in observations} | {'boundary:'+b for b in scope['boundaries']}
    if set(features) != expected:
        raise ValueError('incomplete source feature inventory')
    # Public compiler must verify full part/vertex identity itself; correct
    # road-contact counts do not compensate for an omitted fixed-support lane.
    for key,f in features.items():
        if key.startswith('lane:'):
            sid=key[5:];o=observations[sid];layer=o.get('identity_resolution',{}).get('selected_layer')
            rows=[r for r in o['raw_records'] if layer is None or r['layer']==layer]
            owners=[sid];kind='lane_path'
        else:
            bid=key[len('boundary:'):];rows=scope['boundaries'][bid]['records']
            owners=sorted(s for s,o in observations.items() if any(r['boundary_key']==bid for r in o['boundary_relations']))
            kind='physical_boundary'
        if len(rows)!=1 or not rows[0]['parts'] or f['kind']!=kind or sorted(f['source_lane_ids'])!=owners:
            raise ValueError('source feature role/owner/identity mismatch')
        if len(rows[0]['parts'])!=len(f['parts']): raise ValueError('source part inventory mismatch')
        for raw,part in zip(rows[0]['parts'],f['parts']):
            if not np.array_equal(np.asarray(raw),np.asarray(part['raw_vertices'])):
                raise ValueError('source vertex inventory mismatch')
            q=_project(np.asarray(raw),lat,lon);stations=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
            if (not np.isfinite(stations).all() or len(stations)!=len(part['source_vertex_s_m'])
                    or not np.allclose(stations,part['source_vertex_s_m'],atol=1e-8,rtol=0)
                    or abs(stations[-1]-part['length_m'])>1e-8):
                raise ValueError('source arclength inventory mismatch')
    bindings = _bindings(scope, domain)
    issues, conflicts, nodes = [], [], {}
    world = {k: [_project(np.asarray(p['raw_vertices']), lat, lon) for p in f['parts']]
             for k, f in features.items()}
    charts = {r.get('id'): (linear_chart if chart_mode=='line-only-v1' else exact_parent_chart)(r) for r in root.findall('road')
              if r.get('id') in scope['mutable_roads']}
    tf = source.L['topo']['fields']; topo = defaultdict(list); raw_edges = []
    for row in source.layer_raw_records('topo'):
        a = str(row['attributes'][tf['from']]).strip(); b = str(row['attributes'][tf['to']]).strip()
        topo[a].append(b); raw_edges.append((a, b, row))
    road_sources = {rid: {s for s, os in bindings.items() if 'road:'+rid in os} for rid in charts}

    def lane_record(sid):
        obs = observations[sid]; raw = obs['raw_records']
        layer = obs.get('identity_resolution', {}).get('selected_layer')
        rows = [r for r in raw if layer is None or r['layer'] == layer]
        if len(rows) != 1: raise ValueError('source lane record identity ambiguous')
        specs = [spec for name, spec in source.L.items() if name in ('lane', 'lane_merge')
                 and spec['file'] == rows[0]['layer']]
        if len(specs) != 1: raise ValueError('source lane layer identity ambiguous')
        return rows[0], specs[0]['fields']

    external_records = {}
    def original_lane_record(sid):
        if sid in observations: return lane_record(sid)
        if sid not in external_records:
            raw = source.lane_raw_records(sid)
            primary = [r for r in raw if r['layer'] == source.L['lane']['file']]
            chosen = primary or list(raw)
            if len(chosen) != 1: raise ValueError('external source lane identity ambiguous')
            spec = next(s for name, s in source.L.items() if name in ('lane', 'lane_merge')
                        and s['file'] == chosen[0]['layer'])
            external_records[sid] = {'lane_record': chosen[0], 'boundary_records': source.lane_boundary_records(sid),
                                     'fields': spec['fields'], 'scope_role': 'read_only_external_contact_witness'}
        item = external_records[sid]
        return item['lane_record'], item['fields']

    def lane_node(sid, role):
        row, fields = lane_record(sid)
        name = fields.get(role+'_node')
        node = _node_id(row['attributes'],name)
        if not node: raise ValueError('explicit original lane endpoint node ID mapping required')
        return node

    # An omitted sibling within an original source Link cannot vanish merely
    # because the old exported road did not mention its LANE_PID.
    for rid, sids in road_sources.items():
        links = {source.lane(s).link_pid for s in sids}
        for link in sorted(links):
            for rec in source.lanes_of(link):
                if rec.lane_pid not in sids:
                    issues.append({'code': 'SOURCE_LINK_SIBLING_MISSING_FROM_SCOPE', 'road': rid,
                                   'source_lane_id': rec.lane_pid, 'source_link': link})

    def tips(sid, role):
        q = world.get('lane:'+sid, [])
        if len(q) != 1 or len(q[0]) < 2: raise ValueError('source lane path requires explicit part/direction model')
        direction = q[0][-1]-q[0][0]
        if np.linalg.norm(direction) < 1e-8: raise ValueError('source path direction unresolved')
        rows = observations[sid]['boundary_relations']; result = {}
        if len(rows) != 2 or {r['declared_side'] for r in rows} != {'left', 'right'}:
            raise ValueError('boundary ports require one original feature per declared side')
        for row in rows:
            key = 'boundary:'+row['boundary_key']; parts = world[key]
            if len(parts) != 1: raise ValueError('multipart boundary port needs explicit part declaration')
            p = parts[0]; dot = float((p[-1]-p[0]) @ direction)
            if abs(dot) < 1e-8: raise ValueError('boundary endpoint orientation ambiguous')
            index = 0 if (role == 'start') == (dot > 0) else len(p)-1
            node_key = key+'/part0/vertex'+str(index)
            fields = source.L['boundary']['fields']
            node_field = fields.get('start_node' if index == 0 else 'end_node')
            original = scope['boundaries'][row['boundary_key']]['records'][0]
            endpoint_id = _node_id(original['attributes'],node_field)
            if not endpoint_id: raise ValueError('explicit original boundary endpoint node ID mapping required')
            if node_key not in nodes:
                nodes[node_key] = {'feature': key, 'part_index': 0, 'vertex_index': index,
                                   'xy': p[index].tolist(), 'source_roles': [], 'original_node_id': endpoint_id}
            use = {'source_lane_id': sid, 'source_contact': role, 'declared_side': row['declared_side'],
                   'orientation_basis': 'original_boundary_chord_vs_original_lane_chord',
                   'boundary_reversed_from_lane': dot < 0}
            if use not in nodes[node_key]['source_roles']: nodes[node_key]['source_roles'].append(use)
            result[row['declared_side']] = node_key
        return result

    def known_zero(sid, role):
        prefix = 'start' if role == 'start' else 'end'; o = observations[sid]
        return bool(o[prefix+'_width_known']) and o[prefix+'_width_mm'] == 0

    def gap(a, b): return float(np.linalg.norm(np.asarray(nodes[a]['xy'])-nodes[b]['xy']))

    transitions, pairs = [], []
    for rid, sids in sorted(road_sources.items()):
        for a, b, raw in raw_edges:
            if a not in sids or b not in sids: continue
            row = {'road': rid, 'from_source': a, 'to_source': b, 'topology_record': raw}
            if topo[a].count(b) != 1:
                issues.append({'code': 'DUPLICATE_SOURCE_EVENT_EDGE', 'from_source': a, 'to_source': b}); continue
            try:
                # Exhaust raw S/S, S/E, E/S, E/E. Source geometry digitization
                # direction need not equal traffic direction; TOPO stays intact.
                endpoint_pairs = []
                for ar, ai in (('start', 0), ('end', -1)):
                    for br, bi in (('start', 0), ('end', -1)):
                        an, bn = lane_node(a, ar), lane_node(b, br)
                        qa, qb = world['lane:'+a][0][ai], world['lane:'+b][0][bi]
                        if an == bn and np.linalg.norm(qa-qb) <= contact_tolerance_m:
                            endpoint_pairs.append((ar,br,an,qa,qb))
                if len(endpoint_pairs) != 1:
                    raise ValueError('TOPO requires one node-ID and coordinate-supported endpoint pair')
                ar,br,an,qa,qb=endpoint_pairs[0]
                aa, bb = tips(a, ar), tips(b, br)
                za, zb = known_zero(a, ar), known_zero(b, br)
                if za and gap(aa['left'], aa['right']) > contact_tolerance_m:
                    raise ValueError('declared zero end width but original boundaries do not collapse')
                if zb and gap(bb['left'], bb['right']) > contact_tolerance_m:
                    raise ValueError('declared zero start width but original boundaries do not collapse')
                matches = [(sa, sb, ka, kb, gap(ka, kb)) for sa, ka in aa.items() for sb, kb in bb.items()
                           if (sa == sb or za or zb) and gap(ka, kb) <= contact_tolerance_m
                           and nodes[ka]['original_node_id'] == nodes[kb]['original_node_id']]
                if not za and not zb:
                    valid = {(m[0], m[1]) for m in matches} == {('left', 'left'), ('right', 'right')}
                    kind = 'ordinary_continuation'
                elif zb and not za:
                    valid = len({m[0] for m in matches}) == 1 and {m[1] for m in matches} == {'left', 'right'}
                    kind = 'zero_width_birth'
                elif za and not zb:
                    valid = {m[0] for m in matches} == {'left', 'right'} and len({m[1] for m in matches}) == 1
                    kind = 'zero_width_death'
                else:
                    valid = len(matches) == 4; kind = 'collapsed_continuation'
                if not valid: raise ValueError('original side/zero-width relations do not prove all required contacts')
                contacts = [{'from_endpoint': ka, 'to_endpoint': kb, 'from_side': sa, 'to_side': sb,
                             'gap_m': distance} for sa, sb, ka, kb, distance in matches]
                pairs.extend((m['from_endpoint'], m['to_endpoint']) for m in contacts)
                xyz = np.array([nodes[k]['xy'] for m in contacts for k in (m['from_endpoint'], m['to_endpoint'])])
                frame = charts[rid]
                s = ((_arc_axis(frame).project(xyz)[:,0]) if chart_mode=='exact-line-arc-v1'
                     else (xyz-frame['origin']) @ frame['tangent'])
                legacy = [float(root.find("road[@id='"+rid+"']").findall('lanes/laneSection')[e['section']+1].get('s'))
                          for e in scope['source_links'] if e['kind'] == 'ordinary_section'
                          and e['road'] == rid and e['from_source'] == a and e['to_source'] == b]
                row.update(kind=kind, status='SOURCE_C0_CONTACT_PROVEN', contacts=contacts,
                           source_lane_contact_node_id=an,
                           from_source_contact=ar,to_source_contact=br,
                           source_station_band_m=[float(s.min()), float(s.max())],
                           legacy_section_stations_m=legacy, source_path_tip_gap_m=float(np.linalg.norm(qa-qb)),
                           derivative_constraints='NOT_SOLVED', station_band_is_not_a_new_laneSection=True)
            except (ValueError, KeyError) as exc:
                row.update(status='UNRESOLVED', reason=str(exc))
                issues.append({'code': 'SOURCE_BOUNDARY_CONTACT_UNRESOLVED', 'road': rid,
                               'from_source': a, 'to_source': b, 'reason': str(exc)})
            transitions.append(row)
    matched = {k for pair in pairs for k in pair}
    contacted_lane_ports = {(e['from_source'], e['from_source_contact']) for e in transitions if e['status']=='SOURCE_C0_CONTACT_PROVEN'}
    contacted_lane_ports |= {(e['to_source'], e['to_source_contact']) for e in transitions if e['status']=='SOURCE_C0_CONTACT_PROVEN'}

    # Role qualification is an inventory check, not a by-product of TOPO
    # pairing. Check BOTH ends of EVERY retained original lane, including
    # package-external ends and fixed/connector supports. Do not invent a
    # transition, union endpoints, or extend geometry for an unpaired end.
    endpoint_inventory = []
    for sid in sorted(observations):
        obs = observations[sid]
        for role, index in (('start', 0), ('end', -1)):
            item = {'source_lane_id': sid, 'contact': role,
                    'logical_consumers': bindings.get(sid, []),
                    'has_compiled_transition': (sid, role) in contacted_lane_ports,
                    'width_known': bool(obs[role+'_width_known']),
                    'width_mm': obs[role+'_width_mm'],
                    'source_point_retained': True}
            item['status'] = 'NONZERO_WIDTH' if item['width_known'] else 'WIDTH_UNKNOWN_NOT_ZERO'
            if known_zero(sid, role):
                try:
                    edge_tips = tips(sid, role)
                    left, right = (nodes[edge_tips[s]] for s in ('left', 'right'))
                    if (left['original_node_id'] != right['original_node_id']
                            or gap(edge_tips['left'], edge_tips['right']) > contact_tolerance_m):
                        raise ValueError('zero-width endpoint requires both original boundary node identity and coordinate contact')
                    point = world['lane:'+sid][0][index]
                    center = np.mean([left['xy'], right['xy']], axis=0)
                    distance = float(np.linalg.norm(point-center))
                    item.update(boundary_endpoints=edge_tips, original_boundary_node_id=left['original_node_id'],
                                boundary_tip_gap_m=gap(edge_tips['left'], edge_tips['right']),
                                path_to_boundary_gap_m=distance, status='ZERO_WIDTH_POINT_ROLES_WITHIN_BUDGET')
                    if distance > source_error_budget_m:
                        item['status'] = 'SOURCE_ROLE_DECISION_REQUIRED'
                        conflicts.append({'code': 'LANE_PATH_VS_COLLAPSED_BOUNDARY_ROLE_CONFLICT',
                            'source_lane_id': sid, 'contact': role, 'source_path_xy': point.tolist(),
                            'collapsed_boundary_xy': center.tolist(), 'gap_m': distance,
                            'path_role': 'UNRESOLVED_NOT_ASSUMED_PHYSICAL_CENTER',
                            'source_point_retained': True, 'automatic_override_allowed': False,
                            'conditional_infeasibility': {
                                'assumption': 'lane_path_tip_is_physical_center_at_same_zero_width_boundary_tip',
                                'necessary_common_tube_radius_m': distance/2.,
                                'configured_source_tube_radius_m': source_error_budget_m,
                                'distance_excess_over_two_tubes_m': distance-2*source_error_budget_m,
                                'meaning': 'triangle_inequality_only_under_this_point_role_binding_not_global_no_solution'}})
                except (ValueError, KeyError) as exc:
                    item.update(status='UNRESOLVED_ZERO_WIDTH_ENDPOINT', reason=str(exc))
                    issues.append({'code': 'SOURCE_ZERO_WIDTH_ENDPOINT_UNRESOLVED',
                                   'source_lane_id': sid, 'contact': role, 'reason': str(exc)})
            endpoint_inventory.append(item)

    def external_witnesses(key, feature, part_index, vertex_index):
        """Readonly neighbors outside the preparation, not new geometry owners."""
        witnesses = []
        target_xy = world[key][part_index][vertex_index]
        for sid in feature['source_lane_ids']:
            row, fields = lane_record(sid)
            if feature['kind'] == 'lane_path':
                roles = ['start' if vertex_index == 0 else 'end']
            else:
                roles = []
                for role in ('start', 'end'):
                    try: pp = tips(sid, role)
                    except ValueError: continue
                    target = key+'/part'+str(part_index)+'/vertex'+str(vertex_index)
                    if target in pp.values(): roles.append(role)
            for role in roles:
                lane_contact = _node_id(row['attributes'],fields.get(role+'_node'))
                if not lane_contact: continue
                for a, b, edge_row in raw_edges:
                    if sid not in (a,b): continue
                    neighbor = a if b==sid else b
                    if neighbor in observations: continue
                    try:
                        nr, nf = original_lane_record(neighbor)
                        if len(nr['parts']) != 1: continue
                        # Test raw S and E using node IDs; reversed geometry is
                        # evidence to record, never permission to rename sources.
                        for nr_role, ni in (('start', 0), ('end', -1)):
                            if _node_id(nr['attributes'],nf.get(nr_role+'_node')) != lane_contact: continue
                            if feature['kind'] == 'lane_path':
                                matches = [('lane:'+neighbor, ni,
                                    _project(np.asarray(nr['parts'][0]), lat, lon)[ni], lane_contact)]
                                target_id = lane_contact
                            else:
                                target_node = nodes[key+'/part'+str(part_index)+'/vertex'+str(vertex_index)]
                                target_id = target_node['original_node_id']; matches = []
                                bf = source.L['boundary']['fields']
                                for relation in external_records[neighbor]['boundary_records']:
                                    if len(relation['records']) != 1: continue
                                    rr = relation['records'][0]
                                    if len(rr['parts']) != 1: continue
                                    for b_role, bi in (('start', 0), ('end', -1)):
                                        nid = _node_id(rr['attributes'],bf.get(b_role+'_node'))
                                        if nid == target_id:
                                            matches.append(('boundary:'+rr['layer']+':'+relation['boundary_id'], bi,
                                                _project(np.asarray(rr['parts'][0]), lat, lon)[bi], nid))
                            for endpoint_key, index, xy, nid in matches:
                                distance = float(np.linalg.norm(target_xy-xy))
                                if distance <= contact_tolerance_m:
                                    witnesses.append({'source_lane_id': sid, 'source_contact': role,
                                        'external_lane_id': neighbor, 'external_lane_contact': nr_role,
                                        'external_feature': endpoint_key, 'external_vertex_index': index,
                                        'original_node_id': nid, 'xy_gap_m': distance, 'topology_record': edge_row,
                                        'scope_expansion_authorized': False, 'written_geometry_coverage_proven': False})
                    except (ValueError, KeyError, StopIteration):
                        continue  # Unproven stays a policy issue below, never successful coverage.
        return witnesses

    supports, old_gaps = {}, []
    for key, feature in sorted(features.items()):
        consumers = sorted({o for sid in feature['source_lane_ids'] for o in bindings.get(sid, [])})
        if not consumers: issues.append({'code': 'SOURCE_FEATURE_HAS_NO_LOGICAL_SUPPORT', 'feature': key})
        support = {'source_lane_ids': feature['source_lane_ids'], 'logical_consumers': consumers,
                   'role': 'original_observation_support_not_written_geometry_ownership', 'parts': []}
        for pi, part in enumerate(feature['parts']):
            support['parts'].append({'part_index': pi, 'source_s_m': [0., part['length_m']],
                                    'vertex_indices': list(range(len(part['raw_vertices'])))})
            for atom in part['atoms']:
                if atom['status'] != 'UNASSIGNED_REQUIRES_SCOPE_DECISION': continue
                lo, hi = atom['source_s_m']; end = None
                if abs(lo) < 1e-8: end = 0
                elif abs(hi-part['length_m']) < 1e-8: end = len(part['raw_vertices'])-1
                if feature['kind'] == 'lane_path':
                    contact = 'start' if end == 0 else 'end'
                    proven = end is not None and any((sid, contact) in contacted_lane_ports for sid in feature['source_lane_ids'])
                else:
                    proven = end is not None and key+'/part'+str(pi)+'/vertex'+str(end) in matched
                external = external_witnesses(key, feature, pi, end) if end is not None and not proven else []
                old_gaps.append({'feature': key, 'part_index': pi, 'source_s_m': [lo, hi],
                    'length_m': hi-lo, 'logical_consumers': consumers,
                    'classification': 'SOURCE_CONTACT_BEYOND_LEGACY_CUT' if proven else
                                      'PACKAGE_EXTERNAL_SOURCE_CONTACT' if external else
                                      'SOURCE_DOMAIN_ENDPOINT_REQUIRES_EXTENT_POLICY' if end is not None else
                                      'INTERNAL_SOURCE_INTERVAL_UNRESOLVED',
                    'external_contact_evidence': external,
                    'old_written_geometry_fixed': False, 'source_observation_retained': True})
        supports[key] = support
    totals = defaultdict(float)
    for g in old_gaps: totals[g['classification']] += g['length_m']
    try:
        groups = contact_groups(nodes, pairs, contact_tolerance_m)
    except ValueError as exc:
        groups = []; issues.append({'code': 'CONTACT_GROUP_CONFLICT', 'reason': str(exc)})
    result = {'schema': 'mapforge.source-contact-model/v1', 'status': 'BLOCKED',
              'scope_sha256': scope['content_sha256'], 'source_domain_sha256': domain['content_sha256'],
              'policy': {'contact_tolerance_m': contact_tolerance_m, 'source_error_budget_m': source_error_budget_m,
                         'source_roles_automatically_changed': False, 'source_feature_is_curve_primitive': False},
              'source_bindings': bindings, 'full_source_support': supports,
              'transition_events': transitions, 'boundary_endpoints': nodes, 'contact_groups': groups,
              'source_endpoint_inventory': endpoint_inventory,
              'source_endpoint_inventory_policy': 'all-retained-original-lanes-both-ends/v1',
              'external_read_only_sources': external_records,
              'previous_unassigned_intervals': old_gaps, 'previous_unassigned_by_class_m': dict(totals),
              'issues': issues, 'role_conflicts': conflicts,
              'source_support_compiled': not issues, 'geometry_solver_ran': False, 'export_allowed': False,
              'joint_solver_admission': 'REQUIRES_SOURCE_ROLE_AND_DOMAIN_ENDPOINT_POLICY'}
    if chart_mode!='line-only-v1':
        result['schema']='mapforge.source-contact-model/v2'
        result['chart_mode']=chart_mode
        result['exact_parent_charts']=charts
    result['content_sha256'] = digest(result)
    return result
