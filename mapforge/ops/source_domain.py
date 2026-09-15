"""Original movement inventory and exact Line/Arc-chart source accounting.

This module never fits, edits, exports, or approves geometry. Source arclength
is measured on original polyline parts, not the fitted road's station. Cuts
are explicit normal planes of the CURRENT structural mouths/sections. They
must be recomputed when shared mouths move in a future joint solver.
"""
from collections import Counter, defaultdict
import math

import numpy as np

from mapforge.ops.port_dependencies import PortDependencies
from mapforge.ops.reconstruction_scope import (_lane_at, _source_id, _source_ids, digest, source_chain,
                                               require_solver_input)
from mapforge.validate.shp_boundary_fidelity import _origin, _project


def original_movements(source, junction_id, budget=10000):
    """Traverse raw TOPO from all declared ingress lanes to declared exits.

    Only Profile-declared interior roads are traversable between the ports.
    Polygon containment is NOT used: measured via ends can extend outside the
    source intersection polygon. Any dead end, cycle or truncation is an issue.
    """
    issues = []
    js = source.L['junction']; jf = js['fields']
    junctions = [r for r in source.layer_raw_records('junction')
                 if str(r['attributes'].get(jf['id'], '')).strip() == str(junction_id)]
    if len(junctions) != 1:
        raise ValueError('source junction identity missing or duplicated')
    attrs = junctions[0]['attributes']
    sep = js.get('list_sep', ';')
    enter = [v.strip() for v in str(attrs[jf['enter_roads']]).split(sep) if v.strip()]
    leave = [v.strip() for v in str(attrs[jf['leave_roads']]).split(sep) if v.strip()]
    if not enter or not leave or set(enter) & set(leave):
        raise ValueError('ambiguous original ingress/egress declaration')
    if len(set(enter)) != len(enter) or len(set(leave)) != len(leave):
        raise ValueError('duplicate original road port')

    # Lane roster is derived independently from original mapped layer rows.
    raw_lanes = defaultdict(list)
    for name in ('lane', 'lane_merge'):
        if name not in source.L:
            continue
        fields = source.L[name]['fields']
        for row in source.layer_raw_records(name):
            a = row['attributes']; sid = str(a.get(fields['id'], '')).strip()
            raw_lanes[sid].append((name, str(a.get(fields['road'], '')).strip(), row))
    primary_policy = source.p.get('lane_identity') == 'primary-with-supplement-records'
    lane_road, identity_issues = {}, {}
    for sid, rows in raw_lanes.items():
        primary = [r for r in rows if r[0] == 'lane']
        chosen = (primary or rows) if primary_policy else rows
        if len(chosen) != 1 or not sid or max(Counter(r[0] for r in rows).values()) > 1:
            identity_issues[sid] = 'SOURCE_LANE_IDENTITY_AMBIGUOUS'
        elif chosen[0][1]:
            lane_road[sid] = chosen[0][1]
    ingress = sorted(sid for sid, road in lane_road.items() if road in enter)
    egress = sorted(sid for sid, road in lane_road.items() if road in leave)
    for road in enter + leave:
        if not any(r == road for r in lane_road.values()):
            issues.append({'code': 'SOURCE_PORT_HAS_NO_UNIQUE_LANES', 'road': road})
    for sid, rows in raw_lanes.items():
        if sid in identity_issues and any(r[1] in enter + leave for r in rows):
            issues.append({'code': identity_issues[sid], 'source_lane_id': sid})

    rs = source.L['road_center']; rf = rs['fields']
    values = {str(v) for v in rs.get('interior_values', ())}
    if not values or 'is_junction_interior' not in rf:
        raise ValueError('explicit Profile interior-road classification required')
    road_flags = defaultdict(set)
    road_rows = defaultdict(list)
    for row in source.layer_raw_records('road_center'):
        a = row['attributes']; rid = str(a.get(rf['road'], '')).strip()
        road_flags[rid].add(str(a.get(rf['is_junction_interior'], '')).strip())
        road_rows[rid].append(row)
    interior = {r for r, flags in road_flags.items() if flags and flags <= values}

    tf = source.L['topo']['fields']; edges = defaultdict(list)
    for row in source.layer_raw_records('topo'):
        a = row['attributes']; start = str(a.get(tf['from'], '')).strip()
        end = str(a.get(tf['to'], '')).strip()
        edges[start].append((end, row))
    visited, paths, expansions = set(), [], 0
    for start in ingress:
        stack = [[start]]
        while stack:
            path = stack.pop(); current = path[-1]
            if current in egress:
                paths.append(path); continue
            visited.add(current); expansions += 1
            if expansions > budget:
                raise ValueError('source traversal budget exceeded; incomplete inventory')
            outgoing = edges.get(current, [])
            if not outgoing:
                issues.append({'code': 'SOURCE_MOVEMENT_DEAD_END', 'path': path}); continue
            if len(outgoing) != len(set(v for v, _ in outgoing)):
                issues.append({'code': 'SOURCE_TOPO_DUPLICATE_EDGE', 'source_lane_id': current})
            for nxt in sorted(set(v for v, _ in outgoing), reverse=True):
                if nxt in path:
                    issues.append({'code': 'SOURCE_MOVEMENT_CYCLE', 'path': path + [nxt]})
                elif nxt not in lane_road:
                    issues.append({'code': identity_issues.get(nxt, 'SOURCE_TOPO_LANE_MISSING'),
                                   'path': path + [nxt]})
                elif nxt not in egress and lane_road[nxt] not in interior:
                    issues.append({'code': 'SOURCE_MOVEMENT_LEAVES_DECLARED_INTERIOR',
                                   'path': path + [nxt], 'road': lane_road[nxt]})
                else:
                    stack.append(path + [nxt])
    used = set(ingress + egress) | {sid for p in paths for sid in p} | visited
    return {'source_junction_id': str(junction_id), 'junction_record': junctions[0],
            'ingress_roads': enter, 'egress_roads': leave, 'ingress_lanes': ingress,
            'egress_lanes': egress, 'paths': sorted(paths), 'issues': issues,
            'source_lane_rows': {s: [r[2] for r in raw_lanes.get(s, [])] for s in sorted(used)},
            'interior_road_records': {r: road_rows[r] for r in sorted(
                {lane_road[s] for s in used if s in lane_road} & interior)},
            'topology_records': [row for s in sorted(visited) for _, row in edges.get(s, [])]}


def compare_movements(root, source, inventory, xodr_junction_id):
    """Compare full ORIGINAL port+via+port tuples, not via IDs or counts alone."""
    graph = PortDependencies(root); graph.validate_junction_table()
    if sum(j.get('id') == str(xodr_junction_id) for j in root.findall('junction')) != 1:
        raise ValueError('explicit XODR junction binding missing or duplicated')
    topo = defaultdict(list)
    tf = source.L['topo']['fields']
    for row in source.layer_raw_records('topo'):
        a = row['attributes']
        topo[str(a[tf['from']]).strip()].append(str(a[tf['to']]).strip())
    expected = {tuple(p) for p in inventory['paths']}
    actual, issues = [], []
    for cid, (a, b) in sorted(graph.connections.items()):
        if graph.roads[cid].get('junction') != str(xodr_junction_id):
            continue
        try:
            start, end = _source_id(_lane_at(graph, a)), _source_id(_lane_at(graph, b))
            via = _source_id(graph.roads[cid].find('lanes/laneSection/right/lane'))
            # Resolve against COMPLETE source paths already enumerated from
            # all ingress ports. An interior branch is legal; ambiguity means
            # multiple complete paths match these endpoints and via identity.
            candidates = []
            ports = set(inventory['ingress_lanes'] + inventory['egress_lanes'])
            inside = {s for p in expected for s in p[1:-1]}
            for path in sorted(expected):
                if via not in path: continue
                try:
                    before = source_chain(topo, start, path[0])
                    after = source_chain(topo, path[-1], end)
                except ValueError: continue
                if (set(before[:-1]) | set(after[1:])) & (inside | ports): continue
                candidates.append((list(path), before[:-1] + list(path) + after[1:]))
            if len(candidates) != 1:
                raise ValueError('original movement has '+str(len(candidates))+' complete matching paths')
            movement, chain = candidates[0]
            actual.append({'connector': cid, 'source_path': chain, 'movement': movement,
                           'incoming_port': vars(a), 'outgoing_port': vars(b)})
        except ValueError as exc:
            issues.append({'code': 'WRITTEN_MOVEMENT_UNRESOLVED', 'connector': cid, 'reason': str(exc)})
    counts = Counter(tuple(a['movement']) for a in actual)
    missing = sorted(expected - counts.keys()); extra = sorted(counts.keys() - expected)
    duplicates = sorted(p for p, n in counts.items() if n > 1)
    for code, rows in [('SOURCE_MOVEMENT_MISSING_FROM_XODR', missing),
                       ('XODR_MOVEMENT_NOT_IN_SOURCE', extra), ('XODR_MOVEMENT_DUPLICATED', duplicates)]:
        issues.extend({'code': code, 'movement': list(p)} for p in rows)
    return {'xodr_junction_id': str(xodr_junction_id), 'expected_count': len(expected),
            'written_count': sum(r.get('junction') == str(xodr_junction_id)
                                 and r.get('name') != 'junction_paving' for r in graph.roads.values()),
            'resolved_count': len(actual), 'actual': actual,
            'missing': missing, 'extra': extra, 'duplicates': duplicates, 'issues': issues,
            'status': 'MATCH' if not issues and not inventory['issues'] else 'REJECTED'}


def linear_chart(road):
    geoms = road.findall('planView/geometry')
    if len(geoms) != 1 or geoms[0].find('line') is None:
        raise ValueError('interval prototype requires one exact Line parent chart')
    g = geoms[0]; length = float(road.get('length'))
    values = [float(g.get(k)) for k in ('x', 'y', 'hdg', 's', 'length')]
    if not all(math.isfinite(v) for v in values + [length]) or length <= 0:
        raise ValueError('invalid linear chart')
    if abs(values[3]) > 1e-9 or abs(values[4]-length) > 1e-7:
        raise ValueError('linear chart station/length mismatch')
    return {'origin': values[:2], 'tangent': [math.cos(values[2]), math.sin(values[2])],
            'length': length}


def exact_parent_chart(road):
    """An exact single Line/Arc chart, never a sampled or straightened curve.

    Arc normal-plane clipping is valid on a single forward angular branch.
    Source support is checked on that branch before any interval is assigned.
    Compound/spiral charts require a separate, explicit domain model.
    """
    geoms = road.findall('planView/geometry')
    if len(geoms) != 1 or len(geoms[0]) != 1:
        raise ValueError('source accounting requires one exact Line/Arc chart')
    g = geoms[0]
    if g[0].tag == 'line':
        return dict(linear_chart(road), kind='line', curvature=0.)
    if g[0].tag != 'arc':
        raise ValueError('unsupported source chart; no sampled or Line fallback')
    length = float(road.get('length'))
    x, y, h, s, span = [float(g.get(k)) for k in ('x', 'y', 'hdg', 's', 'length')]
    k = float(g[0].get('curvature'))
    if not np.isfinite([x, y, h, s, span, k, length]).all() or length <= 0:
        raise ValueError('invalid exact Arc chart')
    if abs(s) > 1e-9 or abs(span-length) > 1e-7:
        raise ValueError('Arc chart station/length mismatch')
    if abs(k*length) >= math.pi/2:
        raise ValueError('Arc chart exceeds single forward branch')
    return dict(origin=[x, y], tangent=[math.cos(h), math.sin(h)],
                length=length, kind='arc', curvature=k)


def _arc_axis(chart):
    from mapforge.ops.arc_source_chart import ArcChart
    return ArcChart(tuple(chart['origin']), math.atan2(*chart['tangent'][::-1]),
                    chart.get('curvature', 0.))


def _normal_plane(chart, station):
    point, tangent, _ = _arc_axis(chart).frame(station)
    return (float(tangent[0]), float(tangent[1]), float(tangent @ point))


def _require_chart_support(chart, points):
    # 1-k*v is affine on each ORIGINAL line segment. Positive at every vertex
    # therefore proves the entire part lies on this projection branch.
    # No point deletion, normal wrapping, or silent clipping of unsupported data.
    if chart.get('curvature', 0.) != 0:
        _arc_axis(chart).project(points)


def clipped_intervals(points, planes=()):
    """Exact piecewise-linear clipping: every plane is n dot XY >= offset.

    Return original part arclength intervals. No nearest sampled point, no
    resampling and no implicit connector across multipart is introduced.
    """
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 2 or not np.isfinite(p).all():
        raise ValueError('invalid source part')
    if any(len(plane) != 3 or not all(math.isfinite(v) for v in plane)
           or math.hypot(*plane[:2]) < 1e-12 for plane in planes):
        raise ValueError('invalid clipping plane')
    lengths = np.linalg.norm(np.diff(p, axis=0), axis=1)
    stations = np.r_[0., np.cumsum(lengths)]
    if stations[-1] <= 1e-9:
        raise ValueError('zero-length source part')
    intervals = []
    for i, length in enumerate(lengths):
        if length <= 1e-12:
            continue  # Original duplicate vertex still retained in stations.
        lo, hi = 0., 1.
        for nx, ny, offset in planes:
            a = nx*p[i, 0] + ny*p[i, 1] - offset
            d = nx*(p[i+1, 0]-p[i, 0]) + ny*(p[i+1, 1]-p[i, 1])
            if abs(d) < 1e-12:
                if a < -1e-9: hi = -1.; break
            elif d > 0: lo = max(lo, -a/d)
            else: hi = min(hi, -a/d)
        if hi > lo + 1e-12:
            segment = [float(stations[i]+lo*length), float(stations[i]+hi*length)]
            if intervals and abs(intervals[-1][1]-segment[0]) < 1e-8:
                intervals[-1][1] = segment[1]
            else:
                intervals.append(segment)
    return stations.tolist(), intervals


def _span_planes(chart, lo, hi):
    if not (math.isfinite(lo) and math.isfinite(hi) and 0 <= lo < hi <= chart['length'] + 1e-7):
        raise ValueError('invalid ordinary section interval')
    if chart.get('curvature', 0.) != 0:
        return [_normal_plane(chart, lo), tuple(-v for v in _normal_plane(chart, hi))]
    nx, ny = chart['tangent']; x, y = chart['origin']; base = nx*x + ny*y
    return [(nx, ny, base+lo), (-nx, -ny, -base-hi)]


def _outside_plane(chart, contact):
    if contact not in ('start', 'end'):
        raise ValueError('unknown contact')
    if chart.get('curvature', 0.) != 0:
        plane = _normal_plane(chart, 0. if contact == 'start' else chart['length'])
        return [tuple(-v for v in plane) if contact == 'start' else plane]
    nx, ny = chart['tangent']; x, y = chart['origin']; base = nx*x + ny*y
    if contact == 'start': return [(-nx, -ny, -base)]
    if contact == 'end': return [(nx, ny, base+chart['length'])]
    raise ValueError('unknown contact')


def source_projection(root):
    """The existing explicit local-equidistant research pipeline, not CRS approval."""
    lat0, lon0 = _origin(root)
    reference = root.findtext('header/geoReference') or ''
    pairs = [word.lstrip('+').split('=', 1) for word in reference.split()]
    spec = {v[0]: v[1] if len(v) == 2 else True for v in pairs}
    allowed = {'proj', 'lat_0', 'lon_0', 'lat_ts', 'R', 'units', 'no_defs'}
    radius, lat_ts = float(spec.get('R', 0)), float(spec.get('lat_ts', math.inf))
    if (len(spec) != len(pairs) or set(spec)-allowed or spec.get('proj') != 'eqc'
            or not all(math.isfinite(v) for v in (radius, lat_ts))
            or spec.get('units') != 'm' or abs(radius-6378137.) > 1e-8
            or abs(lat_ts-lat0) > 1e-10
            or not (-90 < lat0 < 90 and -180 <= lon0 <= 180)):
        raise ValueError('unsupported projection; no silent local-XY assumption')
    return lat0, lon0, reference


def allocate_source_intervals(root, scope, comparison, chart_mode='line-only-v1'):
    """Account for full original parts; allocation is NOT geometric fit."""
    if chart_mode not in ('line-only-v1', 'exact-line-arc-v1'):
        raise ValueError('unsupported source chart mode')
    lat0, lon0, reference = source_projection(root)
    graph = PortDependencies(root)
    charts, consumers, issues = {}, [], []
    def chart(rid):
        if rid not in charts:
            charts[rid] = (linear_chart if chart_mode == 'line-only-v1' else exact_parent_chart)(graph.roads[rid])
        return charts[rid]
    for u in scope['occurrences']:
        if u['scope_role'] == 'connector': continue
        rid, si = u['road'], u['section']
        sections = graph.roads[rid].findall('lanes/laneSection')
        lo = float(sections[si].get('s')); hi = (float(sections[si+1].get('s'))
               if si+1 < len(sections) else float(graph.roads[rid].get('length')))
        consumers.append({'owner': 'ordinary:'+rid+':'+str(si), 'role': u['scope_role'],
                          'source_lane_id': u['source_lane_id'], 'lane': u['lane'],
                          'planes': _span_planes(chart(rid), lo, hi)})
    # A fixed endpoint's source feature may continue through several earlier
    # sections of the same UNEDITED parent. Record those existing consumers as
    # fixed context, without expanding movable scope or calling them fitted.
    # Only explicit same-ID occurrences qualify; nearby geometry does not.
    fixed_roads = {p['road'] for p in scope['fixed_ports']} - set(scope['mutable_roads'])
    known = {(c['owner'], c['source_lane_id'], c.get('lane')) for c in consumers}
    for rid in sorted(fixed_roads):
        sections = graph.roads[rid].findall('lanes/laneSection')
        for si, sec in enumerate(sections):
            lo = float(sec.get('s')); hi = (float(sections[si+1].get('s'))
                    if si+1 < len(sections) else float(graph.roads[rid].get('length')))
            for lane in sec.findall('left/lane') + sec.findall('right/lane'):
                try:
                    ids = (_source_ids(lane) if scope.get('identity_mode') == 'source-chain-v1'
                           else [_source_id(lane)])
                except ValueError: continue
                for sid in ids:
                    key = ('ordinary:'+rid+':'+str(si), sid, int(lane.get('id')))
                    if sid not in scope['observations'] or key in known: continue
                    known.add(key)
                    consumers.append({'owner': key[0], 'role': 'fixed_context_not_edited',
                                      'source_lane_id': sid, 'lane': key[2],
                                      'planes': _span_planes(chart(rid), lo, hi)})
    resolved = {a['connector']: a for a in comparison['actual']}
    for cid in scope['connectors']:
        if cid not in resolved:
            issues.append({'code': 'CONNECTOR_HAS_NO_RESOLVED_SOURCE_DOMAIN', 'connector': cid}); continue
        a = resolved[cid]; path = a['source_path']
        if len(path) != len(set(path)) or len(path) < 2:
            issues.append({'code': 'SOURCE_SUPPORT_CHAIN_NOT_SIMPLE', 'connector': cid}); continue
        for i, sid in enumerate(path):
            planes = []
            if i == 0:
                p = a['incoming_port']; planes += _outside_plane(chart(p['road']), p['contact'])
            if i == len(path)-1:
                p = a['outgoing_port']; planes += _outside_plane(chart(p['road']), p['contact'])
            consumers.append({'owner': 'connector:'+cid, 'role': 'connector',
                              'source_lane_id': sid, 'planes': planes,
                              'support_role': 'full_original_via_or_intermediate' if 0 < i < len(path)-1
                              else 'source_tail_beyond_structural_mouth'})

    features = {}
    for sid, obs in scope['observations'].items():
        rows = obs.get('raw_records', ())
        layer = obs.get('identity_resolution', {}).get('selected_layer')
        chosen = [r for r in rows if layer is None or r['layer'] == layer]
        if len(chosen) != 1 or chosen[0]['geometry_origin'] != 'field':
            issues.append({'code': 'SOURCE_PATH_REQUIRES_EXPLICIT_GEOMETRY_MODEL', 'source_lane_id': sid}); continue
        key = 'lane:'+sid
        features[key] = {'records': chosen, 'lane_ids': [sid], 'kind': 'lane_path'}
    for key, obj in scope['boundaries'].items():
        features['boundary:'+key] = {'records': obj['records'], 'kind': 'physical_boundary',
                                    'lane_ids': scope['boundary_owners'][key]}
    output = {}
    for key, feature in sorted(features.items()):
        if len(feature['records']) != 1:
            issues.append({'code': 'INTERVAL_FEATURE_IDENTITY_AMBIGUOUS', 'feature': key}); continue
        rr = feature['records'][0]; parts = []
        if not rr['parts']:
            issues.append({'code': 'INTERVAL_FEATURE_GEOMETRY_MISSING', 'feature': key})
        for pi, raw in enumerate(rr['parts']):
            xy = _project(np.asarray(raw), lat0, lon0)
            stations, _ = clipped_intervals(xy)
            ranges, evidence = defaultdict(list), []
            for consumer in consumers:
                if consumer['source_lane_id'] not in feature['lane_ids']: continue
                if chart_mode == 'exact-line-arc-v1':
                    if consumer['owner'].startswith('ordinary:'):
                        parents = [consumer['owner'].split(':')[1]]
                    else:
                        cid = consumer['owner'].split(':')[1]
                        path = resolved[cid]['source_path']
                        parents = []
                        if consumer['source_lane_id'] == path[0]:
                            parents.append(resolved[cid]['incoming_port']['road'])
                        if consumer['source_lane_id'] == path[-1]:
                            parents.append(resolved[cid]['outgoing_port']['road'])
                    for rid in parents:
                        _require_chart_support(chart(rid), xy)
                _, intervals = clipped_intervals(xy, consumer['planes'])
                ranges[consumer['owner']].extend(intervals)
                evidence.append(dict(consumer, intervals_m=intervals))
            cuts = sorted(set([0., stations[-1]] + [v for rows in ranges.values() for p in rows for v in p]))
            atoms = []
            for lo, hi in zip(cuts[:-1], cuts[1:]):
                if hi-lo < 1e-8: continue
                mid = (lo+hi)/2
                owners = sorted(o for o, rows in ranges.items() if any(a <= mid <= b for a, b in rows))
                ordinary = [o for o in owners if o.startswith('ordinary:')]
                if len(ordinary) > 1 or ordinary and len(owners) > 1:
                    status = 'CONFLICTING_STRUCTURAL_OWNERS'
                elif owners:
                    status = 'SHARED_CONNECTOR_SUPPORT' if len(owners) > 1 else 'ASSIGNED'
                else:
                    status = 'UNASSIGNED_REQUIRES_SCOPE_DECISION'
                atoms.append({'source_s_m': [lo, hi], 'consumers': owners, 'status': status})
            parts.append({'part_index': pi, 'raw_vertices': raw, 'source_vertex_s_m': stations,
                          'length_m': stations[-1], 'atoms': atoms, 'consumer_cuts': evidence,
                          'vertices': [{'index': i, 'source_s_m': s,
                                        'consumers': sorted(o for o, rows in ranges.items()
                                                           if any(a-1e-8 <= s <= b+1e-8 for a, b in rows))}
                                       for i, s in enumerate(stations)]})
        output[key] = {'kind': feature['kind'], 'layer': rr['layer'],
                       'record_index': rr['record_index'], 'source_lane_ids': feature['lane_ids'],
                       'parts': parts}
    totals = Counter()
    for f in output.values():
        for p in f['parts']:
            for a in p['atoms']: totals[a['status']] += a['source_s_m'][1] - a['source_s_m'][0]
    unresolved = totals['UNASSIGNED_REQUIRES_SCOPE_DECISION'] + totals['CONFLICTING_STRUCTURAL_OWNERS']
    result = {'scope': 'observations_in_selected_dependency_packet_not_entire_map',
            'projection': {'type': 'local_eqc', 'lat_0': lat0, 'lon_0': lon0, 'radius_m': 6378137.,
                           'declared_proj_pipeline': reference,
                           'absolute_crs_verified': False},
            'cut_policy': 'current_ordinary_Line_normal_planes_not_source_shape_edits',
            'status': 'ACCOUNTED_NOT_GEOMETRY_VALIDATED' if unresolved < 1e-8 and not issues
                      else 'UNRESOLVED',
            'features': output, 'length_by_status_m': dict(totals), 'issues': issues,
            'shared_boundary_contact_binding': 'NOT_SOLVED', 'geometry_fit_validated': False}
    if chart_mode != 'line-only-v1':
        result.update(chart_mode=chart_mode, exact_parent_charts=charts,
                      cut_policy='current_exact_Line_Arc_normal_planes_not_source_shape_edits')
    return result


def source_speed_intervals(root, scope, partition):
    """Compare speeds only on the ORIGINAL support of each written speed span.

    A long lane chain can contain several source speed limits. Comparing every
    source ID to every speed record of every section creates false mismatches.
    This audit clips full source segments, never changes speed or invents a
    design driving condition. Target/source geometry quality is still separate.
    """
    graph = PortDependencies(root); charts = {}; points = {}; rows = []
    lat, lon, _ = source_projection(root)
    for use in scope['occurrences']:
        if use['scope_role'] == 'connector':
            continue  # Movement design speed is not an ordinary source limit.
        rid, si, sid = use['road'], use['section'], use['source_lane_id']
        if rid not in charts:
            charts[rid] = exact_parent_chart(graph.roads[rid])
        if sid not in points:
            feature = partition['features'].get('lane:'+sid)
            points[sid] = ([_project(np.asarray(p['raw_vertices']), lat, lon) for p in feature['parts']]
                           if feature else [])
        chart = charts[rid]
        sections = graph.roads[rid].findall('lanes/laneSection')
        start = float(sections[si].get('s'))
        end = float(sections[si+1].get('s')) if si+1 < len(sections) else chart['length']
        speeds = use['written_speeds']
        offsets = [float(s['sOffset']) for s in speeds]
        if any(not math.isfinite(v) or not 0 <= v < end-start for v in offsets) or any(
                b <= a for a, b in zip(offsets[:-1], offsets[1:])):
            raise ValueError('invalid source speed span ordering/domain')
        for xy in points[sid]:
            _require_chart_support(chart, xy)
        def measure(lo, hi):
            return sum(b-a for xy in points[sid] for a, b in clipped_intervals(
                xy, _span_planes(chart, lo, hi))[1]) if hi > lo else 0.
        covered = measure(start, end)
        missing = measure(start, start+offsets[0]) if offsets else covered
        mismatches = []; intervals = []
        source_speed = scope['observations'][sid].get('source_max_speed_kmh')
        for i, item in enumerate(speeds):
            lo, hi = start+offsets[i], start+offsets[i+1] if i+1 < len(offsets) else end
            overlap = measure(lo, hi)
            if overlap <= 1e-7:
                continue
            record = dict(written_s_m=[lo, hi], original_overlap_m=overlap,
                          original_limit_kmh=source_speed, written_limit_kmh=item['kmh'])
            intervals.append(record)
            if source_speed is None or abs(item['kmh']-source_speed) > 1e-6:
                mismatches.append(record)
        verdict = ('SOURCE_GEOMETRY_MISSING' if not points[sid] else 'NO_ORIGINAL_OVERLAP' if covered <= 1e-7
                   else 'MISMATCH' if mismatches else 'MISSING_WRITTEN_SPEED' if missing > 1e-7 else 'MATCH')
        rows.append(dict(road=rid, section=si, lane=use['lane'], source_lane_id=sid,
                         status=verdict, overlap_m=covered, missing_written_speed_m=missing,
                         intervals=intervals, mismatches=mismatches))
    counts = Counter(r['status'] for r in rows)
    return dict(scope='ordinary_original_support_only_not_driving_dynamics_or_full_geometry',
                counts=dict(counts), rows=rows, source_speed_changed=False, geometry_validated=False)


def prepare_source_domain(root, source, scope, source_junction_id, xodr_junction_id,
                          chart_mode='line-only-v1'):
    inventory = original_movements(source, source_junction_id)
    comparison = compare_movements(root, source, inventory, xodr_junction_id)
    partition = allocate_source_intervals(root, scope, comparison, chart_mode)
    result = {'schema': 'mapforge.source-domain/v1' if chart_mode == 'line-only-v1' else 'mapforge.source-domain/v2',
              'scope_sha256': scope['content_sha256'],
              'inventory': inventory, 'comparison': comparison, 'partition': partition,
              'status': 'BLOCKED', 'geometry_solver_ran': False, 'export_allowed': False}
    if scope.get('identity_mode') == 'source-chain-v1' and chart_mode == 'exact-line-arc-v1':
        result['source_speed_intervals'] = source_speed_intervals(root, scope, partition)
    result['content_sha256'] = digest(result)
    return result


def require_domain_replay(packet, root, source, scope):
    # Fresh source acquisition is part of replay, not a caller promise. An
    # existing ProfileSource may have stale topology AND lane geometry caches.
    fresh_source = source.fresh_reader()
    require_solver_input(scope, root, fresh_source)
    if digest({k: v for k, v in packet.items() if k != 'content_sha256'}) != packet.get('content_sha256'):
        raise ValueError('source domain digest mismatch')
    fresh = prepare_source_domain(root, fresh_source, scope, packet['inventory']['source_junction_id'],
                                  packet['comparison']['xodr_junction_id'],
                                  packet['partition'].get('chart_mode', 'line-only-v1'))
    if fresh['content_sha256'] != packet['content_sha256']:
        raise ValueError('source domain differs from fresh original inputs')
    return True  # Replay integrity, NOT source/geometry acceptance.
