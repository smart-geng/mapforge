"""E1: read-only source/anchor admission for the reviewed road11 event.

This is NOT a new fitter, source-role decision or a source ownership repair.
Replay the existing whole-network ledger on the actual XML. Keep original
arclength distinct from road station; unassigned fragments cannot disappear
because they are small or because a drawing has a nearby edge.
"""
from collections import Counter
from dataclasses import asdict
import math

import numpy as np
from pyproj import Transformer

from mapforge.ops.port_dependencies import LanePort, PortDependencies
from mapforge.ops.reconstruction_scope import digest as object_digest
from mapforge.ops.source_domain import allocate_source_intervals, source_projection
from mapforge.repair_web.model import digest, parse
from mapforge.validate.junction_edges import audit as junction_audit
from scripts.check_outer_event_written import boundary_poly, max_abs
from scripts.internal_edge_jets import states

START, END = 61.8028, 211.7074107
BIRTHS = {3: (131.0543, '2023041111104128071'), 4: (151.6089, '2023041111104060474')}
CONNECTORS = tuple(str(i) for i in range(106, 112))
REFERENCE_SHA = '5d1cec2baea21200fbdcf78ca9267bfe7bb94e9cf441539d759370fffdaaea98'
LIMITS = dict(position_m=.01, heading_deg=.1, curvature_per_m=1e-7)


def checked_payload(value):
    """A valid signature is necessary, not a substitute for structural replay."""
    if object_digest({k: v for k, v in value.items() if k != 'content_sha256'}) != value['content_sha256']:
        raise ValueError('Content signature mismatch')
    return value


def slope_class(value):
    # Classification of floating point zero only, NOT a relaxed shape guard.
    if not math.isfinite(value):
        raise ValueError('Nonfinite derivative')
    return 'numerical_zero' if abs(value) <= 1e-10 else 'increasing' if value > 0 else 'decreasing'


def world_delta(a, b):
    out = dict(position_m=math.hypot(a[0]-b[0], a[1]-b[1]),
               heading_deg=math.degrees(abs(math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2])))),
               curvature_per_m=abs(a[3]-b[3]))
    out['pass'] = all(math.isfinite(out[k]) and out[k] <= v for k, v in LIMITS.items())
    return out


def collect_sources(root, packet):
    road = root.find("road[@id='11']")
    if road is None or abs(float(road.get('length'))-END) > 1e-8:
        raise ValueError('Unexpected parent domain')
    geoms = road.findall('planView/geometry')
    if len(geoms) != 1 or geoms[0][0].tag != 'line':
        raise ValueError('E1 requires the reviewed exact Line chart')
    _, _, projection = source_projection(root)
    off = root.find('header/offset')
    if off is not None and any(float(off.get(k, '0')) != 0 for k in ('x', 'y', 'z', 'hdg')):
        raise ValueError('Unregistered coordinate offset')
    proj = Transformer.from_crs(4326, projection, always_xy=True)
    g = geoms[0]; h = float(g.get('hdg'))
    origin = np.array([float(g.get('x')), float(g.get('y'))])
    basis = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])

    def project(raw):
        xy = np.asarray([proj.transform(float(p[0]), float(p[1])) for p in raw])
        return (xy-origin)@basis

    owners = {}; edges = {}
    for u in packet['occurrences']:
        if u['road'] != '11' or u['scope_role'] != 'ordinary':
            continue
        sid, lid = u['source_lane_id'], u['lane']
        if lid >= 0 or sid in owners and owners[sid] != lid:
            raise ValueError('Ambiguous original lane identity')
        owners[sid] = lid
        for rel in packet['observations'][sid]['boundary_relations']:
            edge = -lid if rel['declared_side'] == 'left' else -lid-1 if rel['declared_side'] == 'right' else None
            key = rel['boundary_key']
            if edge is None or key in edges and edges[key] != edge:
                raise ValueError('Ambiguous original boundary ownership')
            edges[key] = edge
    if len(owners) != 19 or len(edges) != 25:
        raise ValueError('Reviewed 19-path/25-boundary scope changed')
    features = {}
    for key, edge in sorted(edges.items()):
        records = packet['boundaries'][key]['records']
        if len(records) != 1:
            raise ValueError('Ambiguous source record')
        features['boundary:'+key] = dict(kind='physical_boundary', edge=edge, record=records[0])
    for sid, lid in sorted(owners.items()):
        obs = packet['observations'][sid]
        rows = [r for r in obs['raw_records'] if r['layer'] == obs['identity_resolution']['selected_layer']]
        if len(rows) != 1 or rows[0]['geometry_origin'] != 'field':
            raise ValueError('Explicit original path record required')
        features['lane:'+sid] = dict(kind='movement_path', lane=lid, record=rows[0],
                                    path_sha256=object_digest(obs['lane_path']))
    for f in features.values():
        f['parts_st'] = [project(p) for p in f['record']['parts']]
        if not f['parts_st'] or any(not np.all(np.isfinite(st)) or len(st) < 2 for st in f['parts_st']):
            raise ValueError('Missing/nonfinite original geometry')
    return road, features


def partition_rows(features, ledger):
    """Full original-part accounting. No clamping to [0, END] or edit domain."""
    rows = []
    for key, f in features.items():
        lf = ledger['features'].get(key)
        rr = f['record']
        if lf is None or lf['record_index'] != rr['record_index'] or lf['layer'] != rr['layer']:
            raise ValueError('Missing or mismatched original feature: '+key)
        if len(lf['parts']) != len(rr['parts']):
            raise ValueError('Part was removed from source accounting')
        for part, raw, st in zip(lf['parts'], rr['parts'], f['parts_st']):
            if part['raw_vertices'] != raw:
                raise ValueError('Raw vertices differ; clipping source is forbidden')
            intrinsic = np.asarray(part['source_vertex_s_m'])
            if len(intrinsic) != len(st) or np.any(np.diff(intrinsic) < 0):
                raise ValueError('Invalid original arclength index')
            previous = 0.
            for atom in part['atoms']:
                a, b = atom['source_s_m']
                if b <= a or abs(a-previous) > 1e-8 or b > part['length_m']+1e-8:
                    raise ValueError('Source atoms do not partition the full part')
                previous = b
                ss = [float(np.interp(v, intrinsic, st[:, 0])) for v in (a, b)]
                mids = st[(intrinsic > a) & (intrinsic < b), 0]
                lo, hi = min(ss+list(mids)), max(ss+list(mids))
                rows.append(dict(feature=key, kind=f['kind'], record_index=rr['record_index'],
                    part=part['part_index'], source_arclength_m=[a, b], parent_s_range=[float(lo), float(hi)],
                    length_m=b-a, consumers=atom['consumers'], status=atom['status'],
                    intersects_edit_domain=hi > START and lo < END,
                    beyond_parent=lo < -1e-8 or hi > END+1e-8,
                    **({'edge': f['edge']} if 'edge' in f else {'lane': f['lane']})))
            if abs(previous-part['length_m']) > 1e-8:
                raise ValueError('Original tail missing from ledger')
    return rows


def source_anchors(road, features):
    rows = []
    for s, edge_ids in ((START, range(3)), (END, range(5))):
        for edge in edge_ids:
            c = boundary_poly(road, edge, s, left=True)
            support = []
            for key, f in features.items():
                if f.get('edge') != edge: continue
                for pi, st in enumerate(f['parts_st']):
                    for p, q in zip(st[:-1], st[1:]):
                        if min(p[0], q[0]) <= s <= max(p[0], q[0]) and q[0] != p[0]:
                            slope = (q[1]-p[1])/(q[0]-p[0]); t = p[1]+slope*(s-p[0])
                            support.append(dict(feature=key, part=pi, source_segment_s=[float(p[0]), float(q[0])],
                                point_error_m=float(abs(c[0]-t)), chord_slope=float(slope),
                                chord_is_surveyed_tangent=False))
            lane = -max(1, edge); side = 0 if edge == 0 else 1
            world = states(road, lane, s, True)[side]
            seam = world_delta(world, states(road, lane, s, False)[side]) if s == START else None
            rows.append(dict(s=s, edge=edge, boundary_jets=[float(c[0]), float(c[1]), float(2*c[2])],
                world_jets=list(world), derivative_class=slope_class(c[1]), source=support,
                source_point_within_existing_event_budget=bool(support) and max(v['point_error_m'] for v in support) <= .75+1e-7,
                upstream_seam=seam, full_anchor_admitted=False))
    return rows


def upstream_envelope(road, features):
    cuts = {0., START}
    cuts.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
    for sec in road.findall('lanes/laneSection'):
        s = float(sec.get('s')); cuts.add(s)
        cuts.update(s+float(w.get('sOffset')) for w in sec.findall('.//width'))
    rows = []
    for key, f in features.items():
        if f['kind'] != 'physical_boundary': continue
        for pi, raw_st in enumerate(f['parts_st']):
            st = raw_st if raw_st[-1, 0] > raw_st[0, 0] else raw_st[::-1]
            if np.any(np.diff(st[:, 0]) <= 0): raise ValueError('Unsupported source chart')
            for p, q in zip(st[:-1], st[1:]):
                lo, hi = max(0., p[0]), min(START, q[0])
                if hi <= lo: continue
                stations = sorted({lo, hi} | {v for v in cuts if lo < v < hi})
                slope = (q[1]-p[1])/(q[0]-p[0])
                for a, b in zip(stations[:-1], stations[1:]):
                    src = np.array([p[1]+slope*(a-p[0]), slope, 0., 0.])
                    rows.append(dict(feature=key, part=pi, edge=f['edge'], s=[float(a), float(b)],
                        max_m=max_abs(boundary_poly(road, f['edge'], a)-src, b-a)))
    return dict(scope='full boundary support INSIDE frozen [0,61.8028]; external heads remain in ownership rows',
                max_m=max(r['max_m'] for r in rows), rows=rows,
                tolerance_m=.75, tolerance_origin='existing L01 event budget; not visual acceptance',
                pass_existing_budget=all(r['max_m'] <= .75+1e-7 for r in rows))


def prefix_to_plane(curve_st, stop):
    """First forward prefix ending on a measured source end plane, not nearest via."""
    if curve_st[0, 0] >= stop:
        raise ValueError('Source tail does not extend ahead of the actual mouth')
    for i, (a, b) in enumerate(zip(curve_st[:-1], curve_st[1:])):
        if b[0] < a[0]-1e-8:
            raise ValueError('Written prefix reverses before source tail end')
        if a[0] < stop <= b[0]:
            endpoint = a+(b-a)*((stop-a[0])/(b[0]-a[0]))
            return np.vstack([curve_st[:i+1], endpoint])
    raise ValueError('Written prefix never reaches original tail end plane')


def tail_diagnostics(root, packet, ledger, features, rows):
    """Same-ID source tails vs actual finite connector prefixes, all consumers.

    Sampling is for diagnostics only, not exported geometry or a continuous
    Hausdorff certificate. Paths are reported independently of physical edges.
    """
    from shapely.geometry import LineString, Point
    from scripts.review_measured_ribbon import target_curves
    road = root.find("road[@id='11']"); g = road.find('planView/geometry')
    h = float(g.get('hdg')); origin = np.array([float(g.get('x')), float(g.get('y'))])
    basis = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    graph = PortDependencies(root); cache = {}; result = []
    for r in rows:
        if r['parent_s_range'][0] < END-1e-7: continue
        f = features[r['feature']]; pi = r['part']; st = f['parts_st'][pi]
        part = ledger['features'][r['feature']]['parts'][pi]
        ss = np.asarray(part['source_vertex_s_m']); a, b = r['source_arclength_m']
        # Include every raw interior vertex and both exact ledger cuts.
        query = np.unique(np.r_[a, ss[(ss > a) & (ss < b)], b])
        source = np.c_[np.interp(query, ss, st[:, 0]), np.interp(query, ss, st[:, 1])]
        if source[-1, 0] < source[0, 0]: source = source[::-1]
        length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(source, axis=0), axis=1))]
        dense_s = np.unique(np.r_[length, np.arange(0., length[-1], .05)])
        dense = np.c_[np.interp(dense_s, length, source[:, 0]), np.interp(dense_s, length, source[:, 1])]
        for consumer in r['consumers']:
            if not consumer.startswith('connector:'): raise ValueError('Unexpected tail consumer')
            cid = consumer.split(':')[1]; port = graph.connections[cid][0]
            if port.road != '11' or port.contact != 'end': raise ValueError('Unexpected incoming tail')
            field = 'center' if f['kind'] == 'movement_path' else 'left' if f['edge'] == -port.lane-1 else 'right' if f['edge'] == -port.lane else None
            if field is None: raise ValueError('Tail assigned to an unrelated physical edge')
            if cid not in cache:
                cache[cid] = {k: (xy-origin)@basis for k, xy in target_curves(graph.roads[cid], .05).items()}
            item = dict(feature=r['feature'], part=pi, consumer=consumer, field=field,
                        source_arclength_m=[a, b], source_st=source.tolist())
            try:
                target = prefix_to_plane(cache[cid][field], float(source[-1, 0]))
                target_line, source_line = LineString(target), LineString(source)
                st_error = [Point(p).distance(target_line) for p in dense]
                ts_error = [Point(p).distance(source_line) for p in target]
                item.update(status='SAMPLED_NOT_ACCEPTED', target_st=target.tolist(),
                    source_to_target_max_m=float(max(st_error)), target_to_source_max_m=float(max(ts_error)))
            except ValueError as exc:
                item.update(status='UNRESOLVED_PREFIX', reason=str(exc))
            result.append(item)
    return dict(scope='all explicit same-source consumers; finite first prefix; original vertices retained',
                sample_step_m=.05, continuous_certificate=False, acceptance_threshold_added=False, rows=result)


def admit(reference, packet, domain, decision):
    if digest(reference) != REFERENCE_SHA:
        raise ValueError('Reference changed: requires a new explicit scope review')
    checked_payload(packet); checked_payload(domain)
    if domain['scope_sha256'] != packet['content_sha256']:
        raise ValueError('Source ledger belongs to a different reconstruction packet')
    root = parse(reference); road, features = collect_sources(root, packet)
    graph = PortDependencies(root); graph.validate_junction_table()
    closure = graph.closure({LanePort('11', 'end', lid) for lid in (-1, -2, -3, -4)})
    if closure['connectors'] != frozenset(CONNECTORS):
        raise ValueError('Six-connector dependency closure changed')
    resolved = {r['connector']: r for r in domain['comparison']['actual']}
    for cid in CONNECTORS:
        expected = graph.connections[cid]
        if [resolved[cid]['incoming_port'], resolved[cid]['outgoing_port']] != [asdict(p) for p in expected]:
            raise ValueError('Ledger and actual connector ports disagree')
    # Recompute ALL existing structural cuts; does not fit or reassign a source.
    replay = allocate_source_intervals(root, packet, domain['comparison'], domain['partition'].get('chart_mode', 'line-only-v1'))
    if object_digest(replay) != object_digest(domain['partition']):
        raise ValueError('Actual XML no longer reproduces the frozen source ledger')
    rows = partition_rows(features, replay)
    unresolved = [r for r in rows if r['status'] not in ('ASSIGNED', 'SHARED_CONNECTOR_SUPPORT')]
    birth_rows = []
    approvals = {(r['source_lane_id'], r['contact']) for r in decision['decisions']
        if r['physical_authority'] == 'original_left_right_boundaries' and r['lane_path_role'] == 'movement_path_observation'}
    for edge, (s, sid) in BIRTHS.items():
        sec = next((v for v in road.findall('lanes/laneSection') if abs(float(v.get('s'))-s) < 1e-8), None)
        if sec is None: raise ValueError('Birth station changed')
        lane = sec.find(f"right/lane[@id='{-edge}']")
        obs = packet['observations'][sid]
        if lane is None or lane.find("userData[@code='mapforge.source_lane']").get('value') != sid or (sid, 'start') not in approvals or not obs['start_width_known'] or obs['start_width_mm'] != 0:
            raise ValueError('Existing exact source-role approval required')
        jet = world_delta(states(road, -(edge-1), s, True)[1], states(road, -edge, s, False)[1])
        birth_rows.append(dict(edge=edge, station=s, source_lane_id=sid, original_role_preserved=True, contact=jet))
    anchors = source_anchors(road, features)
    if len(anchors) != 8 or any(not a['source'] for a in anchors):
        raise ValueError('Anchor lacks same-identity original support')
    junction = junction_audit(root)
    seams = [r for r in junction['rows'] if r['connecting_road'] in CONNECTORS]
    errors = [r for r in junction['unresolved'] if r['connecting_road'] in CONNECTORS]
    if len(seams) != 24 or errors:
        raise ValueError('Incomplete six-connector world-edge readback')
    seam_failures = [r for r in seams if any(not math.isfinite(r[k]) or r[k] > v for k, v in LIMITS.items())]
    upstream = upstream_envelope(road, features)
    tails = tail_diagnostics(root, packet, replay, features, rows)
    blockers = []
    if unresolved: blockers.append('SOURCE_PARTITION_HAS_UNASSIGNED_OR_CONFLICTING_INTERVALS')
    if seam_failures: blockers.append('FROZEN_CONNECTOR_EDGE_CONTACT_FAILED')
    if any(a['upstream_seam'] and not a['upstream_seam']['pass'] for a in anchors): blockers.append('UPSTREAM_WORLD_CONTACT_FAILED')
    if any(not a['source_point_within_existing_event_budget'] for a in anchors): blockers.append('FROZEN_ANCHOR_SOURCE_POINT_FAILED')
    if not upstream['pass_existing_budget']: blockers.append('FROZEN_UPSTREAM_SOURCE_ENVELOPE_FAILED')
    if any(not b['contact']['pass'] for b in birth_rows): blockers.append('EXISTING_BIRTH_CONTACT_FAILED')
    # Endpoint continuity/ownership do not validate 6m tails or surveyed jets.
    blockers.append('FULL_ASSIGNED_TAIL_GEOMETRY_AND_ANCHOR_SHAPE_NOT_ADMITTED')
    totals = Counter()
    for r in unresolved: totals[r['kind']] += r['length_m']
    return dict(schema='mapforge/split-event-admission/v1', status='BLOCKED_E1', blockers=blockers,
        contract=dict(reference_sha256=digest(reference), packet_sha256=packet['content_sha256'],
            source_domain_sha256=domain['content_sha256'], source_role_sha256=object_digest(decision),
            parent='11', edit_domain=[START, END], frozen_upstream=[0., START], mutable_edges=list(range(5)),
            frozen_axis=True, frozen_mouth=True, births=birth_rows, dependent_connectors=list(CONNECTORS),
            connector_geometry_edit_allowed=False, source_target_retained=dict(edge=3, s=178.,
                feature='IBD_LANE_BOUNDARY:2023041110502726490'), new_trial_registered=False),
        source=dict(boundaries=25, movement_paths=19, source_ledger_replayed=True,
            original_parts_retained=True, rows=rows, unresolved=unresolved,
            unresolved_length_by_kind_m=dict(totals), unresolved_feature_arclength_sum_m=sum(totals.values()),
            note='sum over different original features, NOT one road gap or missing asphalt area',
            tails=[r for r in rows if r['parent_s_range'][0] >= END-1e-7],
            external_heads=[r for r in rows if r['parent_s_range'][1] <= 1e-7 and r['beyond_parent']]),
        anchors=anchors, frozen_upstream=upstream,
        tail_geometry=tails,
        dependency=dict(connectors=list(CONNECTORS), fixed_other_ports=[asdict(p) for p in sorted(closure['fixed_ports'])],
            thresholds=LIMITS, edge_contacts=seams, failures=seam_failures,
            connector_geometry_sha256={cid: object_digest(_tree(graph.roads[cid])) for cid in CONNECTORS}),
        next_step='close explicit source intervals and full tail geometry within E1; no solver authorization',
        optimizer_calls=0, new_xodr=False, web_changed=False, map_accepted=False,
        absolute_crs='NOT_VERIFIED', driving_cases='MISSING_BLOCKS_DELIVERY', old_failures='RETAINED')


def _tree(e):
    return (str(e.tag), sorted(e.attrib.items()), (e.text or '').strip(), [_tree(c) for c in e])
