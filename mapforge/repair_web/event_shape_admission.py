"""E2 necessary-condition checks, never a geometry fitter or a new tolerance.

An immutable dependency containing a violating actual edge point cannot be
repaired by changing only its parent. Distances here use the complete SAME
original feature, not a cropped tail or an unrelated nearest boundary.
"""
import math

import numpy as np

from mapforge.ops.port_dependencies import PortDependencies
from mapforge.ops.reconstruction_scope import digest as object_digest
from mapforge.repair_web.model import digest, parse
from mapforge.repair_web.split_event_admission import (
    checked_payload, collect_sources, partition_rows, START, END, CONNECTORS, REFERENCE_SHA,
)
from scripts.internal_edge_jets import states


TAIL_BUDGET_M = .35  # Existing final ordinary-tail bound; not via P95 or L01 .75.
NUMERIC_ALLOWANCE_M = 1e-7  # Floating-point comparison allowance, not source accuracy.


def point_to_original(point, polyline):
    """Point to every finite segment of one already identity-bound raw part."""
    point = np.asarray(point, float); raw = np.asarray(polyline, float)
    if (point.shape != (2,) or raw.ndim != 2 or raw.shape[1] != 2 or len(raw) < 2
            or not np.isfinite(point).all() or not np.isfinite(raw).all()):
        raise ValueError('Finite point and full original planar part required')
    delta = np.diff(raw, axis=0); lengths2 = np.sum(delta*delta, axis=1)
    if np.any(lengths2 <= 0): raise ValueError('Degenerate original segment')
    u = np.clip(np.sum((point-raw[:-1])*delta, axis=1)/lengths2, 0., 1.)
    feet = raw[:-1]+u[:, None]*delta
    distance = np.linalg.norm(point-feet, axis=1); i = int(np.argmin(distance))
    return dict(distance_m=float(distance[i]), source_segment=i,
                segment_fraction=float(u[i]), closest_original_st=feet[i].tolist())


def check_frozen_points(points, original, budget):
    """A found violation is evidence; finite points cannot establish PASS."""
    if not math.isfinite(budget) or budget <= 0: raise ValueError('Invalid budget')
    rows = [dict(point_st=np.asarray(p, float).tolist(), **point_to_original(p, original)) for p in points]
    if not rows: raise ValueError('No actual finite witness points')
    worst = max(rows, key=lambda r: r['distance_m'])
    failed = worst['distance_m'] > budget+NUMERIC_ALLOWANCE_M
    return dict(status='FIXED_POINT_VIOLATES_SOURCE_BOUND' if failed else 'NO_VIOLATION_FOUND_NOT_PASS',
                worst=worst, budget_m=budget, exceeds_m=max(0., worst['distance_m']-budget),
                continuous_maximum_proven=False, whole_interval_pass=False)


def dependency_witnesses(reference, packet, domain, source_binding, *, step_m=.25):
    if digest(reference) != REFERENCE_SHA: raise ValueError('Unreviewed XML revision')
    for payload in (packet, domain, source_binding): checked_payload(payload)
    if (domain['scope_sha256'] != packet['content_sha256']
            or source_binding['reference_sha256'] != REFERENCE_SHA
            or source_binding['packet_sha256'] != packet['content_sha256']
            or source_binding['domain_sha256'] != domain['content_sha256']
            or source_binding['source_relation_status'] != 'BOUND'
            or source_binding['source_constraint_status'] != 'READY_FOR_SHAPE_DESIGN'):
        raise ValueError('Source binding is not the reviewed event contract')
    if not math.isfinite(step_m) or step_m <= 0 or step_m > .5:
        raise ValueError('Invalid diagnostic step; not an output curve interval')
    root = parse(reference); parent, features = collect_sources(root, packet)
    rows = partition_rows(features, domain['partition'])
    graph = PortDependencies(root); graph.validate_junction_table()
    g = parent.find('planView/geometry'); h = float(g.get('hdg'))
    origin = np.array([float(g.get('x')), float(g.get('y'))])
    basis = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    result = []
    for row in rows:
        if row['kind'] != 'physical_boundary' or row['parent_s_range'][0] < END-1e-7: continue
        raw = features[row['feature']]['parts_st'][row['part']]
        if np.any(np.diff(raw[:, 0]) <= 0): raise ValueError('Unsupported original chart direction')
        for consumer in row['consumers']:
            if not consumer.startswith('connector:'): raise ValueError('Tail consumer is not a connector')
            cid = consumer.split(':')[1]
            if cid not in CONNECTORS: raise ValueError('Unreviewed dependent')
            port = graph.connections[cid][0]
            if port.road != '11' or port.contact != 'end': raise ValueError('Not the reviewed incoming port')
            side = 0 if row['edge'] == -port.lane-1 else 1 if row['edge'] == -port.lane else None
            if side is None: raise ValueError('Source identity does not own this actual side')
            road = graph.roads[cid]; length = float(road.get('length'))
            sections = road.findall('lanes/laneSection')
            if (len(sections) != 1 or sections[0].findall('left/lane')
                    or [l.get('id') for l in sections[0].findall('right/lane')] != ['-1']):
                raise ValueError('Explicit single-right-lane connector required')
            stop = float(raw[-1, 0]); actual = []; reached_end_plane = False; last_s = None
            for s in np.r_[np.arange(0., length, step_m), length]:
                # Evaluate exact reference primitive + width/offset at s. No
                # interpolation of a rendered mesh or sampled target polyline.
                world = np.asarray(states(road, -1, float(s), False)[side][:2])
                st = (world-origin)@basis
                if last_s is not None and st[0] < last_s-1e-7:
                    raise ValueError('Connector prefix reversed before source end; explicit admission required')
                last_s = float(st[0])
                if st[0] > stop+1e-9:
                    reached_end_plane = True; break
                if st[0] < END-1e-7: raise ValueError('Written prefix starts behind the shared mouth')
                actual.append(dict(connector_s_m=float(s), st=st.tolist(), xy=world.tolist()))
            if not reached_end_plane or not actual: raise ValueError('Incomplete finite prefix support')
            check = check_frozen_points([p['st'] for p in actual], raw, TAIL_BUDGET_M)
            worst = check['worst']; i = next(i for i, p in enumerate(actual) if p['st'] == worst['point_st'])
            result.append(dict(feature=row['feature'], part=row['part'], edge=row['edge'], consumer=consumer,
                field='left' if side == 0 else 'right', source_arclength_m=row['source_arclength_m'],
                original_full_part_st=raw.tolist(), actual_prefix_points=actual,
                witness=dict(**actual[i], **worst), **{k: v for k, v in check.items() if k != 'worst'},
                source_not_cropped=True, independent_movement_tested=False))
    if len(result) != 12 or {r['consumer'] for r in result} != {'connector:'+c for c in CONNECTORS}:
        raise ValueError('Incomplete six-connector physical-edge inventory')
    return result


def review(reference, packet, domain, source_binding):
    rows = dependency_witnesses(reference, packet, domain, source_binding)
    failures = [r for r in rows if r['status'] == 'FIXED_POINT_VIOLATES_SOURCE_BOUND']
    roads = sorted({r['consumer'].split(':')[1] for r in failures}, key=int)
    report = dict(schema='mapforge/event-shape-admission/v1',
        status='BLOCKED_FIXED_DEPENDENCY_SOURCE_CONFLICT' if failures else 'E2_SHAPE_CONTRACT_NOT_YET_ADMITTED',
        reference_sha256=digest(reference), source_binding_sha256=source_binding['content_sha256'],
        proposed_edit_domain=[START, END], mutable_edges=[0, 1, 2, 3, 4],
        frozen_connectors=list(CONNECTORS), frozen_mouth=True, witnesses=rows,
        failed_edge_consumer_count=len(failures), failed_connectors=roads,
        tail_budget_m=TAIL_BUDGET_M,
        budget_origin='existing ordinary-tail final maximum in joint_connector_fit; not via percentile or L01 research envelope',
        constraint_argument='At least one unchanged actual edge point exceeds its complete same-source boundary budget. Parent-only edits cannot change that point.',
        shape_representation_proposal=dict(reference='existing one-Line chart; no change authorized',
            physical_geometry='five shared C2 cubic boundary functions; laneOffset=B0; width_i=B(i-1)-Bi',
            births='two existing stations and prior source roles unchanged; zero width with linked jets',
            knot_policy='few registered long feature intervals, not one knot per sample; no layout selected while frozen evidence fails',
            independent_minimum_span_m=6., output_record_policy='audit semantic re-origin records separately from independent geometry DOFs',
            source_support='retain 35 bound intervals, 4 finite endpoint observations, 19 independent paths, assigned tails',
            fidelity='full source segments plus reverse checks; raw sparse chord slope is not a surveyed tangent',
            shape_intent='full split/shift/settling trend, explicit per-feature extrema and reversal limits before solving',
            old_failures='retain original-domain guards, s178 regression, dynamics failures and missing operating cases',
            writer='one shared boundary model compiled to actual width/offset; zero edit returns original XML bytes'),
        unresolved_before_trial=['dependency edit scope requires explicit decision',
            'per-feature shape intent/bounds and long layout not yet admitted',
            'remaining anchor/fidelity and whole-surface acceptance'],
        next_scope_proposal=dict(connectors=list(CONNECTORS), parent='11',
            purpose='joint source-faithful long-curve reconstruction, not six independent patches',
            mouth_movement_authorized=False, keep_mouth_fixed_first=True,
            source_role_changes=0, status='PROPOSAL_NOT_AUTHORIZED'),
        solver_calls=0, new_xodr=False, new_knots=0, web_changed=False, map_accepted=False,
        formal_certificate=False, all_source_coverage_accepted=False)
    report['content_sha256'] = object_digest(report)
    return report
