"""S1 real-input contract tests; no source fit, file export or Web mutation."""
from dataclasses import replace
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle, ScopeError, prepare_edit_scope
from mapforge.repair_web.model import digest, json_bytes
from scripts.check_outer_event_control import inputs, POINT
from scripts.check_outer_event_written import boundary_poly, max_abs, semantic


@pytest.fixture(scope='module')
def context():
    event, control = inputs()
    request = EditScopeRequest(control.reference_sha256, '11', (3,),
                               (event.scope.knots[5], event.end), (ScopeHandle(POINT),))
    return event, control.reference, request


@pytest.fixture(scope='module')
def prepared(context):
    return prepare_edit_scope(*context)


def test_ready_is_capacity_not_feasibility_or_map(prepared):
    report = prepared.report
    assert report['status'] == 'EDIT_SCOPE_READY_NOT_FEASIBILITY'
    assert report['linear_free'] == report['control_rank'] == 1
    assert report['shape_free_after_controls'] == 0
    assert report['nonlinear_feasibility'] == 'NOT_EVALUATED'
    assert not report['map_accepted'] and not report['export_allowed']
    assert report['optimizer_calls'] == 0 and not report['new_xodr']


def test_noop_exact_bytes_and_readonly_state(context, prepared):
    assert prepared.replay_noop() is context[1]
    assert digest(prepared.replay_noop()) == context[2].reference_sha256
    for array in (prepared.space, prepared.reference_state, prepared.frozen_matrix):
        with pytest.raises(ValueError): array.flat[0] = 100
    changed = prepared.report
    changed['status'] = 'MAP_PASS'
    assert prepared.report['status'] != 'MAP_PASS'


def test_whole_unselected_polynomials_fixed(context, prepared):
    event, _, _ = context
    assert prepared.report['frozen_edges'] == [0, 1, 2, 4]
    assert np.max(abs(prepared.frozen_matrix@prepared.space)) < 1e-8
    assert np.max(abs(event.E@prepared.space)) < 1e-8
    for edge in (0, 1, 2, 4):
        for s in sorted(set(event.splines[edge].t))[:-1]:
            assert np.max(abs(event.power(edge, s)@prepared.space)) < 1e-8
    assert all(c['edge'] == 3 and 157.6089 <= c['interval_m'][0] < c['interval_m'][1] <= event.end
               for c in prepared.report['actual_support'])


def test_neighbour_lane_derivation_is_complete_but_not_neighbour_edge_motion(prepared):
    report = prepared.report
    assert {r['lane'] for r in report['derived_lanes']} == {-3, -4}
    assert all(r['source_lane_id'] and r['movement_observation'] == 'UNCHANGED' for r in report['derived_lanes'])
    assert report['source_roles_changed'] is False
    assert 'junctions' in report['locked_objects']
    assert all(t['edge'] == 3 for t in report['selected_source_occurrences'])


def test_three_spans_refused_without_automatic_expansion(context):
    event, ref, request = context
    short = replace(request, interval=(event.scope.knots[6], event.end))
    result = prepare_edit_scope(event, ref, short)
    assert result.report['status'] == 'EDIT_NO_DOF'
    assert result.report['request']['interval'] == list(short.interval)
    assert result.space.shape[1] == 0
    assert result.replay_noop() == ref


def test_point_and_slope_cannot_be_offered_independently_in_one_dof(context):
    event, ref, request = context
    result = prepare_edit_scope(event, ref, replace(request, handles=(ScopeHandle(POINT, ('position', 'slope')),)))
    assert result.report['status'] == 'EDIT_INTENT_NOT_INDEPENDENT'
    assert result.report['requested_control_count'] == 2
    assert result.report['control_rank'] == 1
    assert result.space.shape[1] == 0  # no usable state escapes refusal


def test_two_edges_must_be_explicit_and_derived_lane_union_is_correct(context):
    event, ref, request = context
    trace = next(t for t in event.traces if t['edge'] == 4 and t['st'][0, 0] < 178 < t['st'][-1, 0])
    p = replace(POINT, edge=4, source_key=trace['key'], record=trace['record'], part=trace['part'])
    result = prepare_edit_scope(event, ref, replace(request, edges=(3, 4), handles=(ScopeHandle(POINT), ScopeHandle(p))))
    assert result.report['status'] == 'EDIT_SCOPE_READY_NOT_FEASIBILITY'
    assert result.report['linear_free'] == result.report['control_rank'] == 2
    assert result.report['frozen_edges'] == [0, 1, 2]
    assert {r['lane'] for r in result.report['derived_lanes']} == {-3, -4}


def test_birth_scope_returns_dependencies_without_approving_roles(context):
    event, ref, request = context
    wide = replace(request, interval=(event.start, event.end))
    result = prepare_edit_scope(event, ref, wide)
    assert result.report['status'] == 'EDIT_DEPENDENCY_CONFIRMATION_REQUIRED'
    required = tuple(result.report['missing_acknowledgements'])
    assert set(required) == {'road:11/birth:3', 'road:11/birth:4'}
    assert result.space.shape[1] == 0
    admitted = prepare_edit_scope(event, ref, replace(wide, acknowledged_dependencies=required))
    assert admitted.report['status'] == 'EDIT_SCOPE_READY_NOT_FEASIBILITY'
    assert admitted.report['selected_edges'] == [3]
    assert admitted.report['source_roles_changed'] is False
    assert admitted.report['linear_free'] == 2


@pytest.mark.parametrize('changes,code', [
    (dict(reference_sha256='0'*64), 'EDIT_STALE_REFERENCE'),
    (dict(road='10'), 'EDIT_UNSUPPORTED_ROAD'),
    (dict(edges=(True,)), 'EDIT_INVALID_SCOPE'),
    (dict(edges=(3, 3)), 'EDIT_INVALID_SCOPE'),
    (dict(edges=(7,)), 'EDIT_INVALID_SCOPE'),
    (dict(edges=()), 'EDIT_INVALID_SCOPE'),
    (dict(interval=(90., 201.7074107)), 'EDIT_INVALID_SCOPE'),
    (dict(interval=(160., float('nan'))), 'EDIT_INVALID_SCOPE'),
    (dict(interval=(160., float('inf'))), 'EDIT_INVALID_SCOPE'),
    (dict(interval=(180., 170.)), 'EDIT_INVALID_SCOPE'),
    (dict(handles=()), 'EDIT_INVALID_REQUEST'),
    (dict(handles=(ScopeHandle(POINT, ('position', 'position')),)), 'EDIT_DUPLICATE_CONTROL'),
    (dict(handles=(ScopeHandle(POINT, ('heading',)),)), 'EDIT_INVALID_HANDLE'),
    (dict(acknowledged_dependencies=('approve-new-source-role',)), 'EDIT_UNKNOWN_DEPENDENCY'),
])
def test_malformed_and_unapproved_scope_rejected(context, changes, code):
    event, ref, request = context
    with pytest.raises(ScopeError) as exc:
        prepare_edit_scope(event, ref, replace(request, **changes))
    assert exc.value.code == code


@pytest.mark.parametrize('changes,code', [
    (dict(edge=2), 'EDIT_INVALID_HANDLE'),
    (dict(record=True), 'EDIT_INVALID_HANDLE'),
    (dict(part=-1), 'EDIT_INVALID_HANDLE'),
    (dict(station=157.6089), 'EDIT_INVALID_HANDLE'),
    (dict(source_key='wrong'), 'EDIT_SOURCE_IDENTITY'),
])
def test_handle_not_reassigned_to_nearest_source(context, changes, code):
    event, ref, request = context
    with pytest.raises(ScopeError) as exc:
        prepare_edit_scope(event, ref, replace(request, handles=(ScopeHandle(replace(POINT, **changes)),)))
    assert exc.value.code == code


def test_whole_document_semantics_checked_including_junction(context):
    event, ref, request = context
    root = ET.fromstring(ref)
    junction = root.find('junction')
    assert junction is not None
    junction.set('name', 'tampered')
    changed = ET.tostring(root)
    with pytest.raises(ScopeError) as exc:
        prepare_edit_scope(event, changed, replace(request, reference_sha256=digest(changed)))
    assert exc.value.code == 'EDIT_SEMANTIC_DRIFT'


def test_hidden_short_width_bump_cannot_be_a_new_baseline(context):
    event, ref, request = context
    root = ET.fromstring(ref)
    road = root.find("road[@id='11']")
    sec = max((s for s in road.findall('lanes/laneSection') if float(s.get('s')) <= 177.123), key=lambda s: float(s.get('s')))
    ET.SubElement(sec.find("right/lane[@id='-3']"), 'width', sOffset=str(177.123-float(sec.get('s'))), a='2', b='0', c='0', d='0')
    changed = ET.tostring(root)
    with pytest.raises(ScopeError) as exc:
        prepare_edit_scope(event, changed, replace(request, reference_sha256=digest(changed)))
    assert exc.value.code == 'EDIT_REFERENCE_NOT_ADMITTED'


def test_no_optimizers_called_and_source_model_unchanged(context, monkeypatch):
    import mapforge.repair_web.outer_event as module
    event, ref, request = context
    before = digest(json_bytes(event.packet)), event.data, event.E.copy(), event.Z.copy()
    def forbidden(*args, **kwargs):
        raise AssertionError('S1 must not solve or fit a new shape')
    monkeypatch.setattr(module, 'minimize', forbidden)
    monkeypatch.setattr(module, 'linprog', forbidden)
    monkeypatch.setattr(event, 'solve', forbidden)
    result = prepare_edit_scope(event, ref, request)
    assert result.report['status'] == 'EDIT_SCOPE_READY_NOT_FEASIBILITY'
    assert digest(json_bytes(event.packet)) == before[0] and event.data == before[1]
    np.testing.assert_array_equal(event.E, before[2])
    np.testing.assert_array_equal(event.Z, before[3])


def test_float_microperturbation_roundtrip_preserves_frozen_curves_in_memory_only(context, prepared):
    # Algebra/serializer regression, not a proposed repair. No XML is saved.
    event, ref, request = context
    direction = prepared.space[:, 0]
    changed = event.compile(prepared.reference_state + direction*1e-7)
    old, new = ET.fromstring(ref), ET.fromstring(changed)
    before, after = (root.find("road[@id='11']") for root in (old, new))
    cuts = {0., float(before.get('length')), *event.scope.knots, *request.interval}
    for road in (before, after):
        cuts.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
        for sec in road.findall('lanes/laneSection'):
            s = float(sec.get('s')); cuts.add(s)
            cuts.update(s+float(w.get('sOffset')) for w in sec.findall('.//width'))
    worst_frozen, moved = 0., 0.
    for a, b in zip(sorted(cuts), sorted(cuts)[1:]):
        for edge in range(5):
            delta = boundary_poly(after, edge, a)-boundary_poly(before, edge, a)
            error = max_abs(delta, b-a)
            if edge != 3 or a < request.interval[0] or b > request.interval[1]:
                worst_frozen = max(worst_frozen, error)
            else:
                moved = max(moved, error)
    assert worst_frozen < 1e-8 and moved > 1e-8
    for a, b in zip(old.findall('road'), new.findall('road')):
        if a.get('id') != '11': assert semantic(a) == semantic(b)
    assert [semantic(j) for j in old.findall('junction')] == [semantic(j) for j in new.findall('junction')]


def test_rank_uncertainty_does_not_return_usable_space(context, monkeypatch):
    import mapforge.repair_web.edit_scope as module
    original = module._svd
    def unstable(matrix):
        null, ranks, threshold = original(matrix)
        return null, [ranks[0], ranks[1], ranks[2]+1], threshold
    monkeypatch.setattr(module, '_svd', unstable)
    result = prepare_edit_scope(*context)
    assert result.report['status'] == 'EDIT_NUMERIC_UNCERTAIN'
    assert result.space.shape[1] == 0


def test_partial_cubic_freeze_reduces_support_without_moving_knots(context):
    event, ref, request = context
    partial = replace(request, interval=(request.interval[0]+.1, request.interval[1]))
    result = prepare_edit_scope(event, ref, partial)
    assert result.report['status'] == 'EDIT_NO_DOF'
    assert result.report['representation']['knots'] == list(event.scope.knots)
    assert result.report['representation']['inserted_knots'] == 0
    assert result.report['request']['interval'] == list(partial.interval)


def test_deterministic_contract_and_complete_source_partition(context, prepared):
    again = prepare_edit_scope(*context)
    assert again.report == prepared.report
    np.testing.assert_array_equal(again.space, prepared.space)
    report = again.report
    fingerprint = report.pop('contract_sha256')
    assert digest(json_bytes(report)) == fingerprint
    assert len(report['selected_source_occurrences'])+len(report['frozen_source_occurrences']) == len(context[0].inventory)
    assert digest(np.ascontiguousarray(again.space).tobytes()) == report['usable_basis_sha256']


def test_all_derived_sections_and_width_center_rows_follow_shared_edge(context, prepared):
    from mapforge.repair_web.model import intervals, lanes
    event = context[0]
    reported = {(row['section_s'], row['lane']) for row in prepared.report['derived_lanes']}
    measured = set()
    for sec, lo, hi in intervals(event.road):
        a, b = max(lo, context[2].interval[0]), min(hi, context[2].interval[1])
        if b <= a: continue
        for lid in lanes(sec):
            s = (a+b)/2; edge = -lid
            width = (event.row(edge-1, s)-event.row(edge, s))@prepared.space
            center = (event.row(edge-1, s)+event.row(edge, s))@prepared.space/2
            if max(np.linalg.norm(width), np.linalg.norm(center)) > 1e-9:
                measured.add((lo, lid))
    assert reported == measured and {lid for _, lid in measured} == {-3, -4}
