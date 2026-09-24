"""S2: evaluate the unique shape of one admitted scalar position request.

No layout search, optimisation, target clipping or guard relaxation. A failed
shape is retained as numeric evidence, never compiled through a bypass.
"""
import math
import time

import numpy as np

from .edit_scope import ScopeError, preservation_rows, _same_semantics
from .model import digest, json_bytes, parse
from .outer_event_shape import reversal_support
from mapforge.validate.line_boundary_shape import span_shape, written_boundary_shape
from scripts.check_outer_event_written import boundary_poly, max_abs, check


METRICS = ('max_abs_curvature', 'max_abs_curvature_rate', 'curvature_total_variation')
METRIC_EPS = np.array([1e-9, 1e-9, 1e-8])


def unique_position_state(reference, space, handle, target):
    """An algebraic equality, not a least-squares surrogate for a target."""
    reference, space, handle = map(lambda a: np.asarray(a, float), (reference, space, handle))
    if (reference.ndim != 1 or space.shape != (len(reference), 1) or handle.shape != reference.shape
            or not all(np.isfinite(a).all() for a in (reference, space, handle))
            or type(target) not in (int, float) or not math.isfinite(target)):
        raise ScopeError('EDIT_NOT_UNIQUE', 'One finite scalar control in a one-dimensional scope required')
    response = float(handle@space[:, 0])
    if abs(response) <= 1e-10:
        raise ScopeError('EDIT_NO_DOF', 'Position has no reliable response')
    parameter = (target-float(handle@reference))/response
    state = reference+space[:, 0]*parameter
    if not np.isfinite(state).all() or abs(float(handle@state)-target) > 1e-8:
        raise ScopeError('EDIT_NUMERIC_UNCERTAIN', 'Unique target equality failed')
    return state, parameter


def shape_metrics(event, state):
    result = []
    for edge, bs in event.splines.items():
        cuts = sorted(set(bs.t))
        rows = [dict(edge=edge, interval_m=[float(a), float(b)],
                     **span_shape(event.power(edge, a)@state, b-a)) for a, b in zip(cuts, cuts[1:])]
        jumps = sum(abs(a['curvature_end']-b['curvature_start']) for a, b in zip(rows, rows[1:]))
        result.append(dict(edge=edge, max_abs_curvature=max(r['max_abs_curvature'] for r in rows),
                           max_abs_curvature_rate=max(r['max_abs_curvature_rate'] for r in rows),
                           curvature_total_variation=sum(r['curvature_variation'] for r in rows)+jumps,
                           join_jump_sum=jumps, intervals=rows))
    return result


def width_metrics(event, state):
    result = []
    for edge in range(1, event.count+1):
        lo = event.births.get(edge, event.start)
        cuts = sorted({lo, event.end} | {s for s in event.scope.knots if lo < s < event.end})
        for a, b in zip(cuts, cuts[1:]):
            c = (event.power(edge-1, a)-event.power(edge, a))@state
            roots = np.polynomial.polynomial.polyroots([c[1], 2*c[2], 3*c[3]])
            points = [0., b-a]+[float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < b-a]
            at = min(points, key=lambda s: np.polynomial.polynomial.polyval(s, c))
            result.append(dict(lane=-edge, interval_m=[a, b], minimum_width_m=float(np.polynomial.polynomial.polyval(at, c)),
                               witness_s=a+at))
    return result


def frozen_readback(data, reference, event, report):
    """Whole actual XML polynomial intervals, not the control-space matrix."""
    if not _same_semantics(event, data):
        raise ScopeError('EDIT_SEMANTIC_DRIFT', 'Written XML changed unrelated semantics')
    roads = [parse(d).find(f"road[@id='{event.scope.road}']") for d in (data, reference)]
    lo, hi = report['request']['interval']; selected = report['selected_edges']
    cuts = {0., float(roads[0].get('length')), lo, hi}
    for road in roads:
        cuts.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
        for sec in road.findall('lanes/laneSection'):
            s = float(sec.get('s')); cuts.add(s)
            cuts.update(s+float(w.get('sOffset')) for w in sec.findall('.//width'))
    errors = []
    for a, b in zip(sorted(cuts), sorted(cuts)[1:]):
        for edge in range(event.count+1):
            if edge in selected and lo <= a and b <= hi:
                continue
            errors.append(max_abs(boundary_poly(roads[0], edge, a)-boundary_poly(roads[1], edge, a), b-a))
    return max(errors, default=0.)


def evaluate_unique_edit(control, prepared, displacement, *, max_seconds=90.):
    """Retain old guard values; only the representation's local space changes.

Returns (state, report, xml_or_none). A state can be rejected; callers must
not treat its presence as export permission. The report distinguishes source
conditions from research nonregression caps and operating-dynamics gaps.
"""
    started = time.monotonic(); event = control.event; scope = prepared.report
    if type(displacement) not in (int, float) or not math.isfinite(displacement) or abs(displacement) > 2:
        raise ScopeError('EDIT_INVALID_TARGET', 'Finite absolute displacement within original two-metre control cap required')
    if type(max_seconds) not in (int, float) or not 0 < max_seconds <= 90:
        raise ScopeError('EDIT_INVALID_BUDGET', 'Original maximum 90-second total budget required')
    record = dict(scope); identity = record.pop('contract_sha256')
    if (digest(json_bytes(record)) != identity or scope['status'] != 'EDIT_SCOPE_READY_NOT_FEASIBILITY'
            or scope['linear_free'] != 1 or scope['control_rank'] != 1 or len(scope['controls']) != 1
            or scope['controls'][0]['component'] != 'position' or scope['missing_acknowledgements']):
        raise ScopeError('EDIT_NOT_UNIQUE', 'An admitted single-position S1 contract is required')
    p = control.point; owned = scope['controls'][0]
    if any(owned[k] != getattr(p, k) for k in ('source_key', 'record', 'part', 'edge', 'station')):
        raise ScopeError('EDIT_SOURCE_IDENTITY', 'S2 cannot replace the S1 source handle')
    F, _ = preservation_rows(event, scope['selected_edges'], *scope['request']['interval'])
    if (scope['reference_sha256'] != control.reference_sha256 or prepared.reference != control.reference
            or scope['baseline_sha256'] != digest(event.data)
            or scope['source_packet_sha256'] != digest(json_bytes(event.packet))
            or scope['representation']['knots'] != list(event.scope.knots)
            or scope['usable_basis_sha256'] != digest(np.ascontiguousarray(prepared.space).tobytes())
            or not np.array_equal(prepared.frozen_matrix, F)
            or not np.array_equal(prepared.control_matrix, control.handle[None, :])
            or not np.allclose(prepared.reference_state, control.current, rtol=0., atol=1e-9)):
        raise ScopeError('EDIT_STALE_REFERENCE', 'Source, representation, baseline or S1 space drift')
    target = float(control.reference_t+displacement)
    state, parameter = unique_position_state(prepared.reference_state, prepared.space, control.handle, target)
    source = event.source_error(state)
    widths = width_metrics(event, state)
    shapes = shape_metrics(event, state)
    reference = written_boundary_shape(control.reference, event.scope.__dict__)
    caps = np.array([[reference['by_edge'][str(e)][k] for k in METRICS] for e in range(event.count+1)])
    actual = np.array([[r[k] for k in METRICS] for r in shapes])
    reversal, _ = reversal_support(event, state)
    _, _, C, low, high, source_count, _, _ = event.linear_model()
    value = C@state
    linear = float(max(0., np.max(low-value), np.max(value-high)))
    bernstein_violation = float(max(0., np.max(low[source_count:]-value[source_count:])))
    frozen = float(np.max(abs(F@(state-prepared.reference_state)))) if F.size else 0.
    equality = float(np.max(abs(event.E@state-event.e)))
    failures = []
    def fail(code, origin, value, limit):
        failures.append(dict(code=code, origin=origin, observed=float(value), limit=float(limit)))
    if source['max_m'] > event.scope.source_tolerance_m+1e-7:
        fail('SOURCE_ENVELOPE', 'existing_L01_source_policy', source['max_m'], event.scope.source_tolerance_m)
    if min(r['minimum_width_m'] for r in widths) < -1e-7:
        fail('NEGATIVE_WIDTH', 'physical_width', min(r['minimum_width_m'] for r in widths), 0.)
    if bernstein_violation > 1e-7:
        fail('WIDTH_BERNSTEIN_GUARD', 'existing_conservative_width_guard', bernstein_violation, 0.)
    if frozen > 1e-8:
        fail('FROZEN_CURVE_CHANGE', 'S1_explicit_scope', frozen, 1e-8)
    if equality > 1e-7:
        fail('EVENT_EQUALITIES', 'registered_ports_and_births', equality, 1e-7)
    for edge in range(event.count+1):
        for j, metric in enumerate(METRICS):
            if actual[edge, j] > caps[edge, j]+METRIC_EPS[j]:
                fail(f'EDGE_{edge}_{metric}', 'research_reference_nonregression', actual[edge, j], caps[edge, j])
        if reversal[edge] > control.budgets[edge]+1e-7:
            fail(f'EDGE_{edge}_SOURCE_DIRECTION_REVERSAL', 'research_reference_nonregression', reversal[edge], control.budgets[edge])
    # Preserve the original sampled source/whole-span-width guard too. This is
    # separate from exact source extrema, never a replacement for them.
    if linear > 1e-7 and not any(r['code'] in ('SOURCE_ENVELOPE', 'WIDTH_BERNSTEIN_GUARD') for r in failures):
        fail('ORIGINAL_LINEAR_GUARD', 'existing_L01_guard', linear, 1e-7)
    if time.monotonic()-started > max_seconds:
        fail('TIME_BUDGET', 'registered_execution_budget', time.monotonic()-started, max_seconds)
    data = None; readback = None
    if not failures:
        data = control.reference if displacement == 0 else event.compile(state)
        readback = check(data, event.data, event.packet, event.scope.__dict__, event.inventory)
        actual_frozen = frozen_readback(data, control.reference, event, scope)
        written = written_boundary_shape(data, event.scope.__dict__)
        written_values = np.array([[written['by_edge'][str(e)][k] for k in METRICS] for e in range(event.count+1)])
        road = parse(data).find(f"road[@id='{event.scope.road}']")
        written_target = float(boundary_poly(road, p.edge, p.station)[0])
        if (readback['status'] != 'PASS_LOCAL_EVENT_NOT_MAP' or actual_frozen > 1e-8
                or abs(written_target-target) > 1e-8 or np.any(written_values > caps+METRIC_EPS)):
            fail('WRITTEN_READBACK_REJECTED', 'actual_XML', 1., 0.)
            data = None
        readback.update(frozen_scope_max_m=actual_frozen, target_error_m=abs(written_target-target), world_shape=written)
    if time.monotonic()-started > max_seconds and not any(r['code'] == 'TIME_BUDGET' for r in failures):
        fail('TIME_BUDGET', 'registered_execution_budget', time.monotonic()-started, max_seconds)
        data = None
    report = dict(schema='mapforge/unique-local-edit-check/v1',
                  status='UNIQUE_TARGET_REJECTED' if failures else 'UNIQUE_LOCAL_CANDIDATE_NOT_MAP',
                  scope_contract_sha256=identity, reference_sha256=control.reference_sha256,
                  requested_m=displacement, target_t_m=target, achieved_m=float(control.handle@state-control.reference_t),
                  target_error_m=abs(float(control.handle@state)-target), parameter=parameter,
                  linear_free_before=1, shape_free_after_position=0, optimizer_calls=0, iterations=0,
                  candidate_count=1, inserted_knots=0, elapsed_seconds=time.monotonic()-started,
                  source=source, widths=widths, shape_by_edge=shapes, reference_caps=caps.tolist(),
                  reversal_by_edge_m=reversal.tolist(), reference_reversal_m=control.budgets.tolist(),
                  frozen_polynomial_change=frozen, event_equalities_residual=equality,
                  linear_violation=linear, width_bernstein_violation=bernstein_violation,
                  failures=failures, readback=readback, candidate_xml_sha256=None if data is None else digest(data),
                  map_accepted=False, operating_dynamics='NOT_EVALUATED', formal_certificate=False,
                  decision_scope='one fixed representation, explicit scope, exact target and unchanged guard set; not general impossibility')
    return state, report, data
