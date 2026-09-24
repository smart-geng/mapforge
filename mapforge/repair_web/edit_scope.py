"""S1: SHA-bound local edit admission, not a fitter or map exporter.

The registered Line/C2 cubic event is retained. Unselected physical curves
are fixed by whole-polynomial equalities, not a minimum-deformation penalty.
Widths and geometric centres are derived from the same shared boundaries.
No nonzero XML export, knot change or Web transaction is provided here.
"""
from __future__ import annotations

import copy
from dataclasses import asdict, dataclass, field
import math

import numpy as np

from .model import digest, intervals, json_bytes, lanes, parse
from .outer_event_control import ControlPoint, recover_state
from scripts.check_outer_event_written import semantic


class ScopeError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class ScopeHandle:
    point: ControlPoint
    components: tuple[str, ...] = ('position',)


@dataclass(frozen=True)
class EditScopeRequest:
    reference_sha256: str
    road: str
    edges: tuple[int, ...]
    interval: tuple[float, float]
    handles: tuple[ScopeHandle, ...]
    # Dependency acknowledgement is NOT approval to alter source roles.
    acknowledged_dependencies: tuple[str, ...] = ()


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _readonly(value):
    result = np.array(value, dtype=float, copy=True)
    result.setflags(write=False)
    return result


def _svd(matrix):
    """Stable ranks with a bounded scale floor; near-zero rows stay zero."""
    if not np.isfinite(matrix).all():
        raise ScopeError('EDIT_NUMERIC_INVALID', 'Nonfinite control matrix')
    if min(matrix.shape) == 0:
        return np.eye(matrix.shape[1]), [0, 0, 0], 0.
    A = matrix / np.maximum(np.linalg.norm(matrix, axis=1), 1e-6)[:, None]
    _, singular, vh = np.linalg.svd(A, full_matrices=True)
    threshold = max(A.shape)*np.finfo(float).eps*max(1., float(singular[0]))
    ranks = [int(np.sum(singular > threshold*k)) for k in (1., 10., 100.)]
    return vh[ranks[1]:].T, ranks, threshold


def preservation_rows(event, edges, lo, hi):
    """Freeze all four coefficients on each nonempty unselected interval.

Freezing even part of a cubic fixes that whole polynomial. A request ending
inside a span therefore shrinks actual support; it NEVER grows the scope.
These cuts are algebraic checking cells, not new geometry/width records.
"""
    rows, cells = [], []
    for edge, bs in event.splines.items():
        cuts = sorted(set(bs.t) | {s for s in (lo, hi) if min(bs.t) < s < max(bs.t)})
        for a, b in zip(cuts, cuts[1:]):
            if edge in edges and lo <= a and b <= hi:
                continue
            rows.extend(event.power(edge, a)*np.array([1., b-a, (b-a)**2, (b-a)**3])[:, None])
            cells.append(dict(edge=edge, interval_m=[float(a), float(b)]))
    return np.asarray(rows).reshape(-1, event.nvar), cells


def _same_semantics(event, reference):
    """Protect the WHOLE XML, including junctions, not only the parent road."""
    values = []
    for data in (event.data, reference):
        root = parse(data)
        road = root.find(f"road[@id='{event.scope.road}']")
        if road is None:
            return False
        for parent in road.iter():
            for child in list(parent):
                if child.tag in ('width', 'laneOffset'):
                    parent.remove(child)
        values.append(semantic(root))
    return values[0] == values[1]


def _validate(event, reference, request):
    if not isinstance(request, EditScopeRequest) or type(reference) is not bytes:
        raise ScopeError('EDIT_INVALID_REQUEST', 'Typed request and immutable reference bytes required')
    if request.reference_sha256 != digest(reference):
        raise ScopeError('EDIT_STALE_REFERENCE', 'Reference SHA does not match the requested baseline')
    if request.road != event.scope.road:
        raise ScopeError('EDIT_UNSUPPORTED_ROAD', 'No implicit road or chart selection')
    if (type(request.edges) is not tuple or not request.edges
            or any(type(e) is not int or e not in event.splines for e in request.edges)
            or len(set(request.edges)) != len(request.edges)
            or type(request.interval) is not tuple or len(request.interval) != 2
            or not all(_finite(s) for s in request.interval)
            or not event.start <= request.interval[0] < request.interval[1] <= event.end):
        raise ScopeError('EDIT_INVALID_SCOPE', 'Explicit distinct edges and contained finite interval required')
    if (type(request.handles) is not tuple or not request.handles
            or type(request.acknowledged_dependencies) is not tuple
            or any(type(s) is not str for s in request.acknowledged_dependencies)
            or len(set(request.acknowledged_dependencies)) != len(request.acknowledged_dependencies)):
        raise ScopeError('EDIT_INVALID_REQUEST', 'Explicit handles and dependency IDs required')
    seen, rows, identities = set(), [], []
    lo, hi = request.interval
    for handle in request.handles:
        if not isinstance(handle, ScopeHandle) or not isinstance(handle.point, ControlPoint):
            raise ScopeError('EDIT_INVALID_HANDLE', 'Typed source-owned handle required')
        p = handle.point
        if (type(p.edge) is not int or p.edge not in request.edges
                or not _finite(p.station) or not lo < p.station < hi
                or not event.births.get(p.edge, event.start) < p.station < event.end
                or type(p.source_key) is not str or type(p.record) is not int or p.record < 0
                or type(p.part) is not int or p.part < 0
                or type(handle.components) is not tuple or not handle.components
                or any(c not in ('position', 'slope') for c in handle.components)):
            raise ScopeError('EDIT_INVALID_HANDLE', 'Handle must be inside selection, with position or dt/ds slope')
        matches = [t for t in event.traces if (t['key'], t['record'], t['part'], t['edge']) ==
                   (p.source_key, p.record, p.part, p.edge) and t['st'][0, 0] <= p.station <= t['st'][-1, 0]]
        if len(matches) != 1:
            raise ScopeError('EDIT_SOURCE_IDENTITY', 'One exact original boundary occurrence required; no nearest-line fallback')
        for component in handle.components:
            key = (p.edge, p.station, component)
            if key in seen:
                raise ScopeError('EDIT_DUPLICATE_CONTROL', 'Repeated control is not independent capacity')
            seen.add(key)
            rows.append(event.row(p.edge, p.station, 0 if component == 'position' else 1))
            identities.append(dict(**asdict(p), component=component,
                                   units='m' if component == 'position' else 'dt/ds'))
    return np.asarray(rows), identities


def _dependencies(event, request):
    lo, hi = request.interval
    return [dict(id=f'road:{request.road}/birth:{edge}', station_m=s,
                 edges=[edge-1, edge], role='existing zero-width birth, shared C2 jets',
                 source_roles_changed=False)
            for edge, s in event.births.items()
            if lo <= s <= hi and {edge-1, edge}.intersection(request.edges)]


def _support(event, space):
    result = []
    for edge, bs in event.splines.items():
        cuts = sorted(set(bs.t))
        for a, b in zip(cuts, cuts[1:]):
            power = event.power(edge, a)*np.array([1., b-a, (b-a)**2, (b-a)**3])[:, None]
            if space.shape[1] and np.max(abs(power@space)) > 1e-9:
                result.append(dict(edge=edge, interval_m=[float(a), float(b)]))
    return result


def _derived(event, support):
    """Exact laneSection ownership; preserve raw movement observations."""
    result = []
    for sec, a, b in intervals(event.road):
        for lid, lane in lanes(sec).items():
            edge = -lid
            cells = [(max(a, c['interval_m'][0]), min(b, c['interval_m'][1])) for c in support
                     if c['edge'] in (edge-1, edge)]
            cells = sorted({(l, h) for l, h in cells if h > l})
            if not cells:
                continue
            source = [e.get('value') for e in lane.findall('userData') if e.get('code') == 'mapforge.source_lane']
            if len(source) != 1 or source[0] not in event.packet['observations']:
                raise ScopeError('EDIT_SOURCE_IDENTITY', 'Derived lane lacks exact source identity')
            result.append(dict(road=event.scope.road, section_s=a, lane=lid, source_lane_id=source[0],
                               intervals_m=[list(c) for c in cells],
                               updates=['width', 'geometric_center', 'boundary_attached_marks'],
                               movement_observation='UNCHANGED', traffic_topology='UNCHANGED'))
    return result


@dataclass(frozen=True)
class PreparedEditScope:
    reference: bytes
    reference_state: np.ndarray = field(repr=False)
    space: np.ndarray = field(repr=False)
    frozen_matrix: np.ndarray = field(repr=False)
    control_matrix: np.ndarray = field(repr=False)
    _report_json: bytes = field(repr=False)

    @property
    def report(self):
        import json
        return json.loads(self._report_json)

    def replay_noop(self):
        """Always return the exact original bytes, never reserialize a seed."""
        return self.reference


def prepare_edit_scope(event, reference: bytes, request: EditScopeRequest):
    """Build a local linear space. READY never means nonlinear feasibility.

Malformed/stale/source-mismatched inputs raise ScopeError. Well-formed but
unusable scopes return explicit refusal status and no usable space. There is
no export/apply method and no implicit acceptance or widening of edit scope.
"""
    controls, identities = _validate(event, reference, request)
    if not _same_semantics(event, reference):
        raise ScopeError('EDIT_SEMANTIC_DRIFT', 'Only bound event widths/offset may differ from the source baseline')
    try:
        state = recover_state(event, reference)
    except ValueError as exc:
        raise ScopeError('EDIT_REFERENCE_NOT_ADMITTED', str(exc)) from exc
    lo, hi = request.interval
    F, frozen = preservation_rows(event, request.edges, lo, hi)
    null, ranks, threshold = _svd(F@event.Z)
    space = event.Z@null
    control_space = controls@space
    _, control_ranks, _ = _svd(control_space)
    dependencies = _dependencies(event, request)
    required = {d['id'] for d in dependencies}
    acknowledged = set(request.acknowledged_dependencies)
    if not acknowledged <= required:
        raise ScopeError('EDIT_UNKNOWN_DEPENDENCY', 'Acknowledgement cannot add unrelated events or source roles')
    missing = sorted(required-acknowledged)
    residual = float(np.max(abs(F@space))) if F.size and space.size else 0.
    original_residual = float(np.max(abs(event.E@space))) if space.size else 0.
    support = _support(event, space)
    status = 'EDIT_SCOPE_READY_NOT_FEASIBILITY'
    reasons = []
    if len(set(ranks)) != 1 or len(set(control_ranks)) != 1 or max(residual, original_residual) > 1e-8:
        status = 'EDIT_NUMERIC_UNCERTAIN'
        reasons.append('Control rank or whole-polynomial preservation is numerically uncertain')
    elif missing:
        status = 'EDIT_DEPENDENCY_CONFIRMATION_REQUIRED'
        reasons.append('Explicit birth dependency acknowledgement required; selection is not expanded')
    elif space.shape[1] == 0 or any(np.linalg.norm(r) <= 1e-10 for r in control_space):
        status = 'EDIT_NO_DOF'
        reasons.append('At least one requested control has no response with the stated curves frozen')
    elif control_ranks[1] < len(controls):
        status = 'EDIT_INTENT_NOT_INDEPENDENT'
        reasons.append('Requested point/slope components cannot be independently controlled in this fixed scope')
    # Zero-change algebra must never silently expand the requested domain.
    if any(c['edge'] not in request.edges or c['interval_m'][0] < lo or c['interval_m'][1] > hi for c in support):
        raise ScopeError('EDIT_SCOPE_LEAK', 'A nonzero polynomial escaped the explicit selection')
    for row, response in zip(identities, control_space):
        row['individually_responsive'] = bool(np.linalg.norm(response) > 1e-10)
    report = dict(schema='mapforge/local-edit-scope/v1', status=status, reasons=reasons,
                  request=asdict(request), reference_sha256=digest(reference), baseline_sha256=digest(event.data),
                  source_packet_sha256=digest(json_bytes(event.packet)),
                  representation=dict(chart='fixed-Line', basis='existing-C2-cubic', knots=list(event.scope.knots),
                                      births=list(event.scope.births), inserted_knots=0),
                  selected_edges=list(request.edges), frozen_edges=sorted(set(event.splines)-set(request.edges)),
                  selected_source_occurrences=[copy.deepcopy(t) for t in event.inventory if t['edge'] in request.edges],
                  frozen_source_occurrences=[copy.deepcopy(t) for t in event.inventory if t['edge'] not in request.edges],
                  actual_support=support, frozen_polynomial_cells=frozen,
                  whole_road_outside_event='UNCHANGED_BY_ADMISSION',
                  dependencies=dependencies, missing_acknowledgements=missing,
                  derived_lanes=_derived(event, support),
                  locked_objects=['reference_axis', 'road_ports', 'junctions', 'topology', 'source_observations', 'speeds'],
                  linear_free=space.shape[1], rank_by_tolerance=ranks, rank_threshold=threshold,
                  requested_control_count=len(controls), control_rank=control_ranks[1],
                  control_rank_by_tolerance=control_ranks,
                  shape_free_after_controls=max(0, space.shape[1]-control_ranks[1]), controls=identities,
                  frozen_polynomial_residual=residual, event_constraint_residual=original_residual,
                  source_roles_changed=False, nonlinear_feasibility='NOT_EVALUATED',
                  dynamics='NOT_EVALUATED', map_accepted=False, export_allowed=False,
                  optimizer_calls=0, new_xodr=False,
                  diagnostic_basis_sha256=digest(np.ascontiguousarray(space).tobytes()))
    report['contract_sha256'] = digest(json_bytes(report))
    # Denied requests retain diagnostic capacity but cannot provide a usable
    # basis to downstream code. No numeric state from a refusal may be applied.
    usable = space if status == 'EDIT_SCOPE_READY_NOT_FEASIBILITY' else np.zeros((event.nvar, 0))
    report['usable_basis_sha256'] = digest(np.ascontiguousarray(usable).tobytes())
    # The contract digest includes the usable/refused basis identity.
    report.pop('contract_sha256')
    report['contract_sha256'] = digest(json_bytes(report))
    return PreparedEditScope(reference, _readonly(state), _readonly(usable), _readonly(F),
                             _readonly(controls), json_bytes(report))
