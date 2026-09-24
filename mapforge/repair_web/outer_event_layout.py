"""Finite source-anchored LONG layout candidates, never implicit map migration.

A moved knot generally cannot represent the old piecewise cubic exactly.
The original XML therefore remains the reference/undo/no-op value. Projection
only supplies coefficients to a separate candidate solver; it is NOT a map.
Only the existing Line-chart event is admitted, no road/port/source movement.
"""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from .model import parse
from .outer_event import OuterEvent
from .outer_event_control import _road
from .outer_event_fairing import CurvatureFairing
from scripts.check_outer_event_written import boundary_poly


def xml_cuts(road, start, end):
    values = {start, end}
    values.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
    for section in road.findall('lanes/laneSection'):
        s = float(section.get('s')); values.add(s)
        values.update(s+float(e.get('sOffset')) for e in section.findall('.//width'))
    return sorted(s for s in values if start <= s <= end)


def reference_objective(event, reference):
    """Exact piecewise degree-six quadrature against ORIGINAL written geometry.

Split integration at the union of old XML and new layout breaks. These are
integration cells ONLY, never new independent or written geometry knots.
"""
    road = _road(reference, event)
    nodes, weights = np.polynomial.legendre.leggauss(4)
    rows, rhs = [], []
    for edge, bs in event.splines.items():
        start, end = min(bs.t), max(bs.t)
        cuts = sorted(set(xml_cuts(road, start, end)) | set(bs.t))
        for lo, hi in zip(cuts, cuts[1:]):
            c = boundary_poly(road, edge, lo)
            for u, w in zip((nodes+1)*(hi-lo)/2, weights*(hi-lo)/2):
                for derivative, scale in ((0, 1.), (1, 10.)):
                    rows.append(event.row(edge, lo+u, derivative)*np.sqrt(w)*scale)
                    value = np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(c, derivative))
                    rhs.append(float(value*np.sqrt(w)*scale))
    return np.array(rows), np.array(rhs)


def migration_error(event, reference, state):
    """Whole-cell max difference to original XML; no sparse sample shortcut."""
    road = _road(reference, event); rows = []
    for edge, bs in event.splines.items():
        cuts = sorted(set(xml_cuts(road, min(bs.t), max(bs.t))) | set(bs.t))
        worst = 0.; witness = None
        for lo, hi in zip(cuts, cuts[1:]):
            c = event.power(edge, lo)@state-boundary_poly(road, edge, lo)
            roots = np.polynomial.polynomial.polyroots([c[1], 2*c[2], 3*c[3]])
            points = [0., hi-lo]+[float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<hi-lo]
            at = max(points, key=lambda u: abs(np.polynomial.polynomial.polyval(u, c)))
            error = abs(float(np.polynomial.polynomial.polyval(at, c)))
            if error > worst: worst, witness = error, lo+at
        rows.append(dict(edge=edge, max_m=worst, witness_s=witness))
    return dict(max_m=max(r['max_m'] for r in rows), by_edge=rows,
                scope='whole original XML/new-layout cell intersection; floating extrema, not formal proof')


def proposals(control):
    """Two deterministic alternatives, same knot count and min 6m separation.

Only the LAST TWO nonsemantic knots may change. Candidate one allocates long
support around the source plateau's end; candidate two balances tail spans.
No per-source-point insertion, random starts or hidden parent/port movement.
"""
    old = control.event.scope; knots = list(old.knots)
    if len(knots) != 10 or old.minimum_span_m != 6.:
        raise ValueError('Only the registered ten-knot long event is admitted')
    s = control.point.station
    vertices = control.trace['st'][:, 0]
    right = vertices[vertices>s]
    if not len(right): raise ValueError('Original target source lacks a following vertex')
    fixed = knots[-4]; end = knots[-1]; gap = old.minimum_span_m
    plateau_end = float(right[0])
    first = knots.copy()
    # First movable break is a long-support location, not another source node.
    first[-3] = fixed+gap
    first[-2] = plateau_end
    second = knots.copy()
    second[-3:] = [fixed+(end-fixed)/3, fixed+2*(end-fixed)/3, end]
    result = []
    for name, candidate in (('source-plateau-long-support', first), ('balanced-long-support', second)):
        if min(np.diff(candidate)) < gap-1e-10: raise ValueError('Proposed long span is too short')
        if any(s not in candidate for _,s in old.births): raise ValueError('Birth anchor moved')
        result.append((name, replace(old, knots=tuple(candidate))))
    return result


class LayoutFairing(CurvatureFairing):
    """Rebuild sources/basis/ports together; original XML stays immutable."""
    def __init__(self, original_control, scope, *, approved_roles):
        original = original_control.event
        # This admission is intentionally narrower than a general layout editor.
        # Existing lane births and interval scope are not parameterisation knobs.
        if (scope.road != original.scope.road or scope.births != original.scope.births
                or scope.knots[:7] != original.scope.knots[:7]
                or scope.knots[-1] != original.end or len(scope.knots) != len(original.scope.knots)
                or scope.minimum_span_m != original.scope.minimum_span_m
                or scope.source_tolerance_m != original.scope.source_tolerance_m
                or not np.isfinite(scope.knots).all()):
            raise ValueError('Only two nonsemantic long tail knots may move')
        event = OuterEvent(original.data, original.packet, scope, approved_roles=approved_roles)
        if event.nvar != original.nvar or event.inventory != original.inventory or event.paths != original.paths:
            raise ValueError('Layout changed source ownership, movement or number of variables')
        if event.axis != original.axis:
            # ArcChart is a dataclass of tuple/scalar fields in this admitted case.
            raise ValueError('Layout must not change reference chart')
        M, y = reference_objective(event, original_control.reference)
        z, _, rank, _ = np.linalg.lstsq(M@event.Z, y-M@event.origin, rcond=None)
        if rank != event.Z.shape[1]: raise ValueError('Rank-deficient reference projection')
        seed = event.origin+event.Z@z
        point = original_control.point
        matches = [t for t in event.traces if (t['key'],t['record'],t['part'],t['edge']) ==
                   (point.source_key,point.record,point.part,point.edge)]
        if len(matches) != 1 or not np.array_equal(matches[0]['st'], original_control.trace['st']):
            raise ValueError('Control source occurrence or station correspondence drifted')
        control = SimpleNamespace(event=event, reference=original_control.reference, current=seed,
                                  point=point, trace=matches[0], handle=event.row(point.edge, point.station),
                                  reference_t=original_control.reference_t,
                                  reference_sha256=original_control.reference_sha256,
                                  budgets=original_control.budgets.copy())
        super().__init__(control)
        self.M, self.objective_reference = M, y
        self.migration = migration_error(event, control.reference, seed)
        self.preflight = dict(schema='mapforge/long-layout-admission/v1',
                              old_knots=list(original.scope.knots), new_knots=list(scope.knots),
                              minimum_span_m=float(min(np.diff(scope.knots))), nvar=event.nvar,
                              free_variables=event.Z.shape[1], projected_seed_error=self.migration,
                              projected_seed_is_map=False, projected_source_max_m=event.source_error(seed)['max_m'],
                              projection_energy=float(np.sum((M@seed-y)**2)),
                              source_ownership_unchanged=True, chart_unchanged=True,
                              scope_and_semantic_anchors_unchanged=True,
                              no_op_policy='return original XML bytes; never apply projected coefficients',
                              map_accepted=False)

    def unchanged_xml(self):
        return self.control.reference

    def solve(self, displacement, **kwargs):
        # Zero pointer intent must not alter geometry through basis projection.
        # Caller must use unchanged_xml, not compile the projected seed.
        if displacement == 0 and not isinstance(displacement, bool):
            raise ValueError('Zero edit replays unchanged_xml; no projected state may be applied')
        state, report = super().solve(displacement, **kwargs)
        report.update(layout_changed=True, old_reference_sha256=self.control.reference_sha256,
                      seed_is_reference_geometry=False, original_xml_objective=True,
                      projected_seed_error_m=self.migration['max_m'])
        return state, report
