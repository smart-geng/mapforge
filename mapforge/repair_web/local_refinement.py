"""R1: local long-cubic delta and a diagnostic-only, two-width writer.

This is a SHA-bound representation adapter, NOT a fitter, target acceptance
gate or Web export API. Old source/shape failures remain visible. Nonzero
output is explicitly a compiler probe, never a deliverable repaired map.
"""
from __future__ import annotations

from bisect import bisect_left, bisect_right
import copy
from dataclasses import asdict, dataclass
import json
import math
from xml.etree import ElementTree as ET

import numpy as np
from scipy.interpolate import BSpline

from .edit_scope import EditScopeRequest, ScopeError, _svd, prepare_edit_scope
from .model import coefficients, complexity, digest, extrema, intervals, json_bytes, lanes, parse
from scripts.check_outer_event_written import boundary_poly, check, max_abs, semantic

LO, HI, EDGE = 157.6089, 201.7074107, 3
KNOT = (187.5+HI)/2
REFERENCE_SHA = '5d1cec2baea21200fbdcf78ca9267bfe7bb94e9cf441539d759370fffdaaea98'
CUTS = (LO, 163.6089, 173.3754, 187.5, KNOT, HI)


def _fail(code, message):
    raise ScopeError(code, message)


def _state(value):
    a = np.asarray(value, dtype=object)
    if (a.shape != (2,) or any(isinstance(v, (bool, np.bool_)) or
            not isinstance(v, (int, float, np.integer, np.floating)) for v in a)):
        _fail('REFINEMENT_INVALID_STATE', 'Two finite real coefficients required; no clipping')
    a = np.asarray(a, dtype=float)
    if not np.isfinite(a).all():
        _fail('REFINEMENT_INVALID_STATE', 'Nonfinite coefficients')
    return a


def _shift(c, distance):
    a, b, c2, d = np.asarray(c)
    return np.array([a+b*distance+c2*distance**2+d*distance**3,
                     b+2*c2*distance+3*d*distance**2, c2+3*d*distance, d])


def _cuts(road):
    cuts = {0., float(road.get('length'))}
    cuts.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
    for sec in road.findall('lanes/laneSection'):
        s = float(sec.get('s')); cuts.add(s)
        cuts.update(s+float(w.get('sOffset')) for w in sec.findall('.//width'))
    return sorted(cuts)


def _protected_semantics(data):
    root = parse(data)
    road = root.find("road[@id='11']")
    if road is None:
        _fail('REFINEMENT_SEMANTIC_DRIFT', 'Registered road removed')
    for sec, lo, hi in intervals(road):
        if hi <= LO or lo >= HI:
            continue
        for lid, lane in lanes(sec).items():
            if lid in (-3, -4):
                for width in list(lane.findall('width')):
                    s = lo+float(width.get('sOffset'))
                    if LO <= s < HI:
                        lane.remove(width)
    return semantic(root)


def _record_layout(road):
    return {(lo, lid): tuple(lo+float(w.get('sOffset')) for w in lane.findall('width'))
            for sec, lo, hi in intervals(road) for lid, lane in lanes(sec).items()}


@dataclass(frozen=True)
class LocalRefinement:
    reference: bytes
    baseline: bytes
    _packet: bytes
    _inventory: bytes
    _event_scope: bytes
    _powers: bytes
    _reference_cells: bytes
    _contract: bytes

    @property
    def contract(self):
        return json.loads(self._contract)

    @property
    def contract_sha256(self):
        return digest(self._contract)

    def _verify(self):
        c = self.contract
        pairs = {'reference_sha256': self.reference, 'baseline_sha256': self.baseline,
                 'source_packet_sha256': self._packet, 'inventory_sha256': self._inventory,
                 'scope_sha256': self._event_scope, 'local_power_sha256': self._powers,
                 'reference_cells_sha256': self._reference_cells}
        if any(digest(value) != c[key] for key, value in pairs.items()):
            _fail('REFINEMENT_STALE_BINDING', 'Immutable reference, source or basis changed')

    def delta_power(self, station, state, *, left=False):
        u = _state(state)
        if isinstance(station, bool) or not isinstance(station, (float, int, np.floating)) or not math.isfinite(station):
            _fail('REFINEMENT_INVALID_STATION', 'Finite station required')
        idx = (bisect_left(CUTS, station) if left else bisect_right(CUTS, station))-1
        if idx < 0 or idx >= len(CUTS)-1:
            return np.zeros(4)
        power = np.asarray(json.loads(self._powers))[idx]@u
        return _shift(power, station-CUTS[idx])

    def power(self, edge, station, state, *, left=False):
        u = _state(state)
        if type(edge) is not int or not 0 <= edge <= 4:
            _fail('REFINEMENT_INVALID_EDGE', 'Only registered physical boundaries 0..4')
        cells = json.loads(self._reference_cells)
        cuts = cells['cuts']
        if (isinstance(station, bool) or not isinstance(station, (float, int, np.floating))
                or not math.isfinite(station) or not 0 <= station <= cuts[-1]):
            _fail('REFINEMENT_INVALID_STATION', 'No extrapolation outside the actual parent')
        idx = (bisect_left(cuts, station) if left else bisect_right(cuts, station))-1
        idx = min(max(idx, 0), len(cuts)-2)
        c = _shift(cells['powers'][idx][edge], station-cuts[idx])
        return c+self.delta_power(station, u, left=left) if edge == EDGE else c

    def _serialize_probe(self, state):
        """Private mechanical writer. Public entry enforces readback and purpose."""
        u = _state(state)
        if not np.any(u):
            return self.reference
        root = parse(self.reference)
        road = root.find("road[@id='11']")
        for sec, lo, hi in intervals(road):
            if hi <= LO or lo >= HI:
                continue
            for lid, lane in lanes(sec).items():
                if lid not in (-3, -4):
                    continue
                old = lane.findall('width')
                for index, width in enumerate(old):
                    a = lo+float(width.get('sOffset'))
                    b = lo+float(old[index+1].get('sOffset')) if index+1 < len(old) else hi
                    if b <= LO or a >= HI:
                        continue
                    # Admission guarantees every old local breakpoint is
                    # already present. Only KNOT may add a new record.
                    if a < LO or b > HI or any(a < s < b for s in CUTS if s != KNOT):
                        _fail('REFINEMENT_UNREGISTERED_LAYOUT', 'Writer would need an unapproved split')
                    split = [a]+([KNOT] if a < KNOT < b else [])+[b]
                    offset = list(lane).index(width)
                    lane.remove(width)
                    for j, (start, end) in enumerate(zip(split, split[1:])):
                        c = _shift(coefficients(width), start-a)
                        c += (-1 if lid == -3 else 1)*self.delta_power(start, u)
                        if not np.isfinite(c).all() or extrema(c, end-start) < -1e-7:
                            _fail('REFINEMENT_NEGATIVE_WIDTH', 'Negative/nonfinite whole-interval width')
                        new = copy.deepcopy(width)
                        new.set('sOffset', format(start-lo, '.17g'))
                        for key, value in zip('abcd', c):
                            new.set(key, format(float(value), '.17g'))
                        lane.insert(offset+j, new)
        return ET.tostring(root, encoding='utf-8', xml_declaration=True)

    def audit_compiled(self, data, state):
        """Read actual XML, not the writer's staged attributes or sampled image."""
        self._verify(); u = _state(state)
        if type(data) is not bytes:
            _fail('REFINEMENT_INVALID_XML', 'Immutable XML bytes required')
        if _protected_semantics(data) != _protected_semantics(self.reference):
            _fail('REFINEMENT_SEMANTIC_DRIFT', 'Unselected XML or source/ID/TOPO/speed/marks changed')
        roots = [parse(d) for d in (self.reference, data)]
        old, road = [r.find("road[@id='11']") for r in roots]
        before, after = _record_layout(old), _record_layout(road)
        for sec, lo, hi in intervals(road):
            if hi <= LO or lo >= HI:
                continue
            for lid, lane in lanes(sec).items():
                if lid not in (-3, -4):
                    continue
                for width in lane.findall('width'):
                    if (set(width.attrib) != set(('sOffset', 'a', 'b', 'c', 'd')) or len(width)
                            or not all(math.isfinite(float(v)) for v in width.attrib.values())):
                        _fail('REFINEMENT_INVALID_XML', 'Unsupported/nonfinite written width')
        if before.keys() != after.keys():
            _fail('REFINEMENT_STRUCTURE_DRIFT', 'Lane/section layout changed')
        for key, starts in before.items():
            should_add = bool(np.any(u) and key in ((181.7074107, -3), (181.7074107, -4)))
            expected = tuple(sorted(starts+((KNOT,) if should_add else ())))
            if after[key] != expected:
                _fail('REFINEMENT_STRUCTURE_DRIFT', 'Unexpected record, station or record order')
        counts = [complexity(r) for r in roots]
        change = {k: counts[1][k]-counts[0][k] for k in ('geometry', 'width', 'laneSection', 'laneOffset')}
        if change != dict(geometry=0, width=2 if np.any(u) else 0, laneSection=0, laneOffset=0):
            _fail('REFINEMENT_STRUCTURE_DRIFT', 'Diagnostic writer exceeded registered record budget')
        if np.any(u):
            for key in ((181.7074107, -3), (181.7074107, -4)):
                values = after[key]; i = values.index(KNOT)
                if min(KNOT-values[i-1], values[i+1]-KNOT) < 6.-1e-9:
                    _fail('REFINEMENT_SHORT_RECORD', 'Inserted width produces a short record')
        ss = sorted(set(_cuts(old)+_cuts(road)+list(CUTS)))
        max_frozen = max_error = max_coeff = 0.
        min_width = float('inf')
        for a, b in zip(ss, ss[1:]):
            scale = (b-a)**np.arange(4)
            for edge in range(5):
                actual = boundary_poly(road, edge, a)
                expected = self.power(edge, a, u)
                max_error = max(max_error, max_abs(actual-expected, b-a))
                max_coeff = max(max_coeff, float(np.max(abs((actual-expected)*scale))))
                if edge != EDGE or b <= LO or a >= HI:
                    max_frozen = max(max_frozen, max_abs(actual-boundary_poly(old, edge, a), b-a))
                if edge in (3, 4) and LO <= a < HI:
                    min_width = min(min_width, extrema(boundary_poly(road, edge-1, a)-actual, b-a))
        jets = []
        for s in CUTS:
            right, left = boundary_poly(road, EDGE, s), boundary_poly(road, EDGE, s, left=True)
            jets.append(dict(station_m=s, jumps=(abs(right[:3]-left[:3])*[1., 1., 2.]).tolist()))
        if max(max_frozen, max_error, max_coeff, *(max(r['jumps']) for r in jets)) > 1e-8:
            _fail('REFINEMENT_GEOMETRY_DRIFT', 'Frozen curves, whole polynomial or C2 readback failed')
        if min_width < -1e-7:
            _fail('REFINEMENT_NEGATIVE_WIDTH', 'Written physical width is negative')
        source = check(data, self.baseline, json.loads(self._packet),
                       json.loads(self._event_scope), json.loads(self._inventory))
        if source['status'] != 'PASS_LOCAL_EVENT_NOT_MAP':
            _fail('REFINEMENT_SOURCE_READBACK', 'Independent source/event readback rejected probe')
        return dict(status='R1_WRITER_CONSISTENT_NOT_SHAPE_ACCEPTANCE', reference_sha256=digest(self.reference),
                    xodr_sha256=digest(data), contract_sha256=self.contract_sha256,
                    state=u.tolist(), noop_original_bytes=data == self.reference,
                    whole_polynomial_error_m=max_error, scaled_coefficient_error_m=max_coeff,
                    frozen_curve_change_m=max_frozen, minimum_edited_width_m=min_width,
                    joint_jets=jets, structure_change=change, source_readback=source,
                    map_accepted=False, export_allowed=False, user_target_checked=False,
                    operating_dynamics='NOT_EVALUATED', shape_acceptance='NOT_EVALUATED',
                    purpose='R1_COMPILER_PROBE', formal_certificate=False)

    def compile_probe(self, state, *, contract_sha256, purpose):
        if contract_sha256 != self.contract_sha256 or purpose != 'R1_COMPILER_PROBE':
            _fail('REFINEMENT_DIAGNOSTIC_ONLY', 'Bound R1 compiler probe, not a repair/export operation')
        self._verify(); u = _state(state)
        data = self._serialize_probe(u)
        return data, self.audit_compiled(data, u)


def prepare_local_refinement(event, reference, request: EditScopeRequest):
    """Admit the reviewed fixed case only; never guess a scope or source role."""
    if (type(reference) is not bytes or digest(reference) != REFERENCE_SHA
            or not isinstance(request, EditScopeRequest)):
        _fail('REFINEMENT_UNREGISTERED_SCOPE', 'Typed scope and exact reviewed reference required')
    # Let S1 validate nested types and exact source occurrence first; dataclass
    # annotations alone are not validation of a caller-supplied request.
    s1 = prepare_edit_scope(event, reference, request)
    if (request.edges != (EDGE,)
            or request.interval != (LO, HI) or request.road != '11'
            or len(request.handles) != 1 or request.handles[0].components != ('position',)
            or asdict(request.handles[0].point) != dict(source_key='IBD_LANE_BOUNDARY:2023041110502726490',
                                                       record=0, part=0, edge=3, station=178.)
            or event.scope.minimum_span_m != 6.
            or event.scope.knots != (100., 115., 131.0543, 141.3316, 151.6089, LO,
                                     163.6089, 173.3754, 187.5, HI)):
        _fail('REFINEMENT_UNREGISTERED_SCOPE', 'Only reviewed source/reference/long-layout scope')
    if s1.report['status'] != 'EDIT_SCOPE_READY_NOT_FEASIBILITY':
        _fail('REFINEMENT_SCOPE_REJECTED', 'Original scope/source admission failed')
    root = parse(reference); road = root.find("road[@id='11']")
    actual_cuts = _cuts(road)
    t = (LO,)*4+CUTS[1:-1]+(HI,)*4
    bs = BSpline(t, np.eye(len(t)-4), 3, extrapolate=False)
    locks = np.array([bs(s, nu=d)*(HI-LO)**d for s in (LO, HI) for d in range(3)])
    space, ranks, _ = _svd(locks)
    if len(set(ranks)) != 1 or space.shape[1] != 2 or np.max(abs(locks@space)) > 1e-9:
        _fail('REFINEMENT_CAPACITY_UNCERTAIN', 'Two stable clamped local directions required')
    powers = json_bytes([np.array([bs(s, nu=d)/math.factorial(d) for d in range(4)]).dot(space).tolist()
                         for s in CUTS[:-1]])
    for sec, lo, hi in intervals(road):
        if hi <= LO or lo >= HI:
            continue
        for lid in (-3, -4):
            lane = lanes(sec).get(lid)
            if lane is None or lane.get('type') != 'driving' or lane.findall('border'):
                _fail('REFINEMENT_UNSUPPORTED_LAYOUT', 'Registered adjacent width lanes required')
            starts = [lo+float(w.get('sOffset')) for w in lane.findall('width')]+[hi]
            if any(s not in starts for s in CUTS if s != KNOT and lo <= s < hi):
                _fail('REFINEMENT_UNREGISTERED_LAYOUT', 'Old local cuts must already be written')
            for width in lane.findall('width'):
                if (set(width.attrib) != set(('sOffset', 'a', 'b', 'c', 'd')) or len(width)
                        or not all(math.isfinite(float(v)) for v in width.attrib.values())):
                    _fail('REFINEMENT_UNSUPPORTED_WIDTH_EXTENSION', 'Width extensions require an explicit adapter; no dropping')
    cells = json_bytes(dict(cuts=actual_cuts, powers=[
        [boundary_poly(road, edge, s).tolist() for edge in range(5)] for s in actual_cuts[:-1]]))
    packet, inventory, scope = json_bytes(event.packet), json_bytes(event.inventory), json_bytes(asdict(event.scope))
    contract = json_bytes(dict(schema='mapforge/local-cubic-refinement/v1',
        reference_sha256=digest(reference), baseline_sha256=digest(event.data),
        source_packet_sha256=digest(packet), inventory_sha256=digest(inventory), scope_sha256=digest(scope),
        local_power_sha256=digest(powers), reference_cells_sha256=digest(cells),
        s1_contract_sha256=s1.report['contract_sha256'], request=asdict(request),
        local_cuts_m=list(CUTS), inserted_simple_knot=KNOT, free_directions=2,
        frozen_physical_edges=[0, 1, 2, 4], derived_lanes=[-3, -4],
        extra_records_max=dict(geometry=0, width=2, laneSection=0, laneOffset=0),
        new_record_minimum_m=6., boundary_role='shared edge, not outermost edge4',
        status='R1_REPRESENTATION_ONLY', target_feasibility='NOT_EVALUATED', map_accepted=False))
    return LocalRefinement(reference, event.data, packet, inventory, scope, powers, cells, contract)
