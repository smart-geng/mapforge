"""Design-only long-knot review: no edited state, fitting or XODR export.

One proposed simple knot is the midpoint of the final existing long span.
Preserve the old spline by Boehm insertion in memory, measure clamped local
capacity, and inspect actual XML record splitting before accepting a design.
This is NOT permission to apply the representation or retry S2.
"""
import json
import math
from pathlib import Path
import sys

import numpy as np
import scipy
from scipy.interpolate import BSpline

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.edit_scope import _svd
from mapforge.repair_web.model import atomic, digest, json_bytes, intervals, lanes, parse
from scripts.check_outer_event_control import inputs, REFERENCE
from scripts.check_outer_event_written import boundary_poly

S2 = ROOT/'out/node4-unique-local-target-s2-20260915-r1'
DEST = ROOT/'out/node4-long-refinement-design-20260917'
LO, HI, EDGE = 157.6089, 201.7074107, 3
MIN_SPAN = 6.


def insert_long_knot(spline, station, minimum=MIN_SPAN):
    if (type(station) not in (int, float) or not math.isfinite(station)
            or type(minimum) not in (int, float) or not math.isfinite(minimum) or minimum <= 0):
        raise ValueError('Finite simple knot and positive minimum span required')
    cuts = sorted(set(spline.t))
    if not cuts[0] < station < cuts[-1] or station in cuts:
        raise ValueError('No repeated knots, endpoint changes or extrapolation')
    if min(np.diff(sorted(cuts+[station]))) < minimum-1e-9:
        raise ValueError('Short independently controlled span rejected')
    return spline.insert_knot(station, m=1)


def clamped_capacity(cuts, station):
    """C2 delta is zero outside the interval; zero 2-jets on both ends."""
    if not cuts[0] < station < cuts[-1] or min(np.diff(cuts)) <= 0:
        raise ValueError('Strict ordered support and interior handle required')
    t = [cuts[0]]*4+list(cuts[1:-1])+[cuts[-1]]*4
    bs = BSpline(t, np.eye(len(t)-4), 3, extrapolate=False)
    length = cuts[-1]-cuts[0]
    locks = np.asarray([bs(s, nu=d)*length**d for s in (cuts[0], cuts[-1]) for d in range(3)])
    space, ranks, _ = _svd(locks)
    if len(set(ranks)) != 1:
        raise ValueError('Uncertain endpoint-lock rank')
    control_ranks = []
    for components in (1, 2, 3):
        rows = np.asarray([bs(station, nu=d)*length**d for d in range(components)])@space
        _, rank, _ = _svd(rows)
        if len(set(rank)) != 1:
            raise ValueError('Uncertain handle rank')
        control_ranks.append(rank[0])
    return dict(independent_spans=len(cuts)-1, free_directions=space.shape[1],
                endpoint_rank_by_tolerance=ranks, position_slope_curvature_jet_ranks=control_ranks,
                free_after_one_position=space.shape[1]-control_ranks[0],
                endpoint_jet_residual=float(np.max(abs(locks@space))),
                nonlinear_feasibility='NOT_EVALUATED', map_accepted=False)


def power(spline, s):
    return np.array([spline(s, nu=d)/math.factorial(d) for d in range(4)])


def migration_review(old, refined, road):
    """Compare every polynomial coefficient on the union with actual XML cuts."""
    cuts = set(old.t) | set(refined.t)
    cuts.update(float(e.get('s')) for e in road.findall('lanes/laneOffset'))
    for sec in road.findall('lanes/laneSection'):
        s = float(sec.get('s')); cuts.add(s)
        cuts.update(s+float(e.get('sOffset')) for e in sec.findall('.//width'))
    cuts = sorted(s for s in cuts if old.t[0] <= s <= old.t[-1])
    old_errors, xml_errors = [], []
    for a, b in zip(cuts, cuts[1:]):
        scale = np.array([1., b-a, (b-a)**2, (b-a)**3])
        old_errors.append(float(np.max(abs((power(refined, a)-power(old, a))*scale))))
        xml_errors.append(float(np.max(abs((power(refined, a)-boundary_poly(road, EDGE, a))*scale))))
    return dict(cells=len(cuts)-1, max_scaled_coefficient_change_m=max(old_errors),
                max_scaled_coefficient_difference_from_xml_m=max(xml_errors),
                comparison='all coefficients per union cell; floating-point check, not formal proof')


def record_split_review(road, station):
    """Predict only selected boundary's two derived width records, not a write."""
    affected, existing = [], []
    for sec, lo, hi in intervals(road):
        for lid, lane in lanes(sec).items():
            if lid not in (-3, -4):
                continue
            cuts = sorted({lo+float(w.get('sOffset')) for w in lane.findall('width')} | {hi})
            for a, b in zip(cuts, cuts[1:]):
                if b > LO and a < HI:
                    existing.append(b-a)
                if a < station < b:
                    affected.append(dict(lane=lid, section_s=lo, before=[a, b],
                                         after_lengths_m=[station-a, b-station]))
    if len(affected) != 2:
        raise ValueError('Knot must split exactly two registered derived width records')
    shortest = min(n for r in affected for n in r['after_lengths_m'])
    return dict(affected_records=affected, predicted_extra_width_records=2,
                predicted_extra_geometry=0, predicted_extra_sections=0, predicted_extra_lane_offset=0,
                baseline_min_record_in_scope_m=min(existing), newly_split_min_record_m=shortest,
                new_records_at_least_6m=shortest >= MIN_SPAN-1e-9,
                actual_export='NOT_PERFORMED')


def review(event, control):
    if (event.scope.road != '11' or control.point.edge != EDGE or control.point.station != 178.
            or event.scope.knots[-2:] != (187.5, HI) or LO not in event.scope.knots
            or event.scope.minimum_span_m != MIN_SPAN):
        raise ValueError('Design only covers the frozen S2 representation and scope')
    station = (event.scope.knots[-2]+HI)/2
    old = BSpline(event.splines[EDGE].t.copy(), control.current[event.slices[EDGE]].copy(), 3, extrapolate=False)
    refined = insert_long_knot(old, station)
    road = next(r for r in parse(control.reference).findall('road') if r.get('id') == '11')
    cuts = [s for s in event.scope.knots if LO <= s <= HI]
    before = clamped_capacity(cuts, control.point.station)
    after = clamped_capacity(sorted(cuts+[station]), control.point.station)
    migration = migration_review(old, refined, road)
    if migration['max_scaled_coefficient_difference_from_xml_m'] > 1e-8:
        raise ValueError('Unacceptable baseline projection loss')
    records = record_split_review(road, station)
    if not records['new_records_at_least_6m']:
        raise ValueError('Long mathematical span is insufficient: written record is short')
    # The preceding long span admits a mathematical split, but its mandatory
    # laneSection cut creates a new 1.27m width record. Reject on structure,
    # without generating/evaluating another edit or searching knot positions.
    excluded_midpoint = (173.3754+187.5)/2
    return dict(schema='mapforge/long-refinement-design-review/v1', status='STRUCTURAL_REVIEW_ONLY',
                reference_sha256=control.reference_sha256, road='11', edge=EDGE,
                interval_m=[LO, HI], proposed_simple_knot_s=station,
                insertion_policy='one fixed midpoint of final long span; no knot search',
                new_independent_span_lengths_m=[station-187.5, HI-station],
                minimum_independent_span_m=min(np.diff(sorted(set(refined.t)))),
                before=before, after=after, migration=migration, written_layout=records,
                excluded_preceding_midpoint=dict(station_m=excluded_midpoint,
                    record_check=record_split_review(road, excluded_midpoint),
                    status='EXCLUDED_BEFORE_ANY_SHAPE_TRIAL'),
                nonzero_edited_states=0, optimizer_calls=0, new_xodr=False,
                implementation_approved=False, target_feasibility='NOT_EVALUATED',
                map_accepted=False, scipy_version=scipy.__version__)


def main():
    if DEST.exists():
        raise ValueError('Design evidence exists: inspect it; do not overwrite')
    binding = json.loads((S2/'binding.json').read_bytes())
    status = json.loads((S2/'status.json').read_bytes())
    for name, sha in status['artifact_sha256'].items():
        binding[str(S2/name)] = sha
    for path in (S2/'status.json', S2/'binding.json', Path(__file__).resolve(),
                 ROOT/'tests/test_long_edit_refinement_review.py'):
        binding[str(path)] = digest(path.read_bytes())
    live = {}
    for name in ('node4-local-repair-20260915', 'node4-repair-ui-v2-20260915'):
        path = ROOT/'out'/name/'project/project.json'
        data = json.loads(path.read_bytes())
        live[name] = dict(project_sha256=digest(path.read_bytes()), bindings=len(data['binding']))
        binding[str(path)] = digest(path.read_bytes())
        for key, sha in data['binding'].items():
            if key in binding and binding[key] != sha:
                raise ValueError('Conflicting frozen binding: '+key)
            binding[key] = sha
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes()) != sha:
                raise ValueError('Frozen input/code drift: '+path)
    verify()
    event, control = inputs()
    report = review(event, control)
    verify()
    DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json', json_bytes(binding))
    atomic(DEST/'review.json', json_bytes(report))
    atomic(DEST/'status.json', json_bytes(dict(status=report['status'], input_bindings=len(binding),
        input_drift=0, live_projects_read_only=live, map_accepted=False, new_xodr=False,
        artifact_sha256={p.name: digest(p.read_bytes()) for p in DEST.iterdir()})))
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
