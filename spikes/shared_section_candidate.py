"""A/B the new coupled ribbon kernel on actual SHP connectors.

Fixed references/parent ports isolate the representation change. This is
NOT the proposed joint whole-network solver or a production export path.
No source, speed, topology, ordinary road or primitive length is changed.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from pyclothoids import Clothoid
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.ops.shared_section import solve_shared_section, width_control_bounds
from mapforge.validate.smoothness import _geoms
from mapforge.validate.shp_boundary_fidelity import _origin, _project
from spikes.connector_cross_section import edge_jet
from spikes.measured_connector_caps import (
    frames, raw_curves, composite_sources, sample, distances,
    source_slacks, write_ribbon, chain, needs_cap,
)
from spikes.trial_xml import replace_road, write_trial
from scripts.gen_all import shp_source
from scripts.review_measured_ribbon import run as review


def fit_fixed_reference(root, road, raw, via, *, joint_reference=False, parameters=None):
    a, b = frames(root, road)
    cls = [Clothoid.StandardParams(x, y, h, k0, (k1-k0)/length, length)
           for _, x, y, h, length, k0, k1 in _geoms(road)]
    lengths = [c.length for c in cls]
    nodes = [cls[0].KappaStart]+[c.KappaEnd for c in cls]
    start = [edge_jet(a, a['edges'][side], cls[0].KappaStart, cls[0].dk) for side in ('left', 'right')]
    end = [edge_jet(b, b['edges'][side], cls[-1].KappaEnd, cls[-1].dk) for side in ('left', 'right')]
    speeds = []
    for e in road.findall('.//lane/speed'):
        unit = e.get('unit', 'm/s')
        if unit not in ('m/s', 'km/h'):
            raise ValueError('unsupported speed unit')
        speeds.append(float(e.get('max'))/(3.6 if unit == 'km/h' else 1.))
    speed = max([15/3.6, *speeds])
    required = (needs_cap(a), needs_cap(b))
    nref = len(cls)
    if joint_reference and nref != 3+sum(required):
        raise ValueError('joint reference requires the existing adaptive-cap layout')
    first = int(required[0])
    shape_seed = np.r_[lengths, cls[first].KappaEnd*20, cls[first+1].KappaEnd*20]
    cache = {}
    def evaluate(z):
        key = tuple(z)
        if key not in cache:
            cc = chain(z[:nref+2], a, b, required) if joint_reference else cls
            q = z[nref+2:] if joint_reference else z
            ll = [c.length for c in cc]
            kk_nodes = [cc[0].KappaStart]+[c.KappaEnd for c in cc]
            aa = [edge_jet(a, a['edges'][side], cc[0].KappaStart, cc[0].dk) for side in ('left', 'right')]
            bb = [edge_jet(b, b['edges'][side], cc[-1].KappaEnd, cc[-1].dk) for side in ('left', 'right')]
            section = solve_shared_section(ll, kk_nodes, aa, bb, q)
            co = dict(zip(('left', 'right'), section.coefficients))
            pts, jets, dyn = sample(cc, section.knots, co)
            errors = {k: {'source_to_target': distances(raw[k], pts[k]),
                          'target_to_source': distances(pts[k], raw[k])} for k in raw}
            errors.update({'raw_via:'+k: {'source_to_target': distances(v, pts[k])} for k, v in via.items()})
            # Convex hull bound for ALL cubic widths, not endpoint sampling.
            controls = width_control_bounds(section).ravel()
            ratio = float(np.max(np.abs(dyn)*np.array([speed**2/2.5, speed**3])))
            # A=1-k*t is polynomial degree <=4 on each common interval;
            # this sampled check is only a construction filter, not a proof.
            kk = []
            ref = np.r_[0., np.cumsum(ll)]
            ss = np.unique(np.r_[np.arange(0, ref[-1], .5), section.knots, ref])
            for s in ss:
                i = min(len(cc)-1, np.searchsorted(ref, s, side='right')-1)
                kk.append(cc[i].KappaStart+cc[i].dk*(s-ref[i]))
            regular = min(float(np.min(1-np.array(kk)*jets[k][:, 0])) for k in ('left', 'right'))
            cache.clear()
            cache[key] = (section, errors, controls, ratio, regular, cc)
        return cache[key]
    def objective(q):
        return sum(np.mean(e**2) for field in evaluate(q)[1].values() for e in field.values())
    def geometry_limits(q):
        _, _, controls, _, regular, _ = evaluate(q)
        return np.r_[controls-.1, regular-.1]
    def all_limits(q):
        _, errors, _, ratio, _, _ = evaluate(q)
        return np.r_[geometry_limits(q), source_slacks(errors, .001), .98-ratio]
    def closure(z):
        c = evaluate(z)[-1][-1]
        angle = c.ThetaEnd-b['pose'][2]
        return np.array([c.XEnd-b['pose'][0], c.YEnd-b['pose'][1],
                         20*np.arctan2(np.sin(angle), np.cos(angle))])
    zero = np.r_[shape_seed, np.zeros(nref-1)] if joint_reference else np.zeros(nref-1)
    bounds = ([(6., 100.)]*nref+[(-6., 6.)]*2 if joint_reference else [])+[(-.5, .5)]*(nref-1)
    if parameters is not None:
        replay = np.asarray(parameters, float)
        if replay.shape != zero.shape or not np.isfinite(replay).all() or any(
                v < lo-1e-8 or v > hi+1e-8 for v, (lo, hi) in zip(replay, bounds)):
            raise ValueError('invalid saved research parameters')
        zero = replay
    history = []
    pool = [zero]
    current = zero
    phases = [] if parameters is not None else [('positive-width-source-warm-start', geometry_limits),
                                               ('unchanged-source-and-dynamics', all_limits)]
    for phase, constraint in phases:
        constraints = [{'type': 'ineq', 'fun': constraint}]
        if joint_reference:
            constraints.append({'type': 'eq', 'fun': closure})
        result = minimize(objective, current, method='SLSQP', bounds=bounds,
                          constraints=constraints, options={'maxiter': 150, 'ftol': 1e-9})
        current = result.x
        pool.append(current.copy())
        history.append({'phase': phase, 'success': bool(result.success), 'message': str(result.message),
                        'iterations': int(result.nit), 'objective': float(objective(current)),
                        'geometry_slack': float(min(geometry_limits(current))),
                        'complete_slack': float(min(all_limits(current))),
                        'parameters': current.tolist(), 'closure_max': float(max(abs(closure(current))))})
        print(road.get('id'), history[-1], flush=True)
    closed = [q for q in pool if not joint_reference or max(abs(closure(q))) < 1e-6]
    # Construction already reserves 1 mm source and 2% dynamics margins;
    # tolerate equality-scale solver roundoff, NOT final gate violations.
    feasible = [q for q in closed if min(all_limits(q)) >= -1e-8]
    positive = [q for q in closed if min(geometry_limits(q)) >= 0]
    # A positive-width but source-invalid trial is diagnostic, never accepted.
    # Among rejected diagnostics, prioritize feasibility deficit, NOT the
    # prettiest source fit that may have much worse vehicle dynamics.
    chosen = (min(feasible, key=objective) if feasible else
              min(positive or closed or pool, key=lambda q: (-min(all_limits(q)), objective(q))))
    section, errors, controls, ratio, regular, cls = evaluate(chosen)
    shears = chosen[nref+2:] if joint_reference else chosen
    lengths = [c.length for c in cls]
    candidate = write_ribbon(road, cls, section.knots, dict(zip(('left', 'right'), section.coefficients)))
    ud = candidate.find("lanes/laneSection/right/lane/userData[@code='mapforge.provenance/v1']")
    if ud is not None:
        p = json.loads(ud.get('value'))
        p.update(cross_section_fit='shared-shear-world-G2-cubic-ribbon',
                 shears=shears.tolist(), transverse_c2_required=False,
                 independent_shear_variables=len(shears))
        ud.set('value', json.dumps(p, separators=(',', ':')))
    row = {'road': road.get('id'), 'status': 'CANDIDATE' if feasible else 'REJECTED',
           'shared_shears': shears.tolist(), 'optimizer_phases': history,
           'joint_reference_optimization': joint_reference,
           'parameters_replayed_without_optimization': parameters is not None,
           'construction_roundoff_tolerance': 1e-8,
           'selected_parameters': chosen.tolist(), 'closure_max': float(max(abs(closure(chosen)))),
           'reference_primitives': len(cls), 'reference_lengths_m': lengths,
           'width_records': len(section.knots)-1, 'minimum_width_record_span_m': float(min(np.diff(section.knots))),
           'old_fixed_reference_transverse_dof': 0, 'new_shared_transverse_dof': len(shears),
           'equality_rank_per_boundary': section.rank, 'equilibrated_condition': section.condition,
           'equality_residual': section.residual,
           'minimum_width_bernstein_bound_m': float(min(controls)),
           'sampled_minimum_normal_regularity': regular, 'construction_dynamics_ratio': ratio,
           'evaluation_speed_kmh': speed*3.6, 'speed_fields_unchanged': True,
           'source': {k: {d: {'median_m': float(np.median(e)), 'p95_m': float(np.percentile(e, 95)),
                             'max_m': float(max(e))} for d, e in field.items()} for k, field in errors.items()},
           'joint_parent_port_optimization': False, 'production_promoted': False}
    return candidate, row


def run(source, target, road_ids, *, joint_reference=False, replay_report=None):
    if source.resolve() == target.resolve() or target.exists():
        raise ValueError('choose a new isolated target, never overwrite source/artifacts')
    tree = ET.parse(source)
    root = tree.getroot()
    available = {r.get('id'): r for r in root.findall('road')}
    if not road_ids or set(road_ids)-available.keys():
        raise ValueError('explicit existing connector IDs required')
    src = shp_source()
    lat, lon = _origin(root)
    rows = []
    saved = {}
    if replay_report is not None:
        replay = json.loads(replay_report.read_text(encoding='utf-8'))
        if replay.get('source_sha256') != hashlib.sha256(source.read_bytes()).hexdigest():
            raise ValueError('saved parameters are for a different parent artifact')
        saved = {r['road']: r for r in replay['roads']}
        if any(rid not in saved or saved[rid].get('joint_reference_optimization') != joint_reference for rid in road_ids):
            raise ValueError('saved road/model layout mismatch')
    for rid in road_ids:
        road = available[rid]
        e = road.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']")
        if e is None:
            raise ValueError('measured via required')
        raw, info = composite_sources(root, road, src, lambda x: _project(x, lat, lon))
        via = raw_curves(src, e.get('value'), lambda x: _project(x, lat, lon))
        candidate, row = fit_fixed_reference(root, road, raw, via, joint_reference=joint_reference,
                                             parameters=saved[rid]['selected_parameters'] if rid in saved else None)
        row['source_composite'] = info
        replace_road(root, road, candidate)
        rows.append(row)
    xsd = write_trial(tree, target)
    report = {'status': 'BLOCKED', 'candidate_only': True, 'source': str(source),
              'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'sha256': hashlib.sha256(target.read_bytes()).hexdigest(), 'roads': rows,
              'source_support_mode': 'source-linked-composite', 'xsd': xsd,
              'scope': 'joint reference/ribbon with fixed parent ports' if joint_reference else 'fixed-reference A/B',
              'whole_network_joint_reconstruction': False}
    target.with_suffix('.ribbon.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    review(source, target, target.parent/'review')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('target', type=Path)
    p.add_argument('--road', required=True, action='append')
    p.add_argument('--joint-reference', action='store_true')
    p.add_argument('--replay-report', type=Path)
    a = p.parse_args()
    run(a.source, a.target, a.road, joint_reference=a.joint_reference, replay_report=a.replay_report)
    raise SystemExit(2)  # always a BLOCKED research artifact, even local PASS
