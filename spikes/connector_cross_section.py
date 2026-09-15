"""Isolated joint reference-frame/edge-end-state connector experiment.

Three clothoids and one cubic edge record per primitive, not sampled patches.
All ordinary roads, topology and speeds remain immutable. This experiment is
not a delivery path: both edges AND the center must pass independent audits.
"""
import copy
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import root as find_root

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.ops.refline_fit import solve_g2_balanced
from mapforge.validate.smoothness import (
    _geoms, _sections, lane_edges_kinematics_at, lane_endpoint_state,
    junction_lane_interfaces, _edge_world_curvature,
)
from mapforge.validate.junction_edges import endpoint_edges, audit as edge_audit
from pyclothoids import Clothoid


def endpoint_frame(road, lid, contact, forward):
    """Same physical cross section, centered on this lane, travel orientation."""
    _, x, y, h, length, k0, k1 = _geoms(road)[0 if contact == 'start' else -1]
    dk = (k1-k0)/length
    k = k0
    if contact == 'end':
        c = Clothoid.StandardParams(x, y, h, k0, dk, length)
        x, y, h, k = c.XEnd, c.YEnd, c.ThetaEnd, c.KappaEnd
    s = 0. if contact == 'start' else float(road.get('length'))
    sec = _sections(road)[0 if contact == 'start' else -1]
    idx = [v[0] for v in sec[2 if lid > 0 else 1]].index(lid)
    es = lane_edges_kinematics_at(road, s, 'left' if lid > 0 else 'right')
    tc = (es[idx][0]+es[idx+1][0])/2
    a = 1-k*tc
    if a <= .1:
        raise ValueError('singular/near-singular parent parallel frame')
    direction = 1 if forward else -1
    return {'pose': (x-tc*math.sin(h), y+tc*math.cos(h), h+(0 if forward else math.pi)),
            'k': direction*k/a, 'dk': dk/a**3,
            'edges': endpoint_edges(road, lid, contact, forward),
            'center': lane_endpoint_state(road, lid, contact, forward=forward)}


def edge_jet(frame, edge, curvature, sharpness):
    """Invert world tangent/curvature into transverse jets of the new frame."""
    x, y, h = frame['pose']
    t = -(edge['x']-x)*math.sin(h)+(edge['y']-y)*math.cos(h)
    along = (edge['x']-x)*math.cos(h)+(edge['y']-y)*math.sin(h)
    if abs(along) > 1e-7:
        raise ValueError('non-shared endpoint cross section')
    a = 1-curvature*t
    angle = (edge['heading']-h+math.pi)%(2*math.pi)-math.pi
    if a <= .1 or abs(angle) >= math.pi/3:
        raise ValueError('non-forward endpoint edge')
    d1 = a*math.tan(angle)
    d2 = (edge['curvature']*(a*a+d1*d1)**1.5/a-curvature*a
          -d1/a*(sharpness*t+2*curvature*d1))
    return np.array([t, d1, d2])


def jet(c, u):
    a, b, c2, d = c
    return np.array([a+b*u+c2*u*u+d*u**3, b+2*c2*u+3*d*u*u, 2*c2+6*d*u])


def fit_edge(cls, start, end):
    """Three cubic records: exact end jets, C1, world G2 across k' jumps."""
    lens = np.array([c.length for c in cls])
    length = sum(lens)
    spans = lens/length
    ks = [c.KappaEnd for c in cls[:-1]]
    jumps = np.diff([(c.KappaEnd-c.KappaStart)/c.length for c in cls])
    scale = np.array([1., length, length**2])
    def residual(co, nonlinear=True):
        co = co.reshape(3, 4)
        rr = [*(jet(co[0], 0)-start*scale), *(jet(co[-1], spans[-1])-end*scale)]
        for i in range(2):
            left, right = jet(co[i], spans[i]), jet(co[i+1], 0)
            rr.extend(right-left)
            if nonlinear:
                rr[-1] += left[0]*left[1]*length*jumps[i]/(1-ks[i]*left[0])
        return np.array(rr)
    zero = np.zeros(12)
    y = residual(zero, False)
    mat = np.column_stack([residual(np.eye(12)[i], False)-y for i in range(12)])
    seed = np.linalg.solve(mat, -y)
    solved = find_root(residual, seed, tol=1e-10)
    if np.max(abs(residual(solved.x))) > 1e-7:
        raise ValueError('edge world-G2 spline solve failed')
    return solved.x.reshape(3, 4)/np.array([1., length, length**2, length**3])


def endpoint_g3_chain(a, b, cap_length):
    """Research-only five-primitive boundary-jet construction.

    Current inferred-connector profile allows THREE primitives. This FIVE
    primitive experiment must remain blocked by that unchanged gate even if
    all differential geometry tests pass.
    """
    first = Clothoid.StandardParams(*a['pose'], a['k'], a['dk'], cap_length)
    back = Clothoid.StandardParams(b['pose'][0], b['pose'][1], b['pose'][2]+math.pi,
                                   -b['k'], b['dk'], cap_length)
    last = Clothoid.StandardParams(back.XEnd, back.YEnd, back.ThetaEnd-math.pi,
                                   -back.KappaEnd, b['dk'], cap_length)
    mid, tuning = solve_g2_balanced((first.XEnd, first.YEnd, first.ThetaEnd),
                                   (last.XStart, last.YStart, last.ThetaStart),
                                   first.KappaEnd, last.KappaStart)
    return (first, *mid, last), dict(tuning, method='research-five-end-G3-clothoids')


def flat_join_spline(cls, start, end):
    """C2 transverse cubics, t'=0 at the four reference sharpness jumps.

    Then world G2 holds for both boundaries AND their average (lane center).
    Seven spans fixed by five primitives, without freely placed local repairs.
    """
    lens = np.array([c.length for c in cls])
    length = sum(lens)
    ref_knots = np.r_[0., np.cumsum(lens)]
    knots = np.sort(np.r_[ref_knots, lens[0]/2, length-lens[-1]/2])
    normalized = knots/length
    spans = np.diff(normalized)
    flat_indices = [int(np.argmin(abs(knots-s))) for s in ref_knots[1:-1]]
    n = len(spans)
    scale = np.array([1., length, length**2])
    def linear(v):
        co = v.reshape(n, 4)
        rr = [*jet(co[0], 0), *jet(co[-1], spans[-1])]
        for i in range(n-1):
            rr.extend(jet(co[i+1], 0)-jet(co[i], spans[i]))
        rr.extend(co[i, 1] for i in flat_indices)
        return np.array(rr)
    rhs=np.r_[start*scale,end*scale,np.zeros(3*(n-1)+len(flat_indices))]
    # Assemble the LINEAR operator directly. Subtracting large endpoint jets
    # from each basis probe loses small matrix entries through cancellation.
    mat = np.column_stack([linear(np.eye(4*n)[i]) for i in range(4*n)])
    physical_scale=np.r_[np.tile(scale,n+1),np.full(len(flat_indices),length)]
    scaled=mat/physical_scale[:,None];target=rhs/physical_scale
    solved=np.linalg.solve(scaled,target)
    for _ in range(2):solved+=np.linalg.solve(scaled,(rhs-linear(solved))/physical_scale)
    if np.max(abs((linear(solved)-rhs)/physical_scale)) > 1e-9:
        raise ValueError('seven-span linear C2 solve failed')
    return knots, solved.reshape(n, 4)/np.array([1., length, length**2, length**3])


def replace(road, a, b, *, mode='three', cap_length=6.):
    if mode == 'three':
        cls, tuning = solve_g2_balanced(a['pose'], b['pose'], a['k'], b['k'])
    elif mode == 'five':
        cls, tuning = endpoint_g3_chain(a, b, cap_length)
    else:
        raise ValueError('unknown experiment mode')
    co = {}
    for side in ('left', 'right'):
        first, last = cls[0], cls[-1]
        start = edge_jet(a, a['edges'][side], first.KappaStart,
                         (first.KappaEnd-first.KappaStart)/first.length)
        end = edge_jet(b, b['edges'][side], last.KappaEnd,
                       (last.KappaEnd-last.KappaStart)/last.length)
        if mode == 'three':
            co[side] = fit_edge(cls, start, end)
            knots = np.r_[0., np.cumsum([c.length for c in cls])]
        else:
            knots, co[side] = flat_join_spline(cls, start, end)
    geoms = road.find('planView')
    geoms.clear()
    group = road.find('lanes')
    for v in group.findall('laneOffset'):
        group.remove(v)
    lane = road.find('lanes/laneSection/right/lane')
    for v in lane.findall('width')+lane.findall('border'):
        lane.remove(v)
    s = 0.
    for i, c in enumerate(cls):
        g = ET.SubElement(geoms, 'geometry', s=str(s), x=str(c.XStart), y=str(c.YStart),
                          hdg=str(c.ThetaStart), length=str(c.length))
        ET.SubElement(g, 'spiral', curvStart=str(c.KappaStart), curvEnd=str(c.KappaEnd))
        s += c.length
    for i, so in enumerate(knots[:-1]):
        group.insert(i, ET.Element('laneOffset', s=str(so),
                                  **dict(zip('abcd', map(str, co['left'][i])))))
        lane.insert(i+1, ET.Element('width', sOffset=str(so),
                                   **dict(zip('abcd', map(str, co['left'][i]-co['right'][i])))))
    road.set('length', str(s))
    ud = lane.find("userData[@code='mapforge.provenance/v1']")
    if ud is not None:
        p = json.loads(ud.get('value'))
        p.update(candidate_only=True, cross_section_fit='shared-frame-world-edge-G2',
                 curve_fit=tuning, center_continuity='requires-independent-gate')
        ud.set('value', json.dumps(p, ensure_ascii=False, separators=(',', ':')))
    return {'road': road.get('id'), 'primitive_lengths': [float(c.length) for c in cls],
            'width_records': len(knots)-1, 'minimum_width_span_m': float(min(np.diff(knots))),
            'center_endpoint_curvature_residual': [
                lane_endpoint_state(road, -1, c)['curvature']-f['center']['curvature']
                for c, f in (('start', a), ('end', b))]}


def build(root, **kwargs):
    """Trial clone; callers never receive a partly changed source root."""
    result = copy.deepcopy(root)
    roads = {r.get('id'): r for r in result.findall('road')}
    rows = []
    for cr in roads.values():
        if cr.get('junction') == '-1' or cr.get('name') == 'junction_paving':
            continue
        if len(cr.findall('lanes/laneSection')) != 1 or cr.findall('lanes/laneSection/left/lane'):
            raise ValueError('one right lane connector only')
        lanes = cr.findall('lanes/laneSection/right/lane')
        if len(lanes) != 1 or lanes[0].get('id') != '-1':
            raise ValueError('one right lane connector only')
        frames = []
        for role in ('predecessor', 'successor'):
            link = cr.find('link/'+role)
            lid = int(lanes[0].find('link/'+role).get('id'))
            contact = link.get('contactPoint')
            frames.append(endpoint_frame(roads[link.get('elementId')], lid, contact,
                          contact == ('end' if role == 'predecessor' else 'start')))
        rows.append(replace(cr, *frames, **kwargs))
    return result, rows


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('target', type=Path)
    p.add_argument('--mode', choices=('three', 'five'), default='three')
    p.add_argument('--cap-length', type=float, default=6.)
    args = p.parse_args()
    result, rows = build(ET.parse(args.source).getroot(), mode=args.mode, cap_length=args.cap_length)
    args.target.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(result)
    ET.ElementTree(result).write(args.target, encoding='utf-8', xml_declaration=True)
    edges = edge_audit(result)
    centers = junction_lane_interfaces(result)
    summary = {'candidate_only': True, 'rows': rows, 'edge_contacts': edges,
               'center_contacts': centers, 'delivery': 'BLOCKED'}
    args.target.with_suffix('.cross-section.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({'edges': edges['maxima'], 'center_maxima': {
        k: max(r[k] for r in centers) for k in ('position_m', 'heading_deg', 'curvature_per_m')}}))
