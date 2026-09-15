"""Independent junction tangent/curvature checks from esmini world XY samples.

No mapforge lane evaluator is used. esmini's heading field can reflect the road
frame; therefore derivatives are estimated directly from its lane positions.
Two sample spacings expose finite-difference instability. This is a geometry
consumer test, not a closed-loop vehicle-controller acceptance test.
"""
import argparse
import ctypes
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.esmini_rm_check import DLL, RMPos, _bind


def endpoint(rm, handle, rid, lid, contact, forward, step, field='center'):
    if field not in ('center', 'left', 'right'):
        raise ValueError('unknown sampled lane field')
    length = rm.RM_GetRoadLength(rid)
    distance = min(step, length/20.)
    offsets = np.arange(6, dtype=float)*distance
    if contact == 'end': offsets = -offsets
    elif contact != 'start': raise ValueError('invalid contactPoint')
    endpoint_s = length if contact == 'end' else 0.
    points = []
    for offset in offsets:
        s = endpoint_s+offset
        transverse = 0.
        if field != 'center':
            width = ctypes.c_double()
            if rm.RM_GetLaneWidthByRoadId(rid, lid, ctypes.c_double(s), ctypes.byref(width)) != 0:
                raise ValueError('esmini lane width lookup failed')
            if not math.isfinite(width.value) or width.value <= 0.:
                raise ValueError('esmini returned nonpositive/non-finite edge width')
            transverse = width.value/2*(1 if field == 'left' else -1)*(1 if forward else -1)
        code = rm.RM_SetLanePosition(handle, rid, lid, ctypes.c_double(transverse), ctypes.c_double(s), True)
        if code < 0: raise ValueError(f'RM_SetLanePosition failed: {code}')
        pd = RMPos()
        if rm.RM_GetPositionData(handle, ctypes.byref(pd)) < 0:
            raise ValueError('RM_GetPositionData failed')
        if pd.roadId != rid or pd.laneId != lid:
            raise ValueError(f'esmini snapped requested lane {rid}/{lid} to {pd.roadId}/{pd.laneId}')
        points.append([pd.x, pd.y])
    points = np.asarray(points)
    # Scale s to avoid an ill-conditioned power basis at centimetre spacing.
    fit = np.polynomial.polynomial.polyfit(offsets/distance, points-points[0], 4)
    velocity, acceleration = fit[1]/distance, 2*fit[2]/distance**2
    k = (velocity[0]*acceleration[1]-velocity[1]*acceleration[0])/np.linalg.norm(velocity)**3
    return {'x': float(points[0, 0]), 'y': float(points[0, 1]),
            'heading_rad': float(math.atan2(velocity[1], velocity[0])+(0. if forward else math.pi)),
            'curvature_per_m': float(k if forward else -k)}


def check(path, *, edges=False):
    root = ET.parse(path).getroot(); roads = {r.get('id'): r for r in root.findall('road')}
    rm = ctypes.CDLL(str(DLL)); _bind(rm)
    if rm.RM_Init(str(path.resolve()).encode('utf-8')) != 0: raise ValueError('esmini load failed')
    rows = []
    try:
        handle = rm.RM_CreatePosition()
        cache = {}
        def sample(key, step, field):
            full = (*key, step, field)
            if full not in cache: cache[full] = endpoint(rm, handle, *key, step, field)
            return cache[full]
        for connection in root.findall('junction/connection'):
            rid = connection.get('connectingRoad'); cr = roads[rid]
            cp = connection.get('contactPoint'); forward = cp == 'start'
            entry_role, exit_role = ('predecessor', 'successor') if forward else ('successor', 'predecessor')
            for ll in connection.findall('laneLink'):
                try:
                    if len(cr.findall('lanes/laneSection')) != 1:
                        raise ValueError('independent probe currently requires one-section connector')
                    lane_id = int(ll.get('to'))
                    lane = cr.find(f"lanes/laneSection/{'left' if lane_id > 0 else 'right'}/lane[@id='{lane_id}']")
                    entry, exit_link = cr.find('link/'+entry_role), cr.find('link/'+exit_role)
                    inc, out = int(entry.get('elementId')), int(exit_link.get('elementId'))
                    inc_contact, out_contact = entry.get('contactPoint'), exit_link.get('contactPoint')
                    pairs = [('entry', (inc, int(ll.get('from')), inc_contact, inc_contact == 'end'),
                              (int(rid), lane_id, cp, forward)),
                             ('exit', (int(rid), lane_id, 'end' if forward else 'start', forward),
                              (out, int(lane.find('link/'+exit_role).get('id')), out_contact, out_contact == 'start'))]
                    fields = ('center','left','right') if edges else ('center',)
                    for which, a, b, field in ((which,a,b,field) for which,a,b in pairs for field in fields):
                        results = []
                        for step in (.02, .01):
                            aa, bb = sample(a, step, field), sample(b, step, field)
                            results.append({'step_m': step,
                                'position_m': math.hypot(aa['x']-bb['x'], aa['y']-bb['y']),
                                'heading_deg': math.degrees(abs((aa['heading_rad']-bb['heading_rad']+math.pi)%(2*math.pi)-math.pi)),
                                'curvature_per_m': abs(aa['curvature_per_m']-bb['curvature_per_m'])})
                        stable = abs(results[0]['curvature_per_m']-results[1]['curvature_per_m']) < 1e-6
                        fine = results[-1]
                        passed = stable and fine['position_m'] < .01 and fine['heading_deg'] < .1 and fine['curvature_per_m'] < 1e-6
                        rows.append({'road': rid, 'interface': which, 'field': field, 'from_lane': ll.get('from'),
                                     'status': 'PASS' if passed else 'FAIL', 'stable': stable, 'samples': results})
                except (AttributeError, ValueError, KeyError) as exc:
                    rows.append({'road': rid, 'status': 'FAIL', 'error': str(exc)})
    finally:
        rm.RM_Close()
    report = {'schema': 'mapforge/esmini-lane-interface/v1', 'engine': 'esminiRMLib',
              'artifact': str(path), 'artifact_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'method': 'world XY one-sided degree-4 polynomial derivatives; 0.02/0.01m spacing',
              'scope': 'junction lane interfaces, one-section connecting roads; not controller testing',
              'fields': ['center','left','right'] if edges else ['center'],
              'thresholds': {'position_m': .01, 'heading_deg': .1, 'curvature_per_m': 1e-6},
              'rows': rows, 'status': 'PASS' if rows and all(x['status'] == 'PASS' for x in rows) else 'FAIL'}
    suffix = '.esmini-edge-interfaces.json' if edges else '.esmini-interfaces.json'
    path.with_suffix(suffix).write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({'status': report['status'], 'interfaces': len(rows),
                      'maxima': {field: max((x['samples'][-1][field] for x in rows if 'samples' in x), default=None)
                                 for field in ('position_m', 'heading_deg', 'curvature_per_m')},
                      'failures': [x for x in rows if x['status'] != 'PASS']}))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('xodr', type=Path)
    p.add_argument('--edges', action='store_true', help='also query both edges using esmini lane widths')
    a = p.parse_args()
    sys.exit(0 if check(a.xodr, edges=a.edges)['status'] == 'PASS' else 1)
