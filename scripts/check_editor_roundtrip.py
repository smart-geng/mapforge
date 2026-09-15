"""Independent no-edit editor admission check on the two FINAL XML files.

Same road/lane identity and same reference s; no nearest-road reassignment or
registration fitting. Sampled shape evidence is not continuous certification.
Unsupported primitives / CRS changes fail closed for geometry comparison.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from lxml import etree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.internal_edge_jets import states, section


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    root = ET.parse(str(path)).getroot()
    for e in root.iter():
        if isinstance(e.tag, str):
            e.tag = ET.QName(e).localname
    return root


def normalized(e):
    if e is None:
        return None
    return (e.tag, tuple(sorted(e.attrib.items())), (e.text or '').strip(),
            tuple(normalized(c) for c in e if isinstance(c.tag, str)))


def summary(root):
    gs = root.findall('road/planView/geometry')
    lengths = np.array([float(g.get('length')) for g in gs])
    return dict(roads=len(root.findall('road')), junctions=len(root.findall('junction')),
                connections=len(root.findall('junction/connection')),
                lane_links=len(root.findall('junction/connection/laneLink')),
                lane_sections=len(root.findall('road/lanes/laneSection')),
                widths=len(root.findall('.//lane/width')),
                lane_offsets=len(root.findall('road/lanes/laneOffset')),
                lane_speeds=len(root.findall('.//lane/speed')),
                source_userdata=len(root.findall('.//userData')),
                geometry=len(gs), kinds=dict(Counter(g[0].tag for g in gs)),
                min_geometry_m=float(min(lengths)), median_geometry_m=float(np.median(lengths)),
                geometry_lt_2m=int(sum(lengths < 2)), geometry_lt_5m=int(sum(lengths < 5)),
                revision=dict(root.find('header').attrib))


def cuts(road):
    out = {0., float(road.get('length'))}
    out.update(float(g.get('s')) for g in road.findall('planView/geometry'))
    out.update(float(g.get('s')) for g in road.findall('lanes/laneOffset'))
    for sec in road.findall('lanes/laneSection'):
        s = float(sec.get('s')); out.add(s)
        for name in ('width', 'border', 'speed'):
            out.update(s + float(x.get('sOffset')) for x in sec.findall('.//' + name))
    return out


def lanes_at(road, s, left):
    sec = section(road, s, left)
    return sec, {int(l.get('id')): l for l in sec.findall('left/lane') + sec.findall('right/lane')}


def speed_at(lane, s, left):
    records = [x for x in lane.findall('speed')
               if float(x.get('sOffset')) < s or (not left and float(x.get('sOffset')) <= s)]
    if not records:
        return None
    e = max(records, key=lambda x: float(x.get('sOffset')))
    value = e.get('max'); unit = e.get('unit', 'm/s')
    if value in ('undefined', 'no limit'):
        return value
    return float(value) * {'m/s': 1., 'km/h': 1/3.6, 'mph': .44704}[unit]


def compare(source, target, output):
    a, b = load(source), load(target)
    ar = {r.get('id'): r for r in a.findall('road')}
    br = {r.get('id'): r for r in b.findall('road')}
    rows, curve_errors, shape_samples = [], [], []
    changed_crs = (a.findtext('header/geoReference') or '').strip() != (b.findtext('header/geoReference') or '').strip()
    offsets = lambda r: tuple(float(r.find('header/offset').get(k, '0')) if r.find('header/offset') is not None else 0. for k in ('x','y','z','hdg'))
    changed_offset = offsets(a) != offsets(b)
    if changed_crs or changed_offset:
        raise ValueError('Different coordinate frames: comparison must not silently realign')
    for rid in sorted(ar.keys() & br.keys(), key=int):
        ra, rb = ar[rid], br[rid]
        la, lb = float(ra.get('length')), float(rb.get('length'))
        kinds = {g[0].tag for r in (ra, rb) for g in r.findall('planView/geometry')}
        if not kinds <= {'line', 'arc', 'spiral'}:
            curve_errors.append(dict(road=rid, reason='unsupported', kinds=sorted(kinds)))
            continue
        stop = min(la, lb)
        ss = set(np.linspace(0., stop, max(2, math.ceil(stop/.5)+1)))
        ss |= {x for x in cuts(ra) | cuts(rb) if 0 <= x <= stop}
        vals, speeds, missing, types = [], [], [], []
        rounding_count, rounding_max, worst = 0, 0., None
        for s in sorted(ss):
            # Explicit one-sided states at record boundaries, not epsilon guessing.
            sides = (False,) if s == 0 else ((True,) if s == stop else (True, False))
            for left in sides:
                sa, lanes_a = lanes_at(ra, s, left)
                sb, lanes_b = lanes_at(rb, s, left)
                if lanes_a.keys() != lanes_b.keys():
                    missing.append(dict(s_m=s, left_limit=left,
                                        before=sorted(lanes_a), after=sorted(lanes_b)))
                for lid in lanes_a.keys() & lanes_b.keys():
                    x, y = lanes_a[lid], lanes_b[lid]
                    if x.get('type') != y.get('type'):
                        types.append(dict(s_m=s, lane=lid, before=x.get('type'), after=y.get('type')))
                    v1 = speed_at(x, s-float(sa.get('s')), left)
                    v2 = speed_at(y, s-float(sb.get('s')), left)
                    delta = abs(v1-v2) if isinstance(v1, float) and isinstance(v2, float) else None
                    # ORBIT serializes m/s to four decimal places. Track that
                    # representational loss separately from missing/changed limits.
                    eq = (delta <= .0000501 if delta is not None else v1 == v2)
                    if eq and delta:
                        rounding_count += 1
                        rounding_max = max(rounding_max, delta)
                    if not eq:
                        speeds.append(dict(s_m=s, lane=lid, before_ms=v1, after_ms=v2))
                    try:
                        aa, bb = states(ra, lid, s, left), states(rb, lid, s, left)
                        for edge, (p, q) in enumerate(zip(aa, bb)):
                            d = math.hypot(p[0]-q[0], p[1]-q[1])
                            vals.append(d)
                            shape_samples.append((p[0], p[1], q[0], q[1], d, int(rid)))
                            if worst is None or d > worst['error_m']:
                                worst = dict(s_m=s, lane=lid, edge=edge, left_limit=left,
                                             before_xy=p[:2], after_xy=q[:2], error_m=d)
                    except (ValueError, ZeroDivisionError) as exc:
                        curve_errors.append(dict(road=rid, lane=lid, s_m=s, error=str(exc)))
        rows.append(dict(road=rid, name=ra.get('name'), length_delta_m=lb-la,
                         geometry_before=len(ra.findall('planView/geometry')),
                         geometry_after=len(rb.findall('planView/geometry')),
                         edge_sample_count=len(vals), edge_max_m=max(vals) if vals else None,
                         edge_p95_m=float(np.percentile(vals, 95)) if vals else None,
                         lane_domain_mismatches=len(missing), lane_domain_examples=missing[:5],
                         lane_type_mismatches=len(types), lane_type_examples=types[:5],
                         speed_state_mismatches=len(speeds), speed_examples=speeds[:5],
                         speed_rounding_samples=rounding_count, speed_rounding_max_ms=rounding_max,
                         worst_edge_witness=worst,
                         road_links_equal=normalized(ra.find('link')) == normalized(rb.find('link'))))
    before, after = summary(a), summary(b)
    movements = lambda root: sorted((j.get('id'), c.get('incomingRoad'), c.get('connectingRoad'), c.get('contactPoint'), l.get('from'), l.get('to')) for j in root.findall('junction') for c in j.findall('connection') for l in c.findall('laneLink'))
    stable_movements = movements(a) == movements(b)
    report = dict(schema='mapforge/editor-roundtrip-admission/v1', source=str(source.resolve()),
                  target=str(target.resolve()), source_sha256=sha(source), target_sha256=sha(target),
                  before=before, after=after, missing_roads=sorted(ar.keys()-br.keys()),
                  added_roads=sorted(br.keys()-ar.keys()), movement_tuples_equal=stable_movements,
                  movement_before=movements(a), movement_after=movements(b),
                  rows=rows, errors=curve_errors, sample_step_m=.5,
                  speed_serialization_rounding_bound_ms=.0000501,
                  shape_scope='same road/lane id at same s, common longitudinal domain only; no nearest matching; extra/lost length separate',
                  map_accepted=False, GUI_tested=False,
                  status='REJECTED_FOR_LOSSLESS_EDITING' if (
                      curve_errors or ar.keys()!=br.keys() or not stable_movements or
                      any(r['lane_domain_mismatches'] or r['lane_type_mismatches'] or r['speed_state_mismatches'] or
                          r['edge_max_m'] is None or r['edge_max_m'] > .01 for r in rows) or
                      after['geometry_lt_2m'] > before['geometry_lt_2m'] or
                      before['source_userdata'] != after['source_userdata']) else 'SAMPLED_NO_EDIT_CHECK_ONLY')
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    np.savez_compressed(output.with_suffix('.npz'), edges=np.array(shape_samples))
    print(json.dumps(dict(status=report['status'], before=before, after=after,
                          max_edge_m=max(r['edge_max_m'] or 0 for r in rows),
                          speed_mismatches=sum(r['speed_state_mismatches'] for r in rows),
                          movements_equal=stable_movements)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('target', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.npz').exists():
        raise FileExistsError('Use a fresh result path')
    compare(args.source, args.target, args.output)
