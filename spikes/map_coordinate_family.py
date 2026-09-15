"""MAP candidate: complete two-way source domain on a simple coordinate spine.

The coordinate reference is not claimed to reproduce Link.points. Actual
lane centers remain constrained by all raw Lane.points and independent G8.
Only monotone, moderate-turn corridors can use a straight spine; other roads
retain the existing long-primitive fitter. No production default changes.
"""
import argparse
import json
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import CASES, POLICY
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml, parse_map_xml_all
from mapforge.ops import map_to_xodr as converter
from mapforge.ops.refline_fit import PlanView, PlanSeg
from spikes.map_single_primitive import accurate_single, pin_speed_floor
from mapforge.report.decision import finalize_opendrive_g8


def source_mouth_heading(lane_points,span=30.):
    """One heading estimate from original incoming terminal observations.

    This proposes a chart, not a change of raw samples or a boundary truth.
    Segment interpolation only locates a point at the requested arc station.
    All actual sources remain constraints in the complete-road solver.
    """
    if not np.isfinite(span) or span<=0:raise ValueError('positive terminal support span required')
    vectors=[];rows=[]
    for sid,points in lane_points:
        xy=np.asarray(points,float)
        if xy.ndim!=2 or xy.shape[1]!=2 or len(xy)<2 or not np.isfinite(xy).all():
            raise ValueError('terminal heading requires complete finite lane observations')
        lengths=np.linalg.norm(np.diff(xy,axis=0),axis=1)
        if np.any(lengths<=1e-9):raise ValueError('duplicate terminal observation requires source review')
        s=np.r_[0.,np.cumsum(lengths)];start=max(0.,s[-1]-span)
        first=np.array([np.interp(start,s,xy[:,k]) for k in range(2)])
        vector=xy[-1]-first;used=float(s[-1]-start)
        if np.linalg.norm(vector)<.9*used:
            raise ValueError('terminal support reverses or bends beyond a regular chart estimate')
        vectors.append(vector)
        rows.append(dict(source_lane_id=sid,used_arc_span_m=used,raw_vertex_count=len(xy),
                         heading_rad=float(np.arctan2(vector[1],vector[0]))))
    if not vectors:raise ValueError('no incoming source lanes for terminal heading')
    total=np.sum(vectors,axis=0);coherence=float(np.linalg.norm(total)/sum(np.linalg.norm(v) for v in vectors))
    if coherence<.98:raise ValueError('inconsistent incoming terminal headings')
    return float(np.arctan2(total[1],total[0])),dict(requested_span_m=span,coherence=coherence,lanes=rows)


def coordinate_axis(link, support, *, heading=None):
    link, support = np.asarray(link, float), np.asarray(support, float)
    chord = link[-1]-link[0]
    norm = np.linalg.norm(chord)
    if norm < 1.:
        return None
    if heading is not None and not np.isfinite(heading):raise ValueError('finite chart heading required')
    t = chord/norm if heading is None else np.array([np.cos(heading),np.sin(heading)])
    n = np.array([-t[1], t[0]])
    delta = np.diff(support, axis=0); size = np.linalg.norm(delta, axis=1)
    meaningful = size > .1
    if np.any((delta[meaningful] @ t)/size[meaningful] < .5):
        return None
    # Limit validity of the chosen coordinate chart, NOT the source error budget.
    offset = float(max(abs((support-link[-1]) @ n)))
    if offset > 15.:
        return None
    length = float(-min((support-link[-1]) @ t))
    if length < 1.:
        return None
    start = link[-1]-length*t
    pv = PlanView(*start, float(np.arctan2(t[1], t[0])), [PlanSeg('line', length)])
    error = float(max(abs((link-link[-1]) @ n)))
    pv.fit_meta.update(fit_selection='coordinate-axis-not-source-road-centerline',
                       link_to_axis_max_m=error, reference_support_to_axis_max_m=offset,
                       final_lane_fidelity_requires_raw_g8=True,
                       impulse_filter={'removed_count': 0})
    return pv, error, False


def run(label, output):
    raw = [ROOT/'v2x_map_xml'/file for _, file in CASES]
    main = parse_map_xml(str(ROOT/'v2x_map_xml'/dict(CASES)[label]))
    neighbors = [n for p in raw for n in parse_map_xml_all(str(p))
                 if (n.region, n.node_id) != (main.region, main.node_id)]
    output.mkdir(parents=True, exist_ok=True)
    path = output/f'{label}.xodr'
    if path.resolve().is_relative_to((ROOT/'out/m2x').resolve()):
        raise ValueError('cannot replace formal output')
    selections = []
    def choose(support, **kwargs):
        lk = next(x for x in main.links if len(x.points) > 1 and
                  np.linalg.norm(converter._project(x.points, main.ref_lat, main.ref_lon)[-1]-support[-1]) < 1e-6)
        result = coordinate_axis(converter._project(lk.points, main.ref_lat, main.ref_lon), support)
        if result is None:
            result = accurate_single(support, **kwargs)
        selections.append({'link': lk.name, **result[0].fit_meta})
        return result
    with patch.object(converter, 'fit_leg_refline', choose):
        stats = converter.build_xodr(main, path, neighbors=neighbors)
    raised = pin_speed_floor(path, ROOT/'out/m2x'/f'{label}.xodr')
    final = finalize_opendrive_g8(path, stats['source_lane_manifest'], POLICY, raw_map_paths=raw)
    final['decision']['status'] = 'BLOCKED'
    final['decision']['blocked_reasons'].append({'code': 'candidate_not_promoted'})
    report = {'candidate_only': True, 'selections': selections,
              'speed_floor_adjustments': raised,
              'conversion': {k: v for k, v in stats.items() if k != 'source_lane_manifest'},
              'gates': {k: v['status'] for k, v in final['quality']['gates'].items()}}
    for suffix, value in (('.coordinate.json', report), ('.delivery-decision.json', final['decision']),
                          ('.quality-report.json', final['quality'])):
        path.with_suffix(suffix).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    print(label, report['gates'], flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('case', choices=dict(CASES))
    parser.add_argument('output', type=Path); args = parser.parse_args()
    run(args.case, args.output)
