"""Isolated full-source experiment: a simple coordinate spine, then lane-family QP.

Outside the short Link support, the reference is not a measured centreline.
Try a straight coordinate axis only for a nearly straight measured Link; all
raw Lane.points stay authoritative in G8 and are fitted by the boundary family.
No source geometry, speed, topology or final tolerance can be changed to pass.
"""
import argparse
import json
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.ops import map_to_xodr as converter
from mapforge.ops.refline_fit import PlanView, PlanSeg
from spikes.map_single_primitive import accurate_single, pin_speed_floor
from scripts.gen_all import CASES, POLICY
from mapforge.report.decision import finalize_opendrive_g8


def straight_coordinate_spine(link, support):
    link = np.asarray(link, float); support = np.asarray(support, float)
    delta = link[-1]-link[0]
    tangent = delta/np.linalg.norm(delta)
    normal = np.array([-tangent[1], tangent[0]])
    deviation = float(max(abs((link-link[-1]) @ normal)))
    if deviation > .35:
        return None
    length = float(-min((support-link[-1]) @ tangent))
    start = link[-1]-length*tangent
    pv = PlanView(*start, float(np.arctan2(tangent[1], tangent[0])), [PlanSeg('line', length)])
    pv.fit_meta.update(fit_selection='straight-coordinate-spine-full-lane-support',
                       measured_link_max_m=deviation,
                       source_fidelity_deferred_to_full_lane_g8=True,
                       impulse_filter={'removed_count': 0})
    return pv, deviation, False


def run(label, output):
    name = dict(CASES)[label]
    source_path = ROOT/'v2x_map_xml'/name
    node = parse_map_xml(str(source_path))
    path = output/f'{label}.xodr'
    if path.resolve().is_relative_to((ROOT/'out/m2x').resolve()):
        raise ValueError('cannot overwrite production artifacts')
    output.mkdir(parents=True, exist_ok=True)
    selections = []
    def choose(support, **kwargs):
        hit = next((lk for lk in node.links if len(lk.points) >= 2 and
                    np.linalg.norm(converter._project(lk.points, node.ref_lat, node.ref_lon)[-1]-support[-1]) < 1e-6), None)
        result = None
        if hit is not None and len(support) > len(hit.points):
            result = straight_coordinate_spine(converter._project(hit.points, node.ref_lat, node.ref_lon), support)
        if result is None:
            result = accurate_single(support, **kwargs)
        selections.append(result[0].fit_meta)
        return result
    with patch.object(converter, 'fit_leg_refline', choose):
        stats = converter.build_xodr(node, path)
    speeds = pin_speed_floor(path, ROOT/'out/map-joint-review-v138'/f'{label}.xodr')
    final = finalize_opendrive_g8(path, stats['source_lane_manifest'], POLICY, raw_map_paths=[source_path])
    # This intermediate artifact deliberately precedes final joint-mouth repair.
    final['decision']['blocked_reasons'].append({'code': 'candidate_not_promoted'})
    final['decision']['status'] = 'BLOCKED'
    for suffix, value in (('.delivery-decision.json', final['decision']),
                          ('.quality-report.json', final['quality']),
                          ('.full-source.json', {'selections': selections, 'speeds_restored': speeds,
                            'extensions': stats.get('reference_support_extensions', []),
                            'gates': {k: v['status'] for k, v in final['quality']['gates'].items()}})):
        path.with_suffix(suffix).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v['status'] for k, v in final['quality']['gates'].items()}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('case', choices=dict(CASES)); parser.add_argument('output', type=Path)
    args = parser.parse_args(); run(args.case, args.output)
