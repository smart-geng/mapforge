"""Isolated SHP coordinate-spine/physical-boundary experiment.

SHP is read directly. MAP only locates the junction, never supplies its lane
geometry. Real via connectors are NOT replaced by synthetic MAP connectors.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import CASES, POLICY, shp_source
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.ops import shp_to_xodr as converter
from mapforge.report.decision import finalize_opendrive_g8
from mapforge.validate.g11 import load_policy, _audit_d
from mapforge.validate.shp_boundary_fidelity import _project, _origin
from spikes.map_coordinate_family import coordinate_axis
from spikes.map_single_primitive import pin_speed_floor
from spikes import road_boundary_family as family
from spikes.clarabel_joint_candidate import interior_qp


def main(label, ordered_boundaries=False, output_tag=None):
    source = shp_source(); node = parse_map_xml(str(ROOT/'v2x_map_xml'/dict(CASES)[label]))
    junction, distance = source.find_junction(node.ref_lon, node.ref_lat)
    if distance > 50.:
        raise ValueError('no close original SHP junction')
    tag = output_tag or ('shp-source-order-v141' if ordered_boundaries else 'shp-coordinate-v141')
    if Path(tag).name != tag or not tag.endswith('-v141'):
        raise ValueError('output tag must be a single directory name ending in -v141')
    folder = ROOT/'out'/tag; folder.mkdir(parents=True, exist_ok=True)
    path = folder/f'{label}.xodr'; fallback = converter.fit_leg_refline; selections = []
    def choose(points, **kwargs):
        result = coordinate_axis(points, points)
        if result is None:
            result = fallback(points, **kwargs)
        selections.append(result[0].fit_meta)
        return result
    with patch.object(converter, 'fit_leg_refline', choose):
        stats = converter.build_junction_xodr(source, junction, path)
    raised = pin_speed_floor(path, ROOT/'out/direct_xodr'/f'{label}.xodr')
    final = finalize_opendrive_g8(path, stats['source_lane_manifest'], POLICY)
    final['decision']['status'] = 'BLOCKED'
    final['decision']['blocked_reasons'].append({'code': 'candidate_not_promoted'})
    path.with_suffix('.delivery-decision.json').write_text(json.dumps(final['decision'], indent=2), encoding='utf-8')
    path.with_suffix('.coordinate.json').write_text(json.dumps({'candidate_only': True, 'selections': selections,
        'speed_floor_adjustments': raised, 'source_scope': 'existing SHP manifest, not all raw layers',
        'conversion': {k:v for k,v in stats.items() if k != 'source_lane_manifest'},
        'gates': {k:v['status'] for k,v in final['quality']['gates'].items()}}, indent=2), encoding='utf-8')
    print('SHP COORDINATE', {k:v['status'] for k,v in final['quality']['gates'].items()}, flush=True)
    tree = etree.parse(str(path)); lat, lon = _origin(tree.getroot()); reports = []
    policy = load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    policy['dynamics']['sample_step_m'] = .02
    with patch.object(family, '_convex_qp', interior_qp):
        for road in tree.findall('road'):
            if road.get('junction') != '-1':
                continue
            attempts = []
            for phase in (None, 0., 3., 6., 9., 12.):
                report = family.solve_road(road, source, lambda p: _project(p,lat,lon),
                    source_mode='boundaries', junction_endpoint_mode='parallel',
                    knot_phase=phase, restore_dynamics=True,
                    boundary_association='source-order' if ordered_boundaries else 'nearest-target')
                attempts.append(dict(report, phase=phase))
                if report['status'] == 'CANDIDATE':
                    break
                if (report.get('reason','').startswith('short physical boundary')
                        or report.get('reason') == 'source boundary association unresolved'):
                    break
            report['attempts'] = attempts; report['road'] = road.get('id')
            if report['status'] == 'CANDIDATE':
                single = etree.Element('OpenDRIVE'); single.append(etree.fromstring(etree.tostring(road)))
                dense = _audit_d(single, policy)
                report['focused_2cm_dynamics'] = dense
            reports.append(report)
            print('SHP ROAD', road.get('id'), report['status'], report.get('reason'), flush=True)
    out = ROOT/'out'/tag.replace('-v141','-joint-v141'); out.mkdir(parents=True, exist_ok=True)
    target = out/f'{label}.xodr'; tree.write(str(target),encoding='utf-8',xml_declaration=True)
    final = finalize_opendrive_g8(target, stats['source_lane_manifest'], POLICY)
    final['decision']['status'] = 'BLOCKED'; final['decision']['blocked_reasons'] += [
        {'code': 'candidate_not_promoted'}, {'code': 'real_via_and_mouth_joint_model_unfinished'}]
    target.with_suffix('.delivery-decision.json').write_text(json.dumps(final['decision'],indent=2),encoding='utf-8')
    target.with_suffix('.fit.json').write_text(json.dumps({'candidate_only': True, 'delivery': 'BLOCKED',
        'roads': reports, 'input_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'output_sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
        'unmodified_rejected_roads_are_not_solutions': True,
        'gates': {k:v['status'] for k,v in final['quality']['gates'].items()}}, indent=2),encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('case', choices=dict(CASES))
    parser.add_argument('--ordered-boundaries', action='store_true')
    parser.add_argument('--output-tag')
    args = parser.parse_args(); main(args.case, args.ordered_boundaries, args.output_tag)
