"""Isolated MAP experiment: use an accurate single primitive before a G2 chain.

No production default changes. Single-primitive candidates retain all points;
legacy multi-primitive fallback still reports its source-filter exceptions.
Written lane speeds cannot be reduced relative to the baseline to gain a PASS.
All outputs are marked candidates, including otherwise passing audit results.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
from lxml import etree
from pyclothoids import Clothoid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mapforge.ops.refline_fit as fitting
import mapforge.ops.map_to_xodr as converter
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml, parse_map_xml_all
from mapforge.report.decision import finalize_opendrive_g8
from mapforge.validate.g11 import load_policy, _audit_d
from scripts.gen_all import CASES, POLICY


def accurate_single(points, **kwargs):
    """Same strict fidelity budget as the existing multi-primitive search.

    Lowest segment count first, then lowest bidirectional maximum deviation.
    Single-primitive trials keep every original point. A legacy fallback's
    impulse-filter metadata must remain visible and is not an all-source PASS.
    """
    src = np.asarray(points, float)
    cap = kwargs.get('kappa_cap', .009)
    sharp = kwargs.get('sharp_cap', .000216)
    hard = kwargs.get('hard_dev_cap', 1.5)
    tol = kwargs.get('dev_tol') or hard
    if not 0 < tol <= hard:
        raise ValueError('invalid deviation budget')
    chord = src[-1] - src[0]
    h = float(np.arctan2(chord[1], chord[0]))
    line = fitting.PlanView(*src[0], h, [fitting.PlanSeg('line', float(np.linalg.norm(chord)))])
    candidates = [('single-line', line, min(.35, tol), .20, .30)]
    arc = fitting._single_arc_candidate(src)
    if arc is not None:
        candidates.append(('single-arc', arc, min(1., tol), .35, .75))
    h0 = fitting._robust_endpoint_heading(src, True)
    h1 = fitting._robust_endpoint_heading(src, False)
    cl = Clothoid.G1Hermite(*src[0], h0, *src[-1], h1)
    spiral = fitting.PlanView(*src[0], h0, [fitting.PlanSeg(
        'spiral', cl.length, cl.KappaStart, cl.KappaEnd)])
    candidates.append(('single-spiral', spiral, min(1., tol), .35, .75))
    valid = []
    for name, pv, limit, median, p95 in candidates:
        ok, metrics = fitting._leg_candidate_ok(
            pv, src, tol=limit, kappa_cap=cap, sharp_cap=sharp,
            median_tol=median, p95_tol=p95)
        if ok:
            valid.append((name, pv, metrics))
    if not valid:
        return fitting.fit_leg_refline(points, **kwargs)
    name, pv, metrics = min(valid, key=lambda row: (
        max(row[2][d]['max_m'] for d in ('source_to_target', 'target_to_source')),
        max(row[2][d]['p95_m'] for d in ('source_to_target', 'target_to_source'))))
    pv.fit_meta.update({'fit_selection': name, 'source_fidelity': metrics,
                       'impulse_filter': {'removed_count': 0, 'removed_indices': []}})
    return pv, max(metrics[d]['max_m'] for d in ('source_to_target', 'target_to_source')), True


def pin_speed_floor(candidate, baseline):
    """Freeze road/lane speed floor, even if semantic section stations changed."""
    previous = etree.parse(str(baseline))
    floor = {}
    def metres_per_second(speed):
        factor = {'m/s': 1., 'km/h': 1./3.6, 'mph': .44704}.get(speed.get('unit', 'm/s'))
        if factor is None:
            raise ValueError('unknown baseline speed unit')
        return float(speed.get('max'))*factor
    for road in previous.findall('road'):
        for lane in road.findall('.//lane'):
            for speed in lane.findall('speed'):
                key = (road.get('id'), lane.get('id'))
                floor[key] = max(floor.get(key, 0.), metres_per_second(speed))
    tree = etree.parse(str(candidate)); raised = []
    for road in tree.findall('road'):
        for lane in road.findall('.//lane'):
            key = (road.get('id'), lane.get('id'))
            for speed in lane.findall('speed'):
                old = metres_per_second(speed)
                if key in floor and old < floor[key]:
                    speed.set('max', str(floor[key]))
                    speed.set('unit', 'm/s')
                    raised.append({'road': key[0], 'lane': key[1],
                                   'before_mps': old, 'after_mps': floor[key]})
    tree.write(str(candidate), encoding='UTF-8', xml_declaration=True, pretty_print=True)
    return raised


def run(label, output_dir):
    name = dict(CASES)[label]
    node = parse_map_xml(str(ROOT/'v2x_map_xml'/name))
    neighbors = [n for _, file in CASES for n in parse_map_xml_all(str(ROOT/'v2x_map_xml'/file))
                 if (n.region, n.node_id) != (node.region, node.node_id)]
    baseline = ROOT/'out/m2x'/f'{label}.xodr'
    out = output_dir/f'{label}.xodr'
    if out.resolve() == baseline.resolve():
        raise ValueError('experiment cannot overwrite production artifact')
    out.parent.mkdir(parents=True, exist_ok=True)
    selections = []
    def choose(points, **kwargs):
        pv, dev, approx = accurate_single(points, **kwargs)
        selections.append(pv.fit_meta)
        return pv, dev, approx
    with patch.object(converter, 'fit_leg_refline', choose):
        stats = converter.build_xodr(node, out, neighbors=neighbors)
    raised = pin_speed_floor(out, baseline)
    final = finalize_opendrive_g8(out, stats['source_lane_manifest'], POLICY,
                                raw_map_paths=[ROOT/'v2x_map_xml'/file for _, file in CASES])
    tree = etree.parse(str(out))
    schema = etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    dense_policy = load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    dense_policy['dynamics']['sample_step_m'] = .02
    dense = _audit_d(tree.getroot(), dense_policy)
    report = {'schema': 'mapforge/map-single-primitive-candidate/v1',
              'candidate_only': True, 'production_promoted': False,
              'sha256': hashlib.sha256(out.read_bytes()).hexdigest(),
              'baseline_sha256': hashlib.sha256(baseline.read_bytes()).hexdigest(),
              'selections': selections, 'speed_floor_adjustments': raised,
              'xsd_15m': 'PASS' if schema.validate(tree) else 'FAIL',
              'g8_status': final['gate']['status'], 'g8_metrics': final['gate']['metrics'],
              'g11_status': final['g11']['status'], 'g11_summary': final['g11']['summary'],
              'dense_2cm_dynamics': dense,
              'conversion_stats': {k: v for k, v in stats.items() if k != 'source_lane_manifest'}}
    final['decision']['status'] = 'BLOCKED'
    final['decision']['candidate_only'] = True
    final['decision']['blocked_reasons'].append({'code': 'candidate_not_promoted'})
    final['quality']['delivery_decision'] = final['decision']
    for suffix, value in (('.experiment.json', report), ('.delivery-decision.json', final['decision']),
                          ('.quality-report.json', final['quality'])):
        out.with_suffix(suffix).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in (
        'conversion_stats', 'dense_2cm_dynamics')}, ensure_ascii=True))
    print('DENSE', dense['status'], json.dumps(dense['roads']))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('case', choices=dict(CASES))
    p.add_argument('out_dir', type=Path)
    a = p.parse_args(); r = run(a.case, a.out_dir)
    sys.exit(0 if r['g11_status'] == 'PASS' and r['dense_2cm_dynamics']['status'] == 'PASS' else 1)
