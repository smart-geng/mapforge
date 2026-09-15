"""Verify NODE5 complete raw coverage and joint-mouth candidate without promotion."""
import hashlib
import argparse
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.gen_all import CASES, POLICY
from spikes.joint_mouth_candidate import run as joint
from spikes.map_corner_surface import build as surface
from scripts.freeze_joint_mouth_candidates import source_plot, screenshots
from scripts.internal_edge_jets import audit as internal_audit
from scripts.esmini_lane_interfaces import check as consumer
from scripts.map_paving_audit import inspect as outline
from mapforge.report.decision import finalize_opendrive_g8
from mapforge.validate.g11 import load_policy, _audit_d
from mapforge.validate.map_source import audit_map_source_manifest


def verify(name, source, previous, target, images, *, baseline_title='v1.38 truncated source domain',
           candidate_title='v1.39 full original lane domain'):
    images.mkdir(parents=True, exist_ok=True)
    if not joint(source, target):
        raise RuntimeError('joint source candidate rejected')
    raw = [ROOT/'v2x_map_xml'/file for _, file in CASES]
    manifest = json.loads(target.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
    old_manifest = json.loads(previous.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
    root = ET.parse(target).getroot()
    paving = surface(root)
    ET.indent(root); ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    final = finalize_opendrive_g8(target, manifest, POLICY, raw_map_paths=raw)
    policy = load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    policy['dynamics']['sample_step_m'] = .02
    dense = _audit_d(root, policy)
    inside = internal_audit(root)
    engine = consumer(target, edges=True)
    envelope = outline(target, images/f'{name}-outline.png')
    source_plot(target, previous, images/f'{name}-source.png', baseline_title=baseline_title,
                candidate_title=candidate_title)
    screenshots(target, images/f'{name}-esmini.png')
    vertices = json.loads((images/f'{name}-source.json').read_text(encoding='utf-8'))['raw_vertex_fidelity']
    schema = etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    structures = []
    for road in root.findall('road'):
        if road.get('name') == 'junction_paving':
            continue
        lengths = [float(g.get('length')) for g in road.findall('planView/geometry')]
        structures.append({'road': road.get('id'), 'junction': road.get('junction'),
                            'reference_primitive_count': len(lengths), 'reference_lengths_m': lengths})
    report = {'candidate_only': True, 'delivery': 'BLOCKED',
              'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
              'gates': {k: v['status'] for k, v in final['quality']['gates'].items()},
              'old_manifest_raw_integrity': audit_map_source_manifest(old_manifest, raw),
              'internal_edges': inside['status'], 'dense_2cm': dense['status'],
              'esmini_center_and_edges': engine['status'], 'esmini_contacts': len(engine['rows']),
              'xsd': schema.validate(etree.parse(str(target))), 'structures': structures,
              'max_jerk_mps3': max(r['lateral_jerk_mps3'] for r in dense['roads']),
              'G8_metrics': final['gate']['metrics'], 'raw_vertices': vertices,
              'surface': paving, 'envelope': {k: v for k, v in envelope.items() if k != 'mouths'}}
    decision = final['decision']; decision['status'] = 'BLOCKED'; decision['candidate_only'] = True
    decision['blocked_reasons'] += [{'code': 'candidate_not_promoted'}, {'code': 'sim_auxiliary_only_not_ad_strict'}]
    for key in ('internal_edges', 'dense_2cm', 'esmini_center_and_edges'):
        if report[key] != 'PASS':
            decision['blocked_reasons'].append({'code': key+'_failed'})
    if not report['xsd']:
        decision['blocked_reasons'].append({'code': 'xsd_failed'})
    for suffix, value in (('.verification.json', report), ('.dense-all.json', dense),
                          ('.internal-edges.json', inside), ('.quality-report.json', final['quality']),
                          ('.delivery-decision.json', decision)):
        target.with_suffix(suffix).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in (
        'surface', 'envelope', 'structures', 'raw_vertices', 'G8_metrics', 'old_manifest_raw_integrity')}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', choices=dict(CASES), default='NODE5')
    parser.add_argument('--coordinate-v140', action='store_true')
    args = parser.parse_args(); name = args.case
    if args.coordinate_v140:
        verify(name, ROOT/'out/map-coordinate-v140'/f'{name}.xodr',
               ROOT/'out/m2x'/f'{name}.xodr', ROOT/'out/map-coordinate-review-v140'/f'{name}.xodr',
               ROOT/'out/preview/map-coordinate-review-v140', baseline_title='previous formal output',
               candidate_title='full source / joint coordinate candidate')
    else:
        verify(name, ROOT/'out/map-full-source-axis-v139'/f'{name}.xodr',
               ROOT/'out/map-joint-review-v138'/f'{name}.xodr', ROOT/'out/map-full-source-review-v139'/f'{name}.xodr',
               ROOT/'out/preview/map-full-source-review-v139')
