"""Freeze explicitly selected MAP experiments; never replace formal outputs."""
import json
import hashlib
import sys
from pathlib import Path
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.report.decision import finalize_opendrive_g8
from mapforge.validate.g11 import _audit_d, load_policy
from scripts.esmini_lane_interfaces import check as esmini_check

SELECTED = {'node3': 'map-single-endpoint-candidate', 'NODE5': 'map-family-t010',
            'node13': 'map-family-t020', 'node16': 'map-family-candidate'}


def main():
    folder = ROOT/'out/map-review-v136'; folder.mkdir(parents=True, exist_ok=True)
    policy = load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    policy['dynamics']['sample_step_m'] = .02
    summary = []
    for case, source_folder in SELECTED.items():
        source = ROOT/'out'/source_folder/(case+'.xodr'); target = folder/(case+'.xodr')
        tree = etree.parse(str(source)); corrected = 0
        # Earlier converter speed-cap metadata is historical after the experiment
        # raised speeds back to the baseline floor. Do not leave it claiming that
        # a lower safety cap remains the actual written speed.
        for lane in tree.findall('.//lane'):
            item = lane.find("userData[@code='mapforge.provenance/v1']")
            if item is None: continue
            prov = json.loads(item.get('value'))
            speed = max((float(s.get('max')) for s in lane.findall('speed')), default=0.)
            cap = prov.get('inferred_speed_cap_ms')
            if cap is not None and speed > float(cap)+1e-8:
                keys = ('speed_adjustment', 'inferred_speed_cap_ms', 'speed_cap_basis')
                previous = {k: prov.pop(k) for k in keys if k in prov}
                prov['candidate_previous_speed_decision'] = previous
                prov['speed_adjustment'] = 'candidate-baseline-speed-floor'
                prov['candidate_written_speed_ms'] = speed
                prov['candidate_only'] = True
                item.set('value', json.dumps(prov, ensure_ascii=False, separators=(',',':')))
                corrected += 1
        tree.write(str(target), encoding='UTF-8', xml_declaration=True, pretty_print=True)
        manifest = json.loads(source.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
        final = finalize_opendrive_g8(target, manifest, ROOT/'profiles/validation/g8-opendrive-jinfeng-v1.yaml')
        dense = _audit_d(tree.getroot(), policy)
        consumer = esmini_check(target)
        decision = final['decision']; decision['status'] = 'BLOCKED'
        decision['candidate_only'] = True
        decision['blocked_reasons'] += [{'code':'candidate_not_promoted'}, {'code':'visual_outer_edge_steps_unresolved'}]
        if dense['status'] != 'PASS': decision['blocked_reasons'].append({'code':'dense_dynamics_failed'})
        if consumer['status'] != 'PASS': decision['blocked_reasons'].append({'code':'independent_interface_failed'})
        final['quality']['delivery_decision'] = decision
        report = {'case':case, 'source_experiment':str(source), 'candidate_only':True,
                  'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
                  'speed_metadata_corrected':corrected, 'G8':final['gate']['status'],
                  'G11':final['g11']['status'], 'dense_2cm':dense['status'],
                  'esmini_interfaces':consumer['status'], 'delivery':'BLOCKED',
                  'source_scope':final['gate']['scope'],
                  'source_fidelity':final['gate']['metrics'],
                  'interfaces':final['g11']['groups']['G11-C']['metrics'],
                  'max_dense_jerk_mps3':max(x['lateral_jerk_mps3'] for x in dense['roads'])}
        for suffix, data in (('.dense-all.json',dense),('.verification.json',report),
                             ('.delivery-decision.json',decision),('.quality-report.json',final['quality'])):
            target.with_suffix(suffix).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        summary.append(report); print('FROZEN',case,report['G11'],report['dense_2cm'],flush=True)
    (folder/'review-summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__': main()
