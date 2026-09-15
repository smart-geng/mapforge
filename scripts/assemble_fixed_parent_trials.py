"""Combine disjoint turn candidates only when their complete parent XML matches.

No geometric approval is inherited. The result still needs the full independent
source, shape, interface and rendered review.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump
from scripts.research_code_revision import bind_current_code
from scripts.review_fixed_parent_replacement import require_fixed_parent
from spikes.trial_xml import replace_road,write_trial


def lineage_to_parent(parent,trial,base_root,chosen,allow_current_code):
    """Trace exact hashed refinement seeds; all stages keep the same parents."""
    current=trial;seen=set();hashes={};changes=[]
    for _ in range(8):
        if current['input_sha256']==parent['sha256']:
            return hashes,changes
        if current['input_sha256'] in seen:raise ValueError('cyclic candidate ancestry')
        seen.add(current['input_sha256'])
        if 'input' not in current:raise ValueError('candidate uses another parent state without refinement lineage')
        path=Path(current['input']);record=path.parent/'report.json'
        if _sha256(path)!=current['input_sha256']:raise ValueError('refinement seed changed')
        previous=json.loads(record.read_text(encoding='utf-8'))
        if previous['sha256']!=current['input_sha256'] or Path(previous['artifact']).resolve()!=path.resolve():
            raise ValueError('refinement record does not match seed')
        require_fixed_parent(base_root,ET.parse(path).getroot(),chosen)
        for group in ('source_hashes','code_sha256'):
            checked,revisions=bind_current_code(previous[group]);unchanged(checked)
            if revisions and not allow_current_code:raise ValueError('explicit --allow-current-code required for refinement lineage')
            changes.extend(revisions)
        for item in (path,record):hashes[str(item)]=_sha256(item)
        current=previous
    raise ValueError('candidate ancestry exceeds bounded refinement depth')


def run(baseline, trials, output, allow_current_code=False):
    baseline=Path(baseline).resolve(); output=Path(output).resolve()
    parent=json.loads((baseline/'report.json').read_text(encoding='utf-8'))
    path=Path(parent['artifact'])
    if _sha256(path)!=parent['sha256']:raise ValueError('baseline changed')
    hashes, changes=bind_current_code(parent['source_hashes']);unchanged(hashes)
    original=ET.parse(path); combined=copy.deepcopy(original); rows=[]; seen=set()
    provenance=[]; codes={str(Path(__file__)):_sha256(Path(__file__))}
    for folder in map(lambda p:Path(p).resolve(),trials):
        report_path=folder/'report.json'; trial=json.loads(report_path.read_text(encoding='utf-8'))
        if _sha256(Path(trial['artifact']))!=trial['sha256']:raise ValueError('candidate changed')
        checked,source_changes=bind_current_code(trial['source_hashes'])
        current_codes,code_changes=bind_current_code(trial['code_sha256'])
        if (source_changes or code_changes) and not allow_current_code:
            raise ValueError('explicit --allow-current-code required to independently recheck historical implementations')
        unchanged(checked);unchanged(current_codes)
        chosen={r['road'] for r in trial['connectors']}
        if seen & chosen or len(chosen)!=len(trial['connectors']):raise ValueError('duplicate turn replacement')
        ancestry,ancestry_changes=lineage_to_parent(parent,trial,original.getroot(),chosen,allow_current_code)
        hashes.update(ancestry)
        tree=ET.parse(trial['artifact'])
        require_fixed_parent(original.getroot(),tree.getroot(),chosen)
        for rid in chosen:
            old=next(r for r in combined.getroot().findall('road') if r.get('id')==rid)
            new=next(r for r in tree.getroot().findall('road') if r.get('id')==rid)
            replace_road(combined.getroot(),old,copy.deepcopy(new))
        rows.extend(trial['connectors']);seen|=chosen;codes.update(current_codes)
        for p in (report_path,Path(trial['artifact'])):hashes[str(p)]=_sha256(p)
        provenance.append(dict(report=str(report_path),sha256=_sha256(report_path),roads=sorted(chosen),
            refinement_ancestry=ancestry,implementation_changes_for_new_review=source_changes+code_changes+ancestry_changes))
    require_fixed_parent(original.getroot(),combined.getroot(),seen)
    output.mkdir(parents=True,exist_ok=False);target=output/'node4-review.xodr';write_trial(combined,target)
    unchanged(hashes);unchanged(codes)
    result=dict(status='REVIEW_NOT_DELIVERY',input=str(path),input_sha256=parent['sha256'],
        artifact=str(target),sha256=_sha256(target),source_hashes=hashes,code_sha256=codes,
        connectors=rows,combined_trials=provenance,input_code_revision_changes=changes,
        production_accepted=False,source_changed=False,unchanged_other_roads=True)
    dump(output/'report.json',result);print('COMBINED_NOT_ACCEPTED',sorted(seen),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('baseline');p.add_argument('output')
    p.add_argument('--trial',action='append',required=True);p.add_argument('--allow-current-code',action='store_true')
    a=p.parse_args();run(a.baseline,a.trial,a.output,a.allow_current_code)
