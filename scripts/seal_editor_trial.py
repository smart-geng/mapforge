"""Seal a completed ORBIT admission trial, without promoting a map."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.check_editor_roundtrip import sha, load, normalized


def run(folder):
    folder=folder.resolve(); target=folder/'seal.json'
    if target.exists(): raise FileExistsError(target)
    bound={}; stages=[]
    for case in ('shp','map'):
        path=folder/case
        reg=json.loads((path/'registration.json').read_text(encoding='utf-8'))
        bound.update(reg['input_hashes'])
        comp=json.loads((path/'comparison-r2.json').read_text(encoding='utf-8'))
        if sha(comp['source'])!=comp['source_sha256'] or sha(comp['target'])!=comp['target_sha256']:
            raise ValueError('comparison files changed')
        es=json.loads((path/'esmini-witnesses.json').read_text(encoding='utf-8'))
        assert es['comparison_sha256']==sha(path/'comparison-r2.json')
        a,b=(load(path/(n+'.xodr')) for n in ('direct','reopened'))
        a.find('header').attrib.pop('date',None);b.find('header').attrib.pop('date',None)
        stages.append(dict(case=case,status=comp['status'],
                           direct_reopen_equal_except_date=normalized(a)==normalized(b),
                           esmini_load_codes=[s['load_code'] for s in es['stages']]))
    changed=[p for p,h in bound.items() if sha(p)!=h]
    if changed: raise ValueError('immutable files changed: '+repr(changed))
    vendor=folder/'vendor/ORBIT-8bd191af7934d7e7f236229648c5716108ec042f'
    src=vendor/'orbit-core/src/orbit_core'
    installed=folder/'venv/Lib/site-packages/orbit_core'
    py=list(src.rglob('*.py'))
    mismatched=[str(p.relative_to(src)) for p in py if sha(p)!=sha(installed/p.relative_to(src))]
    if mismatched: raise ValueError('installed ORBIT differs from upstream: '+repr(mismatched))
    outputs={str(p.relative_to(folder)):sha(p) for case in ('shp','map') for p in (folder/case).iterdir() if p.is_file()}
    outputs['no-edit-comparison.png']=sha(folder/'no-edit-comparison.png')
    scripts=['orbit_roundtrip_worker','check_editor_roundtrip','render_editor_roundtrip','check_editor_esmini','seal_editor_trial']
    data=dict(status='ORBIT_DEFAULT_CORE_REJECTED_NO_MAP_DELIVERY',stages=stages,
              unchanged_bound_files=len(bound),installed_python_files_match_upstream=len(py),
              output_hashes=outputs,script_hashes={s:sha(ROOT/'scripts'/(s+'.py')) for s in scripts},
              immutable_files_changed=changed,gui_executed=False,manual_edits=0,
              source_crs_approved=False,map_accepted=False,
              xsd_1_8_validated=False,driving_controller_tested=False,
              note='r1 comparison speed-count superseded by r2 serialization-aware diagnostics')
    target.write_text(json.dumps(data,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in data.items() if k not in ('output_hashes','script_hashes')}))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    run(p.parse_args().folder)
