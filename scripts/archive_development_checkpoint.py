"""Local-only checkpoint: committed code, immutable data and selected evidence.

Never mutates a source, saved project or running server. Not a release builder.
Session bearer tokens and third-party environments are excluded. The complete
out/ tree is deliberately NOT archived; explicit coverage lives in the manifest.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request, urlopen
import zipfile

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_ROOTS = [
    'out/checkpoint-20260915/validation',
    'out/node4-global-model-preflight-20260914',
    'out/node4-all-source-tail-readback-v171',
    'out/node4-whole-source-role-binding-20260914',
    'out/node4-whole-source-model-r4-20260914',
    'out/node4-whole-derivatives-r5-20260914',
    'out/node4-whole-joint-step-r2-20260914',
    'out/node4-whole-dynamics-r1-20260914',
    'out/node4-north-exit30-stable-source-v174',
    'out/node4-north-exit30-regular-domain-v174',
    'out/node4-local-repair-20260915',
    'out/node4-repair-ui-v2-20260915',
    'out/node4-orbit-evaluation-20260915/map',
    'out/node4-orbit-evaluation-20260915/shp',
]


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf8')


def git(*args):
    return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf8').strip()


def safe_relative(path):
    p=path.resolve()
    if not p.is_relative_to(ROOT):raise ValueError('Path outside workspace: '+str(p))
    if path.is_symlink():raise ValueError('Symlink is not an archive source: '+str(path))
    return p.relative_to(ROOT).as_posix()


def excluded(path):
    parts=set(path.relative_to(ROOT).parts)
    return bool(parts & {'__pycache__','.venv','venv','vendor','xml2xodr','.git'}) or path.name=='session.json' or path.suffix=='.pyc'


def live_snapshots():
    results=[]
    for folder in ('node4-local-repair-20260915','node4-repair-ui-v2-20260915'):
        saved=ROOT/'out'/folder/'project/session.json'
        try:
            url=urlsplit(json.loads(saved.read_text(encoding='utf8'))['url'])
            if url.scheme!='http' or url.hostname!='127.0.0.1':raise ValueError('Non-loopback URL rejected')
            token=parse_qs(url.fragment)['token'][0]
            req=Request(f'http://{url.netloc}/api/project',headers={'Authorization':'Bearer '+token})
            with urlopen(req,timeout=30) as r:state=json.load(r)['state']
            results.append({'project':folder,'status':'read_only_snapshot',**{k:state[k] for k in
                ('revision','saved_revision','values','sha256','can_undo','can_redo')},
                'full_in_memory_history_archived':False})
        except Exception as exc:
            # No exception text: URLs/headers must never leak credentials.
            results.append({'project':folder,'status':'unavailable','exception_type':type(exc).__name__})
    return results


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args();dest=args.destination.resolve()
    if not dest.is_relative_to(ROOT/'archives') or dest.exists():
        parser.error('destination must be a NEW child directory of workspace archives/')
    if git('diff','--cached','--name-only'):parser.error('commit staged changes before archiving')
    head=git('rev-parse','HEAD')
    if shutil.disk_usage(ROOT).free < 3*1024**3:parser.error('need at least 3 GiB free')
    selected=set();missing=[]
    for relative in ['shp_0222-0326','v2x_map_xml',*EVIDENCE_ROOTS]:
        path=ROOT/relative
        if not path.exists():missing.append(relative);continue
        for f in path.rglob('*'):
            if f.is_file() and not excluded(f):safe_relative(f);selected.add(f)
    for relative in ['OpenDRIVE_1.4H.xsd','OpenDRIVE_1.5M.xsd','esmini/version.txt',
        'out/node4-orbit-evaluation-20260915/seal.json',
        'out/node4-orbit-evaluation-20260915/no-edit-comparison.png']:
        p=ROOT/relative
        if p.is_file():selected.add(p)
        else:missing.append(relative)
    # Include the frozen execution dependency bytes for saved repair-project replay.
    bindings={}
    for folder in ('node4-local-repair-20260915','node4-repair-ui-v2-20260915'):
        saved=ROOT/'out'/folder/'project/project.json'
        value=json.loads(saved.read_text(encoding='utf8'))
        for name,digest in value['binding'].items():
            path=Path(name);safe_relative(path)
            if not path.is_file() or sha(path)!=digest:raise ValueError('Frozen input drift: '+name)
            selected.add(path);bindings[safe_relative(path)]=digest
    if missing:raise ValueError('Required evidence missing: '+', '.join(missing))
    files=sorted(selected,key=lambda p:safe_relative(p))
    total=sum(p.stat().st_size for p in files)
    if total>2*1024**3:raise ValueError('Selected evidence exceeds 2 GiB budget')
    dest.mkdir(parents=True)
    write_json(dest/'live-snapshots.json',live_snapshots())
    print(f'Archiving {len(files)} selected files, {total/1024**2:.1f} MiB before compression',flush=True)
    inventory=[]
    with zipfile.ZipFile(dest/'data-and-evidence.zip','x',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        for i,p in enumerate(files):
            before=sha(p);size=p.stat().st_size;relative=safe_relative(p)
            z.write(p,relative)
            if sha(p)!=before:raise ValueError('Source changed while archiving: '+relative)
            inventory.append({'path':relative,'bytes':size,'sha256':before})
            if i%100==0:print(f'  {i+1}/{len(files)} files',flush=True)
    # Validate actual decompressed member hashes, not just ZIP creation success.
    with zipfile.ZipFile(dest/'data-and-evidence.zip') as z:
        for row in inventory:
            h=hashlib.sha256()
            with z.open(row['path']) as f:
                for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
            if h.hexdigest()!=row['sha256']:raise ValueError('Archive readback mismatch: '+row['path'])
    for relative,digest in bindings.items():
        if sha(ROOT/relative)!=digest:raise ValueError('Bound file changed before seal: '+relative)
    git('archive','--format=zip','--output='+str(dest/'code.zip'),head)
    git('bundle','create',str(dest/'repository.bundle'),'--all')
    verify=subprocess.run(['git','bundle','verify',str(dest/'repository.bundle')],cwd=ROOT,capture_output=True,text=True)
    if verify.returncode:raise RuntimeError('Git bundle verification failed')
    with zipfile.ZipFile(dest/'code.zip') as z:
        if z.testzip() is not None:raise ValueError('Code ZIP CRC failure')
    packages=json.loads(subprocess.check_output([sys.executable,'-m','pip','list','--format=json'],text=True))
    write_json(dest/'environment.json',{'python':sys.version,'platform':platform.platform(),'packages':packages,
        'esmini_binary_included':False,'note':'Observed environment, not a minimal requirements lock. Reinstall official esmini separately.'})
    write_json(dest/'inventory.json',{'schema':'mapforge/development-archive/v1','created_at':datetime.now(timezone.utc).isoformat(),
        'commit':head,'branch':git('branch','--show-current'),'original_workspace':str(ROOT),
        'status':'RESEARCH_CHECKPOINT_NOT_MAP_DELIVERY','selected_files':len(files),'uncompressed_bytes':total,
        'coverage':'Original SHP and V2X materials plus explicit key evidence; NOT all out/ history.',
        'evidence_roots':EVIDENCE_ROOTS,'excluded':['runtime tokens','environments','third-party ORBIT code','xml2xodr reference code','esmini binaries','unselected out/ runs','unrelated weekly/daily documents'],
        'bound_input_count':len(bindings),'all_bound_input_hashes_unchanged':True,
        'zip_member_sha256_verified':True,'files':inventory})
    (dest/'START-HERE.md').write_text(
        '# MapFactory development checkpoint\n\nCommit: '+head+'\n\n'
        'This is NOT an accepted smooth map release. Read HANDOFF.md and docs/遗留工作-阶段收口-2026-09-15.md.\n\n'
        'code.zip: committed code and docs; repository.bundle: local Git history (git bundle verify passed).\n'
        'data-and-evidence.zip: original relative paths, verified by decompressed SHA-256; inventory.json lists scope.\n'
        'Restore code first, then data/evidence to the same workspace layout. Frozen projects contain absolute F:\\MapFactory paths; '
        'moving to another path needs explicit audited binding migration, NOT deleting checks.\n'
        'No venv, session bearer tokens, ORBIT source or esmini binaries are bundled. Reinstall official dependencies; environment.json records versions.\n'
        'live-snapshots.json contains only current geometry intent and revision, not full unsaved undo history. Servers were not stopped.\n'
        'All other out/ runs remain untouched in the original workspace and are not promised portable by this archive.\n',encoding='utf8')
    checks={p.name:sha(p) for p in dest.iterdir() if p.is_file()}
    write_json(dest/'SHA256.json',checks)
    print(json.dumps({'archive':str(dest),'commit':head,'files':len(files),'archive_members_verified':True,
        'bound_inputs_unchanged':len(bindings),'total_archive_bytes':sum(p.stat().st_size for p in dest.iterdir() if p.is_file())},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
