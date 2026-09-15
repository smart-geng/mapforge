"""Make a separate UI test project from a live geometry snapshot, never save the original.

Original in-memory undo/redo history stays in the original server. This new project
starts a new history from the snapshot, so it is not a full session migration.
"""
import argparse
import json
from pathlib import Path
import sys
from urllib.parse import urlsplit, parse_qs
from urllib.request import Request, urlopen

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.repair_web.model import Project, atomic, json_bytes


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    dest=args.out.resolve()
    if not dest.is_relative_to(ROOT/'out') or dest.exists():
        parser.error('output must be a new directory below workspace out/')
    session=json.loads(args.session.read_text(encoding='utf8'))
    url=urlsplit(session['url'])
    if url.scheme!='http' or url.hostname!='127.0.0.1':
        parser.error('only a registered loopback session is allowed')
    token=parse_qs(url.fragment)['token'][0]
    request=Request(f'http://{url.netloc}/api/project',headers={'Authorization':'Bearer '+token})
    with urlopen(request,timeout=30) as response:
        snapshot=json.load(response)
    p=Project(ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr',
        ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json',
        ROOT/'out/node4-global-model-preflight-20260914/final-input/run.json',dest/'project')
    p.timeline=[snapshot['state']['values']];p.cursor=0
    if p.snapshot()['sha256']!=snapshot['state']['sha256']:
        raise ValueError('Live geometry did not replay identically; original remains unchanged')
    p.save(p.revision)
    evidence={'purpose':'UI-only separate copy; original server not mutated',
        'original_revision':snapshot['state']['revision'],'values':snapshot['state']['values'],
        'snapshot_sha256':snapshot['state']['sha256'],'original_workspace':snapshot['workspace'],
        'original_saved_revision':snapshot['state']['saved_revision'],
        'bound_inputs':len(p.binding),'history_policy':'new history from live values, original history retained on original server'}
    atomic(dest/'copy-provenance.json',json_bytes(evidence))
    print(json.dumps(evidence,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
