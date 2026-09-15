"""New readonly preparation + contact compilation; never export an XODR."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lxml import etree
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_contacts import compile_source_contacts
from scripts.gen_all import _sha256
from scripts.prepare_joint_reconstruction import prepare, verify_preparation


def build_from_preparation(directory):
    directory = Path(directory)
    check = verify_preparation(directory)
    if not check['source_domain'] or check['source_domain']['movement_status'] != 'MATCH':
        raise ValueError('explicit complete movement inventory required before contact compilation')
    run = json.loads((directory/'run.json').read_text(encoding='utf-8'))
    scope = json.loads((directory/'reconstruction-input.json').read_text(encoding='utf-8'))
    domain = json.loads((directory/'source-domain.json').read_text(encoding='utf-8'))
    source = ProfileSource(run['source_dir'], run['profile'])
    root = etree.parse(run['input']).getroot()
    result = compile_source_contacts(root, source, scope, domain)
    if any(_sha256(Path(f)) != h for f, h in run['input_files_sha256'].items()):
        raise ValueError('input changed during contact compilation')
    return result


def prepare_contacts(xodr, output, source_junction, xodr_junction, roads=('10',)):
    output = Path(output).resolve()
    if output.exists(): raise FileExistsError('use a new output directory; preserve previous evidence')
    # Parent directory is only created by the new preparation run, never erase
    # a prior baseline, accepted input or formal conversion output.
    prepare(xodr, output/'input', roads, source_junction=source_junction, xodr_junction=xodr_junction)
    model = build_from_preparation(output/'input')
    (output/'contact-model.json').write_text(json.dumps(model, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    summary = {'status': 'BLOCKED', 'geometry_solver_ran': False, 'export_allowed': False,
               'model_file_sha256': _sha256(output/'contact-model.json'),
               'preparation_files_sha256': {f: _sha256(output/'input'/f) for f in
                    ('run.json', 'reconstruction-input.json', 'source-domain.json')},
               'counts': {'full_source_features': len(model['full_source_support']),
                          'original_vertices': sum(len(p['vertex_indices']) for f in model['full_source_support'].values() for p in f['parts']),
                          'source_events': len(model['transition_events']),
                          'event_kinds': dict(Counter(e.get('kind','unresolved') for e in model['transition_events'])),
                          'physical_contact_pairs': sum(len(e.get('contacts',[])) for e in model['transition_events']),
                          'issues': model['issues'], 'role_conflicts': model['role_conflicts']},
               'previous_unassigned_by_class_m': model['previous_unassigned_by_class_m']}
    (output/'run.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


def verify_contacts(directory):
    directory = Path(directory)
    run = json.loads((directory/'run.json').read_text(encoding='utf-8'))
    if _sha256(directory/'contact-model.json') != run['model_file_sha256']:
        raise ValueError('contact file hash mismatch')
    expected = {'run.json', 'reconstruction-input.json', 'source-domain.json'}
    if set(run['preparation_files_sha256']) != expected: raise ValueError('incomplete preparation manifest')
    if any(_sha256(directory/'input'/f) != h for f, h in run['preparation_files_sha256'].items()):
        raise ValueError('contact preparation files changed')
    saved = json.loads((directory/'contact-model.json').read_text(encoding='utf-8'))
    if digest({k: v for k, v in saved.items() if k != 'content_sha256'}) != saved['content_sha256']:
        raise ValueError('contact body content mismatch')
    fresh = build_from_preparation(directory/'input')
    if fresh['content_sha256'] != saved['content_sha256']:
        raise ValueError('contact model differs from fresh original inputs')
    return {'fresh_read_match': True, 'status': 'BLOCKED', 'geometry_solver_ran': False, 'export_allowed': False,
            'role_conflicts': len(fresh['role_conflicts']), 'source_issues': len(fresh['issues'])}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('xodr'); p.add_argument('output'); p.add_argument('--road', action='append')
    p.add_argument('--source-junction'); p.add_argument('--xodr-junction'); p.add_argument('--verify', action='store_true')
    a = p.parse_args()
    if a.verify: print(json.dumps(verify_contacts(a.output), indent=2))
    else:
        if not a.source_junction or not a.xodr_junction: p.error('explicit source/XODR junction binding required')
        prepare_contacts(a.xodr, a.output, a.source_junction, a.xodr_junction, a.road or ['10'])
    raise SystemExit(2)
