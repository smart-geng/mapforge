"""Replay raw source contacts and bridge E1's legacy ledger to finite XML targets."""
import json
from pathlib import Path
import sys

from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops.source_contacts import compile_source_contacts
from mapforge.repair_web.event_source_binding import bind_sources
from mapforge.repair_web.model import atomic, digest, json_bytes
from scripts.check_outer_event_control import SOURCE, REFERENCE, PACKET

PRIOR = ROOT/'out/node4-split-event-admission-e1-20260917'
CONTACTS = ROOT/'out/node4-whole-source-role-binding-20260914/whole-contact-model.json'
ROLES = ROOT/'out/node4-whole-source-model-r4-20260914/source-roles.json'
DEST = ROOT/'out/node4-event-source-binding-20260917'


def inputs():
    packet = json.loads(PACKET.read_bytes())
    domain = json.loads(PACKET.with_name('source-domain.json').read_bytes())
    contacts = json.loads(CONTACTS.read_bytes()); roles = json.loads(ROLES.read_bytes())
    return REFERENCE.read_bytes(), packet, domain, contacts, roles


def verify_raw_contacts(packet, domain, contacts):
    run = json.loads(PACKET.with_name('run.json').read_bytes())
    for filename, sha in run['input_files_sha256'].items():
        if digest(Path(filename).read_bytes()) != sha: raise ValueError('Original input/code drift: '+filename)
    source = ProfileSource(run['source_dir'], run['profile'])
    # Contacts are defined on the exact preparation revision, not relabeled r6.
    fresh = compile_source_contacts(etree.parse(str(SOURCE)).getroot(), source, packet, domain,
                                    chart_mode='exact-line-arc-v1')
    if fresh['content_sha256'] != contacts['content_sha256']:
        raise ValueError('Stored contacts differ from freshly read original node/TOPO evidence')


def main():
    if DEST.exists(): raise ValueError('Do not overwrite source binding evidence')
    bindings = json.loads((PRIOR/'binding.json').read_bytes())
    extra = [PRIOR/'admission.json', CONTACTS, ROLES, Path(__file__).resolve(),
        ROOT/'mapforge/repair_web/event_source_binding.py', ROOT/'tests/test_event_source_binding.py',
        ROOT/'mapforge/ops/source_contacts.py', ROOT/'mapforge/ops/source_roles.py',
        ROOT/'mapforge/ops/physical_continuations.py']
    for p in extra:
        sha = digest(p.read_bytes())
        if str(p) in bindings and bindings[str(p)] != sha: raise ValueError('Previously frozen file changed')
        bindings[str(p)] = sha
    def verify():
        for name, sha in bindings.items():
            if digest(Path(name).read_bytes()) != sha: raise ValueError('Input drift: '+name)
    verify(); args = inputs(); verify_raw_contacts(*args[1:4])
    report = bind_sources(*args); verify()
    DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json', json_bytes(bindings)); atomic(DEST/'source-bindings.json', json_bytes(report))
    atomic(DEST/'status.json', json_bytes(dict(status=report['status'], input_bindings=len(bindings), input_drift=0,
        source_contacts_recompiled_from_raw=True, geometry_solver_ran=False, new_xodr=False, map_accepted=False,
        artifact_sha256={p.name: digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()})))
    print(json.dumps(dict(status=report['status'], counts=report['binding_counts'],
        source_relations=report['source_relation_debt_count'],
        endpoint_observations=report['endpoint_observation_count'], source_constraints=report['source_constraint_status'],
        input_bindings=len(bindings))))


if __name__ == '__main__': main()
