"""Read-only E1 evidence, without fitting, exporting or changing old sessions."""
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.model import atomic, digest, json_bytes
from mapforge.repair_web.split_event_admission import admit
from scripts.check_outer_event_control import PACKET, REFERENCE, DECISION

DEST = ROOT/'out/node4-split-event-admission-e1-20260917'
PRIOR = ROOT/'out/node4-split-event-scope-review-20260917'
DOMAIN = PACKET.with_name('source-domain.json')


def load_report():
    return admit(REFERENCE.read_bytes(), json.loads(PACKET.read_bytes()), json.loads(DOMAIN.read_bytes()),
                 yaml.safe_load(DECISION.read_bytes()))


def plot(report, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 3, figsize=(15, 8), layout='constrained')
    for ax, cid in zip(axs.flat, report['dependency']['connectors']):
        entries = [r for r in report['tail_geometry']['rows'] if r['consumer'] == 'connector:'+cid]
        for row in entries:
            source = row['source_st']
            ax.plot([p[0] for p in source], [p[1] for p in source], color='#d58520', lw=2)
            if 'target_st' in row:
                target = row['target_st']
                ax.plot([p[0] for p in target], [p[1] for p in target], color='#3867ae', lw=1.2,
                        ls='--' if row['field'] == 'center' else '-')
        ax.set_title('Connector '+cid+' | edges + path (dashed target center)')
        ax.set_xlabel('road11 chart s (m)'); ax.set_ylabel('t (m)'); ax.grid(alpha=.2)
        ax.set_aspect('equal', adjustable='datalim')
    fig.suptitle('E1 unchanged XML tails | orange: raw source | blue: actual finite prefix\n'
                 '0.05m DIAGNOSTIC samples only; no new geometry; not map acceptance')
    fig.savefig(path, dpi=140); plt.close(fig)


def main():
    if DEST.exists(): raise ValueError('E1 evidence already exists; do not overwrite')
    bindings = json.loads((PRIOR/'binding.json').read_bytes())
    paths = [DOMAIN, PRIOR/'scope-review.json', Path(__file__).resolve(),
        ROOT/'mapforge/repair_web/split_event_admission.py', ROOT/'tests/test_split_event_admission.py',
        ROOT/'mapforge/ops/source_domain.py', ROOT/'mapforge/ops/port_dependencies.py',
        ROOT/'mapforge/ops/reconstruction_scope.py', ROOT/'mapforge/validate/junction_edges.py',
        ROOT/'mapforge/validate/smoothness.py', ROOT/'scripts/review_measured_ribbon.py']
    for path in paths:
        sha = digest(path.read_bytes())
        if str(path) in bindings and bindings[str(path)] != sha: raise ValueError('Previously frozen file changed')
        bindings[str(path)] = sha
    def verify():
        for name, sha in bindings.items():
            if digest(Path(name).read_bytes()) != sha: raise ValueError('Binding drift: '+name)
    verify(); report = load_report(); verify()
    DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json', json_bytes(bindings))
    atomic(DEST/'contract.json', json_bytes(report['contract']))
    atomic(DEST/'admission.json', json_bytes(report))
    plot(report, DEST/'tail-readback.png'); verify()
    atomic(DEST/'status.json', json_bytes(dict(status=report['status'], input_bindings=len(bindings), input_drift=0,
        optimizer_calls=0, new_xodr=False, map_accepted=False, web_changed=False,
        artifact_sha256={p.name: digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()})))
    print(json.dumps(dict(status=report['status'], blockers=report['blockers'], input_bindings=len(bindings),
        unresolved_intervals=len(report['source']['unresolved']),
        unresolved_lengths=report['source']['unresolved_length_by_kind_m'],
        contacts=len(report['dependency']['edge_contacts']), contact_failures=len(report['dependency']['failures']),
        tail_comparisons=len(report['tail_geometry']['rows']))))


if __name__ == '__main__': main()
