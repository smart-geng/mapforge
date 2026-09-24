"""Bounded read-only E2 check. Existing artifacts and services stay untouched."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.event_shape_admission import review
from mapforge.repair_web.model import atomic, digest, json_bytes
from scripts.bind_split_event_sources import inputs

PRIOR = ROOT/'out/node4-event-source-binding-20260917'
DEST = ROOT/'out/node4-event-shape-admission-e2-20260917'


def load_report():
    data, packet, domain, _, _ = inputs()
    bound = json.loads((PRIOR/'source-bindings.json').read_bytes())
    return review(data, packet, domain, bound)


def plot(report, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), layout='constrained')
    for ax, cid in zip(axes.flat, report['frozen_connectors']):
        rows = [r for r in report['witnesses'] if r['consumer'] == 'connector:'+cid]
        for row in rows:
            raw = row['original_full_part_st']; actual = row['actual_prefix_points']
            # Draw original full part, then zoom; data/distance are never cropped.
            ax.plot([p[0] for p in raw], [p[1] for p in raw], color='#d38419', lw=2)
            ax.plot([p['st'][0] for p in actual], [p['st'][1] for p in actual], color='#2965a2', lw=1.5)
            if row['status'] == 'FIXED_POINT_VIOLATES_SOURCE_BOUND':
                w = row['witness']; foot = w['closest_original_st']
                ax.plot([w['st'][0], foot[0]], [w['st'][1], foot[1]], 'o-', color='#bd2528', markersize=3)
                ax.annotate(f"{w['distance_m']:.3f} m", w['st'], xytext=(3, 7), textcoords='offset points', fontsize=8)
        ax.set_title('Frozen connector '+cid); ax.set_xlim(211.45, 218.05)
        ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.2)
        ax.set_xlabel('parent chart s (m)'); ax.set_ylabel('t (m)')
    fig.suptitle('E2 necessary-condition check: unchanged actual XML points vs complete same-source SHP\n'
                 'Orange = source | blue = written edges | red = fixed violating witness | NOT a new map')
    fig.savefig(path, dpi=140); plt.close(fig)


def main():
    if DEST.exists(): raise ValueError('Do not overwrite E2 evidence')
    bindings = json.loads((PRIOR/'binding.json').read_bytes())
    for path in (PRIOR/'source-bindings.json', PRIOR/'status.json', Path(__file__).resolve(),
                 ROOT/'mapforge/repair_web/event_shape_admission.py', ROOT/'tests/test_event_shape_admission.py',
                 ROOT/'mapforge/ops/joint_connector_fit.py'):
        value = digest(path.read_bytes())
        if str(path) in bindings and bindings[str(path)] != value: raise ValueError('Prior binding drift')
        bindings[str(path)] = value
    def verify():
        for path, sha in bindings.items():
            if digest(Path(path).read_bytes()) != sha: raise ValueError('Input drift: '+path)
    verify(); report = load_report(); verify()
    DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json', json_bytes(bindings)); atomic(DEST/'admission.json', json_bytes(report))
    plot(report, DEST/'frozen-tail-witnesses.png'); verify()
    atomic(DEST/'status.json', json_bytes(dict(status=report['status'], input_bindings=len(bindings),
        input_drift=0, solver_calls=0, new_xodr=False, map_accepted=False,
        artifact_sha256={p.name: digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()})))
    print(json.dumps(dict(status=report['status'], failed_edges=report['failed_edge_consumer_count'],
        failed_connectors=report['failed_connectors'], input_bindings=len(bindings),
        max_witness_m=max(r['witness']['distance_m'] for r in report['witnesses']))))


if __name__ == '__main__': main()
