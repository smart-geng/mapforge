"""One node4 scope census: list NEW decisions, never grant or rebind them."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump
from mapforge.validate.shp_boundary_fidelity import _project


def remaining(report):
    if set(report['roads']) != {'10', '11', '12', '13'}:
        raise ValueError('all four ordinary source blocks required')
    inventory = report['raw_endpoint_inventory']
    keys = {(r['source_lane_id'], r['contact']) for r in inventory}
    if len(keys) != len(inventory) or len(keys) != 2*report['raw_lane_count']:
        raise ValueError('incomplete or duplicate original endpoint census')
    if report['missing_compiled_conflicts']:
        raise ValueError('source compiler omitted independently observed conflicts')
    # Existing exact-ID decisions are listed for review ONLY. A larger source
    # packet is not a valid automatic migration of either old decision file.
    return [r for r in report['rows'] if r['requires_source_role_decision']
            and not r['listed_decisions'] and not r['listed_in_existing_north_decision']]


def run(directory, output=None):
    directory = Path(directory).resolve()
    path = directory/'independent-role-review/report.json'
    report = json.loads(path.read_text(encoding='utf-8'))
    unchanged(report['hashes']); pending = remaining(report)
    domain = json.loads((directory/'input/source-domain.json').read_text(encoding='utf-8'))
    projection = domain['partition']['projection']
    hashes = dict(report['hashes'])
    for p in (path, Path(__file__)): hashes[str(p)] = _sha256(p)
    out = Path(output).resolve() if output else directory/'pending-review'
    out.mkdir(exist_ok=False)
    packet = dict(status='NEW_ROLE_DECISIONS_REQUIRED_NOT_APPROVED',
        source_contact_sha256=report['source_contact_sha256'], raw_lanes=report['raw_lane_count'],
        raw_endpoints=report['raw_endpoint_count'], explicit_zero_endpoints=report['explicit_zero_endpoint_count'],
        existing_listed_decisions=len(report['rows'])-len(pending),
        pending=[dict(source_lane_id=r['source_lane_id'], contact=r['contact'], gap_m=r['gap_m'],
                      raw_layer=r['raw_lane']['layer'], raw_record_index=r['raw_lane']['record_index'],
                      raw_boundary_node=r['original_boundary_tips'][0]['node']) for r in pending],
        scope='node4 four ordinary source blocks and original incident TOPO support; not all SHP maps',
        raw_sources_modified=False, new_role_authorizations=0, decisions_rebound=False, hashes=hashes)
    dump(out/'pending-roles.json', packet)
    if not pending: return packet
    import numpy as np
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, len(pending), figsize=(6*len(pending), 9),
                             layout='constrained', squeeze=False)
    for col, r in enumerate(pending):
        for ax in axes[:, col]:
            for label, rec, color, style in [('path', r['raw_lane'], '#176bbb', '--'),
                    ('left boundary', r['raw_boundaries']['left'], '#e78c1a', '-'),
                    ('right boundary', r['raw_boundaries']['right'], '#228b5b', '-')]:
                xy = _project(np.asarray(rec['points']), projection['lat_0'], projection['lon_0'])
                ax.plot(*xy.T, linestyle=style, marker='o', ms=3, color=color, label='Original '+label)
            xy = np.array([r['lane_point_xy'], r['collapsed_point_xy']])
            ax.plot(*xy.T, 'r-', lw=2); ax.set_aspect('equal'); ax.grid(alpha=.2)
            ax.set_xlabel('local x / m'); ax.set_ylabel('local y / m')
        axes[0, col].set_title(r['source_lane_id']+' / start width = 0', fontsize=11)
        lo=xy.min(axis=0)-2; hi=xy.max(axis=0)+2
        axes[1,col].set_xlim(lo[0],hi[0]); axes[1,col].set_ylim(lo[1],hi[1])
        axes[1,col].set_title(f"Unresolved role gap {r['gap_m']:.3f}m")
    axes[0,0].legend(fontsize=8)
    fig.suptitle(f"Node4 complete four-arm census: {packet['raw_lanes']} lanes / {packet['raw_endpoints']} endpoints / "
                 f"{packet['explicit_zero_endpoints']} zero-width contacts\n"
                 f"{packet['existing_listed_decisions']} exact-ID decisions listed; {len(pending)} new contacts below. No source edits.")
    fig.savefig(out/'new-three-originals.png', dpi=140); plt.close(fig)
    unchanged(hashes)
    return packet


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('directory'); p.add_argument('--output'); a=p.parse_args()
    print(json.dumps(run(a.directory,a.output)['pending'], ensure_ascii=False))
