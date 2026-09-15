"""Plot source interval accounting, NOT a fitted map or smoothness verdict."""
import argparse
import json
from collections import Counter
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mapforge.ops.reconstruction_scope import digest
from mapforge.validate.shp_boundary_fidelity import _project


def read_accounting(path):
    packet = json.loads(Path(path).read_text(encoding='utf-8'))
    if digest({k: v for k, v in packet.items() if k != 'content_sha256'}) != packet['content_sha256']:
        raise ValueError('source domain content mismatch')
    parts, totals = [], Counter()
    prj = packet['partition']['projection']
    for key, f in packet['partition']['features'].items():
        for part in f['parts']:
            xy = _project(np.asarray(part['raw_vertices']), prj['lat_0'], prj['lon_0'])
            ss = np.asarray(part['source_vertex_s_m'])
            for a in part['atoms']:
                lo, hi = a['source_s_m']; inner = (ss > lo) & (ss < hi)
                points = np.vstack([[np.interp(lo, ss, xy[:, d]) for d in (0, 1)], xy[inner],
                                    [np.interp(hi, ss, xy[:, d]) for d in (0, 1)]])
                totals[a['status']] += hi-lo
                parts.append({'key': key, 'kind': f['kind'], 'points': points,
                              'status': a['status'], 'length_m': hi-lo})
    return packet, parts, dict(totals)


def render(path, output):
    output = Path(output)
    if output.exists(): raise FileExistsError('do not overwrite previous review')
    packet, parts, totals = read_accounting(path)
    gaps = sorted((p for p in parts if p['status'] == 'UNASSIGNED_REQUIRES_SCOPE_DECISION'),
                  key=lambda p: p['length_m'], reverse=True)
    # Three separated original geometry locations, not three almost identical
    # neighboring edges masquerading as coverage of all problem regions.
    centers = []
    for p in gaps:
        mid = p['points'].mean(axis=0)
        if all(np.linalg.norm(mid-c[0]) > 12 for c in centers):
            centers.append((mid, p))
        if len(centers) == 3: break
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    colors = {'ASSIGNED': '#246bb2', 'SHARED_CONNECTOR_SUPPORT': '#c17d00',
              'UNASSIGNED_REQUIRES_SCOPE_DECISION': '#d52834',
              'CONFLICTING_STRUCTURAL_OWNERS': '#9927bb'}
    for ax in axes.flat:
        for part in sorted(parts, key=lambda p: p['status'] == 'UNASSIGNED_REQUIRES_SCOPE_DECISION'):
            xy = part['points']; color = colors[part['status']]
            ax.plot(xy[:, 0], xy[:, 1], color=color,
                    lw=2.8 if color == '#d52834' else .7, alpha=1 if color == '#d52834' else .6)
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.set_xlabel('local x (m)'); ax.set_ylabel('local y (m)')
    axes[0, 0].set_title('Full admitted source domain (not all-map coverage)')
    for ax, (mid, p) in zip(list(axes.flat)[1:], centers):
        ax.set_xlim(mid[0]-9, mid[0]+9); ax.set_ylim(mid[1]-9, mid[1]+9)
        ax.set_title('Unassigned original interval: %.3f m\n%s' % (p['length_m'], p['key']), fontsize=9)
    for ax in list(axes.flat)[1+len(centers):]: ax.set_visible(False)
    fig.suptitle('Source interval accounting / BLOCKED\n'
                 'Red = unassigned by current structural cuts; NOT lateral fitting error or paving holes', fontsize=13)
    handles = [Line2D([0], [0], color=c, lw=2, label=s) for s, c in colors.items()]
    fig.legend(handles=handles, loc='lower center', ncol=2, fontsize=9)
    fig.tight_layout(rect=(0, .07, 1, .94)); output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150); plt.close(fig)
    return {'source_domain_sha256': packet['content_sha256'], 'length_by_status_m': totals,
            'geometry_validation': 'NOT_RUN', 'output': str(output.resolve())}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__); ap.add_argument('source_domain'); ap.add_argument('output')
    args = ap.parse_args()
    print(json.dumps(render(args.source_domain, args.output), indent=2))
