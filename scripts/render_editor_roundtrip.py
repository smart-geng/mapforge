"""Static side-by-side evidence of FINAL-XML edge samples, not editor screenshots."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def render(folder):
    fig, axes = plt.subplots(2, 2, figsize=(14, 11), dpi=140)
    for row, name in enumerate(('shp', 'map')):
        info = json.loads((folder/name/'comparison-r2.json').read_text(encoding='utf-8'))
        pts = np.load(folder/name/'comparison-r2.npz')['edges']
        # Exclude only the two explicitly named pre-existing auxiliary paving
        # roads from BOTH panels. All matched drivable road IDs remain plotted.
        excluded = [int(r['road']) for r in info['rows'] if r['name'] == 'junction_paving']
        data = pts[~np.isin(pts[:, 5], excluded)]
        for col in (0, 1):
            ax = axes[row, col]
            xy = data[:, col*2:col*2+2]
            ax.scatter(xy[:, 0], xy[:, 1], s=.35, color=('#2563a6' if col==0 else '#c25720'))
            ax.set_xlim(-410, 250); ax.set_ylim(-300, 310); ax.set_aspect('equal')
            ax.set_xlabel('Local x (m)'); ax.set_ylabel('Local y (m)'); ax.grid(alpha=.2)
            ax.set_title(name.upper() + ('-derived input: already REJECTED' if col==0 else ': ORBIT no-edit reopen/export'))
            meta = info['before' if col==0 else 'after']
            ax.text(.02, .98, f"{meta['geometry']} reference primitives (all roads)\n"
                    f"minimum {meta['min_geometry_m']:.4f} m", transform=ax.transAxes,
                    va='top', fontsize=9, bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
    fig.suptitle('Node4 editor admission: NO manual edit performed', fontsize=15)
    fig.text(.5, .025, 'Source: frozen input XODR vs ORBIT 8bd191af reopened XODR | 2026-09-15\n'
             'Same-ID/common-s edge samples <= 0.5 m + record boundaries; both sides checked.\n'
             'Auxiliary paving 90/91 excluded from both plots. Not a SHP fidelity, GUI or esmini acceptance.',
             ha='center', fontsize=9)
    fig.tight_layout(rect=(0,.085,1,.97))
    fig.savefig(folder/'no-edit-comparison.png')
    plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    render(parser.parse_args().folder)
