"""Raw MAP geometric anomaly evidence. Read-only: never repairs or drops a point."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import CASES
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.ops.map_to_xodr import _project, _source_lane_key


def isolated_detours(points, *, offset_m=100., ratio_min=10., neighbor_span_max_m=100.):
    """Flag extreme local detours for REVIEW, not a source-invalid/PASS verdict."""
    pts = np.asarray(points, float)
    findings = []
    for i in range(1, len(pts)-1):
        a, p, b = pts[i-1:i+2]
        base = b-a; span = float(np.linalg.norm(base))
        if span < .01 or span > neighbor_span_max_m:
            continue
        t = float(np.clip((p-a)@base/(span*span), 0., 1.))
        offset = float(np.linalg.norm(p-(a+t*base)))
        detour = float(np.linalg.norm(p-a)+np.linalg.norm(b-p))
        if offset > offset_m and detour/span > ratio_min:
            findings.append({'status': 'REVIEW_REQUIRED', 'point_index_0based': i,
                              'distance_to_neighbor_chord_m': offset,
                              'neighbor_chord_length_m': span, 'detour_ratio': detour/span,
                              'source_action': 'NONE; original point retained'})
    return findings


def main():
    report = {'scope': 'isolated local detour review, not automatic source repair', 'cases': []}
    images = ROOT/'out/preview/map-real-source-review-v140'; images.mkdir(parents=True, exist_ok=True)
    for label, name in CASES:
        path = ROOT/'v2x_map_xml'/name; node = parse_map_xml(str(path)); findings = []
        for link in node.links:
            for lane in link.lanes:
                points = _project(lane.points, node.ref_lat, node.ref_lon) if lane.points else []
                for finding in isolated_detours(points):
                    i = finding['point_index_0based']
                    finding.update(source_lane_id=_source_lane_key(node, link, lane),
                                   longitude_e7=round(lane.points[i][0]*1e7), latitude_e7=round(lane.points[i][1]*1e7),
                                   original_local_xy=np.asarray(points).tolist())
                    findings.append(finding)
                    import matplotlib
                    matplotlib.use('Agg')
                    import matplotlib.pyplot as plt
                    fig, axes = plt.subplots(1, 2, figsize=(13, 6), dpi=150)
                    for ax in axes:
                        for other in link.lanes:
                            xy = _project(other.points, node.ref_lat, node.ref_lon)
                            ax.plot(*xy.T, marker='o', ms=3, lw=1., label=f'raw lane {other.lane_id}')
                        ref = _project(link.points, node.ref_lat, node.ref_lon)
                        ax.plot(*ref.T, '--', color='black', lw=1., label='raw Link.points')
                        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.set(xlabel='local x (m)', ylabel='local y (m)')
                    axes[0].annotate('Raw point, NOT removed', points[i], xytext=(points[i]+[-450., 200.]),
                                      arrowprops={'arrowstyle': '->', 'color': 'red'}, color='red')
                    axes[1].set(xlim=(-10., 90.), ylim=(0., 270.)); axes[1].legend()
                    fig.suptitle(f'{label} {link.name}: retained source detour {finding["distance_to_neighbor_chord_m"]:.1f} m')
                    fig.tight_layout(); fig.savefig(images/f'{label}-{link.name}-raw-detour.png'); plt.close(fig)
        report['cases'].append({'case': label, 'source': str(path),
                                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'findings': findings})
    out = ROOT/'out/map-raw-geometry-review-v140.json'
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps([(c['case'], c['findings']) for c in report['cases'] if c['findings']], ensure_ascii=True))


if __name__ == '__main__':
    main()
