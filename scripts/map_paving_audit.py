"""Compare the inferred G2 mouth envelope with the actual exported road union.

MAP provides no measured curb polygon. The orange envelope is an inference,
not ground truth; this diagnostic isolates errors introduced while exporting it.
"""
import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.validate.smoothness import (
    _ref_kappa_at, lane_edges_kinematics_at, road_surface_polygon,
    sample_road_ref,
)


def xml_mouths(root):
    mouths = []
    for rd in root.findall('road'):
        if rd.get('junction') != '-1':
            continue
        link = rd.find("link/successor[@elementType='junction']")
        if link is None:
            raise ValueError('audit requires ordinary roads ending at junction')
        pts, ss, hh = sample_road_ref(rd)
        s = float(rd.get('length'))
        k, dk = _ref_kappa_at(rd, s)
        mouth = {'road_id':rd.get('id'), 'pose':[*pts[-1], float(hh[-1])]}
        for side in ('left', 'right'):
            t, dt, ddt = lane_edges_kinematics_at(rd, s, side)[-1]
            a, b = 1-k*t, dt
            mouth[side+'_t'] = t
            mouth[side+'_heading'] = float(hh[-1]+math.atan2(b,a))
            mouth[side+'_curvature'] = (a*(k*a+ddt)+b*(dk*t+2*k*dt))/(a*a+b*b)**1.5
        mouths.append(mouth)
    return mouths


def inspect(path, output):
    from spikes.map_corner_surface import analytic_envelope
    root = ET.parse(path).getroot()
    mouths = xml_mouths(root)
    envelope = analytic_envelope(mouths)
    all_roads = [(rd, road_surface_polygon(rd, .05)) for rd in root.findall('road')]
    paving = unary_union([p for r,p in all_roads if r.get('name') == 'junction_paving'])
    legs = unary_union([p for r,p in all_roads if r.get('junction') == '-1'])
    target = envelope.union(legs)
    written = unary_union([p for _,p in all_roads])
    excess = written.difference(target.buffer(.002))
    missing = target.difference(written.buffer(.002))
    bound = envelope.bounds
    fig, axes = plt.subplots(1, 2, figsize=(16,8), dpi=160)
    for ax in axes:
        for r,p in all_roads:
            for part in getattr(p, 'geoms', [p]):
                ax.fill(*part.exterior.xy, color='#777777', alpha=.12)
                if r.get('name') == 'junction_paving':
                    ax.plot(*part.exterior.xy, lw=.65, label='exported auxiliary surface')
        ax.plot(*envelope.exterior.xy, color='#ff8800', lw=1.4, label='inferred G2 envelope')
        for geom, color, label in ((excess,'red','outside envelope/legs'),
                                   (missing,'cyan','uncovered inferred envelope')):
            for p in getattr(geom, 'geoms', [geom]):
                if p.is_empty or not hasattr(p, 'exterior'): continue
                ax.fill(*p.exterior.xy, color=color, alpha=.75, label=label)
        ax.set_aspect('equal'); ax.grid(alpha=.2)
        ax.set(xlabel='local x (m)',ylabel='local y (m)')
    axes[0].set(xlim=(bound[0]-8,bound[2]+8), ylim=(bound[1]-8,bound[3]+8))
    worst = max(getattr(excess, 'geoms', [excess]), key=lambda g:g.area)
    if not worst.is_empty:
        x,y = worst.representative_point().coords[0]
        axes[1].set(xlim=(x-12,x+12),ylim=(y-12,y+12))
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    fig.legend(unique.values(), unique.keys(), loc='lower center', ncol=2)
    report = {'path':str(path), 'envelope_semantics':'INFERRED (not measured MAP curb)',
              'verification_curve_sample_step_m':.025,'comparison_buffer_m':.002,
              'excess_area_m2':float(excess.area), 'missing_area_m2':float(missing.area),
              'envelope_area_m2':float(envelope.area), 'mouths':mouths}
    fig.suptitle(f'{path.stem}: export excess {excess.area:.3f} m2, missing {missing.area:.3f} m2')
    fig.tight_layout(rect=(0,.08,1,.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output); plt.close(fig)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='mouths'}))
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('xodr',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();inspect(a.xodr,a.output)
