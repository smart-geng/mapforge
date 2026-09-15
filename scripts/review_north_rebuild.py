"""Fresh original-source replay of a non-flat north component IN a written map.

The saved optimizer verdict is not reused. This audit covers the whole
ordinary source support, not incident-turn validity or whole-map delivery.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from scripts.build_ordinary_source_road import load_road
from scripts.build_source_geometry_candidate import dump
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged
from mapforge.ops.source_cubic_export import cubic_model,constrain_written_endpoints,constrain_flat_connector_port,compile_road
from mapforge.ops.reconstruction_scope import digest
from mapforge.validate.source_cubic_readback import audit_written_source


def run(directory,map_path,output):
    directory=Path(directory).resolve();map_path=Path(map_path).resolve();output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    state=json.loads((directory/'shared-state.json').read_text(encoding='utf8'))
    if state['road']!='10' or state.get('source_export_domain') is not None:
        raise ValueError('explicit north common-domain component required')
    original,roles,hashes,raw_root=load_road(state['source_directory'],state['decision_file'],'10',
        state.get('previous_source_directory'))
    model=cubic_model(original,minimum_span=state['minimum_width_span'],end_axis=state['end_axis'])
    model=constrain_written_endpoints(model)
    if state['flat_port']:model=constrain_flat_connector_port(model)
    if digest(model.describe())!=state['model_sha']:raise ValueError('source or model changed since saved coefficients')
    hashes.update({str(p):_sha256(p) for p in (map_path,directory/'shared-state.json',directory/'report.json',
        Path(__file__),ROOT/'mapforge/validate/source_cubic_readback.py')})
    x=np.asarray(state['coefficients'])
    _,compiled,_=compile_road(model,x,raw_root.find("road[@id='10']"))
    actual=next(r for r in ET.parse(map_path).getroot().findall('road') if r.get('id')=='10')
    audit=audit_written_source(actual,compiled,model,step=.02)
    from scripts.internal_edge_jets import audit as internal_audit
    sub=ET.Element('OpenDRIVE');sub.append(actual)
    internal=internal_audit(sub)
    # Source speed boundaries are rebuilt from original observations by the
    # compiler; compare every actual entry, not only a nominal maximum.
    expected,_,_=compile_road(model,x,raw_root.find("road[@id='10']"))
    def speeds(road):
        return [(s.get('s'),side,l.get('id'),[tuple(sorted(v.attrib.items())) for v in l.findall('speed')])
            for s in road.findall('lanes/laneSection') for side in ('left','right') for l in s.findall(side+'/lane')]
    source_speeds_match=speeds(actual)==speeds(expected)
    from scripts.basemap_overlay import _lane_lines
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    edges,_=_lane_lines(actual,.02)
    fig,axes=plt.subplots(1,3,figsize=(16,9),layout='constrained')
    for ax in axes:
        for key,xy in model.source_xy.items():
            if key.startswith('boundary:'):ax.plot(*xy.T,c='#ed901c',lw=1.6)
        for xy,*_ in edges:ax.plot(*xy.T,c='#146ab3',lw=1.)
        ax.set_aspect('equal');ax.grid(alpha=.2)
    axes[0].set_title('Whole north physical boundary support')
    xy=np.vstack([g for g,*_ in edges]);ymin=xy[:,1].min()
    axes[1].set_xlim(xy[:,0].min()-1,xy[:,0].max()+1);axes[1].set_ylim(ymin-1,ymin+16)
    axes[1].set_title('Junction end: no artificial flat-port equality')
    axes[2].set_xlim(xy[:,0].min()-1,xy[:,0].max()+1);axes[2].set_ylim(55,87)
    axes[2].set_title('Original zero-width transition region')
    fig.suptitle('ORANGE: raw SHP boundaries | BLUE: actual written XODR\n'
        f"Raw / reverse max: {audit['source_to_written_max_m']:.6f} / {audit['written_to_source_max_m']:.6f} m. "
        'Ordinary component ONLY; no whole-map acceptance.')
    fig.savefig(output/'north-actual-source.png',dpi=150);plt.close(fig)
    report=dict(status='ORDINARY_SOURCE_REVIEW_ONLY',artifact=str(map_path),sha256=_sha256(map_path),
        source_state=str(directory),model_sha=state['model_sha'],flat_port=state['flat_port'],
        written_source=audit,internal_edges=internal,source_speed_entries_match=source_speeds_match,
        source_speed_entry_count=sum(len(v[3]) for v in speeds(actual)),
        source_hashes=hashes,source_roles=roles['resolved'],source_changed=False,
        no_new_role_authorizations=True,incident_connectors_validated=False,
        movement_paths_validated=False,production_accepted=False)
    unchanged(hashes);dump(output/'report.json',report)
    print('NORTH ACTUAL',audit['status'],internal['status'],'SOURCE SPEED',source_speeds_match,flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('map_path');p.add_argument('output')
    a=p.parse_args();run(a.directory,a.map_path,a.output);raise SystemExit(2)
