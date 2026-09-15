"""Fresh original-source replay for an explicit ordinary component in actual XML."""
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
from mapforge.ops.source_cubic_export import cubic_model,constrain_written_endpoints,compile_road
from mapforge.ops.reconstruction_scope import digest
from mapforge.validate.source_cubic_readback import audit_written_source


def run(directory,map_path,output):
    directory,map_path,output=[Path(p).resolve() for p in (directory,map_path,output)]
    output.mkdir(parents=True,exist_ok=False)
    state=json.loads((directory/'shared-state.json').read_text(encoding='utf8'));rid=state['road']
    if state.get('flat_port') or state.get('source_export_domain') is not None:
        raise ValueError('explicit nonflat common-source component required')
    original,roles,hashes,raw_root=load_road(state['source_directory'],state['decision_file'],rid,state.get('previous_source_directory'))
    model=constrain_written_endpoints(cubic_model(original,minimum_span=state['minimum_width_span'],end_axis=state['end_axis']))
    if digest(model.describe())!=state['model_sha']:raise ValueError('source/model changed')
    expected,compiled,_=compile_road(model,np.asarray(state['coefficients']),raw_root.find(f"road[@id='{rid}']"),
        source_direction=state.get('source_direction'))
    actual=ET.parse(map_path).getroot().find(f"road[@id='{rid}']")
    if actual is None:raise ValueError('parent missing from actual XML')
    audit=audit_written_source(actual,compiled,model,step=.02)
    def speeds(road):
        return [(s.get('s'),side,l.get('id'),[tuple(sorted(v.attrib.items())) for v in l.findall('speed')])
            for s in road.findall('lanes/laneSection') for side in ('left','right') for l in s.findall(side+'/lane')]
    from scripts.internal_edge_jets import audit as internal_audit
    from mapforge.validate.g11 import load_policy,_audit_d
    sub=ET.Element('OpenDRIVE');sub.append(actual);internal=internal_audit(sub)
    cfg=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml');cfg['dynamics']['sample_step_m']=.02
    dynamics=_audit_d(sub,cfg)
    from scripts.basemap_overlay import _lane_lines
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    edges,_=_lane_lines(actual,.02)
    fig,axes=plt.subplots(3,1,figsize=(16,10),layout='constrained')
    for ax in axes:
        for key,xy in model.source_xy.items():
            if key.startswith('boundary:'):ax.plot(*xy.T,c='#ed901c',lw=1.6)
        for xy,*_ in edges:ax.plot(*xy.T,c='#146ab3',lw=1.)
        ax.set_aspect('equal');ax.grid(alpha=.2)
    axes[0].set_title('Complete owned source boundary support')
    for ax,station,label in zip(axes[1:],[compiled['chart_start_m'],compiled['chart_end_m']],['Written start / original source tips','Written end / original source tips']):
        p,_,_=model.axis.frame(station);ax.set_xlim(p[0]-12,p[0]+12);ax.set_ylim(p[1]-12,p[1]+12);ax.set_title(label)
    fig.suptitle(f"road{rid}: ORANGE original SHP / BLUE actual written XML\n"
        f"Raw / reverse max {audit['source_to_written_max_m']:.6f} / {audit['written_to_source_max_m']:.6f} m; component ONLY, not entire map")
    fig.savefig(output/'actual-source.png',dpi=140);plt.close(fig)
    hashes.update({str(p):_sha256(p) for p in (map_path,directory/'shared-state.json',Path(__file__),
        ROOT/'mapforge/validate/source_cubic_readback.py',ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')})
    report=dict(status='ORDINARY_SOURCE_REVIEW_ONLY',road=rid,artifact=str(map_path),sha256=_sha256(map_path),
        written_source=audit,internal_edges=internal,source_speed_entries_match=speeds(actual)==speeds(expected),
        source_speed_entry_count=sum(len(v[3]) for v in speeds(actual)),source_speed_dynamics=dynamics,
        source_hashes=hashes,source_changed=False,source_roles=roles['resolved'],incident_connectors_validated=False,
        movement_paths_validated=False,production_accepted=False)
    unchanged(hashes);dump(output/'report.json',report)
    print('PARENT ACTUAL',rid,audit['status'],internal['status'],dynamics['status'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for n in ('directory','map_path','output'):p.add_argument(n)
    a=p.parse_args();run(a.directory,a.map_path,a.output);raise SystemExit(2)
