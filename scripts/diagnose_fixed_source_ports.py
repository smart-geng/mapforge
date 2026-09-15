"""Exact point-to-original-segment lower bounds at immutable opposite ports.

Only a conditional obstruction for the declared fixed-parent trial. It does
not prove that the source road cannot be rebuilt or OpenDRIVE cannot model it.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256,shp_source
from scripts.build_source_geometry_candidate import dump
from spikes.measured_connector_caps import frames,composite_sources,raw_curves,distances
from mapforge.ops.port_dependencies import PortDependencies
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def run(directory,output):
    directory=Path(directory).resolve();output=Path(output).resolve()
    report=json.loads((directory/'report.json').read_text(encoding='utf8'));path=Path(report['artifact'])
    if _sha256(path)!=report['sha256']:raise ValueError('declared fixed map changed')
    root=ET.parse(path).getroot();graph=PortDependencies(root);graph.validate_junction_table()
    src=shp_source();lat,lon=_origin(root);project=lambda p:_project(p,lat,lon);rows=[]
    for rid,ports in graph.connections.items():
        if not any(p.road=='10' for p in ports):continue
        road=graph.roads[rid];ff=frames(root,road)
        _,identity=composite_sources(root,road,src,project)
        for end,p in enumerate(ports):
            if p.road=='10':continue
            sid=identity['source_lane_ids'][0 if end==0 else -1]
            source=raw_curves(src,sid,project)
            for side in ('left','right'):
                edge=ff[end]['edges'][side];point=np.array([[edge['x'],edge['y']]])
                distance=float(distances(point,source[side])[0])
                rows.append(dict(road=rid,parent=p.road,lane=p.lane,contact=p.contact,
                    role='predecessor' if end==0 else 'successor',source_lane_id=sid,field=side,
                    fixed_world_xy=point[0].tolist(),source_vertices=source[side].tolist(),
                    exact_point_to_source_polyline_m=distance,
                    exceeds_ordinary_tail_budget=distance>.35+1e-7))
    conflicts=[r for r in rows if r['exceeds_ordinary_tail_budget']]
    result=dict(status='CONDITIONAL_FIXED_PORT_CONFLICT' if conflicts else 'NO_ENDPOINT_OBSTRUCTION_FOUND',
        artifact=str(path),sha256=_sha256(path),mutable_parent='10',rows=rows,
        conflicts=conflicts,threshold_m=.35,source_changed=False,source_roles_expanded=False,
        scope='fixed opposite physical edge endpoint versus exact original corresponding polyline; includes endpoints',
        whole_road_impossibility_proven=False,source_correction_authorized=False,production_accepted=False)
    if output.exists():raise FileExistsError('preserve prior diagnosis')
    dump(output,result)
    print(result['status'],[{k:r[k] for k in ('road','parent','lane','role','source_lane_id','field','exact_point_to_source_polyline_m')}
                            for r in conflicts],flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output');a=p.parse_args();run(a.directory,a.output)
