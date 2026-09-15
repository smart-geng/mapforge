"""Read-only parent-port vs original TOPO geometry diagnosis.

Polyline headings are observations, not certified smooth endpoint jets. This
report never grants permission to move a parent or proves infeasibility.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256,shp_source
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump
from scripts.research_code_revision import bind_current_code
from spikes.measured_connector_caps import frames,composite_sources
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def observed_heading(points,at_end):
    differences=np.diff(np.asarray(points,float),axis=0)
    lengths=np.linalg.norm(differences,axis=1)
    nonzero=differences[lengths>1e-8]
    if not len(nonzero):raise ValueError('nondegenerate original heading required')
    d=nonzero[-1 if at_end else 0]
    return float(np.arctan2(d[1],d[0]))


def run(directory,output):
    directory=Path(directory).resolve();output=Path(output).resolve()
    packet=directory/'report.json';info=json.loads(packet.read_text(encoding='utf8'))
    path=Path(info['artifact'])
    if _sha256(path)!=info['sha256']:raise ValueError('changed actual XML')
    hashes,changes=bind_current_code(info['source_hashes'])
    for p in (path,packet,Path(__file__)):hashes[str(p)]=_sha256(p)
    root=ET.parse(path).getroot();src=shp_source();lat,lon=_origin(root);records=[]
    for row in info['connectors']:
        road=next(r for r in root.findall('road') if r.get('id')==row['road'])
        ends=frames(root,road);raw,identity=composite_sources(root,road,src,lambda p:_project(p,lat,lon))
        contacts=[]
        for at_end,(role,frame) in enumerate(zip(('predecessor','successor'),ends)):
            fields={}
            for field in ('left','right','center'):
                actual=frame['center'] if field=='center' else frame['edges'][field]
                h=observed_heading(raw[field],bool(at_end));delta=actual['heading']-h
                source_point=raw[field][-1 if at_end else 0]
                fields[field]=dict(parent_heading_deg=float(np.degrees(actual['heading'])),
                    original_polyline_heading_deg=float(np.degrees(h)),
                    absolute_heading_difference_deg=float(abs(np.degrees(np.arctan2(np.sin(delta),np.cos(delta))))),
                    endpoint_separation_m=float(np.linalg.norm(source_point-[actual['x'],actual['y']])),
                    parent_written_curvature_per_m=actual['curvature'])
            contacts.append(dict(role=role,parent_road=road.find('link/'+role).get('elementId'),fields=fields))
        records.append(dict(road=row['road'],source_identity=identity,contacts=contacts))
    unchanged(hashes)
    if output.exists():raise FileExistsError('preserve prior source diagnosis')
    result=dict(status='PORT_OBSERVATION_DIAGNOSTIC_NOT_INFEASIBILITY_PROOF',artifact=str(path),sha256=info['sha256'],
        records=records,hashes=hashes,input_code_revision_changes=changes,source_changed=False,
        production_accepted=False,parent_edit_authorized_by_report=False,
        next_step='Jointly test source-constrained parent end jets and all incident turns; do not move one connector alone.')
    dump(output,result)
    print('PORT_DIAGNOSIS',len(records),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output');a=p.parse_args();run(a.directory,a.output)
