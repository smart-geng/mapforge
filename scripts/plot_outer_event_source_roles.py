"""Show the already-approved distinction: physical borders vs source path.

Read-only source illustration, not a new source-role decision or a vehicle
trajectory proof. The source coordinate transformation is the registered
research frame; it does not validate absolute CRS.
"""
import argparse
import json
import sys
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
from pyproj import Transformer
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.ops.arc_source_chart import ArcChart
from mapforge.repair_web.model import atomic,json_bytes,digest


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path)
    dest=p.parse_args().directory.resolve()
    if not dest.is_relative_to(ROOT/'out'):raise ValueError('workspace out/ only')
    if (dest/'source-role-comparison.png').exists():raise ValueError('do not overwrite source evidence')
    packet_path=ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json'
    packet=json.loads(packet_path.read_text(encoding='utf8'))
    root=ET.parse(ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr')
    road=next(r for r in root.findall('road') if r.get('id')=='11');g=road.find('planView/geometry')
    axis=ArcChart((float(g.get('x')),float(g.get('y'))),float(g.get('hdg')),0)
    project=Transformer.from_crs(4326,root.findtext('header/geoReference'),always_xy=True)
    def local(points):return axis.project(np.array([project.transform(float(p[0]),float(p[1])) for p in points]))
    ids=['2023041111104128071','2023041111104060474','2023041111104058460']
    fig,axes=plt.subplots(3,1,figsize=(12,10),layout='constrained');evidence=[]
    for ax,sid in zip(axes,ids):
        obs=packet['observations'][sid];boundaries=[]
        for rel in obs['boundary_relations']:
            parts=[part for record in packet['boundaries'][rel['boundary_key']]['records'] for part in record['parts']]
            if len(parts)!=1:raise ValueError('explicit multiparts needed for this diagnostic')
            st=local(parts[0]);st=st if st[-1,0]>st[0,0] else st[::-1]
            if np.any(np.diff(st[:,0])<=0):raise ValueError('nonmonotone original boundary')
            ax.plot(st[:,0],st[:,1],'-o',color='#d68b32',markersize=3,label='Original physical boundary' if not boundaries else None)
            boundaries.append(st)
        a,b=boundaries;ss=np.unique(np.r_[a[:,0],b[:,0]])
        ss=ss[(ss>=max(a[0,0],b[0,0])) & (ss<=min(a[-1,0],b[-1,0]))]
        midpoint=(np.interp(ss,a[:,0],a[:,1])+np.interp(ss,b[:,0],b[:,1]))*.5
        ax.plot(ss,midpoint,'--',color='#555',label='Boundary-derived midpoint (not source path)')
        path=local(obs['lane_path']);ax.plot(path[:,0],path[:,1],'-o',color='#25836d',label='Original lane path, unchanged')
        ax.set_title(f'Source lane {sid}: original start/end width {obs["start_width_mm"]}/{obs["end_width_mm"]} mm')
        ax.set_xlabel('Station in fixed research reference chart (m)');ax.set_ylabel('Lateral coordinate t (m)')
        ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8)
        evidence.append({'source_lane':sid,'start_width_mm':obs['start_width_mm'],'end_width_mm':obs['end_width_mm'],
                         'original_path_st':path.tolist(),'original_boundary_st':[v.tolist() for v in boundaries],
                         'source_speed_limit_kmh':obs['source_max_speed_kmh']})
    fig.suptitle('Source paths and physical lane midpoints are distinct near zero-width events\n'
                 'Existing source-role decisions only; no source changes and no new acceptance waiver',fontsize=12)
    dest.mkdir(parents=True,exist_ok=True);fig.savefig(dest/'source-role-comparison.png',dpi=145);plt.close(fig)
    atomic(dest/'source-role-comparison.json',json_bytes({'packet_sha256':digest(packet_path.read_bytes()),
        'coordinate_scope':'registered local research frame, not absolute CRS proof','rows':evidence}))


if __name__=='__main__':main()
