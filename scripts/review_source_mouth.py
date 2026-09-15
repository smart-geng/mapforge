"""Independent original-center/edge review of changed ordinary-mouth windows."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from spikes.mouth_anchor_review import observations,project_curve
from scripts.gen_all import shp_source
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def review(path,folder):
    root=ET.parse(path).getroot();src=shp_source();lat,lon=_origin(root)
    project=lambda x:_project(x,lat,lon)
    meta=json.loads(path.with_suffix('.mouth.json').read_text(encoding='utf-8'))
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    if meta.get('sha256')!=digest:raise ValueError('mouth report SHA mismatch')
    selected=[r for r in meta['rows'] if r['status']=='CANDIDATE']
    if not selected:raise ValueError('no changed source mouth windows')
    fig,axes=plt.subplots(len(selected),1,figsize=(11,4.5*len(selected)),squeeze=False,layout='constrained')
    rows=[]
    for ax,entry in zip(axes.flat,selected):
        road=root.find("road[@id='%s']"%entry['road']);lo,hi=entry['window_original_s']
        stations=list(np.linspace(lo,hi,max(2,int(np.ceil((hi-lo)/.1))+1)))
        sids={e.get('value') for sec in road.findall('lanes/laneSection') if float(sec.get('s'))<=hi
              for e in sec.findall(".//userData[@code='mapforge.source_lane']")}
        # Include every raw vertex that falls in the changed window, not only
        # the regular evaluation grid. This does not prove between-point extrema.
        for sid in sids:
            raw=[src.lane(sid).geometry,*src.lane_boundary_geometries(sid)]
            for curve in raw:
                st=project_curve(curve,road,project)
                stations.extend(st[(st[:,0]>=lo)&(st[:,0]<=hi),0])
        data=[]
        for s in np.unique(stations):data.extend(dict(v,s=float(s)) for v in observations(road,src,project,float(s)))
        good=[v for v in data if v['supported']];tracks=defaultdict(list)
        for v in good:tracks[(v['source'],v['field'])].append(v)
        metrics={}
        for kind in ('center','edges'):
            errors=[v['error_m'] for v in good if (v['field']=='center')==(kind=='center')]
            metrics[kind]={'count':len(errors),'median_m':float(np.median(errors)),
                           'p95_m':float(np.percentile(errors,95)),'max_m':float(max(errors))}
        row={'road':entry['road'],'window_s_m':[lo,hi],'source_metrics':metrics,
             'unsupported_observations':len(data)-len(good),'observations':data,
             'status':'PASS' if len(data)==len(good) and all(m['max_m']<=.35+1e-7 for m in metrics.values()) else 'FAIL'}
        rows.append(row)
        for (sid,field),items in tracks.items():
            ss=[v['s'] for v in items];style='--' if field=='center' else '-'
            ax.plot(ss,[v['source_t'] for v in items],color='#333333',lw=1.1,linestyle=style)
            ax.plot(ss,[v['target_t'] for v in items],color='#176bbe',lw=1.,linestyle=style)
        ax.set_title(f"Road {entry['road']}: window source {row['status']}, center max {metrics['center']['max_m']:.3f}m, edges max {metrics['edges']['max_m']:.3f}m")
        ax.set_xlabel('s in written straight reference frame (m)');ax.set_ylabel('transverse offset t (m)');ax.grid(alpha=.2)
    fig.suptitle('Source mouth windows only: black=original SHP / blue=written geometry\nSolid=physical boundaries, dashed=lane centers; full map BLOCKED')
    folder.mkdir(parents=True,exist_ok=True);fig.savefig(folder/'source-mouth-windows.png',dpi=150);plt.close(fig)
    report={'status':'BLOCKED','artifact':str(path),'sha256':digest,
            'scope':'changed mouth windows at raw stations plus <=0.1m grid; not complete SHP map coverage',
            'rows':rows}
    (folder/'source-mouth-review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print([(r['road'],r['status'],r['source_metrics']) for r in rows]);return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('folder',type=Path)
    a=p.parse_args();review(a.source,a.folder)
