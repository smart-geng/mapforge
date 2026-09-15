"""Read back sparse-path trial parameters and independently recheck raw SHP.

No optimizer, no XODR export. Previous reports are never overwritten.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from lxml import etree
from pyclothoids import Clothoid

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source, _sha256
from scripts.sparse_source_path_trial import source_path
from mapforge.validate.shp_boundary_fidelity import _origin, _project
from spikes.sparse_path_model import Limits, sample, verify
from spikes.straight_transition_model import heading_extrema


CASES={
    'North/right -1':[
        ('Fixed 60 km/h; 3 spirals','sparse-path-v147-north-in',3),
        ('Speed diagnostic; 3 spirals','sparse-path-v147-north-in-frontier',3),
        ('Protected straights; 5 primitives','sparse-path-v147-north-in-protected',5)],
    'North/left +3':[
        ('Fixed 60 km/h; 3 spirals','sparse-path-v147-north-out',3),
        ('Speed diagnostic; 3 spirals','sparse-path-v147-north-out-frontier',3),
        ('Protected straights; 5 primitives','sparse-path-v147-north-out-protected',5)],
    'North/right -3':[
        ('One spiral','sparse-path-v147-north-straight',1),
        ('Line-first; one line','sparse-path-v147-line-first',1)]}


def run(output):
    if output.exists():raise FileExistsError('previous evidence must not be overwritten')
    src=shp_source();report={'status':'BLOCKED','kind':'independent-raw-path-parameter-recheck',
        'scope':'path models only; no lane widths, boundary graph, junction, MAP, XODR or driving approval',
        'untested':['full multi-lane shared reconstruction','source versus midpoint semantics at zero width',
                    'whole-map source coverage and speed','junction surface','MAP','absolute CRS'],
        'cases':{}}
    protected={}
    fig,axes=plt.subplots(2,3,figsize=(19,10),dpi=130,layout='constrained')
    for column,(name,files) in enumerate(CASES.items()):
        rows=[]
        for index,(label,directory,count) in enumerate(files):
            path=ROOT/'out'/directory/'report.json'
            old=json.loads(path.read_text(encoding='utf-8'))
            base=Path(old['input']);digest=_sha256(base)
            if digest!=old['input_sha256']:raise ValueError('trial parent changed')
            protected[str(base)]=digest;protected[str(path)]=_sha256(path)
            for rel,d in old.get('source_files_sha256',{}).items():
                if _sha256(ROOT/rel)!=d:raise ValueError('source changed since trial')
                protected[str(ROOT/rel)]=d
            root=etree.parse(str(base)).getroot();lat,lon=_origin(root)
            road=next(r for r in root.findall('road') if r.get('id')==old['road'])
            raw,ids,speed=source_path(road,src,old['side'],old['lane'],lambda g:_project(g,lat,lon))
            if ids!=old['source_lane_ids'] or not np.array_equal(raw,np.asarray(old['raw_xy'])):
                raise ValueError('stored raw snapshot is not the full current original path')
            trial=next(t for t in old['result']['trials'] if t.get('primitive_count')==count)
            cc=[Clothoid.StandardParams(*p) for p in trial['parameters']]
            limits=Limits(**old['limits'])
            if speed!=limits.speed_ms:raise ValueError('requested speed was changed')
            checked=verify(cc,raw,limits)
            if 'Protected' in label:
                h0=cc[0].ThetaStart;h1=cc[-1].ThetaEnd
                delta=np.array([cc[-1].XEnd-cc[0].XStart,cc[-1].YEnd-cc[0].YStart])
                sign=np.sign(delta@np.array([-np.sin(h0),np.cos(h0)]))
                angles=sign*(heading_extrema(cc)-h0)
                straight_ok=all(c.KappaStart==0 and c.dk==0 for c in (cc[0],cc[-1]))
                envelope_ok=bool(min(angles)>=min(0.,sign*(h1-h0))-1e-8 and max(angles)<=.6+1e-8)
                checked['protected_straights_recheck']={'line_caps':straight_ok,'heading_envelope':envelope_ok}
                if not straight_ok or not envelope_ok:checked['status']='REJECTED'
            checked.update(label=label,trial_path=str(path.relative_to(ROOT)),
                           raw_source_matches=True,original_topology_and_speed_match=True,
                           source_lane_ids=ids,source_speeds_kmh=[src.lane(sid).max_speed_kmh for sid in ids],
                           parameters=[list(c.Parameters) for c in cc])
            rows.append(checked)
            h=float(road.find('planView/geometry').get('hdg'))
            rotation=np.array([[np.cos(h),-np.sin(h)],[np.sin(h),np.cos(h)]])
            chart=lambda p:(np.asarray(p)-raw[0])@rotation
            if index==0:
                axes[0,column].plot(*chart(raw).T,'o-',c='black',lw=2,ms=4,label='Full original path')
            color=('#426dca','#d97820','#15926b')[index]
            axes[0,column].plot(*chart(sample(cc,.1)).T,c=color,label=label,lw=1.6)
            offset=0.
            for ci,c in enumerate(cc):
                axes[1,column].plot([offset,offset+c.length],[c.KappaStart,c.KappaEnd],c=color,
                                   lw=1.6,label=label if ci==0 else None)
                offset+=c.length
        report['cases'][name]=rows
        axes[0,column].set(title=name,xlabel='longitudinal chart x (m)',ylabel='transverse chart y (m)')
        axes[1,column].set(xlabel='actual curve arc length (m)',ylabel='curvature (1/m)')
        for ax in axes[:,column]:ax.legend(fontsize=8);ax.grid(alpha=.2)
    report['input_sha256']=protected
    report['inputs_unchanged']=all(_sha256(Path(p))==d for p,d in protected.items())
    if not report['inputs_unchanged']:raise ValueError('input changed during recheck')
    output.mkdir(parents=True)
    (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    fig.suptitle('Original paths vs few-primitive models - NOT an accepted XODR\nTransverse scale exaggerated; speed diagnostics remain REJECTED at the original 60 km/h')
    fig.savefig(output/'comparison.png');plt.close(fig)
    print(json.dumps({name:[{k:r[k] for k in ('label','status','source_to_curve_max_m',
        'source_tube_upper_bound_m','reverse_tube_upper_bound_m','tube_certified','jerk_mps3','supported_speed_kmh')}
        for r in rows] for name,rows in report['cases'].items()},indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path)
    run(p.parse_args().output)
    raise SystemExit(2)
