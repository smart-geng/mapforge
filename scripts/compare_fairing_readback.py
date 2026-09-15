"""Fixed-scale original-source overlays BEFORE/AFTER, read from actual XML."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scripts.gen_all import _sha256,shp_source
from scripts.build_source_geometry_candidate import dump
from scripts.review_fixed_parent_replacement import require_fixed_parent
from scripts.review_measured_ribbon import target_curves,fidelity
from scripts.review_turn_shape import reverse_turn,headings
from scripts.internal_edge_jets import states
from spikes.measured_connector_caps import composite_sources
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def run(before,after,output,roads):
    before,after,output=map(lambda p:Path(p).resolve(),(before,after,output))
    hashes={str(p):_sha256(p) for p in (before,after,Path(__file__))}
    roots=[ET.parse(p).getroot() for p in (before,after)]
    # A subset plot is only allowed for disjoint connector edits with the same
    # parents. Determine the actual complete changed set for the guard.
    original={r.get('id'):ET.tostring(r) for r in roots[0].findall('road')}
    changed={r.get('id') for r in roots[1].findall('road') if ET.tostring(r)!=original.get(r.get('id'))}
    require_fixed_parent(*roots,changed)
    src=shp_source();lat,lon=_origin(roots[0]);rows=[]
    fig,axes=plt.subplots(len(roads),2,figsize=(12,5*len(roads)),squeeze=False,layout='constrained')
    for i,rid in enumerate(roads):
        curves=[];record=dict(road=rid,versions=[])
        for j,root in enumerate(roots):
            road=next(r for r in root.findall('road') if r.get('id')==rid)
            raw,identity=composite_sources(root,road,src,lambda p:_project(p,lat,lon));actual=target_curves(road,.02)
            grid=np.linspace(0,float(road.get('length')),max(2,int(float(road.get('length'))/.02)+1))
            jets=np.array([states(road,-1,float(s),False) for s in grid])
            checks={k:reverse_turn(headings(raw[k]),jets[:,idx,2] if idx is not None else headings(actual[k]))
                    for k,idx in [('left',0),('right',1),('center',None)]}
            errors=fidelity(raw,actual)
            ax=axes[i,j]
            for field in raw:
                ax.plot(*raw[field].T,color='#df8915',lw=1.4,label='Full original SHP' if field=='left' else None)
                ax.plot(*actual[field].T,color='#1666ac',lw=1.1,ls='--' if field=='center' else '-',
                        label='Actual XODR' if field=='left' else None)
            # Never hide S/compound fields behind the old applicability test.
            worst=max(r['extra_reverse_turn_deg'] for r in checks.values())
            p95=max(v['p95_m'] for d in errors.values() for v in d.values())
            ax.set_title(f'{rid} | {"BEFORE" if j==0 else "AFTER (research only)"}\nmax P95 {p95:.3f}m | added turn (all fields) {worst:.3f}deg',
                         color='#a63226' if worst>1. else '#222222')
            ax.grid(alpha=.2);ax.set_aspect('equal');ax.legend(fontsize=8)
            curves.extend(list(raw.values())+list(actual.values()))
            record['versions'].append(dict(artifact=str((before,after)[j]),source_identity=identity,shape=checks,source=errors))
        if record['versions'][0]['source_identity']!=record['versions'][1]['source_identity']:
            raise ValueError('before/after original topology binding differs')
        allpts=np.vstack(curves);lo=allpts.min(axis=0)-1;hi=allpts.max(axis=0)+1
        for ax in axes[i]:ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1])
        rows.append(record)
    output.mkdir(parents=True,exist_ok=False)
    fig.suptitle('Same scale and complete original observations. Shape is not whole-map/dynamics approval.')
    fig.savefig(output/'comparison.png',dpi=150);plt.close(fig)
    from scripts.fit_source_boundary_block import unchanged
    unchanged(hashes)
    dump(output/'report.json',dict(status='VISUAL_REVIEW_NOT_ACCEPTANCE',records=rows,hashes=hashes,production_accepted=False))
    print(output/'comparison.png',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('before');p.add_argument('after');p.add_argument('output')
    p.add_argument('--road',action='append',required=True);a=p.parse_args();run(a.before,a.after,a.output,a.road)
