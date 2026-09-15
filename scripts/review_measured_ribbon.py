"""Full raw via center/edges + source-linked extensions, independent directions.

The optimizer's pooled/0.5m objective is NOT an acceptance result. This review
keeps all raw vertices, samples written geometry at 0.05m, checks exact cubic
width extrema, and leaves the complete candidate blocked when any road fails.
"""
import argparse
import copy
import hashlib
import json
import sys
import textwrap
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString,Polygon

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source
from spikes.measured_connector_caps import raw_curves,composite_sources,distances
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.validate.smoothness import sample_road_ref,lane_edges_at,junction_lane_interfaces
from mapforge.validate.junction_edges import audit as edge_audit
from mapforge.validate.g11 import load_policy,_audit_d
from scripts.internal_edge_jets import audit as internal_edge_audit


def target_curves(road,step=.05):
    xy,ss,hh=sample_road_ref(road,step);normal=np.c_[-np.sin(hh),np.cos(hh)]
    es=np.array([lane_edges_at(road,float(s),'right') for s in ss])
    if es.shape[1]!=2:raise ValueError('one right lane connector required')
    left=xy+es[:,0,None]*normal;right=xy+es[:,1,None]*normal
    return {'left':left,'right':right,'center':(left+right)/2}


def minimum_width(road):
    """Polynomial critical points, not a sample-grid minimum."""
    widths=road.findall('lanes/laneSection/right/lane/width')
    if not widths:raise ValueError('missing widths')
    values=[]
    for i,e in enumerate(widths):
        hi=(float(widths[i+1].get('sOffset')) if i+1<len(widths) else float(road.get('length')))-float(e.get('sOffset'))
        if hi<=0:raise ValueError('nonpositive width record interval')
        a,b,c,d=[float(e.get(k)) for k in 'abcd'];q=[0.,hi]
        q.extend(float(x.real) for x in np.roots([3*d,2*c,b]) if abs(x.imag)<1e-9 and 0<x.real<hi)
        values.extend(a+x*(b+x*(c+x*d)) for x in q)
    return float(min(values))


def stats(v):
    return {'count':len(v),'median_m':float(np.median(v)),'p95_m':float(np.percentile(v,95)),'max_m':float(max(v))}


def fidelity(source,target):
    result={}
    for field in ('center','left','right'):
        result[field]={direction:stats(distances(a,b)) for direction,a,b in (
            ('source_to_target',source[field],target[field]),('target_to_source',target[field],source[field]))}
    return result


def bound_trial(after):
    report=json.loads(after.with_suffix('.ribbon.json').read_text(encoding='utf-8'))
    if report.get('sha256')!=hashlib.sha256(after.read_bytes()).hexdigest():
        raise ValueError('trial report hash does not match written geometry')
    return report


def independent_design_check(road):
    """Recompute the written shape; do not inherit optimizer dynamics PASS."""
    sub=ET.Element('OpenDRIVE');sub.append(copy.deepcopy(road))
    for speed in sub.findall('.//lane/speed'):
        unit=speed.get('unit','m/s')
        if unit not in ('m/s','km/h'):raise ValueError('unsupported speed unit')
        value=float(speed.get('max'))/(3.6 if unit=='km/h' else 1.)
        speed.set('max',str(max(15/3.6,value)));speed.set('unit','m/s')
    cfg=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    cfg['dynamics']['sample_step_m']=.02
    return _audit_d(sub,cfg)


def sampled_strip_validity(curves):
    """Reject observed loops/crossed strips; finite sampling is not a proof."""
    simple={k:bool(LineString(v).is_simple) for k,v in curves.items()}
    strip=Polygon(np.vstack([curves['left'],curves['right'][::-1]]))
    return {'status':'PASS' if all(simple.values()) and strip.is_valid and strip.area>0 else 'FAIL',
            'curves_simple':simple,'strip_valid':bool(strip.is_valid),'scope':'sampled at 0.05m, not an analytic global intersection proof'}


def run(before,after,folder):
    inp=ET.parse(before).getroot();root=ET.parse(after).getroot();rs={r.get('id'):r for r in root.findall('road')}
    old={r.get('id'):r for r in inp.findall('road')};src=shp_source();lat,lon=_origin(root)
    trial=bound_trial(after)
    edge=edge_audit(root);center=junction_lane_interfaces(root);rows=[]
    selected=trial['roads']
    columns=min(4,len(selected))
    if not columns:raise ValueError('no selected connector to review')
    nr=int(np.ceil(len(selected)/columns))
    fig,axes=plt.subplots(nr,columns,figsize=(4.5*columns,4*nr),layout='constrained',squeeze=False)
    for ax,row in zip(axes.flat,selected):
        rid=row['road'];road=rs[rid];r={'road':rid,'optimizer_trial_status':row['status']}
        try:
            sid=road.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']").get('value')
            raw=raw_curves(src,sid,lambda x:_project(x,lat,lon))
            reference,info=(composite_sources(root,road,src,lambda x:_project(x,lat,lon)) if trial.get('structural_mouth_retreat_m') or trial.get('source_support_mode')=='source-linked-composite'
                            else (raw,{'source_lane_ids':[sid]}))
            out=target_curves(road);previous=target_curves(old[rid]);errors=fidelity(reference,out)
            r.update(source=errors,raw_via_to_target={k:stats(distances(raw[k],out[k])) for k in raw},
                source_identity=info,minimum_width_m=minimum_width(road))
            rr=[e for e in edge['rows'] if e['connecting_road']==rid]
            cc=[c for c in center if c['connecting_road']==rid]
            r['endpoint_maxima']={k:max((v[k] for v in rr+cc),default=float('inf')) for k in ('position_m','heading_deg','curvature_per_m')}
            r['source_pass']=all(v['median_m']<=.35+1e-8 and v['p95_m']<=.75+1e-8 and v['max_m']<=1.5+1e-8 for field in errors.values() for v in field.values())
            r['raw_via_full_pass']=all(v['median_m']<=.35+1e-8 and v['p95_m']<=.75+1e-8 and v['max_m']<=1.5+1e-8 for v in r['raw_via_to_target'].values())
            r['source_pass'] &= r['raw_via_full_pass']
            ends=r['endpoint_maxima'];r['endpoints_pass']=(len(rr)==4 and len(cc)==2 and ends['position_m']<=.01 and ends['heading_deg']<=.1 and ends['curvature_per_m']<=1e-7)
            r['design_dynamics_2cm']=independent_design_check(road)
            r['design_dynamics_pass']=r['design_dynamics_2cm']['status']=='PASS'
            sub=ET.Element('OpenDRIVE');sub.append(copy.deepcopy(road))
            r['internal_edges']=internal_edge_audit(sub)
            r['strip_geometry']=sampled_strip_validity(out)
            r['status']='PASS' if r['source_pass'] and r['endpoints_pass'] and r['design_dynamics_pass'] and r['minimum_width_m']>=.1 and row['status']=='CANDIDATE' and r['internal_edges']['status']=='PASS' and r['strip_geometry']['status']=='PASS' else 'FAIL'
            for field in ('left','right','center'):
                ax.plot(*reference[field].T,color='#292929',lw=1.8,alpha=.7)
                ax.plot(*previous[field].T,color='#b83131',lw=.7,linestyle=':')
                ax.plot(*out[field].T,color='#176bbe',lw=1.1,linestyle='--' if field=='center' else '-')
            ax.set_title(f"Road {rid}: {r['status']} | raw via max {max(v['max_m'] for v in r['raw_via_to_target'].values()):.2f}m",color='#22713b' if r['status']=='PASS' else '#a52121',fontsize=10)
            ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
        except (ValueError,KeyError,AttributeError) as e:
            r.update(status='FAIL',error=str(e));ax.set_title(f'Road {rid}: FAIL');ax.text(.05,.5,str(e),transform=ax.transAxes,wrap=True,fontsize=8)
        rows.append(r)
    for ax in list(axes.flat)[len(selected):]:ax.set_visible(False)
    caption = 'Measured SHP connectors: black=source / red=input / blue=trial. Full original via kept; candidate BLOCKED; no consumer/controller approval.'
    fig.suptitle(textwrap.fill(caption, width=max(42, columns*42)), fontsize=11)
    folder.mkdir(parents=True,exist_ok=True);fig.savefig(folder/'all-connectors.png',dpi=130);plt.close(fig)
    report={'artifact':str(after),'sha256':hashlib.sha256(after.read_bytes()).hexdigest(),'status':'BLOCKED',
        'source_gate':'each direction separately; full raw via also required; .35/.75/1.5m median/P95/max',
        'minimum_width':'exact polynomial extrema','written_geometry_step_m':.05,
        'scope':'selected source-linked connector ribbons, not whole-map acceptance','rows':rows}
    (folder/'review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print([(r['road'],r['status'],r.get('source_pass'),r.get('endpoints_pass')) for r in rows])
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('folder',type=Path)
    a=p.parse_args();run(a.before,a.after,a.folder)
