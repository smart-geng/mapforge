"""Independent written-file/source overlay of one fixed-parent connector trial.

Geometry review is deliberately separate from unchanged-speed dynamics.
Neither this review nor esmini interface success accepts the entire map.
"""
import argparse
import copy
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from lxml import etree

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source,_sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump
from scripts.review_measured_ribbon import (target_curves,minimum_width,fidelity,stats,
    distances,sampled_strip_validity,independent_design_check)
from scripts.internal_edge_jets import audit as internal_audit
from scripts.esmini_lane_interfaces import check as interface_check
from spikes.measured_connector_caps import raw_curves,composite_sources
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def source_ok(values):
    return bool(values) and all(v['median_m']<=.35+1e-8 and v['p95_m']<=.75+1e-8 and v['max_m']<=1.5+1e-8 for v in values)


def written_forward_minimum(road):
    """Rebuild quartics from XML records, not optimizer state or a sample grid."""
    from mapforge.validate.g11 import _primitive
    geoms=road.findall('planView/geometry');offsets=road.findall('lanes/laneOffset')
    widths=road.findall('lanes/laneSection/right/lane/width')
    cuts=sorted({0.,float(road.get('length'))}|{float(g.get('s')) for g in geoms+offsets}|
                {float(w.get('sOffset')) for w in widths})
    def poly(elements,key,s):
        e=max((e for e in elements if float(e.get(key))<=s),key=lambda e:float(e.get(key)))
        u=s-float(e.get(key));a,b,c,d=[float(e.get(k,'0')) for k in 'abcd']
        return np.array([a+u*(b+u*(c+u*d)),b+u*(2*c+3*d*u),c+3*d*u,d])
    minimum=float('inf')
    for a,b in zip(cuts[:-1],cuts[1:]):
        g=max((g for g in geoms if float(g.get('s'))<=a),key=lambda g:float(g.get('s')))
        p=_primitive(g);dk=(p['k1']-p['k0'])/p['length'];k=p['k0']+dk*(a-float(g.get('s')))
        left=poly(offsets,'s',a);right=left-poly(widths,'sOffset',a)
        for t in (left,right):
            q=-np.polynomial.polynomial.polymul([k,dk],t);q[0]+=1
            stationary=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(q))
            ss=[0.,b-a]+[float(r.real) for r in stationary if abs(r.imag)<1e-9 and 0<r.real<b-a]
            minimum=min(minimum,float(min(np.polynomial.polynomial.polyval(ss,q))))
    return minimum


def audit_north_source(path,state_folder):
    from lxml import etree as LE
    from scripts.fit_source_boundary_block import load
    from mapforge.ops.source_cubic_export import cubic_model,constrain_written_endpoints,constrain_flat_connector_port,compile_road
    from mapforge.ops.reconstruction_scope import digest
    from mapforge.validate.source_cubic_readback import audit_written_source
    state_folder=Path(state_folder);state=json.loads((state_folder/'shared-state.json').read_text(encoding='utf8'))
    model,_,_=load(ROOT/'out/source-role-input-v151',ROOT/'profiles/repair/node4-zero-width-source-roles-v1.yaml',3)
    model=constrain_flat_connector_port(constrain_written_endpoints(cubic_model(model,minimum_span=5.5,end_axis=True)))
    if digest(model.describe())!=state['model_sha']:raise ValueError('north source/model revision mismatch')
    unchanged(state['source_hashes'])
    original=LE.parse(str(ROOT/'out/shp-shared-section-final-v145/node4.xodr')).getroot().find("road[@id='10']")
    _,compiled,_=compile_road(model,np.array(state['coefficients']),original)
    actual=ET.parse(path).getroot().find("road[@id='10']")
    audit=audit_written_source(actual,compiled,model)
    audit.update(artifact=str(path),sha256=_sha256(path),model_sha=state['model_sha'],source_hashes=state['source_hashes'])
    unchanged(state['source_hashes']);return audit


def run(folder,skip_consumer=False,parent_state=None):
    folder=Path(folder).resolve();trial=json.loads((folder/'report.json').read_text(encoding='utf8'))
    path=Path(trial['artifact']);before=Path(trial['input'])
    if _sha256(path)!=trial['sha256'] or _sha256(before)!=trial['input_sha256']:raise ValueError('review revision mismatch')
    unchanged(trial['source_hashes']);root=ET.parse(path).getroot();old=ET.parse(before).getroot()
    src=shp_source();lat,lon=_origin(root);records=[]
    # Recheck ALL 12 dependent source turns, not only the modified subset.
    roads=[r for r in root.findall('road') if r.get('junction')!='-1' and any(
        e.get('elementType')=='road' and e.get('elementId')=='10' for e in r.findall('link/*'))]
    if len(roads)!=12:raise ValueError('node4 north review requires all twelve dependent connectors')
    interfaces=None if skip_consumer else interface_check(path,edges=True)
    fig,axes=plt.subplots(3,4,figsize=(17,13),layout='constrained')
    for ax,road in zip(axes.flat,roads):
        rid=road.get('id');sid=road.find("lanes/laneSection/right/lane/userData[@code='mapforge.source_lane']").get('value')
        raw=raw_curves(src,sid,lambda p:_project(p,lat,lon))
        support,identity=composite_sources(root,road,src,lambda p:_project(p,lat,lon))
        actual=target_curves(road,.05);previous=target_curves(old.find(f"road[@id='{rid}']"),.05)
        errors=fidelity(support,actual);via={k:stats(distances(raw[k],actual[k])) for k in raw}
        sub=ET.Element('OpenDRIVE');sub.append(copy.deepcopy(road));internal=internal_audit(sub)
        strip=sampled_strip_validity(actual);width=minimum_width(road);forward=written_forward_minimum(road)
        ends=[r for r in interfaces['rows'] if r['road']==rid] if interfaces else []
        port_ok=len(ends)==6 and all(r['status']=='PASS' for r in ends)
        passed=source_ok([v for f in errors.values() for v in f.values()]+list(via.values()))
        passed=passed and width>=.1 and forward>=.1-1e-8 and strip['status']=='PASS' and internal['status']=='PASS' and port_ok
        speed=src.lane(sid).max_speed_kmh
        written=[float(v.get('max'))*(3.6 if v.get('unit','m/s')=='m/s' else 1.) for v in road.findall('.//lane/speed')]
        speed_ok=bool(written) and all(abs(v-speed)<1e-8 for v in written)
        dynamics=independent_design_check(road)
        row=dict(road=rid,geometry_status='PASS' if passed else 'FAIL',source=errors,raw_via=via,
            source_identity=identity,internal_edges=internal,strip=strip,minimum_width_m=width,minimum_forward_factor=forward,
            interfaces_pass=port_ok,speed_check=dict(source_kmh=speed,written_kmh=written,status='PASS' if speed_ok else 'FAIL'),
            source_speed_dynamics=dynamics,production_accepted=False)
        records.append(row)
        for field in ('left','right','center'):
            ax.plot(*support[field].T,c='#ec8e19',lw=1.9)
            ax.plot(*previous[field].T,c='#aa3854',lw=.8,ls=':')
            ax.plot(*actual[field].T,c='#176bbb',lw=1.1,ls='--' if field=='center' else '-')
        extent=np.vstack([*support.values(),*actual.values()]);low=extent.min(axis=0)-2.;high=extent.max(axis=0)+2.
        previous_points=np.vstack(list(previous.values()))
        clipped=bool(np.any(previous_points<low)|np.any(previous_points>high))
        row['plot_view']=dict(xlim=[float(low[0]),float(high[0])],ylim=[float(low[1]),float(high[1])],
                              historical_baseline_outside_view=clipped,acceptance_metrics_cropped=False)
        ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1])
        if clipped:ax.text(.02,.02,'Old rejected baseline extends outside view',transform=ax.transAxes,fontsize=7,color='#aa3854')
        worst=max(v['max_m'] for f in errors.values() for v in f.values())
        ax.set_title(f"{rid}: geometry {row['geometry_status']} | max {worst:.3f}m",fontsize=10)
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
    for ax in list(axes.flat)[len(roads):]:ax.set_visible(False)
    fig.suptitle('All 12 north-related turns: orange=complete source support; blue=written XODR; red dotted=baseline\n'
                 'Sampled review, median/P95/max <=0.35/0.75/1.5m, NOT maximum 0.35m. Whole map NOT accepted.',fontsize=12)
    fig.savefig(folder/'all-connectors-readback.png',dpi=140);plt.close(fig)
    schema=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    xsd=schema.validate(etree.parse(str(path)))
    report=dict(status='REVIEW_NOT_DELIVERY',sha256=_sha256(path),artifact=str(path),source_hashes=trial['source_hashes'],
        geometry_pass_roads=[r['road'] for r in records if r['geometry_status']=='PASS'],
        geometry_failed_roads=[r['road'] for r in records if r['geometry_status']!='PASS'],
        all_map_internal_edges=internal_audit(root),interfaces=interfaces,rows=records,xsd_pass=xsd,
        xsd_errors=str(schema.error_log),production_accepted=False,
        source_scope='Full raw via, declared original-TOPO adjacent support, each direction separately',
        written_sample_step_m=.05,continuous_certificate=False,
        reviewed_code_sha256={str(p):_sha256(p) for p in [Path(__file__),ROOT/'scripts/review_measured_ribbon.py',
            ROOT/'scripts/internal_edge_jets.py',ROOT/'scripts/esmini_lane_interfaces.py']})
    if parent_state is not None:
        north=audit_north_source(path,parent_state);dump(folder/'north-source-readback.json',north)
        report['north_source_readback']={k:north[k] for k in ('status','source_to_written_max_m','written_to_source_max_m','model_sha')}
    def ids(r):
        return (r.get('id'),r.get('junction'),ET.tostring(r.find('link')) if r.find('link') is not None else b'',
            tuple((sec.get('s'),side,l.get('id'),ET.tostring(l.find('link')) if l.find('link') is not None else b'')
                  for sec in r.findall('lanes/laneSection') for side in ('left','right') for l in sec.findall(side+'/lane')))
    old_ids={r.get('id'):ids(r) for r in old.findall('road')};new_ids={r.get('id'):ids(r) for r in root.findall('road')}
    changed=sorted(k for k in set(old_ids)|set(new_ids) if old_ids.get(k)!=new_ids.get(k))
    report['id_link_diff']=dict(changed_roads=changed,status='PASS' if not changed else 'FAIL')
    unchanged(trial['source_hashes']);dump(folder/'review-validation.json',report)
    print('REVIEW',report['geometry_pass_roads'],'FAIL',report['geometry_failed_roads'],flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');p.add_argument('--skip-consumer',action='store_true')
    p.add_argument('--parent-state',help='explicit saved flat-port north source model for actual-file replay')
    a=p.parse_args();run(a.folder,a.skip_consumer,a.parent_state)
