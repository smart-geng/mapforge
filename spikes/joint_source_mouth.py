"""Source-constrained ordinary mouth windows for measured-via reconstruction.

Window edges share the original road frame and lane identities. Only a long
mouth window may change; the remaining road is restricted exactly, not fitted.
Source observations and written-speed dynamics stay constrained. The trial
does not turn source stop lines into movable geometry. All outputs are BLOCKED
until all affected connectors, source support and whole-map gates are rebuilt.
"""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from lxml import etree

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from spikes.measured_connector_caps import trim_ordinary
from spikes.road_boundary_family import solve_road
from scripts.gen_all import shp_source
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.validate.g11 import load_policy,_audit_d
from scripts.internal_edge_jets import audit as internal_edges
from spikes.trial_xml import replace_road,write_trial


def mouth_contact(road):
    links=[role for role in ('predecessor','successor')
           if road.find('link/'+role) is not None and road.find('link/'+role).get('elementType')=='junction']
    if len(links)!=1:raise ValueError('requires exactly one structural junction end')
    return 'start' if links[0]=='predecessor' else 'end'


def make_window(road,span):
    length=float(road.get('length'));contact=mouth_contact(road)
    if span<=0 or span>=length:raise ValueError('mouth window must be shorter than road')
    lo,hi=(0.,span) if contact=='start' else (length-span,length)
    window=trim_ordinary(road,0.,interval=(lo,hi))
    other='successor' if contact=='start' else 'predecessor'
    links=window.find('link')
    for link in links.findall(other):links.remove(link)
    # A local solver anchor, never written into the joined full road.
    ET.SubElement(links,other,elementType='road',elementId='mouth-window-fixed-cut',
                  contactPoint='start' if other=='successor' else 'end')
    return window,lo,hi


def lane_map(sec):
    return {(side,l.get('id')):l for side in ('left','right') for l in sec.findall(side+'/lane')}


def splice_window(original,window,lo,hi):
    """Restore full original reference, topology, speeds and outside geometry."""
    length=float(original.get('length'))
    if not (lo==0. or hi==length):raise ValueError('only a junction-end window may be spliced')
    fixed=trim_ordinary(original,0.,interval=(hi,length) if lo==0. else (0.,lo))
    before,after=(window,fixed) if lo==0. else (fixed,window)
    shift=hi if lo==0. else lo
    a=copy.deepcopy(before.find('lanes'));b=copy.deepcopy(after.find('lanes'))
    for e in b.findall('laneOffset'):e.set('s',str(float(e.get('s'))+shift))
    for sec in b.findall('laneSection'):sec.set('s',str(float(sec.get('s'))+shift))
    old=lane_map(a.findall('laneSection')[-1]);new=lane_map(b.findall('laneSection')[0])
    if old.keys()!=new.keys():raise ValueError('window cut crosses a lane birth/death; no rank guessing')
    for key,lane in old.items():
        for one,role in ((lane,'successor'),(new[key],'predecessor')):
            links=one.find('link')
            if links is None:links=ET.Element('link');one.insert(0,links)
            for e in links.findall(role):links.remove(e)
            ET.SubElement(links,role,id=key[1])
    group=ET.Element('lanes')
    for side in (a,b):
        for e in side.findall('laneOffset'):group.append(e)
    for side in (a,b):
        for e in side.findall('laneSection'):group.append(e)
    result=copy.deepcopy(original);i=list(result).index(result.find('lanes'))
    result.remove(result.find('lanes'));result.insert(i,group)
    return result


def run(source,target,span=30.,min_span=10.,source_tolerance=.35,mouth_retreat=0.,span_by_road=None,road_ids=None):
    if source.resolve()==target.resolve():raise ValueError('never overwrite input')
    tree=ET.parse(source);root=tree.getroot();src=shp_source();lat,lon=_origin(root)
    selected=set(road_ids) if road_ids is not None else None
    if selected is not None:
        known={r.get('id') for r in root.findall('road') if r.get('junction')=='-1'}
        if selected-known:raise ValueError('unknown ordinary road IDs')
        if mouth_retreat:raise ValueError('structural relocation requires the complete ordinary road set')
    if mouth_retreat:
        for road in list(root.findall('road')):
            if road.get('junction')=='-1':
                trimmed=trim_ordinary(road,mouth_retreat);replace_road(root,road,trimmed)
    rows=[];cfg=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    cfg['dynamics']['sample_step_m']=.02
    for original in list(root.findall('road')):
        if original.get('junction')!='-1':continue
        if selected is not None and original.get('id') not in selected:continue
        row={'road':original.get('id'),'status':'REJECTED'}
        try:
            chosen=(span_by_road or {}).get(original.get('id'),span)
            window,lo,hi=make_window(original,chosen)
            fit=etree.fromstring(ET.tostring(window))
            result=solve_road(fit,src,lambda pts:_project(pts,lat,lon),min_span=min_span,
                              source_tol=source_tolerance,source_error_budget='absolute',
                              boundary_association='source-order',physical_graph=True,
                              junction_endpoint_mode='source-parallel',dynamics=True,all_source_vertices=True)
            row.update(result,road=original.get('id'),window_original_s=[lo,hi],all_source_vertices=True)
            if result['status']=='CANDIDATE':
                fitted=ET.fromstring(etree.tostring(fit));sub=ET.Element('OpenDRIVE');sub.append(copy.deepcopy(fitted))
                row['window_dynamics_2cm']=_audit_d(sub,cfg)
                new=splice_window(original,fitted,lo,hi);check=ET.Element('OpenDRIVE');check.append(new)
                row['internal_edges']=internal_edges(check)
                row['window_internal_edges']=internal_edges(sub)
                # Existing outside defects remain reported, never count as
                # solved. This local construction must not add a cut defect.
                cut=hi if lo==0. else lo
                cut_fail=[r for r in row['internal_edges']['failures'] if abs(r['s_m']-cut)<1e-6]
                row['cut_failures']=cut_fail
                if row['window_dynamics_2cm']['status']=='PASS' and row['window_internal_edges']['status']=='PASS' and not cut_fail:
                    replace_road(root,original,new)
                else:row['status']='REJECTED';row['reason']='independent readback or cut continuity failed'
        except (ValueError,KeyError,IndexError) as exc:
            row.update(status='REJECTED',reason=str(exc))
        rows.append(row);target.parent.mkdir(parents=True,exist_ok=True)
        target.with_suffix('.progress.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
        print('MOUTH',row['road'],row['status'],row.get('reason'),flush=True)
    xsd=write_trial(tree,target)
    report={'status':'BLOCKED','source':str(source),'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
            'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'scope':'ordinary mouth windows only; dependent measured connectors NOT rebuilt',
            'source_stop_lines_unchanged':True,'structural_mouth_retreat_m':mouth_retreat,'rows':rows,'xsd':xsd}
    target.with_suffix('.mouth.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path)
    p.add_argument('--span',type=float,default=30.);p.add_argument('--min-span',type=float,default=10.)
    p.add_argument('--mouth-retreat',type=float,default=0.)
    p.add_argument('--road',action='append',help='fit only this ordinary road; relocation cannot be partial')
    a=p.parse_args();run(a.source,a.target,a.span,a.min_span,mouth_retreat=a.mouth_retreat,road_ids=a.road);raise SystemExit(2)
