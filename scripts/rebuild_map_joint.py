"""Source-first MAP corridor/mouth reconstruction; isolated, atomic candidate.

No cap/floor copied from historical maps. The second stage rebuilds every
connection from one frozen set of corridor mouths. Rejection of one road
prevents emission of a mixed solved/unsolved final XODR.
"""
import argparse
import copy
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import numpy as np
from lxml import etree

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import CASES,POLICY
from scripts.recheck_speed_contract import sha,dump,check_limits
from scripts.map_raw_geometry_review import isolated_detours
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml,parse_map_xml_all
from mapforge.ops import map_to_xodr as converter
from mapforge.validate.map_source import audit_map_source_manifest
from mapforge.validate.g11 import load_policy,_audit_d
from mapforge.report.decision import finalize_opendrive_g8
from spikes.map_coordinate_family import coordinate_axis,source_mouth_heading
from spikes.map_lane_family import ManifestCenters
from spikes import road_boundary_family as family
from spikes.clarabel_joint_candidate import interior_qp
from spikes.joint_mouth_candidate import connectors
from spikes.three_clothoid_minimax import optimize as three_clothoid_minimax
from spikes.map_corner_surface import build as surface


def geometry_stage(source_root,manifest,*,coupled=False,restore=False,rotating_axes=False,source_widths=None,
                   joint_initialization=False):
    """Transactional whole-junction geometry, never mutates rejected input."""
    if restore and not coupled:raise ValueError('joint restoration requires coupled geometry')
    if rotating_axes and not coupled:raise ValueError('rotating axes require coupled geometry')
    if joint_initialization and not (coupled and restore and rotating_axes):
        raise ValueError('joint initialization requires coupled/restoration/rotating axes')
    tree=etree.ElementTree(etree.fromstring(ET.tostring(source_root)))
    observations=ManifestCenters(manifest);rows=[]
    for road in tree.findall('road'):
        if road.get('junction')!='-1':continue
        if joint_initialization:
            rows.append({'road_id':road.get('id'),'status':'JOINT_SEED_ONLY',
                         'reason':'source admission deferred to same complete joint feasibility solve'})
            continue
        left=road.findall('lanes/laneSection/left/lane')
        prov=[l.find("userData[@code='mapforge.provenance/v1']") for l in left]
        mirror=(bool(left) and all(l.find("userData[@code='mapforge.source_lane']") is None for l in left)
                and all(p is not None and json.loads(p.get('value')).get('support_kind')
                        in ('mirror','lane-transition-ribbon') for p in prov))
        with patch.object(family,'_convex_qp',interior_qp):
            result=family.solve_road(road,observations,np.asarray,source_mode='centers',mirror=mirror,
                source_tol=.35,source_error_budget='absolute',all_source_vertices=True,
                source_certificate=True,junction_endpoint_mode='source-parallel',source_widths=source_widths)
        result['road_id']=road.get('id');rows.append(result)
        print('CORRIDOR',road.get('id'),result['status'],result.get('reason'),flush=True)
    report={'method':'source-bounded whole corridors, followed by all dependent connecting roads',
            'joint_nonlinear_optimum_claimed':False,'source_tolerance_m':.35,
            'source_error_budget':'absolute','corridors':rows,'candidate_only':True}
    if not rows or (not joint_initialization and any(r['status']!='CANDIDATE' for r in rows)):
        return None,dict(report,status='REJECTED',reason='at least one complete corridor rejected')
    root=ET.fromstring(etree.tostring(tree))
    if not joint_initialization:
        try:report['connectors']=connectors(root,curve_solver=three_clothoid_minimax)
        except ValueError as exc:return None,dict(report,status='REJECTED',reason=str(exc))
    if coupled:
        from spikes.joint_corridor_connectors import solve as joint_solve
        try:solved,joint_report=joint_solve(root,manifest,target_ratio=.999,restore=restore,rotating_axes=rotating_axes,source_widths=source_widths,
                                          joint_initialization=joint_initialization)
        except ValueError as exc:return None,dict(report,status='REJECTED',reason='joint model rejected: '+str(exc))
        report['coupled_state']=joint_report
        report['corridor_reports_apply_to']=('unqualified initial seeds; no independent corridor PASS'
            if joint_initialization else 'pre-coupled initializer; current checks are in coupled_state and independent readback')
        if solved is None:return None,dict(report,status='REJECTED',reason=joint_report['reason'])
        root,curves=solved
        report['connectors']=connectors(root,frozen_curves=curves)
        report['method']='simultaneous whole-corridor boundary coefficients and all connecting roads, '+('rotating' if rotating_axes else 'fixed')+' Line charts'
    if any(not r['curve_fit']['shape_constraints_satisfied'] for r in report['connectors']):
        return None,dict(report,status='REJECTED',reason='inferred connector shape/minimum-length constraint failed')
    return root,dict(report,status='CANDIDATE')


def topology_speed_signature(root):
    def compact(e):return (str(e.tag),tuple(sorted(e.attrib.items())),tuple(compact(c) for c in e))
    return {'junctions':[compact(j) for j in root.findall('junction')],
            'roads':{r.get('id'):{'link':compact(r.find('link')) if r.find('link') is not None else None,
                'sections':[{'s':s.get('s'),'lanes':[(side,l.get('id'),l.get('type'),
                    tuple(compact(x) for x in l.findall('link')+l.findall('speed')+
                          l.findall("userData[@code='mapforge.source_lane']")))
                    for side in ('left','right') for l in s.findall(side+'/lane')]}
                    for s in r.findall('lanes/laneSection')]} for r in root.findall('road')
                if r.get('name')!='junction_paving'}}


def cover_source_prefix(pv, source_sets):
    """Extend one coordinate Line upstream to all admitted raw observations.

    The junction end remains fixed. This is not extrapolation of source
    observations, nor an additional geometry primitive.
    """
    if len(pv.segs)!=1 or pv.segs[0].kind!='line':
        raise ValueError('prefix coverage requires a single coordinate Line')
    tangent=np.array([np.cos(pv.hdg),np.sin(pv.hdg)])
    start=np.array([pv.x0,pv.y0])
    raw=np.vstack([xy for _,xy in source_sets if len(xy)>=2])
    extension=max(0.,-float(min((raw-start)@tangent)))
    pv.x0,pv.y0=start-extension*tangent
    pv.segs[0].length+=extension
    pv.fit_meta['raw_source_prefix_extension_m']=extension
    return extension


def run(label,directory,*,coupled=False,restore=False,source_axis=False,rotating_axes=False,joint_initialization=False):
    if restore and not coupled:raise ValueError('joint restoration requires coupled geometry')
    if rotating_axes and not coupled:raise ValueError('rotating axes require coupled geometry')
    if joint_initialization and not (coupled and restore and rotating_axes):
        raise ValueError('joint initialization requires coupled/restoration/rotating axes')
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=False)
    raw=[ROOT/'v2x_map_xml'/n for _,n in CASES]
    paths=raw+[POLICY,ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml',ROOT/'OpenDRIVE_1.5M.xsd']
    for folder in ('mapforge','spikes','scripts'):paths+=list((ROOT/folder).rglob('*.py'))
    hashes={str(p.relative_to(ROOT)):sha(p) for p in paths}
    main=parse_map_xml(str(ROOT/'v2x_map_xml'/dict(CASES)[label]))
    nodes=[n for p in raw for n in parse_map_xml_all(str(p))]
    record={'case':label,'status':'BLOCKED','candidate_only':True,'default_promoted':False,
            'joint_initialization':joint_initialization,
            'source_and_code_sha256':hashes,'geometry_stage_completed':False}
    def save_record():
        if any(sha(ROOT/p)!=h for p,h in hashes.items()):raise ValueError('source or implementation changed during run')
        dump(directory/'run.json',record)
    anomaly=[dict(v,source_lane_id=converter._source_lane_key(main,l,k)) for l in main.links for k in l.lanes
             if len(k.points)>2 for v in isolated_detours(converter._project(k.points,main.ref_lat,main.ref_lon))]
    if anomaly:
        record.update(reason='raw-source-detour-requires-explicit-correction',anomalies=anomaly)
        save_record();print(label,record['reason'],flush=True);return 2
    initial=directory/'initial';initial.mkdir();path=initial/(label+'.xodr');selections=[]
    support_sets=[]
    original_support=converter._reference_support
    def track_support(link_points,lane_point_sets):
        support_sets[:]=lane_point_sets
        return original_support(link_points,lane_point_sets)
    def choose(support,**kwargs):
        matches=[l for l in main.links if len(l.points)>1 and
                 np.linalg.norm(converter._project(l.points,main.ref_lat,main.ref_lon)[-1]-support[-1])<1e-6]
        if len(matches)!=1:raise ValueError('ambiguous coordinate support ownership')
        link=matches[0];heading=None;heading_evidence=None
        if source_axis:
            heading,heading_evidence=source_mouth_heading([
                (converter._source_lane_key(main,link,lane),converter._project(lane.points,main.ref_lat,main.ref_lon))
                for lane in link.lanes])
        answer=coordinate_axis(converter._project(link.points,main.ref_lat,main.ref_lon),support,heading=heading)
        if answer is None:raise ValueError('source corridor is not a regular Line chart; no fragmented fallback')
        if heading_evidence is not None:
            answer[0].fit_meta.update(axis_heading_source='original incoming terminal support',
                                      axis_heading_evidence=heading_evidence)
        cover_source_prefix(answer[0],support_sets)
        selections.append(dict(link=link.name,metadata=answer[0].fit_meta))
        return answer
    neighbors=[n for n in nodes if (n.region,n.node_id)!=(main.region,main.node_id)]
    try:
        with patch.object(converter,'fit_leg_refline',choose),patch.object(converter,'_reference_support',track_support):
            stats=converter.build_xodr(main,path,neighbors=neighbors)
    except Exception as exc:
        record.update(reason='initial-coordinate-construction-rejected',error=f'{type(exc).__name__}: {exc}')
        save_record();return 2
    manifest=stats['source_lane_manifest'];dump(path.with_suffix('.source-lanes.json'),manifest)
    admitted=audit_map_source_manifest(manifest,raw);dump(initial/'source-integrity.json',admitted)
    record['reference_selections']=selections
    if admitted['status']!='PASS':
        record.update(reason='raw-source-inventory-failed');save_record();return 2
    limits={converter._source_lane_key(n,l,k):converter._lane_speed_kmh(k) for n in nodes for l in n.links for k in l.lanes}
    source_root=ET.parse(path).getroot()
    before=check_limits(source_root,limits);dump(initial/'source-speed.json',before)
    if before['status']!='PASS':raise ValueError('initial source speed integrity failed')
    from mapforge.validate.map_width import raw_widths
    widths=raw_widths(raw)
    final,fit=geometry_stage(source_root,manifest,coupled=coupled,restore=restore,rotating_axes=rotating_axes,source_widths=widths,
                             joint_initialization=joint_initialization);dump(directory/'fit.json',fit)
    if final is None:
        record.update(reason=fit['reason']);save_record();return 2
    record['geometry_stage_completed']=True
    if topology_speed_signature(final)!=topology_speed_signature(source_root):
        raise ValueError('geometry reconstruction altered speed or traffic topology')
    # Paving is a separate simulation surface, never a driving repair claim.
    try:paving=surface(final)
    except ValueError as exc:
        record.update(reason='surface-rejected',surface_error=str(exc));save_record();return 2
    target=directory/(label+'.xodr');ET.indent(final)
    ET.ElementTree(final).write(target,encoding='utf-8',xml_declaration=True)
    accepted=finalize_opendrive_g8(target,manifest,POLICY,raw_map_paths=raw)
    speed=check_limits(ET.parse(target).getroot(),limits)
    width_review=accepted['quality']['gates']['MAP-explicit-width']
    policy=load_policy(ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml');policy['dynamics']['sample_step_m']=.02
    dense=_audit_d(ET.parse(target).getroot(),policy)
    from scripts.internal_edge_jets import audit as inside
    from scripts.esmini_lane_interfaces import check
    from scripts.freeze_joint_mouth_candidates import source_plot,screenshots
    internal=inside(ET.parse(target).getroot());consumer=check(target,edges=True)
    xsd=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd'))).validate(etree.parse(str(target)))
    source_plot(target,path,directory/'source-overlay.png',baseline_title='fresh coordinate seed',candidate_title='source-constrained joint mouths')
    vertex_review=json.loads((directory/'source-overlay.json').read_text(encoding='utf-8'))
    vertex_max=max(r['max_m'] for r in vertex_review['raw_vertex_fidelity'])
    raw_vertex_gate={'status':'PASS' if vertex_max<=.3501 else 'FAIL',
                     'max_m':vertex_max,'tolerance_m':.35,'sampling_allowance_m':.0001,
                     'scope':'all original vertices to same-identity written curves; not full-path certificate'}
    screenshots(target,directory/'esmini-review.png')
    decision=accepted['decision'];decision['status']='BLOCKED';decision['candidate_only']=True
    decision['blocked_reasons'].append({'code':'research_candidate_not_promoted'})
    for code,value in [('source-speed',speed['status']),('dense-dynamics',dense['status']),
                       ('explicit-source-width',width_review['status']),
                       ('raw-vertices',raw_vertex_gate['status']),('internal-edges',internal['status']),
                       ('consumer',consumer['status']),('xsd','PASS' if xsd else 'FAIL')]:
        if value!='PASS':decision['blocked_reasons'].append({'code':code+'-failed'})
    accepted['quality']['delivery_decision']=decision
    for suffix,data in [('.dense.json',dense),('.source-speed.json',speed),('.internal-edges.json',internal),
                        ('.source-width.json',width_review),
                        ('.consumer.json',consumer),('.quality-report.json',accepted['quality']),('.delivery-decision.json',decision)]:
        dump(target.with_suffix(suffix),data)
    record.update(xodr_sha256=sha(target),gates={k:v['status'] for k,v in accepted['quality']['gates'].items()},
                  raw_vertices=raw_vertex_gate,
                  source_speed=speed['status'],source_width=width_review['status'],dense=dense['status'],dense_metrics=dense['metrics'],
                  internal_edges=internal['status'],consumer=consumer['status'],xsd=xsd,paving=paving,
                  simulation_auxiliary_only=True,crs_absolutely_verified=False)
    save_record();print(label,{k:v for k,v in record.items() if k not in ('source_and_code_sha256','reference_selections','paving')},flush=True)
    return 2


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('case',choices=dict(CASES));p.add_argument('output')
    p.add_argument('--coupled',action='store_true',help='jointly solve all corridor coefficients and connectors')
    p.add_argument('--restore-joint',action='store_true',help='recover nonlinear feasibility before rejecting a coupled candidate')
    p.add_argument('--source-mouth-axis',action='store_true',help='derive chart directions from original incoming terminal observations')
    p.add_argument('--rotating-axes',action='store_true',help='optimize chart angles jointly with complete-source boundaries and all connectors')
    p.add_argument('--joint-initialization',action='store_true',help='restore full source/ports/dynamics jointly without requiring a qualified fixed-axis initializer')
    a=p.parse_args();raise SystemExit(run(a.case,a.output,coupled=a.coupled,restore=a.restore_joint,source_axis=a.source_mouth_axis,
                                        rotating_axes=a.rotating_axes,joint_initialization=a.joint_initialization))
