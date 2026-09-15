"""Bounded simultaneous source-parent / ALL-incident-turn feasibility trial.

No production write, no source/speed/role edits. Failed shared source states
remain coefficient evidence; the ordinary compiler is never bypassed to emit
a map. Feasibility precedes any source-priority/fairness optimization stage.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import numpy as np
from scipy.optimize import least_squares
from scripts.build_ordinary_source_road import load_road
from scripts.build_source_geometry_candidate import dump
from scripts.rebuild_north_source_ports import install_same_identity_parent
from scripts.research_code_revision import bind_current_code
from scripts.fit_source_boundary_block import unchanged
from scripts.gen_all import _sha256,shp_source
from mapforge.ops.source_cubic_export import cubic_model,constrain_written_endpoints,compile_road
from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.coupled_source_junction import SourceParentState,CoupledSourceJunction
from mapforge.ops.port_dependencies import PortDependencies
from mapforge.ops.joint_connector_fit import fit_joint,basis_for,coefficients_to_basis
from mapforge.ops.long_connector_chain import chain
from spikes.measured_connector_caps import frames,needs_cap,composite_sources,raw_curves,write_ribbon
from spikes.trial_xml import replace_road,write_trial
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def written_coefficients(road,knots,basis):
    """Existing XML is an initialization only, never a replacement source."""
    def powers(elements,attr,station):
        row=max((r for r in elements if float(r.get(attr))<=station+1e-8),key=lambda r:float(r.get(attr)))
        delta=station-float(row.get(attr));co=np.array([float(row.get(k)) for k in 'abcd'])
        return np.array([np.polynomial.polynomial.polyval(delta,np.polynomial.polynomial.polyder(co,j))/f
                         for j,f in enumerate((1,1,2,6))])
    left=np.array([powers(road.findall('lanes/laneOffset'),'s',s) for s in knots[:-1]])
    width=np.array([powers(road.findall('lanes/laneSection/right/lane/width'),'sOffset',s) for s in knots[:-1]])
    return coefficients_to_basis(knots,dict(left=left,right=left-width),basis)


def isolated_parent(road):
    road=ET.fromstring(ET.tostring(road))
    for link in road.findall('link'):road.remove(link)
    return road


def run(component,baseline,seed_parent,output,max_evaluations):
    component,baseline,seed_parent,output=map(lambda p:Path(p).resolve(),(component,baseline,seed_parent,output))
    if output.exists():raise FileExistsError('preserve previous evidence')
    output.mkdir(parents=True)
    state=json.loads((component/'shared-state.json').read_text(encoding='utf8'))
    info=json.loads((baseline/'report.json').read_text(encoding='utf8'));path=Path(info['artifact'])
    if _sha256(path)!=info['sha256']:raise ValueError('baseline actual XML changed')
    if state['road']!='10' or state['flat_port'] or state.get('source_export_domain') is not None:
        raise ValueError('explicit source-backed north nonflat common-domain model required')
    original,roles,hashes,source_root=load_road(state['source_directory'],state['decision_file'],'10',
        state.get('previous_source_directory'))
    model=constrain_written_endpoints(cubic_model(original,minimum_span=state['minimum_width_span'],end_axis=state['end_axis']))
    if digest(model.describe())!=state['model_sha']:raise ValueError('source-backed model revision changed')
    old=json.loads(seed_parent.read_text(encoding='utf8'))
    x0=np.asarray(old['coefficients'],float)
    parent,compiled,_=compile_road(model,x0,source_root.find("road[@id='10']"))
    tree=ET.parse(path);root=tree.getroot();untouched={r.get('id'):ET.tostring(r) for r in root.findall('road')}
    old_graph=PortDependencies(root);old_graph.validate_junction_table()
    install_same_identity_parent(root,isolated_parent(parent))
    graph=PortDependencies(root);graph.validate_junction_table()
    if graph.connections!=old_graph.connections:raise ValueError('explicit traffic graph changed')
    shared=SourceParentState(model,x0,compiled)
    source=shp_source();lat,lon=_origin(root);project=lambda points:_project(points,lat,lon)
    kernels={};identity={};jobs=[]
    for rid,ports in graph.connections.items():
        if not any(p.road=='10' for p in ports):continue
        road=graph.roads[rid];a,b=frames(root,road)
        previous=next(row for row in info['connectors'] if row['road']==rid)
        raw,support=composite_sources(root,road,source,project)
        sid=road.find(".//userData[@code='mapforge.source_lane']").get('value')
        via=raw_curves(source,sid,project);q=previous['shape_parameters'];core=previous.get('reference_core_count',3)
        caps=(needs_cap(a),needs_cap(b));cls=chain(q,a,b,caps,core);kn,B,_=basis_for(cls)
        coeff=written_coefficients(road,kn,B)
        tails=[dict(role=role,source_lane_id=s,curves=raw_curves(source,s,project)) for role,s in (
            ('predecessor',support['source_lane_ids'][0]),('successor',support['source_lane_ids'][-1]))]
        kernels[rid]=fit_joint(road,a,b,raw,via,q,initial_coefficients=coeff,fair_world=True,
            core_count=core,source_tails=tails,_problem_only=True)
        identity[rid]=support
        print('ASSEMBLED SHARED TURN',rid,len(kernels[rid]['initial']),flush=True)
    problem=CoupledSourceJunction(graph,{'10':shared},kernels)
    if len(kernels)!=12:raise ValueError('complete north dependency closure required')
    previous_hashes,code_changes=bind_current_code(info['source_hashes'])
    hashes.update(previous_hashes)
    hashes.update({str(p):_sha256(p) for name in ('mapforge','scripts','spikes') for p in (ROOT/name).rglob('*.py')})
    hashes.update({str(p):_sha256(p) for p in (path,baseline/'report.json',component/'shared-state.json',seed_parent)})
    unchanged(hashes)
    obstructions=problem.fixed_source_obstructions()
    if obstructions:
        report=dict(status='FIXED_SOURCE_PORT_CONFLICT',artifact=None,input=str(path),input_sha256=info['sha256'],
            conflicts=obstructions,affected_roads=list(problem.turn_slices),source_hashes=hashes,
            source_changed=False,source_roles_expanded=False,geometry_solver_ran=False,
            global_impossibility_proven=False,production_accepted=False)
        dump(output/'report.json',report);unchanged(hashes)
        print('REFUSE FIXED SOURCE PORTS',obstructions,flush=True)
        return report
    initial=problem.summary(problem.initial);dump(output/'initial.json',initial)
    initial_res=problem.residual(problem.initial)
    started=time.monotonic();jac_count=0
    def jac(v):
        nonlocal jac_count
        jac_count+=1
        print('JOINT JACOBIAN',jac_count,'elapsed',round(time.monotonic()-started,1),flush=True)
        return problem.jacobian(v)
    print('ONE SHARED STATE',len(problem.initial),'parent',shared.nvar,'dependent',len(kernels),flush=True)
    result=least_squares(problem.residual,problem.initial,jac=jac,bounds=problem.bounds.T,
        method='trf',tr_solver='lsmr',x_scale='jac',max_nfev=max_evaluations,
        ftol=1e-9,xtol=1e-9,gtol=1e-9)
    final=problem.summary(result.x);final_res=problem.residual(result.x)
    changed=shared.coefficients(result.x[problem.parent_slices['10']])
    saved_turns=[]
    for rid,sl in problem.turn_slices.items():
        eq,iq,r,full=problem.turn(rid,result.x[sl],problem.contact_jets(result.x,rid))
        k=kernels[rid]
        saved_turns.append(dict(road=rid,source_support=identity[rid],shape_parameters=full[:k['nq']].tolist(),
            joint_coefficients=full[k['nq']:].tolist(),reference_core_count=k['core_count'],caps=list(k['caps']),
            primitive_lengths_m=[c.length for c in r[0]],minimum_width_record_span_m=float(min(np.diff(r[1]))),
            maximum_scaled_equality=float(max(abs(eq))),minimum_construction_slack=float(min(iq))))
    dump(output/'joint-state.json',dict(schema='mapforge/shared-parent-junction-state/v1',model_sha=state['model_sha'],
        parent_coefficients=changed.tolist(),connectors=saved_turns,vector=result.x.tolist(),
        simultaneous_parent_connector_variables=True,source_hashes=hashes,production_accepted=False))
    artifact=None;write_error=None
    try:
        new_parent,_,_=compile_road(model,changed,source_root.find("road[@id='10']"))
        install_same_identity_parent(root,isolated_parent(new_parent))
        for rid,sl in problem.turn_slices.items():
            _,_,r,_=problem.turn(rid,result.x[sl],problem.contact_jets(result.x,rid))
            old_road=next(e for e in root.findall('road') if e.get('id')==rid)
            new_road=write_ribbon(old_road,*r[:3]);replace_road(root,old_road,new_road)
        for r in root.findall('road'):
            if r.get('id') not in kernels and r.get('id')!='10' and ET.tostring(r)!=untouched[r.get('id')]:
                raise ValueError('unaffected road changed')
        PortDependencies(root).validate_junction_table()
        artifact=output/'node4-joint-diagnostic.xodr';write_trial(tree,artifact)
    except ValueError as exc:
        write_error=str(exc)
    report=dict(status='REJECTED_NOT_DELIVERY',artifact=str(artifact) if artifact else None,
        sha256=_sha256(artifact) if artifact else None,input=str(path),input_sha256=info['sha256'],
        north_component=str(component),seed_parent_state=str(seed_parent),source_hashes=hashes,
        initial=initial,final=final,connectors=saved_turns,affected_roads=list(problem.turn_slices),
        simultaneous_parent_connector_variables=True,variable_count=len(problem.initial),
        independent_parent_variables=shared.nvar,source_inequalities=len(model.lower),
        jacobian_method='exact linear source maps times block-local contact-jet numerical derivatives',
        initial_squared_residual=float(initial_res@initial_res),final_squared_residual=float(final_res@final_res),
        optimizer=dict(success=bool(result.success),message=str(result.message),nfev=int(result.nfev),njev=int(result.njev)),
        elapsed_seconds=time.monotonic()-started,write_error=write_error,
        input_code_revision_changes=code_changes,source_changed=False,source_roles_expanded=False,
        source_speed_changed=False,source_priority_stage_ran=False,independent_readback_completed=False,
        whole_surface_accepted=False,MAP_retested=False,production_accepted=False)
    unchanged(hashes);dump(output/'report.json',report)
    print('JOINT END',report['initial_squared_residual'],report['final_squared_residual'],write_error,flush=True)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('component');parser.add_argument('baseline')
    parser.add_argument('seed_parent');parser.add_argument('output');parser.add_argument('--max-evaluations',type=int,default=5)
    args=parser.parse_args()
    if not 1<=args.max_evaluations<=20:raise ValueError('bounded joint experiment required')
    run(args.component,args.baseline,args.seed_parent,args.output,args.max_evaluations)
    raise SystemExit(2)
