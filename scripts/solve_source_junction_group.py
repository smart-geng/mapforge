"""One bounded solve for a prepared group of source-native parents and ALL turns.

Research only: the source compiler must accept every parent before any whole
map is written. Long model initialization is separate from joint feasibility;
neither solver convergence nor a partial source gate approves delivery.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import numpy as np
from scipy.optimize import least_squares
from scripts.build_ordinary_source_road import load_road
from scripts.build_source_geometry_candidate import dump
from scripts.gen_all import _sha256,shp_source
from scripts.research_code_revision import bind_current_code
from scripts.fit_source_boundary_block import unchanged
from scripts.solve_source_parent_junction import isolated_parent,written_coefficients
from scripts.rebuild_north_source_ports import install_same_identity_parent,fresh_seed
from mapforge.ops.source_cubic_export import cubic_model,constrain_written_endpoints,compile_road
from mapforge.ops.reconstruction_scope import digest, model_scope_report
from mapforge.ops.coupled_source_junction import SourceParentState,CoupledSourceJunction
from mapforge.ops.port_dependencies import PortDependencies
from mapforge.ops.joint_connector_fit import fit_joint,basis_for,coefficients_to_basis
from mapforge.ops.long_connector_chain import chain
from spikes.measured_connector_caps import frames,needs_cap,composite_sources,raw_curves,ribbon,write_ribbon
from spikes.trial_xml import replace_road,write_trial
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def written_long_seed(road,old_frames):
    """Derive a declared long family from actual XML, never from source vertices."""
    caps=tuple(needs_cap(f) for f in old_frames)
    geoms=road.findall('planView/geometry');core=len(geoms)-sum(caps)
    if core not in (3,4) or len(geoms)>5:raise ValueError('actual XML does not match a bounded long family')
    lengths=[float(g.get('length')) for g in geoms]
    if not np.isfinite(lengths).all() or min(lengths)<6.-1e-7:
        raise ValueError('short or nonfinite actual reference is not an admissible long seed')
    def end_k(g):
        if g.find('line') is not None:return 0.
        if g.find('arc') is not None:return float(g.find('arc').get('curvature'))
        if g.find('spiral') is not None:return float(g.find('spiral').get('curvEnd'))
        raise ValueError('unsupported actual reference family')
    i=int(caps[0])
    return dict(shape_parameters=lengths+[20*end_k(g) for g in geoms[i:i+core-1]],
                reference_core_count=core,initialization_only=True)


def run(directory,output,max_evaluations=10,solver_method='trf',initialize_references=False,compact_port_families=False,
        whole_junction_id=None):
    if solver_method not in ('trf','hard-source'):raise ValueError('unknown bounded joint solver')
    directory=Path(directory).resolve();output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    info=json.loads((directory/'report.json').read_text(encoding='utf8'))
    path=Path(info['artifact']);basepath=Path(info['input'])
    if _sha256(path)!=info['sha256'] or _sha256(basepath)!=info['input_sha256']:
        raise ValueError('prepared input or retained baseline changed')
    tree=ET.parse(path);root=tree.getroot();oldroot=ET.parse(basepath).getroot()
    if whole_junction_id is not None:
        check=model_scope_report(root,whole_junction_id,[p['road'] for p in info['parents']],info['affected_roads'])
        check.update(input=str(path),input_sha256=info['sha256'])
        dump(output/'whole-junction-preflight.json',check)
        if check['status']!='SCOPE_COMPLETE_NOT_MODEL_FEASIBILITY':
            raise ValueError('incomplete whole-junction model; no initialization or numerical solve started')
    hashes,changes=bind_current_code(info['source_hashes'])
    graph=PortDependencies(root);graph.validate_junction_table()
    oldgraph=PortDependencies(oldroot);oldgraph.validate_junction_table()
    if graph.connections!=oldgraph.connections:raise ValueError('source group changed traffic graph')
    baseline=json.loads((basepath.parent/'report.json').read_text(encoding='utf8'))
    if baseline['sha256']!=info['input_sha256']:raise ValueError('baseline report differs from actual XML')
    previous={r['road']:r for r in baseline['connectors']}
    parents={};models={};states={};originals={}
    for row in info['parents']:
        rid=row['road'];component=Path(row['component'])
        state=json.loads((component/'shared-state.json').read_text(encoding='utf8'))
        if rid in parents or state['road']!=rid or state.get('flat_port') or state.get('source_export_domain') is not None:
            raise ValueError('explicit unique nonflat source state required')
        raw,roles,files,source_root=load_road(state['source_directory'],state['decision_file'],rid,state.get('previous_source_directory'))
        model=constrain_written_endpoints(cubic_model(raw,minimum_span=state['minimum_width_span'],end_axis=state['end_axis']))
        if digest(model.describe())!=row['model_sha'] or state['model_sha']!=row['model_sha']:
            raise ValueError('parent source model changed')
        x=np.array(row['coefficients'],float);original=source_root.find(f"road[@id='{rid}']")
        written,compiled,_=compile_road(model,x,original,source_direction=state.get('source_direction'))
        if digest(compiled)!=digest(row['compiled']):raise ValueError('prepared source domain changed')
        install_same_identity_parent(root,isolated_parent(written))
        parents[rid]=SourceParentState(model,x,compiled);models[rid]=model;states[rid]=state;originals[rid]=original
        hashes.update(files);hashes[str(component/'shared-state.json')]=_sha256(component/'shared-state.json')
    required={rid for rid,ps in graph.connections.items() if any(p.road in parents for p in ps)}
    if required!=set(info['affected_roads']) or len(required)!=len(info['affected_roads']):
        raise ValueError('incomplete or duplicate dependency closure')
    graph=PortDependencies(root)
    untouched={r.get('id'):ET.tostring(r) for r in root.findall('road')}
    src=shp_source();lat,lon=_origin(root);project=lambda pts:_project(pts,lat,lon)
    kernels={};support_rows={};initialization=[]
    for rid in sorted(required,key=int):
        road=graph.roads[rid];a,b=frames(root,road);old_frames=frames(oldroot,oldgraph.roads[rid])
        saved=previous.get(rid) or written_long_seed(oldgraph.roads[rid],old_frames)
        variable_caps=tuple(p.road in parents and parents[p.road].contact_slope_can_vary(p)
            for p in graph.connections[rid])
        caps=tuple(needs_cap(f) or variable for f,variable in zip((a,b),variable_caps))
        q,core,migration=fresh_seed(a,b,saved,old_frames,contact_caps=caps)
        cls=chain(q,a,b,caps,core);kn,B,_=basis_for(cls)
        raw,support=composite_sources(root,road,src,project)
        via=raw_curves(src,road.find(".//userData[@code='mapforge.source_lane']").get('value'),project)
        end=cls[-1];delta=end.ThetaEnd-b['pose'][2]
        closure=float(max(abs(end.XEnd-b['pose'][0]),abs(end.YEnd-b['pose'][1]),
            abs(20*np.arctan2(np.sin(delta),np.cos(delta)))))
        old_caps=tuple(needs_cap(f) for f in old_frames)
        if initialize_references and closure>1e-6:
            coeff=None;seed_method='pending-reference-family-initialization'
        elif caps==old_caps:
            coeff=written_coefficients(road,kn,B);seed_method='actual-written-cubic-seed'
        else:
            kk,co=ribbon(cls,a,b,raw,True)
            if not np.allclose(kk,kn,atol=1e-8):raise ValueError('initial ribbon basis differs from fixed joint basis')
            coeff=coefficients_to_basis(kn,co,B);seed_method='new-cap-source-ribbon-initialization-only'
        bounded_seed=None;family_diagnostics=[]
        if initialize_references and closure>1e-6:
            from mapforge.ops.long_source_seed import initialize,ReferenceInitializationError
            print('CLOSED LONG INITIALIZATION',rid,closure,flush=True)
            try:
                q,coeff,bounded_seed=initialize(a,b,raw,parameters=q,core_count=core,max_iterations=40,
                    contact_caps=caps,allow_open_reference=compact_port_families)
                if compact_port_families and not bounded_seed.get('reference_closed',True) and sum(caps)>=1:
                    family_diagnostics.append(bounded_seed)
                    try:
                        q2,c2,r2=initialize(a,b,raw,parameters=None,core_count=2,max_iterations=40,
                            contact_caps=caps,allow_open_reference=True)
                        if r2['maximum_scaled_closure_residual']<bounded_seed['maximum_scaled_closure_residual']:
                            q,coeff,bounded_seed,core=q2,c2,r2,2
                        else:family_diagnostics.append(r2)
                    except ReferenceInitializationError as compact_error:
                        family_diagnostics.append(compact_error.report)
            except ReferenceInitializationError as exc:
                family_diagnostics.append(exc.report)
                if compact_port_families and sum(caps)>=1:
                    core=2
                    try:
                        q,coeff,bounded_seed=initialize(a,b,raw,parameters=None,core_count=2,
                            max_iterations=40,contact_caps=caps,allow_open_reference=True)
                    except ReferenceInitializationError as compact_error:
                        family_diagnostics.append(compact_error.report)
                if bounded_seed is None:
                    dump(output/'INITIALIZATION-REJECTED.json',dict(road=rid,**family_diagnostics[-1],
                        tested_families=family_diagnostics,
                        prepared_input=str(path),prepared_input_sha256=_sha256(path),
                        source_support=support,source_contact_variable_slope=list(variable_caps),
                        source_changed=False,source_roles_expanded=False,source_speed_changed=False,
                        simultaneous_optimization_started=False))
                    raise
            cls=chain(q,a,b,caps,core)
            seed_method=bounded_seed['status']
        if compact_port_families:
            from mapforge.ops.regular_ribbon_seed import initialize_regular_ribbon
            coeff,regular_seed=initialize_regular_ribbon(cls,a,b,raw)
        else:regular_seed=None
        tails=[dict(role=role,source_lane_id=sid,curves=raw_curves(src,sid,project)) for role,sid in
               [('predecessor',support['source_lane_ids'][0]),('successor',support['source_lane_ids'][-1])]]
        kernels[rid]=fit_joint(road,a,b,raw,via,q,initial_coefficients=coeff,fair_world=True,
            core_count=core,source_tails=tails,_problem_only=True,contact_caps=caps)
        support_rows[rid]=support
        initialization.append(dict(road=rid,seed_migration=migration,coefficient_seed=seed_method,
            previous_reference_closure_residual=closure,bounded_reference_initialization=bounded_seed,
            tested_failed_reference_families=family_diagnostics,regular_ribbon_seed=regular_seed,
            source_contact_variable_slope=list(variable_caps),
            previous_parameters_from='bound-baseline-report' if rid in previous else 'actual-XML-long-primitives',
            caps=list(caps),core_count=core,initial_reference_lengths_m=[c.length for c in cls],
            independent_variables=len(kernels[rid]['initial'])))
        dump(output/'initialization.json',initialization)
        print('GROUP TURN',rid,seed_method,len(kernels[rid]['initial']),flush=True)
    problem=CoupledSourceJunction(graph,parents,kernels)
    conflicts=problem.fixed_source_obstructions()
    if conflicts:raise ValueError('fixed original source ports conflict: '+str(conflicts))
    hashes.update({str(p):_sha256(p) for n in ('mapforge','scripts','spikes') for p in (ROOT/n).rglob('*.py')})
    hashes.update({str(p):_sha256(p) for p in (path,basepath,directory/'report.json',basepath.parent/'report.json')})
    unchanged(hashes)
    x0=problem.initial;initial=problem.summary(x0);r0=problem.residual(x0)
    dump(output/'initial.json',initial);started=time.monotonic();jac_count=0
    regular=problem.regular_geometry(x0);dump(output/'initial-regularity.json',regular)
    def jac(x):
        nonlocal jac_count
        jac_count+=1
        print('GROUP JACOBIAN',jac_count,'elapsed',round(time.monotonic()-started,1),flush=True)
        return problem.jacobian(x)
    print('SIMULTANEOUS',len(x0),'PARENTS',sum(p.nvar for p in parents.values()),'TURNS',len(kernels),flush=True)
    if solver_method=='trf':
        result=least_squares(problem.residual,x0,jac=jac,bounds=problem.bounds.T,
            method='trf',tr_solver='lsmr',x_scale='jac',max_nfev=max_evaluations,ftol=1e-9,xtol=1e-9,gtol=1e-9)
        final_x=result.x
        optimizer=dict(method='trf-lsmr',success=bool(result.success),message=str(result.message),
            nfev=int(result.nfev),njev=int(result.njev))
    elif solver_method=='hard-source':
        from mapforge.ops.source_feasible_gauss_newton import solve
        progress_rows=[]
        def progress(row):
            row=dict(row,elapsed_seconds=time.monotonic()-started)
            progress_rows.append(row);dump(output/'optimizer-progress.json',progress_rows)
            print('SOURCE FEASIBLE STEP',json.dumps(row),flush=True)
        final_x,optimizer=solve(problem,x0,max_iterations=max_evaluations,progress=progress)
    else:raise ValueError('unknown bounded joint solver')
    final=problem.summary(final_x);rf=problem.residual(final_x)
    chosen={rid:p.coefficients(final_x[problem.parent_slices[rid]]) for rid,p in parents.items()}
    rows=[]
    for rid,sl in problem.turn_slices.items():
        eq,iq,r,full=problem.turn(rid,final_x[sl],problem.contact_jets(final_x,rid));k=kernels[rid]
        rows.append(dict(road=rid,source_support=support_rows[rid],shape_parameters=full[:k['nq']].tolist(),
            joint_coefficients=full[k['nq']:].tolist(),reference_core_count=k['core_count'],caps=list(k['caps']),
            primitive_lengths_m=[c.length for c in r[0]],minimum_width_record_span_m=float(min(np.diff(r[1]))),
            maximum_scaled_equality=float(max(abs(eq))),minimum_construction_slack=float(min(iq))))
    saved=dict(schema='mapforge/shared-source-group-state/v1',parents=[dict(road=rid,component=next(
        r['component'] for r in info['parents'] if r['road']==rid),model_sha=states[rid]['model_sha'],
        coefficients=x.tolist()) for rid,x in chosen.items()],connectors=rows,vector=final_x.tolist(),
        simultaneous_parent_connector_variables=True,source_hashes=hashes,production_accepted=False)
    dump(output/'joint-state.json',saved)
    artifact=None;write_error=None
    try:
        for rid,x in chosen.items():
            written,_,_=compile_road(models[rid],x,originals[rid],source_direction=states[rid].get('source_direction'))
            install_same_identity_parent(root,isolated_parent(written))
        for rid,sl in problem.turn_slices.items():
            _,_,r,_=problem.turn(rid,final_x[sl],problem.contact_jets(final_x,rid))
            old=next(e for e in root.findall('road') if e.get('id')==rid)
            replace_road(root,old,write_ribbon(old,*r[:3]))
        for road in root.findall('road'):
            rid=road.get('id')
            if rid not in parents and rid not in kernels and ET.tostring(road)!=untouched[rid]:
                raise ValueError('unaffected road changed')
        if PortDependencies(root).connections!=graph.connections:raise ValueError('explicit traffic graph changed')
        PortDependencies(root).validate_junction_table()
        artifact=output/'node4-joint-diagnostic.xodr';write_trial(tree,artifact)
    except ValueError as exc:write_error=str(exc)
    report=dict(status='REJECTED_NOT_DELIVERY',artifact=str(artifact) if artifact else None,
        sha256=_sha256(artifact) if artifact else None,input=str(basepath),input_sha256=info['input_sha256'],
        prepared_group=str(directory),parent_components={r['road']:r['component'] for r in info['parents']},
        source_hashes=hashes,input_code_revision_changes=changes,initial=initial,final=final,connectors=rows,
        affected_roads=list(problem.turn_slices),simultaneous_parent_connector_variables=True,
        variable_count=len(x0),independent_parent_variables=sum(p.nvar for p in parents.values()),
        source_inequalities=sum(len(m.lower) for m in models.values()),
        initial_squared_residual=float(r0@r0),final_squared_residual=float(rf@rf),
        optimizer=optimizer,
        elapsed_seconds=time.monotonic()-started,write_error=write_error,source_priority_stage_ran=False,
        closed_reference_initialization_requested=initialize_references,
        compact_port_families_enabled=compact_port_families,
        source_changed=False,source_roles_expanded=False,source_speed_changed=False,
        whole_surface_accepted=False,MAP_retested=False,production_accepted=False)
    unchanged(hashes);dump(output/'report.json',report)
    print('GROUP END',report['initial_squared_residual'],report['final_squared_residual'],write_error,flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output');p.add_argument('--max-evaluations',type=int,default=10)
    p.add_argument('--method',choices=('trf','hard-source'),default='trf')
    p.add_argument('--initialize-references',action='store_true')
    p.add_argument('--compact-port-families',action='store_true')
    p.add_argument('--whole-junction-id',help='require models for every explicit parent and turn before any solve')
    a=p.parse_args()
    if not 1<=a.max_evaluations<=20:raise ValueError('bounded group experiment required')
    if a.compact_port_families and not a.initialize_references:raise ValueError('compact family requires explicit bounded initialization')
    run(a.directory,a.output,a.max_evaluations,a.method,a.initialize_references,a.compact_port_families,
        a.whole_junction_id);raise SystemExit(2)
