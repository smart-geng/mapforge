"""Read original node4 inputs and check a bounded, whole-model stencil set.

Zero fitting iterations; no XML writer. The result distinguishes fixed
residual identities, local/full equivalence and numerical differentiability.
It cannot turn a rejected initial geometry into a map acceptance.
"""
import argparse
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.sparse import save_npz

from scripts.prepare_whole_source_model import (prepare_roles,raw_models,register_turns,
    ProfileSource,MovableSourceParentState,WholeSourcePortState,WholeSourceGeometryState,
    original_greville_seed,read,dump,unchanged,_sha256)
from mapforge.ops.whole_source_constraints import WholeSourceConstraintSystem


def run(preparation,review,decision,output):
    preparation,review,decision,output=map(lambda p:Path(p).resolve(),(preparation,review,decision,output))
    output.mkdir(parents=True,exist_ok=False);start=time.monotonic()
    contract=dict(status='REGISTERED_FULL_MODEL_DERIVATIVE_DIAGNOSTIC',fitting_iterations=0,
        xodr_writer_enabled=False,parents=['10','11','12','13','30'],movements=list(map(str,range(100,124))),
        probe_policy=['10 curvature','11 origin-y','12 heading','13 first free knot','30 first coefficient',
            '11 active junction cut','100 first primitive length','104 first core curvature','123 first free edge coefficient'],
        derivative_columns=9,step_scales=[1.,.5],sides_per_column=2,
        full_vs_local_checks=2,all_1681_jacobian_columns_checked=False,
        fidelity_trace_probes=4,
        structural_optimization_candidate_registered=False,
        stop_conditions=['changed original/approval','changed discrete source identity','both stencil directions invalid'])
    dump(output/'registration-contract.json',contract)
    root,scope,domain,contacts,roles,config,hashes,changes=prepare_roles(preparation,review,decision)
    hashes.update({str(p):_sha256(p) for d in ('mapforge','spikes','scripts') for p in (ROOT/d).rglob('*.py')})
    dump(output/'input-binding.json',dict(input_files_sha256=hashes,input_code_revision_changes=changes))
    models,rows=raw_models(root,scope,domain,contacts,roles,config)
    if set(models)!=set(scope['mutable_roads']):raise ValueError('all original parents required')
    parents={rid:MovableSourceParentState(o,m,original_greville_seed(m),source_root=root)
             for rid,(o,m) in models.items()}
    ports=WholeSourcePortState(root,config['whole_junction']['xodr_junction_id'],parents,scope['connectors'])
    snap=ports.evaluate(ports.initial)
    kernels,providers,turn_rows=register_turns(ports,snap,ProfileSource(config['source_dir'],config['profile']),scope,domain)
    geometry=WholeSourceGeometryState(ports,kernels,providers)
    print('REGISTER COMPLETE CERTIFICATE ENVELOPES',flush=True)
    system=WholeSourceConstraintSystem(geometry)
    port_source=[]
    for cid,turn in system.base.full['turns'].items():
        support=turn['source_support']
        gaps=[dict(end=end,side=side,distance_m=float(np.linalg.norm(
            np.array([frame['edges'][side][k] for k in ('x','y')])-support['raw'][side][0 if end==0 else -1])))
            for end,frame in enumerate(turn['frames']) for side in ('left','right')]
        port_source.append(dict(road=cid,side_mapping=support['travel_side_to_declared_side'],
            endpoint_source_gaps=gaps,maximum_m=max(r['distance_m'] for r in gaps)))
    dump(output/'physical-port-source-binding.json',port_source)
    saved=read(ROOT/'out/node4-whole-source-model-r4-20260914/whole-coordinate-state.json')['initial']
    if not np.array_equal(system.initial,saved):raise ValueError('diagnostic silently changed registered initial geometry')
    p=lambda rid,i:ports.slices[rid].start+i
    cut=next(i for i,c in enumerate(('start','end')) if any(port.contact==c for port in parents['11'].port_features))
    columns=[p('10',3),p('11',1),p('12',2),p('13',6),p('30',parents['30'].coefficient_slice.start),
        p('11',4+cut),geometry.turn_slices['100'].start,geometry.turn_slices['104'].start+5,
        geometry.turn_slices['123'].start+7]
    dump(output/'residual-contract.json',dict(row_identities=system.row_identities,columns=columns,
        summary=system.summary(),full_original_state_unchanged=True))
    reports=[];linearizations=[]
    def progress(d):
        print('STENCIL',d['column'],d['owner'],d['method'],'unresolved rows',len(d['left_right_disagreement_rows']),flush=True)
    for scale in contract['step_scales']:
        print('CHECK FULL DEPENDENCIES AT STEP SCALE',scale,flush=True)
        result=system.linearize(columns=columns,step_scale=scale,progress=progress)
        save_npz(output/f'stencil-{scale}.npz',result['matrix'])
        reports.append(dict(step_scale=scale,diagnostics=result['diagnostics']))
        linearizations.append(result['matrix'])
        dump(output/'stencil-diagnostics.json',reports)
    equivalence=[]
    for col in (columns[1],columns[-1]):
        print('INDEPENDENT WHOLE VS LOCAL',col,flush=True)
        delta=system.steps[col]
        local=system.trial_column(system.base,col,delta)
        full=system.evaluate(local.vector)
        error=float(np.max(abs(local.values-full.values)))
        if error>1e-10:raise ValueError('block-local evaluation omitted a whole-state dependency')
        equivalence.append(dict(column=col,maximum_residual_difference=error))
    a,b=[m.toarray() for m in linearizations]
    scales=abs(a-b)/(1+np.maximum(abs(a),abs(b)))
    unstable=np.argwhere(scales>1e-2)
    from scripts.source_stencil_trace import trace_turn
    traces=[]
    for col,direction,turn_ids in ((columns[0],1,('102','103')),(columns[5],-1,('106',))):
        traces.append(dict(column=col,step=0.,turns=[trace_turn(system,system.base,cid) for cid in turn_ids]))
        for scale in (1.,.5):
            delta=direction*system.steps[col]*scale
            trial=system.trial_column(system.base,col,delta)
            traces.append(dict(column=col,step=delta,turns=[trace_turn(system,trial,cid) for cid in turn_ids]))
    dump(output/'source-correspondence-trace.json',traces)
    summary=system.summary()
    summary.update(input_files_sha256=hashes,input_code_revision_changes=changes,
        physical_port_source_binding=port_source,
        probe_columns=columns,stencils=reports,whole_local_equivalence=equivalence,
        step_halving_disagreement_count=len(unstable),
        step_halving_disagreements=[dict(row=int(i),column=int(columns[j]),
            identity=system.row_identities[i],scaled_difference=float(scales[i,j])) for i,j in unstable],
        full_jacobian_checked=False,smoothness_certified=False,xodr_generated=False,
        production_accepted=False,elapsed_seconds=time.monotonic()-start)
    unchanged(hashes);dump(output/'report.json',summary)
    print('CHECK ENDED',summary['status'],'halving disagreement',len(unstable),flush=True)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('preparation','review','decision','output'):parser.add_argument(name)
    args=parser.parse_args();run(args.preparation,args.review,args.decision,args.output)
    raise SystemExit(2)
