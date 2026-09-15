"""Full free-column sided stencils and a preregistered joint trial.

All five parents and 24 turns, source originals re-read. This is a numerical
interface check, not fitting or a candidate search. No XODR writer exists in
this entrypoint. Preserve older evidence and always use a new output folder.
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
from mapforge.ops.whole_source_step import WholeSourceStepModel


def run(preparation,review,decision,output):
    preparation,review,decision,output=map(lambda p:Path(p).resolve(),(preparation,review,decision,output))
    output.mkdir(parents=True,exist_ok=False);start=time.monotonic()
    contract=dict(status='REGISTERED_FULL_SIDED_STENCIL_AND_ATOMIC_TRIAL',fitting_iterations=0,
        xodr_writer_enabled=False,parents=['10','11','12','13','30'],movements=list(map(str,range(100,124))),
        column_policy='every registered free coordinate; no unchecked zero columns',
        derivative_step_scale=1.,sides_per_column=2,maximum_stencil_seconds=1800,
        coefficient_evaluation='all original affine witnesses and exact port basis dependence; axes/cuts/knots rebuild',
        joint_probe_policy='all five parent origin-y +1e-5m and all 24 first primitive lengths +1e-5m',
        full_vs_local_checks=1,coefficient_fast_vs_original_checks=10,
        one_joint_trial=True,structural_optimization_candidate_registered=False,
        stop_conditions=['changed original/approval','changed discrete source identity','both stencil directions invalid',
                         'stencil deadline exceeded; retain incomplete progress, no zero filling'])
    dump(output/'registration-contract.json',contract)
    root,scope,domain,contacts,roles,config,hashes,changes=prepare_roles(preparation,review,decision)
    hashes.update({str(p):_sha256(p) for d in ('mapforge','spikes','scripts') for p in (ROOT/d).rglob('*.py')})
    dump(output/'input-binding.json',dict(input_files_sha256=hashes,input_code_revision_changes=changes))
    models,_=raw_models(root,scope,domain,contacts,roles,config)
    if set(models)!=set(scope['mutable_roads']):raise ValueError('all original parents required')
    parents={rid:MovableSourceParentState(o,m,original_greville_seed(m),source_root=root)
             for rid,(o,m) in models.items()}
    ports=WholeSourcePortState(root,config['whole_junction']['xodr_junction_id'],parents,scope['connectors'])
    snapshot=ports.evaluate(ports.initial)
    kernels,providers,_=register_turns(ports,snapshot,ProfileSource(config['source_dir'],config['profile']),scope,domain)
    system=WholeSourceConstraintSystem(WholeSourceGeometryState(ports,kernels,providers))
    saved=read(ROOT/'out/node4-whole-source-model-r4-20260914/whole-coordinate-state.json')['initial']
    if not np.array_equal(system.initial,saved):raise ValueError('diagnostic changed registered initial geometry')
    dump(output/'residual-contract.json',dict(row_identities=system.row_identities,
        columns=system.free_columns.tolist(),summary=system.summary(),full_original_state_unchanged=True))
    from mapforge.ops.source_coefficient_stencil import ParentCoefficientStencils
    fast=ParentCoefficientStencils(system,system.base);fast_checks=[]
    for rid,sl in ports.slices.items():
        p=parents[rid]
        for local in (p.coefficient_slice.start,p.coefficient_slice.stop-1):
            col=sl.start+local;delta=system.steps[col]
            quick=fast.evaluate(col,delta)
            ordinary=system.trial_column(system.base,col,delta)
            error=float(np.max(abs(quick['values']-ordinary.values),initial=0.))
            if error>1e-10:raise ValueError('all-witness coefficient stencil differs from original evaluator')
            fast_checks.append(dict(column=col,parent=rid,max_residual_difference=error,
                recomputed_blocks=quick['recomputed_blocks']))
    dump(output/'coefficient-equivalence.json',fast_checks)
    diagnostics=[];stencil_start=time.monotonic()
    def progress(row):
        diagnostics.append(row)
        if len(diagnostics)%10==0 or len(diagnostics)==len(system.free_columns):
            print('FULL SIDED COLUMNS',len(diagnostics),'/',len(system.free_columns),
                  'seconds',round(time.monotonic()-stencil_start,1),flush=True)
            dump(output/'stencil-progress.json',dict(completed=len(diagnostics),expected=len(system.free_columns),
                last_column=row['column'],all_free_columns_evaluated=len(diagnostics)==len(system.free_columns)))
        if time.monotonic()-stencil_start>contract['maximum_stencil_seconds']:
            raise TimeoutError('registered full-stencil deadline; incomplete matrix not returned')
    try:
        stencil=system.linearize(progress=progress,coefficient_stencils=True)
    except (ValueError,ArithmeticError,TimeoutError) as exc:
        dump(output/'stencil-diagnostics.json',diagnostics);unchanged(hashes)
        dump(output/'report.json',dict(status='FULL_STENCIL_INCOMPLETE_NOT_MAP',reason=str(exc),
            completed_columns=len(diagnostics),required_columns=len(system.free_columns),
            input_files_sha256=hashes,optimization_ran=False,xodr_generated=False,export_allowed=False))
        raise
    dump(output/'stencil-diagnostics.json',stencil['diagnostics'])
    for name,matrix in dict(central=stencil['matrix'],**stencil['sided_matrices']).items():
        save_npz(output/f'stencil-{name}.npz',matrix)
    dump(output/'sided-validity.json',{k:v.tolist() for k,v in stencil['sided_valid'].items()})
    # This is ONE deterministic all-block motion, never an optimization step
    # or a list of best per-road states assembled into a fictitious success.
    delta=np.zeros(len(system.initial))
    for sl in ports.slices.values():delta[sl.start+1]=1e-5
    for sl in system.geometry.turn_slices.values():delta[sl.start]=1e-5
    radius=100*system.steps;radius[system.fixed_columns]=0.
    dump(output/'joint-trial-registration.json',dict(base=system.initial.tolist(),delta=delta.tolist(),
        radius=radius.tolist(),selection='fixed before seeing stencils; not merit based',geometric_optimization=False))
    model=WholeSourceStepModel(system,system.base,stencil,radius=radius)
    print('VERIFY ALL PARENTS AND TURNS IN ONE ATOMIC TRIAL',flush=True)
    result=model.verify(delta,independent_full=True)
    np.savez_compressed(output/'joint-trial-residuals.npz',before=system.base.values,
        actual=result['snapshot'].values,**model.predict(delta))
    summary=result['report']
    summary.update(input_files_sha256=hashes,input_code_revision_changes=changes,
        coefficient_equivalence=fast_checks,
        stencil_shape=list(stencil['matrix'].shape),free_columns=len(system.free_columns),
        fixed_columns=system.fixed_columns,stencil_seconds=time.monotonic()-stencil_start,
        left_right_disagreement_columns=sum(bool(d['left_right_disagreement_rows']) for d in diagnostics),
        left_right_disagreement_entries=sum(len(d['left_right_disagreement_rows']) for d in diagnostics),
        one_sided_columns=[d['column'] for d in diagnostics if d['method']=='one-sided-domain-stencil'],
        smooth_jacobian_certified=False,original_whole_initial_unchanged=True,
        all_source_rows_retained=True,elapsed_seconds=time.monotonic()-start)
    unchanged(hashes);dump(output/'report.json',summary)
    print('FINISHED NOT MAP',summary['stencil_shape'],summary['independent_full_max_difference'],flush=True)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('preparation','review','decision','output'):parser.add_argument(key)
    args=parser.parse_args();run(args.preparation,args.review,args.decision,args.output)
    raise SystemExit(2)
