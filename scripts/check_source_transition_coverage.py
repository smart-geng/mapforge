"""Recheck historical source-native states after closing ordinary contact gaps.

Previous coefficients are inputs, never a current-code PASS. Free station jets
are NOT curves. This command emits research evidence only, no XODR or new IDs.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_transition_domains import source_transition_domains
from scripts.fit_source_boundary_block import load,unchanged,runtime_versions,draw
from scripts.recheck_speed_contract import sha,dump
from spikes.source_event_basis import event_aligned_model
from spikes.source_jet_relaxation import SourceJetRelaxation


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def previous_states(directory,model,roles,check,event_model,layout):
    directory=Path(directory).resolve();run=read(directory/'run.json');files={}
    for name in ('run.json','report.json','relaxed-jets.json','event-basis.json'):
        files[str(directory/name)]=sha(directory/name)
        if name!='run.json' and run['output_sha256'].get(name)!=sha(directory/name):
            raise ValueError('historical parameter hash mismatch: '+name)
    report=read(directory/'report.json');jets=read(directory/'relaxed-jets.json');event=read(directory/'event-basis.json')
    if report['original_scope_sha']!=model.scope['content_sha256'] or report['roles_sha']!=roles['content_sha256']:
        raise ValueError('historical source/role identity mismatch')
    if digest(jets['variables'])!=digest(check.variables):raise ValueError('historical station variable indexing differs')
    if jets['status']!='NOT_A_CURVE' or jets['export_allowed'] is not False or event['export_allowed'] is not False:
        raise ValueError('historical research state cannot claim map acceptance')
    if digest(layout)!=digest(event['layout']):raise ValueError('historical long event layout differs')
    actual=event_model.describe()
    # New constraints are intentional; coordinate system, basis and source
    # scope MUST match before historical coefficients may be interpreted.
    for key in actual:
        if key!='constraints' and digest(actual[key])!=digest(event['model'].get(key)):
            raise ValueError('historical coefficient interpretation differs: '+key)
    return jets,event,files


def inspect_jets(check,values):
    x=np.asarray(values,float)
    if x.shape!=(check.nvar,) or not np.isfinite(x).all():raise ValueError('invalid free station state')
    A=check.matrix(check.inequalities);delta=np.asarray(A@x)-np.asarray(check.rhs)
    scale=np.maximum(np.asarray(abs(A).sum(axis=1)).ravel(),1.)
    masks={
        'record_support':np.array([r.get('domain_kind')!='source_transition_band' for r in check.labels]),
        'transition_bands':np.array([r.get('domain_kind')=='source_transition_band' for r in check.labels]),
    }
    result={}
    for name,mask in masks.items():
        indices=np.flatnonzero(mask);ordered=indices[np.argsort(-delta[indices]/scale[indices])]
        result[name]=dict(rows=len(indices),violations_over_1e_6=int(np.sum(delta[indices]/scale[indices]>1e-6)),
            maximum_normalized_violation=float(np.max(delta[indices]/scale[indices],initial=0.)),
            worst_rows=[dict(check.labels[i],normalized_violation=float(delta[i]/scale[i])) for i in ordered[:8]])
    result.update(scaled_contact_residual=float(np.max(abs(check.matrix(check.equalities)@x),initial=0.)),
                  curve_reconstructed=False,export_allowed=False)
    return result


def compute(model,roles,previous):
    check=SourceJetRelaxation(model,2.);event_model,layout=event_aligned_model(model)
    jets,event,files=previous_states(previous,model,roles,check,event_model,layout)
    historical=inspect_jets(check,jets['scaled_jet_values'])
    print('OLD FREE STATION STATE',historical['transition_bands']['maximum_normalized_violation'],flush=True)
    x,new_free=check.solve();print('CLOSED FREE STATIONS',new_free['status'],new_free.get('minimum_normalized_slack'),flush=True)
    _,new_basis=check.solve(restrict_basis=True)
    print('CLOSED LONG BASIS',new_basis['status'],new_basis.get('minimum_normalized_slack'),flush=True)
    event_check=SourceJetRelaxation(event_model,2.);_,new_event=event_check.solve(restrict_basis=True)
    print('CLOSED EVENT BASIS',new_event['status'],new_event.get('minimum_normalized_slack'),flush=True)
    old_curve=np.asarray(event['coefficients'],float);audit=event_model.audit(old_curve)
    coverage=source_transition_domains(model);lengths=[r['b']-r['a'] for r in coverage['intervals']]
    result=dict(status='RESEARCH_COVERAGE_RECHECK_NOT_MAP',original_scope_sha=model.scope['content_sha256'],
        roles_sha=roles['content_sha256'],coverage=coverage,
        coverage_summary=dict(ordinary_events=len(coverage['events']),added_check_bands=len(lengths),
            cumulative_lane_length_m=sum(lengths),max_band_m=max(lengths,default=0.),
            excluded_nonordinary_events=len(coverage['excluded_taper_events']),
            station_roundoff_tolerance_m=1e-8,geometry_spans_added=0),
        previous_free_station_recheck=historical,new_free_station_diagnosis=new_free,
        new_uniform_basis_diagnosis=new_basis,new_event_basis_diagnosis=new_event,
        historical_event_curve_recomputed=audit,event_model=event_model.describe(),event_layout=layout,
        source_tolerance_changed=False,source_speed_changed=False,source_removed=False,
        new_geometry_generated=False,xodr_generated=False,export_allowed=False,
        historical_code_claimed_current=False,global_map_impossibility_proven=False,
        remaining=['exterior source-center support','movement observations','relative widening and legal events',
                   'writable whole-road model','incident connectors','MAP width contract','absolute CRS','Web editor'])
    return result,dict(status='NOT_A_CURVE',variables=check.variables,
        scaled_jet_values=None if x is None else x.tolist(),export_allowed=False),event_model,old_curve,files


def context_series(model,context,x,speed,step=.02):
    """Polynomial one-sided limits, not the other branch at an exact knot.

    The input record may end at a branch where this pair of boundaries ceases
    to be the physical lane. Evaluating its right-hand third derivative at
    that exact endpoint can plot an unrelated combination as a jerk spike.
    Match the independent audit's within-span query convention.
    """
    from spikes.road_boundary_family import world_kinematics
    a,b=context['a'],context['b'];left,right=context['left'],context['right']
    lf,rf=[model.families[model.owner[k]] for k in (left,right)]
    cuts=np.unique(np.r_[a,b,lf.knots[(lf.knots>a)&(lf.knots<b)],rf.knots[(rf.knots>a)&(rf.knots<b)]])
    parts=[]
    for lo,hi in zip(cuts[:-1],cuts[1:]):
        ss=np.unique(np.r_[np.nextafter(lo,hi),np.arange(lo,hi,step),np.nextafter(hi,lo)])
        ss=np.clip(ss,np.nextafter(lo,hi),np.nextafter(hi,lo))
        jets=np.array([[.5*(model.expression(left,s,j)+model.expression(right,s,j))@x for j in range(4)] for s in ss])
        v=speed/3.6;values=abs(world_kinematics(jets,0.,0.))*[v*v/2.5,v**3]
        parts.append((ss,values))
    return parts


def draw_gaps(model,x,output):
    """Actual old polynomials with newly covered intervals, NOT free LP jets."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from spikes.shared_boundary_dynamics import audit_transition_bands
    audit=audit_transition_bands(model,x);ranked=sorted(audit['rows'],key=lambda r:r['value']/(2.5 if r['metric']=='ay_mps2' else 1.),reverse=True)
    ids=list(dict.fromkeys(r['event_index'] for r in ranked))[:3]
    fig,axes=plt.subplots(1,3,figsize=(15,4),squeeze=False)
    for ax,index in zip(axes[0],ids):
        d=next(v for v in audit['coverage']['intervals'] if v['event_index']==index)
        first=True
        for context in d['context_domains']:
            for ss,values in context_series(model,context,x,d['source_speed_kmh']):
                ax.plot(ss,values[:,0],color='#246ca5',label='ay / 2.5' if first else None)
                ax.plot(ss,values[:,1],color='#bb2450',label='jerk / 1.0' if first else None)
                first=False
        ax.axvspan(d['a'],d['b'],color='#e7a12b',alpha=.6,label='previously omitted band')
        worst=next(r for r in ranked if r['event_index']==index)
        peak=worst['value']/(2.5 if worst['metric']=='ay_mps2' else 1.)
        ax.scatter([worst['s']],[peak],color='#712c85',s=24,zorder=5)
        if not worst['numerically_resolved_span']:
            ax.annotate('raw knot query <1e-8 m',xy=(worst['s'],peak),xytext=(6,0),
                        textcoords='offset points',fontsize=7,color='#712c85')
        ax.axhline(1,color='black',ls='--');ax.set_xlabel('reference station (m)');ax.set_ylabel('limit ratio')
        ax.set_title('TOPO '+str(d['topology_record']['record_index'])+' / '+format(d['b']-d['a'],'.3f')+' m gap')
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Historical long-curve readback / unchanged source 60 km/h / NOT an accepted map')
    fig.tight_layout();fig.savefig(output,dpi=140);plt.close(fig)


def implementation_files(files):
    for folder in ('mapforge','spikes','scripts'):
        for p in (ROOT/folder).rglob('*.py'):files[str(p)]=sha(p)
    return files


def run(source,decision,previous,output):
    output=Path(output).resolve()
    if output.exists():raise FileExistsError('new evidence directory required')
    model,roles,files=load(source,decision,degree=5);implementation_files(files)
    result,jets,event_model,old_curve,old_files=compute(model,roles,previous);files.update(old_files)
    unchanged(files);output.mkdir(parents=True)
    dump(output/'report.json',result);dump(output/'closed-relaxed-jets.json',jets)
    draw(event_model,old_curve,output/'historical-event-readback.png',diagnostic=True,
         state_label='RED: historical event curve, rechecked and REJECTED (not a new map)')
    draw_gaps(event_model,old_curve,output/'transition-dynamics.png')
    unchanged(files)
    dump(output/'run.json',dict(source=str(Path(source).resolve()),decision=str(Path(decision).resolve()),
        previous=str(Path(previous).resolve()),runtime_versions=runtime_versions(),input_sha256=files,
        output_sha256={p.name:sha(p) for p in output.iterdir() if p.is_file()}))
    print(json.dumps(result['coverage_summary']),flush=True)
    return result


def verify(directory):
    directory=Path(directory);run=read(directory/'run.json');unchanged(run['input_sha256'])
    if run['runtime_versions']!=runtime_versions():raise ValueError('runtime changed')
    names={'report.json','closed-relaxed-jets.json','historical-event-readback.png','transition-dynamics.png'}
    if set(run['output_sha256'])!=names:raise ValueError('output inventory differs')
    unchanged({str(directory/n):h for n,h in run['output_sha256'].items()})
    model,roles,files=load(run['source'],run['decision'],degree=5);implementation_files(files)
    result,jets,_,_,old_files=compute(model,roles,run['previous']);files.update(old_files)
    if files!=run['input_sha256']:raise ValueError('input inventory differs')
    if digest(result)!=digest(read(directory/'report.json')):raise ValueError('fresh readback differs')
    saved=read(directory/'closed-relaxed-jets.json')
    if digest(saved)!=digest(jets):raise ValueError('free station state differs')
    unchanged(files)
    return dict(status='VERIFIED_COVERAGE_RECHECK_NOT_MAP',inputs=len(files),export_allowed=False,
                coverage=result['coverage_summary'],free_status=result['new_free_station_diagnosis']['status'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source');p.add_argument('decision');p.add_argument('previous');p.add_argument('output')
    p.add_argument('--verify',action='store_true');a=p.parse_args()
    if a.verify:print(verify(a.output))
    else:run(a.source,a.decision,a.previous,a.output)
    raise SystemExit(2)
