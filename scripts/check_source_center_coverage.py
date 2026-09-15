"""Rebuild shared source-center constraints; no accepted map or speed change."""
import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_center_support import frame,source_slice,source_xy
from scripts.fit_source_boundary_block import load,unchanged,runtime_versions,draw
from scripts.recheck_speed_contract import sha,dump
from spikes.source_event_basis import event_aligned_model
from spikes.shared_boundary_dynamics import refine_dynamics,verify_refinement


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def historical(directory,model,layout):
    directory=Path(directory).resolve();run=read(directory/'run.json');old=read(directory/'event-basis.json')
    files={str(directory/n):sha(directory/n) for n in ('run.json','report.json','event-basis.json')}
    for name in ('report.json','event-basis.json'):
        if files[str(directory/name)]!=run['output_sha256'][name]:raise ValueError('historical parameter hash mismatch')
    origin=read(directory/'report.json')
    if origin['original_scope_sha']!=model.scope['content_sha256'] or origin['roles_sha']!=model.roles['content_sha256']:
        raise ValueError('historical source or approved roles differ')
    if digest(layout)!=digest(old['layout']):raise ValueError('historical layout differs')
    current=model.describe()
    for key in ('road','variables','polynomial_degree','families','chart','source_lanes','source_boundary_features',
                'long_families','source_role_sha256','physical_continuation_sha256','minimum_independent_span_m'):
        if digest(current[key])!=digest(old['model'][key]):raise ValueError('historical coefficient basis differs: '+key)
    return np.asarray(old['coefficients'],float),files


def calculate(model,old,initial,x,phase,dynamics):
    before=model.audit(old);after=model.audit(x) if x is not None else None
    plan=model.center_support
    return dict(status='RESEARCH_CENTER_COVERAGE_NOT_MAP',scope_sha=model.scope['content_sha256'],
        roles_sha=model.roles['content_sha256'],model=model.describe(),phase=phase,
        previous_coefficients=old.tolist(),candidate_coefficients=x.tolist() if x is not None else None,
        initial_coefficients=initial.tolist() if initial is not None else None,dynamics_refinement=dynamics,
        previous_curve_audit=before,candidate_audit=after,
        coverage=dict(previous_unsupported_records=len(model.legacy_uncovered_centers),
            unresolved_intervals=len(plan['unresolved']),full_interval_checks=len(plan['pieces']),
            exterior_caps=sum(d['kind']=='euclidean_endpoint_cap' for d in plan['endpoint_caps']),
            source_contact_roundoff_caps=sum(d['kind']=='numerical_contact_cap' for d in plan['endpoint_caps']),
            separate_movement_observations=plan['separate_movement_observations'],geometry_spans_added=0),
        new_research_coefficients_solved=x is not None,source_modified=False,source_speed_changed=False,
        source_tolerance_changed=False,source_vertices_removed=0,xodr_generated=False,export_allowed=False,
        reverse_center_fidelity_certified=False,full_curve_dynamics_certified=False,
        next_gates=['unchanged dynamics/design-contract decision','movement observations','writable shared road/all connectors',
                    'MAP width interpretation','absolute CRS','all-map regression','Web editor'])


def draw_caps(model,old,x,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    caps=[d for d in model.center_support['endpoint_caps'] if d['kind']=='euclidean_endpoint_cap']
    caps=sorted(caps,key=lambda d:max(abs(w['longitudinal_m']) for w in d['witnesses']),reverse=True)[:4]
    fig,axes=plt.subplots(2,2,figsize=(10,9))
    for ax,d in zip(axes.flat,caps):
        key=d['feature'];raw=model.raw[key];s=d['host_station'];origin=np.array(d['witnesses'][0]['source_xy'])
        ss=source_slice(model,key,max(raw[0,0],s-1.),min(raw[-1,0],s+1.))[:,0]
        xy=np.array([source_xy(model,key,z) for z in ss])-origin
        ax.plot(xy[:,0],xy[:,1],color='#e38726',lw=3,label='original source (retained)')
        witness=np.array([w['source_xy'] for w in d['witnesses']])-origin
        ax.scatter(witness[:,0],witness[:,1],c='#e38726',s=35,zorder=5)
        families=[model.families[model.owner[k]] for k in (d['left'],d['right'])]
        lo=max(s-1.,*[f.knots[0] for f in families]);hi=min(s+1.,*[f.knots[-1] for f in families])
        query=np.unique(np.r_[s,np.linspace(lo,hi,80)])
        errors=[]
        for state,color,name in ((old,'#bf2948','historical curve'),(x,'#246c9f','new shared candidate')):
            if state is None:continue
            pts=[]
            for z in query:
                p,_,n=frame(model,z);t=.5*(model.expression(d['left'],z)+model.expression(d['right'],z))@state
                pts.append(p+t*n-origin)
            pts=np.asarray(pts);ax.plot(pts[:,0],pts[:,1],color=color,lw=1.5,label=name)
            endpoint=pts[np.argmin(abs(query-s))];ax.scatter(*endpoint,c=color,s=35,zorder=5)
            errors.append(name+': '+format(float(np.max(np.linalg.norm(witness-endpoint,axis=1))),'.3f')+' m')
        ax.add_patch(Circle((0,0),model.source_tol,fill=False,ls='--',color='gray',label='0.35 m total budget'))
        ax.set_xlim(-.55,.55);ax.set_ylim(-.55,.55);ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.set_title(d['source_lane'][-7:]+' / '+d['side']+'\n'+'; '.join(errors),fontsize=9)
        ax.set_xlabel('local east relative to source tip (m)');ax.set_ylabel('local north (m)')
    if caps:
        handles,labels=axes.flat[0].get_legend_handles_labels()
        fig.legend(handles,labels,loc='lower center',ncol=2,fontsize=9)
    fig.suptitle('Actual curve readback at source end cuts / SAME 0.35 m budget\nNo source extension, no short geometry, NOT accepted XODR',fontsize=12)
    fig.tight_layout(rect=(0,.06,1,.92));fig.savefig(path,dpi=150);plt.close(fig)


def files_with_code(files):
    for folder in ('mapforge','spikes','scripts'):
        for p in (ROOT/folder).rglob('*.py'):files[str(p)]=sha(p)
    return files


def run(source,decision,previous,output):
    output=Path(output).resolve()
    if output.exists():raise FileExistsError('new output directory required')
    model,roles,files=load(source,decision,degree=5);model,layout=event_aligned_model(model)
    old,oldfiles=historical(previous,model,layout);files.update(oldfiles);files_with_code(files)
    initial,phase=model.solve();x=initial;dynamics=None
    if initial is not None:
        print('SOURCE FEASIBLE; minimizing unchanged actual dynamics',flush=True)
        x,dynamics=refine_dynamics(model,initial,iterations=12,seconds=90.)
    result=calculate(model,old,initial,x,phase,dynamics)
    unchanged(files);output.mkdir(parents=True);dump(output/'report.json',result)
    draw(model,x if x is not None else old,output/'shared-source-readback.png',diagnostic=True,
         state_label='RED: shared source candidate; dynamics/whole map NOT accepted' if x is not None else 'RED: unchanged historical curve; no candidate')
    draw_caps(model,old,x,output/'source-end-cuts.png');unchanged(files)
    dump(output/'run.json',dict(source=str(Path(source).resolve()),decision=str(Path(decision).resolve()),
        previous=str(Path(previous).resolve()),inputs_sha256=files,runtime_versions=runtime_versions(),
        outputs_sha256={p.name:sha(p) for p in output.iterdir() if p.is_file()}))
    print(result['coverage'],flush=True)
    for name,a in [('old',result['previous_curve_audit']),('candidate',result['candidate_audit'])]:
        if a is not None:print(name,a['full_source_center_support']['maximum_conservative_error_m'],
            a['full_source_center_support']['physical_source_to_center_certified'],a['physical_midpoint_dynamics_worst'],flush=True)
    return result


def verify(directory):
    directory=Path(directory);record=read(directory/'run.json');saved=read(directory/'report.json')
    unchanged(record['inputs_sha256'])
    if record['runtime_versions']!=runtime_versions():raise ValueError('runtime mismatch')
    if set(record['outputs_sha256'])!={'report.json','shared-source-readback.png','source-end-cuts.png'}:
        raise ValueError('incomplete output inventory')
    unchanged({str(directory/n):h for n,h in record['outputs_sha256'].items()})
    model,roles,files=load(record['source'],record['decision'],degree=5);model,layout=event_aligned_model(model)
    old,oldfiles=historical(record['previous'],model,layout);files.update(oldfiles);files_with_code(files)
    if files!=record['inputs_sha256']:raise ValueError('source/parameter/code inventory differs')
    initial,phase=model.solve()
    if digest(initial.tolist() if initial is not None else None)!=digest(saved['initial_coefficients']):
        raise ValueError('fresh hard-geometry initializer differs')
    x=np.asarray(saved['candidate_coefficients'],float) if saved['candidate_coefficients'] is not None else None
    dynamics=saved['dynamics_refinement']
    if initial is not None:
        if x is None or dynamics is None:raise ValueError('missing final shared state')
        verify_refinement(model,initial,x,dynamics)
    elif x is not None or dynamics is not None:raise ValueError('candidate without source-feasible initialization')
    fresh=calculate(model,old,initial,x,phase,dynamics)
    if digest(saved)!=digest(fresh):raise ValueError('fresh source center reconstruction differs')
    unchanged(files)
    return dict(status='VERIFIED_SOURCE_CENTER_RESEARCH_NOT_MAP',inputs=len(files),coverage=fresh['coverage'],
        physical_source_to_center_certified=fresh['candidate_audit']['full_source_center_support']['physical_source_to_center_certified']
            if fresh['candidate_audit'] else False,export_allowed=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source');p.add_argument('decision')
    p.add_argument('previous');p.add_argument('output');p.add_argument('--verify',action='store_true');a=p.parse_args()
    if a.verify:print(verify(a.output))
    else:run(a.source,a.decision,a.previous,a.output)
    raise SystemExit(2)
