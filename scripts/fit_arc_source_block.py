"""Replayable source-native Line/Arc + long cubic boundary model experiment.

Explicit chart trials are model comparisons, NOT global optimization or a map.
No production default, source geometry, speed or already-built XODR is changed.
"""
import argparse
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import minimize

from scripts.fit_source_boundary_block import load, unchanged, runtime_versions
from scripts.gen_all import _sha256
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from spikes.clarabel_joint_candidate import interior_qp
from spikes.shared_boundary_dynamics import refine_dynamics, verify_refinement


def best_source_trial(rows):
    """A numerical LP failure is not a geometric impossibility or a candidate."""
    eligible=[]
    for i,row in enumerate(rows):
        phase=row.get('phase',{});value=phase.get('minimum_uniform_constraint_slack_m')
        if (phase.get('status') in ('FEASIBLE','INFEASIBLE','OPTIMIZATION_FAILED') and
            isinstance(value,(int,float)) and np.isfinite(value) and value>=-1e-7):eligible.append(i)
    return min(eligible,key=lambda i:rows[i]['phase']['minimum_uniform_constraint_slack_m']) if eligible else None


def draw(model, x, path, rejected=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    curves=[];omission=None
    if x is not None:
        try:
            for f in model.families:
                ss=np.linspace(f.knots[0],f.knots[-1],1200);c=BSpline(f.knots,x[f.columns],3)
                curves.append(model.axis.world(np.c_[ss,c(ss)]))
        except ValueError as exc:
            if not rejected:raise
            curves=[];omission=str(exc)
    fig,axes=plt.subplots(1,3,figsize=(14,7))
    for ax in axes:
        for k,xy in model.source_xy.items():
            ax.plot(xy[:,0],xy[:,1],color='#e2861c' if k.startswith('boundary:') else '#92969b',
                    lw=1 if k.startswith('boundary:') else .7,ls='-' if k.startswith('boundary:') else '--')
        for xy in curves:ax.plot(xy[:,0],xy[:,1],color='#bd2753' if rejected else '#1466b8',lw=1)
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('local x (m)');ax.set_ylabel('local y (m)')
    axes[0].set_title('Entire source boundary block')
    for ax,c in zip(axes[1:],model.contacts['role_conflicts']):
        p=c['collapsed_boundary_xy'];ax.set_xlim(p[0]-5,p[0]+5);ax.set_ylim(p[1]-12,p[1]+12)
        ax.set_title('Source taper '+c['source_lane_id'][-7:])
    fig.suptitle('Single Arc/Line + shared LONG cubics / NOT an accepted XODR\n'
                 'Orange: raw boundaries; gray: raw lane paths; '+
                 ('REJECTED nonregular diagnosis omitted; original source ONLY' if omission else
                  'RED: rejected full-source sampled diagnosis (NOT a repair)' if rejected else 'blue: source-feasible candidate'))
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(path,dpi=140);plt.close(fig)
    return dict(candidate_rendered=bool(curves),omitted_reason=omission,rejected_diagnosis=rejected)


def run(source, decision, output, charts=((0.,0.),(0.,-.001),(0.,.001)), optimize_axis=False):
    output=Path(output).resolve()
    if output.exists():raise FileExistsError('new output directory required')
    original,roles,files=load(source,decision,degree=3)
    for tree in ('mapforge','spikes','scripts'):
        files.update({str(p.resolve()):_sha256(p) for p in (ROOT/tree).rglob('*.py')})
    rows=[];models=[];cache={}
    def trial(angle,k):
        if (float(angle),float(k)) in cache:return cache[(float(angle),float(k))]
        record=dict(heading_delta_rad=float(angle),reference_curvature=float(k),export_allowed=False,
                    xodr_generated=False,source_speed_changed=False)
        try:
            model=ArcSourceBoundaryBlock(original,angle,k)
        except ValueError as exc:
            record.update(model=None,reason=str(exc));models.append(None)
        else:
            with patch('spikes.source_contact_fit._convex_qp',interior_qp):x,phase=model.solve()
            _,necessary=model.original_vertex_preflight()
            diagnostic,sampled=model.sampled_source_preflight()
            record.update(model=model.describe(),phase=phase,initial_coefficients=None,coefficients=None,
                          audit=None,dynamics=None,powers=None,original_vertex_preflight=necessary,sampled_source_preflight=sampled,
                          rejected_diagnostic_coefficients=diagnostic.tolist() if x is None else None)
            if x is not None:
                first=x.copy();x,dynamics=refine_dynamics(model,x)
                record.update(initial_coefficients=first.tolist(),coefficients=x.tolist(),
                              audit=model.audit(x),dynamics=dynamics,powers=model.coefficients(x))
            models.append(model)
        rows.append(record)
        cache[(float(angle),float(k))]=len(rows)-1
        print(json.dumps(dict(index=len(rows)-1,angle=angle,curvature=k,
                              phase=record.get('phase',{}).get('status'),reason=record.get('reason'),
                              slack=record.get('phase',{}).get('minimum_uniform_constraint_slack_m'),
                              dynamics=record.get('audit',{}).get('physical_midpoint_dynamics_worst') if record.get('audit') else None),
                         ensure_ascii=False),flush=True)
        return len(rows)-1
    for angle,k in charts:trial(angle,k)
    optimization=None
    if optimize_axis:
        def objective(z):
            i=trial(float(z[0]/10.),float(z[1]/1000.));record=rows[i]
            return float(record.get('phase',{}).get('minimum_uniform_constraint_slack_m',100.))
        answer=minimize(objective,[0.,0.],method='Nelder-Mead',bounds=[(-1.,1.),(-4.,4.)],
                        options=dict(maxfev=24,xatol=1e-4,fatol=1e-7,initial_simplex=[[0.,0.],[.3,0.],[0.,1.]]))
        best=best_source_trial(rows)
        optimization=dict(method='bounded variable projection: continuous heading/curvature + all shared cubic coefficients',
                          function_evaluations=int(answer.nfev),optimizer_success=bool(answer.success),
                          message=str(answer.message),best_trial_index=best,
                          minimum_source_model_slack_m=rows[best]['phase']['minimum_uniform_constraint_slack_m'] if best is not None else None,
                          global_impossibility_proven=False,all_incident_connectors_solved=False,
                          free_source_event_positions_optimized=False,export_allowed=False)
    unchanged(files);output.mkdir(parents=True)
    report=dict(schema='mapforge.arc-source-block-trials/v1',status='RESEARCH_ONLY_NOT_MAP',
                export_allowed=False,xodr_generated=False,whole_network_jointly_solved=False,
                continuous_axis_optimization_ran=bool(optimize_axis),axis_optimization=optimization,source_tolerance_m=.35,
                trials=rows,remaining=['source end-domain closure','movement paths','all incident connectors',
                                      'all roads','production XODR write/readback and independent simulation'])
    def write(name,data):(output/name).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    products=['report.json']
    for i,(m,r) in enumerate(zip(models,rows)):
        if m is not None:
            rejected=r['coefficients'] is None
            name=f'chart-{i}.png'
            r['visualization']=draw(m,np.array(r['rejected_diagnostic_coefficients'] if rejected else r['coefficients']),output/name,rejected)
            products.append(name)
    write('report.json',report)
    write('run.json',dict(source=str(Path(source).resolve()),decision=str(Path(decision).resolve()),
                          charts=[(r['heading_delta_rad'],r['reference_curvature']) for r in rows],
                          optimize_axis=bool(optimize_axis),runtime=runtime_versions(),input_sha256=files,
                          outputs_sha256={n:_sha256(output/n) for n in products}))
    unchanged(files)
    return report


def verify(output):
    output=Path(output)
    run=json.loads((output/'run.json').read_text(encoding='utf-8'));unchanged(run['input_sha256'])
    if run['runtime']!=runtime_versions():raise ValueError('runtime changed')
    for n,h in run['outputs_sha256'].items():
        if Path(n).name!=n or _sha256(output/n)!=h:raise ValueError('changed output')
    report=json.loads((output/'report.json').read_text(encoding='utf-8'))
    expected=dict(schema='mapforge.arc-source-block-trials/v1',status='RESEARCH_ONLY_NOT_MAP',
                  export_allowed=False,xodr_generated=False,whole_network_jointly_solved=False,
                  continuous_axis_optimization_ran=run['optimize_axis'],source_tolerance_m=.35,
                  remaining=['source end-domain closure','movement paths','all incident connectors',
                             'all roads','production XODR write/readback and independent simulation'])
    if {k:v for k,v in report.items() if k not in ('trials','axis_optimization')}!=expected:raise ValueError('unsupported report qualification')
    opt=report['axis_optimization']
    if run['optimize_axis']:
        best=best_source_trial(report['trials'])
        if (opt['best_trial_index']!=best or
            opt['minimum_source_model_slack_m']!=(report['trials'][best]['phase']['minimum_uniform_constraint_slack_m'] if best is not None else None) or
            any(opt[key] is not False for key in ('export_allowed','global_impossibility_proven',
                     'all_incident_connectors_solved','free_source_event_positions_optimized'))):
            raise ValueError('unsupported variable-projection qualification')
    elif opt is not None:raise ValueError('unrequested axis search')
    original,_,files=load(run['source'],run['decision'],degree=3)
    for tree in ('mapforge','spikes','scripts'):
        files.update({str(p.resolve()):_sha256(p) for p in (ROOT/tree).rglob('*.py')})
    if files!=run['input_sha256']:raise ValueError('source/code inventory changed')
    if len(report['trials'])!=len(run['charts']):raise ValueError('missing chart trial')
    products={'report.json'}
    for i,(row,(angle,k)) in enumerate(zip(report['trials'],run['charts'])):
        if any(row.get(key) is not False for key in ('export_allowed','xodr_generated','source_speed_changed')):
            raise ValueError('unsupported chart acceptance claim')
        if row['heading_delta_rad']!=angle or row['reference_curvature']!=k:raise ValueError('chart parameters changed')
        try:m=ArcSourceBoundaryBlock(original,angle,k)
        except ValueError as exc:
            if row!={**{key:row[key] for key in ('heading_delta_rad','reference_curvature','export_allowed','xodr_generated','source_speed_changed')},
                     'model':None,'reason':str(exc)}:raise ValueError('construction rejection differs')
            continue
        if m.describe()!=row['model']:raise ValueError('source model differs')
        with patch('spikes.source_contact_fit._convex_qp',interior_qp):x,phase=m.solve()
        if phase!=row['phase']:raise ValueError('source phase differs')
        _,necessary=m.original_vertex_preflight()
        if necessary!=row['original_vertex_preflight']:raise ValueError('original vertex preflight differs')
        diagnostic,sampled=m.sampled_source_preflight()
        if sampled!=row['sampled_source_preflight']:raise ValueError('sampled source preflight differs')
        if x is None:
            if any(row[key] is not None for key in ('initial_coefficients','coefficients','audit','dynamics','powers')):
                raise ValueError('rejected model claimed geometry')
            if not np.allclose(diagnostic,row['rejected_diagnostic_coefficients'],rtol=0,atol=1e-8):
                raise ValueError('rejected diagnosis differs')
        else:
            if row['rejected_diagnostic_coefficients'] is not None:raise ValueError('unexpected rejected diagnosis')
            if not np.allclose(x,row['initial_coefficients'],rtol=0,atol=1e-8):raise ValueError('initial state differs')
            verify_refinement(m,x,row['coefficients'],row['dynamics'])
            if m.audit(row['coefficients'])!=row['audit'] or m.coefficients(row['coefficients'])!=row['powers']:
                raise ValueError('geometry/compilation differs from independent recomputation')
        draw_x=diagnostic if x is None else np.array(row['coefficients'])
        omission=None
        try:
            for f in m.families:
                ss=np.linspace(f.knots[0],f.knots[-1],1200);c=BSpline(f.knots,draw_x[f.columns],3)
                m.axis.world(np.c_[ss,c(ss)])
        except ValueError as exc:omission=str(exc)
        if row['visualization']!=dict(candidate_rendered=omission is None,omitted_reason=omission,rejected_diagnosis=x is None):
            raise ValueError('visualization qualification differs from curve regularity')
        products.add(f'chart-{i}.png')
        allowed={'heading_delta_rad','reference_curvature','export_allowed','xodr_generated','source_speed_changed',
                 'model','phase','initial_coefficients','coefficients','audit','dynamics','powers',
                 'original_vertex_preflight','sampled_source_preflight','rejected_diagnostic_coefficients','visualization'}
        if set(row)!=allowed:raise ValueError('unexpected chart metadata')
    if set(run['outputs_sha256'])!=products:raise ValueError('output inventory differs')
    return dict(status='VERIFIED_RESEARCH_NOT_MAP',trials=len(report['trials']),input_hashes=len(files),export_allowed=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('decision');p.add_argument('output')
    p.add_argument('--optimize-axis',action='store_true');a=p.parse_args()
    run(a.source,a.decision,a.output,optimize_axis=a.optimize_axis)
    print(json.dumps(verify(a.output),ensure_ascii=False));raise SystemExit(2)
