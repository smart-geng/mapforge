"""Inspect saved layout states, never optimize or write a rejected map."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.check_outer_event_control import inputs, DECISION
from mapforge.repair_web.outer_event_layout import LayoutFairing
from mapforge.repair_web.outer_event_shape import reversal_support
from mapforge.repair_web.outer_event_control import _road
from mapforge.repair_web.model import atomic,json_bytes,digest
from scripts.check_outer_event_written import boundary_poly


def main():
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();dest=args.out.resolve();trial=args.trial.resolve()
    if dest==ROOT/'out' or not dest.is_relative_to(ROOT/'out'):p.error('new workspace out child required')
    dest.mkdir(exist_ok=False)
    binding=json.loads((trial/'binding.json').read_text(encoding='utf8'))
    for path,sha in binding.items():
        if digest(Path(path).read_bytes())!=sha:raise ValueError('Original/code binding drift: '+path)
    summary=json.loads((trial/'summary.json').read_text(encoding='utf8'))
    event,control=inputs();road=_road(control.reference,event)
    roles={(d['source_lane_id'],d['contact']) for d in yaml.safe_load(DECISION.read_text(encoding='utf8'))['decisions']
           if d['physical_authority']=='original_left_right_boundaries' and d['lane_path_role']=='movement_path_observation'}
    fig,axes=plt.subplots(2,2,figsize=(14,9),layout='constrained')
    targets=[None,3,2,None]
    for trace in event.traces:
        for i,ax in enumerate(axes.ravel()[:3]):
            if targets[i] is not None and trace['edge']!=targets[i]:continue
            ax.plot(trace['st'][:,0],trace['st'][:,1],color='#dd9626',lw=2.)
    for edge in range(5):
        ss=np.linspace(max(event.births.get(edge,event.start),150),195,500)
        ts=[boundary_poly(road,edge,s)[0] for s in ss]
        for i,ax in enumerate(axes.ravel()[:3]):
            if targets[i] is not None and edge!=targets[i]:continue
            ax.plot(ss,ts,color='#68798d',lw=1.5,label='original r6 XML' if edge==(targets[i] or 0) else None)
    rows=[]
    for item,color in zip(summary['history'],['#bb283f','#734cc2']):
        path=trial/item['layout'];pre=json.loads((path/'preflight.json').read_text(encoding='utf8'))
        report=json.loads((path/'solve.json').read_text(encoding='utf8'))
        if report.get('rejected_state') is None:continue
        fairing=LayoutFairing(control,replace(event.scope,knots=tuple(pre['new_knots'])),approved_roles=roles)
        state=np.asarray(report['rejected_state'],float)
        source=fairing.event.source_error(state);rev,_=reversal_support(fairing.event,state)
        peaks,_=fairing.continuous(state);tv,_=fairing.total_variation(state)
        rows.append(dict(layout=item['layout'],report_sha256=digest((path/'solve.json').read_bytes()),
                         source_max_m=source['max_m'],source_violations=[r for r in source['rows'] if r['max_m']>.7500001],
                         reversal_delta_m=(rev-control.budgets).tolist(),
                         peak_curvature_rate_excess=(peaks-fairing.caps[:,:2]).tolist(),
                         curvature_variation_delta=(tv-fairing.caps[:,2]).tolist(),
                         target_error_m=float(fairing.control.handle@state-control.reference_t-summary['requested_m']),
                         state_only=True,compiled_xodr=False))
        for edge in range(5):
            ss=np.linspace(max(event.births.get(edge,event.start),150),195,500)
            ts=[fairing.event.row(edge,s)@state for s in ss]
            for i,ax in enumerate(axes.ravel()[:3]):
                if targets[i] is not None and edge!=targets[i]:continue
                ax.plot(ss,ts,color=color,lw=1.25,label=item['layout']+' (rejected state)' if edge==(targets[i] or 0) else None)
        for trace,lo,hi,c in fairing.event.source_cells():
            ss=np.linspace(lo,hi,15)
            yy=[abs(fairing.event.row(trace['edge'],s)@state-np.polynomial.polynomial.polyval(s-lo,c)) for s in ss]
            axes[1,1].plot(ss,yy,color=color,lw=.8)
        worst=max(source['rows'],key=lambda r:r['max_m'])
        axes[1,1].plot(worst['witness_s'],worst['max_m'],'o',color=color,label=item['layout']+' exact max %.6fm'%worst['max_m'])
    axes[0,1].plot(control.point.station,control.reference_t+summary['requested_m'],'x',color='black',ms=10,label='requested source point')
    for ax,title in zip(axes.ravel(),['All five physical edges (orange: original SHP)','Target edge 3: meeting a point is insufficient','Neighbour edge 2: deformation must also be checked','Whole source envelope: unchanged 0.75m gate']):
        ax.set_title(title);ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlabel('reference s (m)')
    for ax in axes.ravel()[:3]:ax.set_xlim(150,195);ax.set_ylabel('lateral t (m)')
    axes[1,1].axhline(.75,color='black',ls='--');axes[1,1].set_xlim(event.start,event.end);axes[1,1].set_ylabel('source error (m)')
    fig.suptitle('REJECTED LONG-LAYOUT STATES | NO NEW XODR | NO WEB APPLY\nOriginal XML reference remains unchanged; no independent consumer acceptance',fontsize=12)
    fig.savefig(dest/'long-layout-review.png',dpi=140);plt.close(fig)
    result=dict(schema='mapforge/long-layout-rejection-inspection/v1',source_report_sha256=digest((trial/'summary.json').read_bytes()),
                verifier_sha256=digest(Path(__file__).read_bytes()),binding_count=len(binding),binding_drift=0,
                history=rows,map_accepted=False,esmini='NOT_RUN_NO_ACCEPTED_XODR',
                method='saved coefficients re-evaluated with same geometry kernel; not third-party verification')
    atomic(dest/'inspection.json',json_bytes(result))
    print(json.dumps(dict(results=[dict(layout=r['layout'],source_max_m=r['source_max_m'],source_violations=len(r['source_violations']),
                                       reversal_delta_m=r['reversal_delta_m']) for r in rows],map_accepted=False)))


if __name__=='__main__':main()
