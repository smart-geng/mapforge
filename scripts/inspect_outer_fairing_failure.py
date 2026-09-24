"""Independent source/shape inspection of a rejected STATE, never an XODR."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.check_outer_event_control import inputs
from mapforge.repair_web.model import atomic,json_bytes,digest
from mapforge.repair_web.outer_event_fairing import CurvatureFairing
from mapforge.repair_web.outer_event_shape import reversal_support
from scripts.check_outer_event_written import boundary_poly


def main():
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();dest=args.out.resolve();trial=args.trial.resolve()
    if dest==ROOT/'out' or not dest.is_relative_to(ROOT/'out'):p.error('new workspace out child required')
    dest.mkdir(exist_ok=False)
    report=json.loads((trial/'solve.json').read_text(encoding='utf8'))
    event,control=inputs();fairing=CurvatureFairing(control)
    # Saved numerical state is immutable evidence. Its producer may be an old
    # code revision; verify ORIGINAL data bindings, record this verifier hash.
    bindings=json.loads((trial/'binding.json').read_text(encoding='utf8'))
    data_bindings={k:v for k,v in bindings.items() if Path(k).suffix.lower() not in ('.py',)}
    for path,sha in data_bindings.items():
        if digest(Path(path).read_bytes())!=sha:raise ValueError('original input drift: '+path)
    step=report['steps'][-1] if 'steps' in report else report
    x=np.array(step['rejected_state'],float)
    if x.shape!=(event.nvar,):raise ValueError('No saved rejected state to inspect')
    source=event.source_error(x);rev,_=reversal_support(event,x)
    peaks,_=fairing.continuous(x);tv,_=fairing.total_variation(x)
    failure=dict(schema='mapforge/rejected-fairing-state-inspection/v1',input_report_sha256=digest((trial/'solve.json').read_bytes()),
                 verifier_sha256=digest(Path(__file__).read_bytes()),data_bindings=data_bindings,
                 source_max_m=source['max_m'],source_violations=[r for r in source['rows'] if r['max_m']>.7500001],
                 reversal_m=rev.tolist(),reference_reversal_m=control.budgets.tolist(),
                 peak_curvature_and_rate=peaks.tolist(),curvature_total_variation=tv.tolist(),reference_caps=fairing.caps.tolist(),
                 geometry_state_only=True,compiled_xodr=False,esmini='NOT_RUN_NO_ACCEPTED_CANDIDATE',map_accepted=False)
    atomic(dest/'failure.json',json_bytes(failure))
    fig,axes=plt.subplots(3,1,figsize=(12,11),layout='constrained')
    for trace in event.traces:
        for ax in axes[:2]:
            if ax is axes[1] and trace['edge']!=3:continue
            ax.plot(trace['st'][:,0],trace['st'][:,1],color='#d99220',lw=2)
    from xml.etree import ElementTree as ET
    road=next(r for r in ET.fromstring(control.reference).findall('road') if r.get('id')=='11')
    for edge in range(5):
        ss=np.linspace(max(150,event.births.get(edge,event.start)),195,601)
        old=np.array([boundary_poly(road,edge,s)[0] for s in ss])
        rejected=np.array([event.row(edge,s)@x for s in ss])
        axes[0].plot(ss,old,color='#738297',lw=1.4,label='r6 XML reference' if edge==0 else None)
        axes[0].plot(ss,rejected,color='#c53240',lw=1.2,label='rejected state (NOT XML)' if edge==0 else None)
        if edge==3:
            axes[1].plot(ss,old,color='#738297',label='r6 XML reference')
            axes[1].plot(ss,rejected,color='#c53240',label='rejected state (NOT XML)')
            axes[1].plot(control.point.station,control.reference_t+report['requested_m'],'x',color='black',ms=9,label='final target NOT delivered')
    for trace,lo,hi,c in event.source_cells():
        ss=np.linspace(lo,hi,25)
        err=[abs(event.row(trace['edge'],s)@x-np.polynomial.polynomial.polyval(s-lo,c)) for s in ss]
        axes[2].plot(ss,err,color='#c53240',lw=1)
    axes[2].axhline(.75,color='black',ls='--',label='unchanged 0.75m source gate')
    worst=max(source['rows'],key=lambda r:r['max_m'])
    axes[2].plot(worst['witness_s'],worst['max_m'],'o',color='red',label='exact polynomial extremum')
    for ax in axes:ax.grid(alpha=.2);ax.legend(loc='best');ax.set_xlabel('Reference s (m)')
    for ax in axes[:2]:ax.set_xlim(150,195);ax.set_ylabel('Lateral t (m)')
    axes[2].set_xlim(event.start,event.end);axes[2].set_ylabel('Source error (m)')
    axes[0].set_title('All five physical boundaries: original SHP in orange')
    axes[1].set_title('The third-boundary bulge remains; no replacement of user map')
    axes[2].set_title('Every original source segment checked; overshoot NOT waived')
    fig.suptitle('REJECTED SOLVER STATE ONLY | NO NEW XODR | NO ESMINI ACCEPTANCE\nSource max %.9fm > 0.75m'%source['max_m'],fontsize=11)
    fig.savefig(dest/'rejected-state-review.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(source_max_m=source['max_m'],source_violations=len(failure['source_violations']),worst_source=worst,
                           reversal_delta_m=(rev-control.budgets).tolist(),compiled_xodr=False)))


if __name__=='__main__':main()
