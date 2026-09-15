"""Replay saved ordinary-source coefficients; this is NOT XODR validation."""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.interpolate import BSpline
from scripts.build_ordinary_source_road import load_road,draw_source_state
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged
from scripts.gen_all import _sha256
from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_cubic_export import cubic_model,constrain_written_endpoints,constrain_flat_connector_port


def endpoint_jet(description,coefficients,feature,station):
    """Evaluate direct stored scalar spline coefficients in world coordinates.

    No optimizer constraint matrix/expression row is reused. This is source
    STATE readback only; no claim about an exported or consumed XODR.
    """
    families=[f for f in description['families'] if feature in f['features']]
    if len(families)!=1:raise ValueError('ambiguous stored boundary family')
    f=families[0];a,b=f['column_range'];kn=np.asarray(f['knots'])
    if station<kn[0]-1e-7 or station>kn[-1]+1e-7:raise ValueError('no source extrapolation')
    s=float(np.clip(station,kn[0],kn[-1]));c=BSpline(kn,np.asarray(coefficients)[a:b],3,extrapolate=False)
    t,dt,ddt=[float(c(s,d)) for d in range(3)]
    chart=description['chart'];origin=np.asarray(chart['origin']);base=np.asarray(chart['tangent'])
    heading=math.atan2(base[1],base[0]);k=float(description['reference_curvature'])
    if k:
        xy=origin+np.array([(math.sin(heading+k*s)-math.sin(heading))/k,
                            -(math.cos(heading+k*s)-math.cos(heading))/k])
    else:xy=origin+s*base
    e=np.array([math.cos(heading+k*s),math.sin(heading+k*s)]);n=np.array([-e[1],e[0]])
    v=(1-k*t)*e+dt*n;acc=-2*k*dt*e+(k*(1-k*t)+ddt)*n
    if np.linalg.norm(v)<1e-8:raise ValueError('singular boundary parameterization')
    return dict(xy=(xy+t*n).tolist(),heading_rad=math.atan2(v[1],v[0]),
                curvature_per_m=float((v[0]*acc[1]-v[1]*acc[0])/np.linalg.norm(v)**3))


def run(directory,output):
    directory=Path(directory).resolve();output=Path(output).resolve()
    if output.exists():raise FileExistsError('preserve prior evidence; use a new output directory')
    state=json.loads((directory/'shared-state.json').read_text(encoding='utf8'))
    saved=json.loads((directory/'report.json').read_text(encoding='utf8'))
    changed_code=[]
    for name,expected in state['source_hashes'].items():
        path=Path(name).resolve()
        if _sha256(path)==expected:continue
        if path.suffix=='.py' and any(parent in path.parents for parent in (ROOT/'scripts',ROOT/'mapforge',ROOT/'spikes')):
            changed_code.append(dict(path=str(path),previous_sha256=expected,current_sha256=_sha256(path)))
        else:raise ValueError('source, configuration or decision changed since this saved state')
    original,roles,hashes,_=load_road(state['source_directory'],state['decision_file'],state['road'],
        state.get('previous_source_directory'))
    model=cubic_model(original,minimum_span=state['minimum_width_span'],end_axis=state['end_axis'])
    reason=None
    try:model=constrain_written_endpoints(model)
    except ValueError as exc:
        if 'full written endpoint budget impossible' not in str(exc):raise
        reason=str(exc)
    if state['flat_port']:model=constrain_flat_connector_port(model)
    description=model.describe()
    if digest(description)!=state['model_sha']:raise ValueError('current model differs; saved coefficients cannot be silently reused')
    if reason!=saved.get('reason'):raise ValueError('export-domain decision changed')
    x=np.asarray(state['coefficients']);audit=model.audit(x)
    if (audit['inequality_violation']>1e-7 or audit['scaled_C2_residual']>1e-7
            or audit['boundary_same_chart_max_m']>.35+1e-7 or audit['exact_width_min_m'] < -1e-7
            or not audit['full_source_center_support']['physical_source_to_center_certified']):
        raise ValueError('saved source coefficients fail the current model')
    rows=[]
    for item in model.contacts['source_endpoint_inventory']:
        if item['source_lane_id'] not in model.source_ids or not item['width_known'] or item['width_mm']!=0:continue
        jets=[]
        for side in ('left','right'):
            point=model.contacts['boundary_endpoints'][item['boundary_endpoints'][side]]
            station=float(model.axis.project(np.asarray(point['xy']))[0])
            jets.append(endpoint_jet(description,x,point['feature'],station))
        a,b=jets;angle=abs(math.remainder(a['heading_rad']-b['heading_rad'],2*math.pi))
        gap=float(np.linalg.norm(np.subtract(a['xy'],b['xy'])))
        dk=abs(a['curvature_per_m']-b['curvature_per_m'])
        rows.append(dict(source_lane_id=item['source_lane_id'],contact=item['contact'],gap_m=gap,
                         heading_gap_rad=angle,curvature_gap_per_m=dk,
                         pass_=gap<=1e-7 and angle<=1e-8 and dk<=1e-8,jets=jets))
    if not all(r['pass_'] for r in rows):raise ValueError('one or more actual zero-width edge joins failed')
    hashes.update({str(p):_sha256(p) for p in (Path(__file__).resolve(),directory/'shared-state.json',directory/'report.json')})
    unchanged(hashes);output.mkdir(parents=True,exist_ok=False)
    draw_source_state(model,x,output/'source-state.png')
    report=dict(status='SOURCE_STATE_REPLAYED_NOT_XODR',source_state=str(directory),model_sha=state['model_sha'],
        source_hashes=hashes,changed_code_revalidated=changed_code,audit=audit,zero_width_joins=rows,
        unresolved_export_domain=reason,source_roles=roles['resolved'],
        xodr_generated=False,independent_consumer_ran=False,whole_map_accepted=False,
        movement_paths_validated=False,source_changed=False)
    unchanged(hashes);report['image_sha256']=_sha256(output/'source-state.png');dump(output/'report.json',report)
    print(json.dumps(dict(status=report['status'],zero_width_joins=rows,unresolved_export_domain=reason),ensure_ascii=False),flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output');a=p.parse_args()
    run(a.directory,a.output);raise SystemExit(2)
