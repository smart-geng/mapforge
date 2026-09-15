"""Evaluate source-bound moving coordinates; NO optimizer and NO XODR writer.

The whole source/contact inventory is checked separately from the two existing
approved component fixtures. Those fixtures never become a whole-map PASS.
"""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from lxml import etree as ET
from scripts.prepare_joint_reconstruction import verify_preparation
from scripts.build_ordinary_source_road import load_road
from scripts.gen_all import _sha256
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops.source_contacts import compile_source_contacts
from mapforge.ops.coupled_source_junction import MovableSourceParentState
from mapforge.ops.port_dependencies import PortDependencies
from spikes.arc_source_boundary import ArcSourceBoundaryBlock


def dump(path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf8')


def component_check(group, row):
    saved = Path(row['component'])/'shared-state.json'
    state = json.loads(saved.read_text(encoding='utf8')); rid = row['road']
    raw, roles, files, _ = load_road(state['source_directory'], state['decision_file'], rid,
                                    state.get('previous_source_directory'))
    raw.min_span = state['minimum_width_span']
    g = ET.parse(group['artifact']).getroot().find(f"road[@id='{rid}']/planView/geometry")
    k = float(g.find('arc').get('curvature')) if g.find('arc') is not None else 0.
    h = float(g.get('hdg'))-k*row['compiled']['chart_start_m']
    delta = h-math.atan2(*raw.chart['tangent'][::-1])
    delta = math.atan2(math.sin(delta), math.cos(delta))
    # Replay the saved coordinate hypothesis, not source_end_axis()/Powell.
    model = ArcSourceBoundaryBlock(raw, delta, k)
    parent = MovableSourceParentState(raw, model, np.array(row['coefficients']), row['compiled'])
    initial = parent.evaluate(parent.initial)
    graph = PortDependencies(ET.parse(group['artifact']).getroot())
    port_uses = [(cid, end, p) for cid, ports in graph.connections.items()
                 for end, p in enumerate(ports) if p.road == rid]
    contacts = {port.contact for _,_,port in port_uses}
    if len(contacts)!=1:
        raise ValueError('component probe requires an explicit single junction-side cut')
    cp=next(iter(contacts))
    rows = []
    # Predeclared finite coordinate probes: no objective, seed search or
    # candidate selection. Failed probes remain failed in the report.
    probes = [('origin_x',0,1e-5), ('heading',2,1e-7), ('curvature',3,1e-9),
              ('junction_cut_'+cp,4 if cp=='start' else 5,1e-5 if cp=='start' else -1e-5)]
    if parent.knot_count: probes.append(('shared_knot',6,1e-5))
    for name, index, step in probes:
        v = parent.initial.copy(); v[index] += step
        try:
            result = parent.evaluate(v)
            shifts = []
            for cid, end, port in port_uses:
                forward = port.contact == ('end' if end==0 else 'start')
                a = parent.frame(initial,port,forward); b = parent.frame(result,port,forward)
                shifts.append(dict(connector=cid, endpoint=end, parent=rid,
                    position_change_m=float(math.hypot(a['center']['x']-b['center']['x'],a['center']['y']-b['center']['y'])),
                    heading_change_rad=float(b['center']['heading']-a['center']['heading']),
                    curvature_change_per_m=float(b['center']['curvature']-a['center']['curvature'])))
            assert model.nvar==result['model'].nvar
            differences={key:float(np.max(abs(model.source_xy[key]-result['model'].source_xy[key])))
                         for key in model.source_xy if not np.array_equal(model.source_xy[key],result['model'].source_xy[key])}
            if differences:
                raise ValueError('original world coordinate replay changed: '+str(max(differences.values()))+'m')
            rows.append(dict(probe=name,status='EVALUATED_NOT_FITTED',step=step,
                source_min_slack_m=float(min(result['source_inequality_slack'])),
                source_equalities_max=float(np.max(abs(result['source_equalities']),initial=0.)),
                minimum_shared_span_m=float(min(min(np.diff(np.unique(f.knots))) for f in result['model'].families)),
                source_points_unchanged=True,dependent_port_changes=shifts))
        except ValueError as exc:
            rows.append(dict(probe=name,status='TRIAL_REJECTED',reason=str(exc),step=step))
    unchanged = all(_sha256(Path(path))==sha for path,sha in files.items())
    if not unchanged: raise ValueError('source/component input changed during evaluation')
    return dict(road=rid,status='COMPONENT_EVALUATION_NOT_WHOLE_CANDIDATE',
        variables=parent.nvar,source_coefficients=model.nvar,free_shared_knots=parent.knot_count,
        source_role_overrides_already_bound=len(roles['resolved']),new_source_role_authorizations=0,
        original_min_slack_m=float(min(initial['source_inequality_slack'])),
        probes=rows,input_hashes_verified=len(files),geometry_solver_ran=False,export_allowed=False)


def run(preparation, component_input, output):
    preparation, component_input, output = map(lambda p:Path(p).resolve(), (preparation, component_input, output))
    output.mkdir(parents=True, exist_ok=False)
    # Persist scope/budget BEFORE doing the probes; none is a fitting trial.
    contract = dict(status='EVALUATOR_IMPLEMENTATION_CHECK_NOT_STRUCTURE_SEARCH',
        numerical_optimization_budget=0,maximum_component_fixtures=2,
        probes_per_component=5,scope='whole source inventory plus existing approved component fixtures',
        stop_conditions=['changed source hash','source/role binding rejected','unsupported fixed-layout branch'],
        missing_full_model=['ordinary roads 11/12/13 shared coefficients','all24 connector shape variables',
                            'whole surface','moving-domain acceptance','nonlinear constrained optimizer'],
        geometry_solver_ran=False,export_allowed=False)
    dump(output/'check-contract.json',contract)
    verify_preparation(preparation)
    config=json.loads((preparation/'run.json').read_text(encoding='utf8'))
    scope=json.loads((preparation/'reconstruction-input.json').read_text(encoding='utf8'))
    domain=json.loads((preparation/'source-domain.json').read_text(encoding='utf8'))
    root=ET.parse(config['input']).getroot()
    source=ProfileSource(config['source_dir'],config['profile'])
    contacts=compile_source_contacts(root,source,scope,domain,chart_mode='exact-line-arc-v1')
    dump(output/'whole-contact-model.json',contacts)
    group=json.loads(component_input.read_text(encoding='utf8'))
    if len(group['parents'])>2 or _sha256(Path(group['artifact']))!=group['sha256']:
        raise ValueError('component fixture budget or saved artifact binding failed')
    components=[]
    for row in group['parents']:
        try: components.append(component_check(group,row))
        except ValueError as exc:
            components.append(dict(road=row['road'],status='COMPONENT_REJECTED',reason=str(exc)))
    unchanged=all(_sha256(Path(path))==sha for path,sha in config['input_files_sha256'].items())
    if not unchanged: raise ValueError('whole source input changed during evaluation')
    report=dict(status='RESEARCH_EVALUATOR_ONLY_NOT_MAP',contract=contract,
        ordinary_roads=scope['mutable_roads'],connectors=scope['connectors'],
        original_features=len(contacts['full_source_support']),source_contact_issues=contacts['issues'],
        source_role_conflicts=contacts['role_conflicts'],components=components,
        source_inputs_unchanged=unchanged,input_hashes_verified=len(config['input_files_sha256']),
        numerical_optimization_ran=False,xodr_generated=False,production_accepted=False)
    dump(output/'report.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('components','source_role_conflicts','contract')},ensure_ascii=False))
    print([(r['road'],r['status'],r.get('reason'),[(p['probe'],p['status']) for p in r.get('probes',[])]) for r in components])
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('preparation');p.add_argument('component_input');p.add_argument('output')
    a=p.parse_args();run(a.preparation,a.component_input,a.output)
    raise SystemExit(2)
