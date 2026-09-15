"""Prepare source-backed shared parents and the complete original turn closure.

This is an INPUT state, not connected geometry and not a new map candidate.
No fixed-parent turn fit is run; all parent coefficients are retained as joint
variables for the subsequent simultaneous solve.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from scripts.build_ordinary_source_road import load_road
from scripts.solve_source_parent_junction import isolated_parent
from scripts.rebuild_north_source_ports import install_same_identity_parent
from scripts.build_source_geometry_candidate import dump
from scripts.gen_all import _sha256,shp_source
from scripts.research_code_revision import bind_current_code
from scripts.fit_source_boundary_block import unchanged
from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_cubic_export import cubic_model,constrain_written_endpoints,compile_road
from mapforge.ops.coupled_source_junction import SourceParentState
from mapforge.ops.port_dependencies import PortDependencies
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from spikes.measured_connector_caps import composite_sources,raw_curves,distances
from spikes.connector_cross_section import endpoint_frame
from spikes.trial_xml import write_trial


def run(baseline,north_component,north_seed,other_component,output):
    baseline,north_component,north_seed,other_component,output=[Path(p).resolve() for p in
        (baseline,north_component,north_seed,other_component,output)]
    output.mkdir(parents=True,exist_ok=False)
    base=json.loads((baseline/'report.json').read_text(encoding='utf8'));path=Path(base['artifact'])
    if _sha256(path)!=base['sha256']:raise ValueError('baseline actual map changed')
    tree=ET.parse(path);root=tree.getroot();old=PortDependencies(root);old.validate_junction_table()
    hashes,changes=bind_current_code(base['source_hashes']);parents={};rows=[]
    for component,seed in ((north_component,north_seed),(other_component,other_component/'shared-state.json')):
        state=json.loads((component/'shared-state.json').read_text(encoding='utf8'));rid=state['road']
        if rid in parents or state.get('flat_port') or state.get('source_export_domain') is not None:
            raise ValueError('distinct nonflat common-source parent states required')
        raw,roles,files,source_root=load_road(state['source_directory'],state['decision_file'],rid,state.get('previous_source_directory'))
        model=constrain_written_endpoints(cubic_model(raw,minimum_span=state['minimum_width_span'],end_axis=state['end_axis']))
        if digest(model.describe())!=state['model_sha']:raise ValueError('parent model changed; explicit rebuild required')
        x=np.array(json.loads(seed.read_text(encoding='utf8'))['coefficients'],float)
        written,compiled,_=compile_road(model,x,source_root.find(f"road[@id='{rid}']"),source_direction=state.get('source_direction'))
        install_same_identity_parent(root,isolated_parent(written))
        shared=SourceParentState(model,x,compiled);parents[rid]=shared
        hashes.update(files);hashes.update({str(p):_sha256(p) for p in (component/'shared-state.json',seed)})
        rows.append(dict(road=rid,component=str(component),seed=str(seed),model_sha=state['model_sha'],
            coefficients=x.tolist(),independent_joint_variables=shared.nvar,source_inequalities=len(model.lower),
            compiled=compiled,source_role_overrides=len(roles['resolved']),new_source_role_authorizations=0))
    graph=PortDependencies(root);graph.validate_junction_table()
    if graph.connections!=old.connections:raise ValueError('traffic topology changed')
    source=shp_source();lat,lon=_origin(root);project=lambda pts:_project(pts,lat,lon)
    turns=[];conflicts=[]
    for rid,ports in graph.connections.items():
        if not any(p.road in parents for p in ports):continue
        curve=graph.roads[rid];raw,support=composite_sources(root,curve,source,project)
        source_ids=[support['source_lane_ids'][0],support['source_lane_ids'][-1]]
        endpoints=[]
        for index,(port,sid) in enumerate(zip(ports,source_ids)):
            forward=port.contact==('end' if index==0 else 'start')
            parent=parents.get(port.road)
            actual=endpoint_frame(graph.roads[port.road],port.lane,port.contact,forward)
            if parent is not None:
                expected=parent.frame(port,parent.jets(port,np.zeros(parent.nvar)),forward)
                for field in ('left','right','center'):
                    a=actual['center'] if field=='center' else actual['edges'][field]
                    b=expected['center'] if field=='center' else expected['edges'][field]
                    if not np.allclose([a[k] for k in ('x','y','heading','curvature')],
                                       [b[k] for k in ('x','y','heading','curvature')],atol=1e-8,rtol=0):
                        raise ValueError('shared source contact map differs from actual written endpoint')
            tails=raw_curves(source,sid,project);errors={}
            for side in ('left','right'):
                edge=actual['edges'][side]
                errors[side]=float(distances([[edge['x'],edge['y']]],tails[side])[0])
                if parent is None and errors[side]>.35+1e-7:
                    conflicts.append(dict(road=rid,parent=port.road,lane=port.lane,contact=port.contact,
                        field=side,source_lane_id=sid,distance_m=errors[side],budget_m=.35))
            endpoints.append(dict(parent=port.road,lane=port.lane,contact=port.contact,source_lane_id=sid,
                shared_variable_parent=parent is not None,world_frame=actual,source_boundary_distances_m=errors))
        turns.append(dict(road=rid,source_support=support,endpoints=endpoints,geometry_rebuilt=False))
    input_path=output/'unconnected-source-parent-input.xodr';write_trial(tree,input_path)
    hashes.update({str(p):_sha256(p) for p in (path,baseline/'report.json',Path(__file__))})
    unchanged(hashes)
    report=dict(status='SHARED_SOURCE_GROUP_INPUT_NOT_CONNECTED' if not conflicts else 'FIXED_SOURCE_PORT_CONFLICT',
        artifact=str(input_path),sha256=_sha256(input_path),input=str(path),input_sha256=base['sha256'],
        parents=rows,affected_roads=[r['road'] for r in turns],turns=turns,fixed_source_port_conflicts=conflicts,
        source_hashes=hashes,input_code_revision_changes=changes,geometry_solver_ran=False,
        simultaneous_parent_connector_optimization_ran=False,source_changed=False,source_roles_expanded=False,
        production_accepted=False,next_required='fixed long model initialization then ALL parents/turns in one source-constrained solve')
    dump(output/'report.json',report)
    print(report['status'],report['affected_roads'],'FIXED CONFLICTS',conflicts,flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('baseline','north_component','north_seed','other_component','output'):p.add_argument(name)
    a=p.parse_args();run(a.baseline,a.north_component,a.north_seed,a.other_component,a.output)
    raise SystemExit(2)
