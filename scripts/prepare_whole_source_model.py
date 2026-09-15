"""Register all original parent boundaries/ports before any numerical fit.

An explicit new user decision is combined with exact prior approvals. Full
originals and all movement dependencies remain in scope. This script does not
fit, compile, or write XODR; incomplete models/negative slack remain visible.
"""
import argparse
import copy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import yaml
from lxml import etree as ET
from mapforge.ops.source_roles import compose_reviewed_source_roles, replay_source_role_packet
from mapforge.ops.source_contacts import compile_source_contacts
from mapforge.adapters.shp.profile_source import ProfileSource
from scripts.prepare_joint_reconstruction import verify_preparation
from scripts.research_code_revision import bind_current_code
from scripts.prepare_whole_source_roles import read, dump
from scripts.fit_source_boundary_block import unchanged
from scripts.gen_all import _sha256
from spikes.whole_source_parent import OriginalParentBoundaryBlock, original_greville_seed
from spikes.arc_source_boundary import ArcSourceBoundaryBlock
from mapforge.ops.coupled_source_junction import MovableSourceParentState, WholeSourcePortState, WholeSourceGeometryState


def prepare_roles(preparation, review, decision_path):
    config = read(preparation / 'run.json')
    hashes, changes = bind_current_code(read(review / 'report.json')['input_files_sha256'])
    # The new decision is an explicit input, never a compiler-generated grant.
    hashes[str(decision_path)] = _sha256(decision_path)
    verify_preparation(preparation)
    scope = read(preparation / 'reconstruction-input.json')
    domain = read(preparation / 'source-domain.json')
    root = ET.parse(config['input']).getroot()
    source = ProfileSource(config['source_dir'], config['profile'])
    contacts = compile_source_contacts(root, source, scope, domain, chart_mode='exact-line-arc-v1')
    old = read(review / 'source-roles.json')
    replay_source_role_packet(scope, domain, contacts, old)
    decision = yaml.safe_load(decision_path.read_text(encoding='utf8'))
    bundles = old['decision']['review_bundles'] + [dict(scope=scope, domain=domain, contacts=contacts, decision=decision)]
    roles = compose_reviewed_source_roles(scope, domain, contacts, bundles)
    replay_source_role_packet(scope, domain, contacts, roles)
    if not roles['role_binding_complete']:
        raise ValueError('unresolved original roles; no whole-source model permitted')
    return root, scope, domain, contacts, roles, config, hashes, changes


def raw_models(root, scope, domain, contacts, roles, config):
    profile = yaml.safe_load(Path(config['profile']).read_text(encoding='utf8'))
    node_fields = {spec['file']: {r: spec['fields'][r+'_node'] for r in ('start', 'end')}
        for name, spec in profile['layers'].items() if name in ('lane', 'lane_merge')
        and all(r+'_node' in spec.get('fields', {}) for r in ('start', 'end'))}
    models = {}; rows = []
    for rid in scope['mutable_roads']:
        print('REGISTER ORIGINAL PARENT', rid, flush=True)
        try:
            original = OriginalParentBoundaryBlock(root, scope, domain, contacts, roles,
                road=rid, degree=3, speed_node_fields=node_fields)
            original.min_span = 5.5  # Existing declared cubic-width floor; not a reference-piece floor.
            # Deterministic coordinate registration, not source_end_axis/Powell.
            model = ArcSourceBoundaryBlock(original, curvature=original.original_parent_chart['curvature'])
            models[rid] = (original, model)
            rows.append(dict(road=rid, status='SOURCE_BASIS_REGISTERED_NOT_FITTED',
                source_ids=original.source_ids, families=len(model.families), coefficients=model.nvar,
                minimum_width_span_m=float(min(min(np.diff(np.unique(f.knots))) for f in model.families)),
                source_inequalities=len(model.lower), source_equalities=len(model.E),
                coordinate_policy=original.source_coordinate_policy, geometry_accepted=False))
        except ValueError as exc:
            rows.append(dict(road=rid, status='SOURCE_BASIS_REJECTED', reason=str(exc), geometry_accepted=False))
    return models, rows


def register_turns(whole, snapshot, source, scope, domain):
    from mapforge.ops.movable_source_support import MovableMovementSupport
    from mapforge.ops.joint_connector_fit import fit_joint, basis_for
    from mapforge.ops.long_connector_chain import chain
    from spikes.connector_cross_section import edge_jet
    providers = {}; kernels = {}; rows = []
    for cid in whole.connectors:
        print('REGISTER LONG MOVEMENT', cid, flush=True)
        try:
            support = MovableMovementSupport(whole.graph, cid, whole.parents, source, scope, domain)
            data = support.evaluate(snapshot['parents'])
            a,b = snapshot['connector_frames'][cid]
            via = data['via']['center']
            length = max(30., float(np.linalg.norm(np.diff(via,axis=0),axis=1).sum() +
                np.linalg.norm(np.array(a['pose'][:2])-via[0]) + np.linalg.norm(np.array(b['pose'][:2])-via[-1])))
            if length > 312.:
                raise ValueError('declared five-long-segment initializer exceeds its domain')
            delta = np.arctan2(np.sin(b['pose'][2]-a['pose'][2]), np.cos(b['pose'][2]-a['pose'][2]))
            q = np.r_[6., np.full(3,(length-12.)/3), 6., np.full(2,20*delta/length)]
            cls = chain(q,a,b,(True,True),3);kn,B,_ = basis_for(cls)
            sites = np.array([np.mean(B.t[i+1:i+4]) for i in range(len(B.c))])
            coefficients = []
            for side in ('left','right'):
                start = edge_jet(a,a['edges'][side],cls[0].KappaStart,cls[0].dk)[0]
                end = edge_jet(b,b['edges'][side],cls[-1].KappaEnd,cls[-1].dk)[0]
                coefficients.extend(start+(end-start)*sites)
            kernel = fit_joint(whole.graph.roads[cid], a,b,data['raw'],data['via'],q,
                initial_coefficients=np.array(coefficients), core_count=3, contact_caps=(True,True),
                fair_world=True, _problem_only=True, source_tails=data['tails'])
            providers[cid] = support; kernels[cid] = kernel
            rows.append(dict(road=cid, status='LONG_GEOMETRY_KERNEL_REGISTERED_NOT_FITTED',
                independent_variables=len(kernel['initial']), reference_primitives=5,
                minimum_reference_span_m=float(min(q[:5])), source_ids=list(support.source_ids),
                separate_movement_ids=support.separate_movement_ids, full_via_preserved=True))
        except (ValueError, ArithmeticError) as exc:
            rows.append(dict(road=cid, status='LONG_GEOMETRY_KERNEL_REJECTED', reason=str(exc)))
    return kernels, providers, rows


def run(preparation, review, decision, output):
    preparation, review, decision, output = map(lambda p: Path(p).resolve(), (preparation, review, decision, output))
    output.mkdir(parents=True, exist_ok=False)
    dump(output / 'registration-contract.json', dict(status='FULL_MODEL_CONSTRUCTION_ONLY',
        numerical_fitting_budget=0, optimization_ran=False, xodr_writer_enabled=False,
        scope=dict(parents=['10','11','12','13','30'], movements=list(map(str, range(100,124)))),
        reference_policy='one-Line-or-Arc-coordinate-frame-per-parent-no-pointwise-pieces',
        minimum_independent_width_span_m=5.5,
        turn_registration=dict(reference_cores=3, end_caps=2, minimum_reference_span_m=6.,
            initialization='deterministic-via-length-and-end-heading-no-root-solve-or-local-fit',
            full_joint_evaluations=1),
        full_parent_incidence_probes=1,
        stop_conditions=['changed originals or role scope', 'unsupported original topology/layout', 'missing full model'],
        structural_optimization_candidate_registered=False))
    root, scope, domain, contacts, roles, config, hashes, changes = prepare_roles(preparation, review, decision)
    code = [p for name in ('mapforge','spikes','scripts') for p in (ROOT/name).rglob('*.py')]
    hashes.update({str(p): _sha256(p) for p in code})
    dump(output/'source-roles.json', roles)
    models, rows = raw_models(root, scope, domain, contacts, roles, config)
    parents = {}; port_rows = []; parent_vectors = {}
    for rid, (original, model) in models.items():
        print('REGISTER SOURCE PORTS', rid, flush=True)
        try:
            x = original_greville_seed(model)
            parent = MovableSourceParentState(original, model, x, source_root=root)
            state = parent.evaluate(parent.initial)
            parents[rid] = parent; parent_vectors[rid] = parent.initial.tolist()
            port_rows.append(dict(road=rid, status='SOURCE_PORTS_REGISTERED_NOT_FEASIBLE',
                variables=parent.nvar, coefficient_count=model.nvar, free_knots=parent.knot_count,
                minimum_source_slack_m=float(min(state['source_inequality_slack'])),
                maximum_scaled_source_equality=float(np.max(abs(state['source_equalities']), initial=0.)),
                full_original_domains=state['source_domain']['original_chain_domains'],
                junction_tails=state['source_domain']['tails'], geometry_accepted=False))
        except ValueError as exc:
            port_rows.append(dict(road=rid, status='SOURCE_PORT_REGISTRATION_REJECTED', reason=str(exc)))
    whole = None; whole_summary = dict(status='INCOMPLETE_SOURCE_MODEL')
    turn_rows = []; geometry_summary = dict(status='INCOMPLETE_GEOMETRY_MODEL')
    if set(parents) == set(scope['mutable_roads']):
        whole = WholeSourcePortState(root, config['whole_junction']['xodr_junction_id'], parents, scope['connectors'])
        state = whole.evaluate(whole.initial)
        whole_summary = dict(status=state['status'], parent_variables=len(whole.initial),
            movements=sorted(state['connector_frames'], key=int),
            frames={rid:list(frames) for rid,frames in state['connector_frames'].items()},
            source_states_share_one_snapshot=True, export_allowed=False)
        # One finite whole-state incidence test, not a new fit or seed search.
        changed = whole.initial.copy()
        changed[whole.slices['11'].start+1] += 1e-5
        moved = whole.evaluate(changed)
        affected = sorted(cid for cid,ps in whole.graph.connections.items() if any(p.road=='11' for p in ps))
        observed=[]
        for cid,fs in state['connector_frames'].items():
            if any(abs(f['center']['y']-g['center']['y'])>1e-12 or abs(f['center']['x']-g['center']['x'])>1e-12
                   for f,g in zip(fs,moved['connector_frames'][cid])): observed.append(cid)
        if sorted(observed)!=affected:raise ValueError('full-state parent change missed or altered an unrelated turn')
        whole_summary['finite_incidence_probe']=dict(parent='11', origin_y_step_m=1e-5,
            expected_movements=affected, changed_movements=sorted(observed), numerical_optimization_ran=False)
        source = ProfileSource(config['source_dir'], config['profile'])
        kernels, providers, turn_rows = register_turns(whole,state,source,scope,domain)
        if set(kernels) == set(whole.connectors):
            geometry = WholeSourceGeometryState(whole,kernels,providers)
            dump(output/'whole-coordinate-state.json', dict(initial=geometry.initial.tolist(),
                parent_size=geometry.parent_size, turn_slices={k:[v.start,v.stop] for k,v in geometry.turn_slices.items()}))
            print('EVALUATE ONE COMPLETE SOURCE/SHAPE STATE', len(geometry.initial), flush=True)
            try:
                full = geometry.evaluate(geometry.initial)
                geometry_summary = dict(status=full['status'], variables=len(geometry.initial),
                    source_variables=geometry.parent_size, turn_variables=len(geometry.initial)-geometry.parent_size,
                    equality_count=len(full['equalities']), inequality_count=len(full['inequalities']),
                    maximum_scaled_equality=float(np.max(abs(full['equalities']),initial=0.)),
                    minimum_inequality_slack=float(min(full['inequalities'])),
                    simultaneous_parent_connector_variables=True, dynamic_source_partition=True,
                    final_domain_dynamics_and_surface_constraints_complete=False,
                    separate_movement_paths_validated=False,
                    turns=[dict(road=cid, **{k:v for k,v in row.items() if k not in ('result','frames','source_support','equalities')})
                           for cid,row in full['turns'].items()], optimization_ran=False, export_allowed=False)
            except (ValueError, ArithmeticError) as exc:
                geometry_summary = dict(status='WHOLE_STATE_EVALUATION_REJECTED', reason=str(exc),
                    variables=len(geometry.initial), optimization_ran=False, export_allowed=False)
    dump(output/'parent-coordinate-state.json', parent_vectors)
    report = dict(status='SOURCE_BASES_ONLY_NOT_CONNECTED_MAP', parents=rows,
        source_roles_complete=True, roles_resolved=len(roles['resolved']),
        new_user_decision_file=str(decision), explicit_decisions_added=len(yaml.safe_load(decision.read_text(encoding='utf8'))['decisions']),
        port_registration=port_rows, whole_source_state=whole_summary,
        turn_registration=turn_rows, whole_geometry_state=geometry_summary,
        unresolved_roles=roles['unresolved'], original_features=len(contacts['full_source_support']),
        original_endpoints=len(contacts['source_endpoint_inventory']),
        input_files_sha256=hashes, input_code_revision_changes=changes,
        optimization_ran=False, xodr_generated=False, production_accepted=False)
    unchanged(hashes); dump(output/'report.json', report)
    print([(r['road'],r['status'],r.get('reason')) for r in rows], flush=True)
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('preparation', 'review', 'decision', 'output'):
        p.add_argument(key)
    a = p.parse_args(); run(a.preparation, a.review, a.decision, a.output)
    raise SystemExit(2)
