"""Read-only seven-road interface/long-layout verification; no optimization."""
import json
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.coupled_event_contract import CoupledEventContract
from mapforge.repair_web.coupled_event_state import CoupledEventState
from mapforge.repair_web.model import atomic, digest, json_bytes
from scripts.bind_split_event_sources import inputs

PRIOR = ROOT/'out/node4-event-shape-admission-e2-20260917'
DEST = ROOT/'out/node4-coupled-event-contract-20260917'
SCOPE = ROOT/'profiles/repair/node4-west-event-and-six-turns-v1.yaml'
ROLES = ROOT/'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml'
BOUND = ROOT/'out/node4-event-source-binding-20260917/source-bindings.json'


def load_contract():
    data, packet, domain, *_ = inputs()
    return CoupledEventContract(data, packet, domain, json.loads(BOUND.read_bytes()),
        yaml.safe_load(SCOPE.read_text(encoding='utf-8')), yaml.safe_load(ROLES.read_text(encoding='utf-8')))


def response_check(contract):
    """One deterministic linear-interface probe; no geometric target or fit."""
    model = CoupledEventState(contract); p = contract.parent
    initial = model.diagnostic.copy(); baseline = model.evaluate(initial)
    requested = np.zeros(10); requested[6] = 1e-4  # edge3 first derivative only
    delta = np.linalg.lstsq(contract.jet_matrix@p.Z, requested, rcond=None)[0]
    probe = initial.copy(); probe[model.parent_slice] += delta
    after = model.evaluate(probe)
    results = []
    for cid in model.turns:
        oldframes, newframes = baseline['frames'][cid], after['frames'][cid]
        keys = ('x', 'y', 'heading', 'curvature')
        old = baseline['turns'][cid]; new = after['turns'][cid]
        frame_delta = max(abs(newframes[0]['edges'][side][key]-oldframes[0]['edges'][side][key])
                          for side in ('left', 'right') for key in keys)
        control_delta = max(np.max(abs(new['controls'][side]-old['controls'][side])) for side in ('left', 'right'))
        results.append(dict(connector=cid, incoming_frame_changed=bool(frame_delta > 1e-9),
            transverse_controls_changed=bool(control_delta > 1e-9), maximum_control_delta=float(control_delta),
            outgoing_frame_unchanged=newframes[1] == oldframes[1],
            primitive_count=len(new['refs']), transverse_span_count=len(new['stations'])-1,
            minimum_span_m=float(min(np.diff(new['stations']))),
            baseline_reference_closure=old['reference_closure'].tolist(),
            baseline_maximum_world_join_delta=np.max(np.abs([r['delta'] for r in old['joins']]), axis=0).tolist(),
            baseline_minimum_width_m=old['minimum_width_m']))
    coordinate_delta = contract.jet_matrix@(after['parent_coefficients']-baseline['parent_coefficients'])
    return dict(status='INTERFACE_PROBE_NOT_CANDIDATE', state_variables=model.nvar,
        parent_free_variables=p.Z.shape[1], turn_variables={c:b.nvar for c,b in model.turns.items()},
        requested_mouth_jet_delta=requested.tolist(), actual_mouth_jet_delta=coordinate_delta.tolist(),
        requested_actual_error=float(max(abs(coordinate_delta-requested))), results=results,
        coordinate_probe_is_map=False, source_fidelity_evaluated=False, actual_xml_written=False,
        shape_admitted=False, optimizer_calls=0)


def main():
    if DEST.exists(): raise ValueError('Do not overwrite interface evidence')
    bindings = json.loads((PRIOR/'binding.json').read_bytes())
    for path in (PRIOR/'admission.json', PRIOR/'status.json', SCOPE, ROLES, BOUND, Path(__file__).resolve(),
            ROOT/'mapforge/repair_web/coupled_event_contract.py', ROOT/'mapforge/repair_web/coupled_event_state.py',
            ROOT/'tests/test_coupled_event_contract.py', ROOT/'mapforge/ops/coupled_source_junction.py',
            ROOT/'mapforge/ops/long_connector_chain.py', ROOT/'mapforge/ops/endpoint_jet_coordinates.py',
            ROOT/'mapforge/ops/source_connector_ribbon.py', ROOT/'spikes/connector_cross_section.py'):
        value = digest(path.read_bytes())
        if str(path) in bindings and bindings[str(path)] != value: raise ValueError('Prior binding drift')
        bindings[str(path)] = value
    def verify():
        for path, sha in bindings.items():
            if digest(Path(path).read_bytes()) != sha: raise ValueError('Input drift: '+path)
    verify(); contract = load_contract(); report = contract.describe(); probe = response_check(contract)
    report['joint_coordinate_kernel_registered'] = True
    report['joint_trial_kernel_admitted'] = False
    report['next_required'] = 'full source/shape/surface and semantic written-layout admission before one registered trial'
    verify(); DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json', json_bytes(bindings)); atomic(DEST/'contract.json', json_bytes(report))
    atomic(DEST/'interface-probe.json', json_bytes(probe)); verify()
    atomic(DEST/'status.json', json_bytes(dict(status=report['status'], input_bindings=len(bindings),
        input_drift=0, solver_calls=0, new_xodr=False, map_accepted=False, web_changed=False,
        artifact_sha256={p.name:digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()})))
    print(json.dumps(dict(status=report['status'], variables=probe['state_variables'],
        affected=[r['connector'] for r in probe['results'] if r['incoming_frame_changed']],
        port_positions_within_budget=sum(r['position_within_budget'] for r in contract.port_sources),
        predicted_short_semantic_records=len(report['predicted_written_layout']['short_semantic_records']),
        input_bindings=len(bindings))))


if __name__ == '__main__': main()
