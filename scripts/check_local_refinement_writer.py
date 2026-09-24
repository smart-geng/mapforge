"""R1 one fixed compiler probe; no target solve or live Web mutation."""
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle
from mapforge.repair_web.local_refinement import prepare_local_refinement, LO, HI, EDGE
from mapforge.repair_web.model import atomic, digest, json_bytes
from mapforge.validate.line_boundary_shape import written_boundary_shape
from scripts.check_outer_event_control import inputs, POINT
from scripts.check_outer_event_shape import written_shape

DESIGN = ROOT/'out/node4-long-refinement-design-20260917'
DEST = ROOT/'out/node4-local-refinement-writer-r1-20260917'
LEGACY = ROOT/'out/node4-outer-event-shape-20260915-r6/dynamics.json'
# Coefficient probes, NOT a displacement towards the failed user/source target.
PROBE = [1e-6, -1e-6]


def main():
    if DEST.exists():
        raise ValueError('R1 evidence directory already exists; inspect, never overwrite')
    binding = json.loads((DESIGN/'binding.json').read_bytes())
    design_status = json.loads((DESIGN/'status.json').read_bytes())
    for name, sha in design_status['artifact_sha256'].items():
        binding[str(DESIGN/name)] = sha
    for path in (DESIGN/'status.json', Path(__file__).resolve(),
                 ROOT/'mapforge/repair_web/local_refinement.py', ROOT/'tests/test_local_refinement.py',
                 ROOT/'OpenDRIVE_1.5M.xsd', ROOT/'esmini/bin/esminiRMLib.dll',
                 ROOT/'scripts/validate_repair_export.py', ROOT/'scripts/esmini_rm_check.py', LEGACY):
        binding[str(path)] = digest(path.read_bytes())
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes()) != sha:
                raise ValueError('Frozen source/code/evidence drift: '+path)
    verify()
    event, control = inputs()
    request = EditScopeRequest(control.reference_sha256, '11', (EDGE,), (LO, HI), (ScopeHandle(POINT),))
    model = prepare_local_refinement(event, control.reference, request)
    kwargs = dict(contract_sha256=model.contract_sha256, purpose='R1_COMPILER_PROBE')
    noop_data, noop_report = model.compile_probe([0., 0.], **kwargs)
    if noop_data is not control.reference:
        raise ValueError('No-op must replay exact original bytes')
    # Exactly one predefined micro-probe, never a nonzero source/drag target.
    data, report = model.compile_probe(PROBE, **kwargs)
    scope = asdict(event.scope)
    comparison = dict(reference=written_boundary_shape(control.reference, scope),
                      probe=written_boundary_shape(data, scope),
                      reference_reversal=written_shape(control.reference, event),
                      probe_reversal=written_shape(data, event))
    failures = []
    for edge in range(5):
        for key, eps in (('max_abs_curvature', 1e-9), ('max_abs_curvature_rate', 1e-9),
                         ('curvature_total_variation', 1e-8)):
            cap = comparison['reference']['by_edge'][str(edge)][key]
            observed = comparison['probe']['by_edge'][str(edge)][key]
            if observed > cap+eps:
                failures.append(dict(edge=edge, metric=key, observed=observed, limit=cap,
                                     origin='existing_research_nonregression'))
        cap = comparison['reference_reversal']['by_edge'][str(edge)]
        observed = comparison['probe_reversal']['by_edge'][str(edge)]
        if observed > cap+1e-7:
            failures.append(dict(edge=edge, metric='source_direction_reversal_m', observed=observed,
                                 limit=cap, origin='existing_research_nonregression'))
    comparison.update(research_nonregression_failures=failures, user_target_checked=False,
                      map_accepted=False, status='PROBE_OBSERVATIONS_ONLY')
    verify()
    DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json', json_bytes(binding))
    atomic(DEST/'contract.json', json_bytes(model.contract))
    atomic(DEST/'basis.json', model._powers)
    atomic(DEST/'noop.json', json_bytes(noop_report))
    atomic(DEST/'probe-only.xodr', data)
    atomic(DEST/'probe-readback.json', json_bytes(report))
    atomic(DEST/'shape-observations.json', json_bytes(comparison))
    atomic(DEST/'consumer-request.json', json_bytes(dict(xodr_sha256=digest(data),
        guard=dict(changed_roads=['11']), purpose='R1_COMPILER_PROBE', map_accepted=False)))
    consumer_path = DEST/'consumer.json'
    proc = subprocess.run([sys.executable, str(ROOT/'scripts/validate_repair_export.py'),
                           str(DEST/'probe-only.xodr'), str(DEST/'consumer-request.json'), str(consumer_path)],
                          cwd=DEST, capture_output=True, timeout=45)
    atomic(DEST/'consumer.log', proc.stdout+proc.stderr)
    consumer = json.loads(consumer_path.read_bytes()) if proc.returncode == 0 and consumer_path.exists() else {}
    legacy = json.loads(LEGACY.read_bytes())
    good = consumer.get('xsd') == consumer.get('esmini') == 'PASS'
    summary = dict(status='R1_WRITER_VERIFIED_NOT_REPAIR' if good else 'R1_CONSUMER_CHECK_FAILED',
                   xodr_sha256=digest(data), artifact_kind='compiler_probe_only_not_repaired_map',
                   contract_sha256=model.contract_sha256, input_bindings=len(binding), input_drift=0,
                   nonzero_probe_count=1, probe_coefficients=PROBE, real_target_candidates=0, optimizer_calls=0,
                   new_xodr_diagnostic_files=1, new_repaired_maps=0, map_accepted=False, web_changed=False,
                   source_event_max_m=report['source_readback']['source_event_max_m'],
                   frozen_curve_change_m=report['frozen_curve_change_m'],
                   whole_polynomial_error_m=report['whole_polynomial_error_m'],
                   structure_change=report['structure_change'], xsd=consumer.get('xsd', 'NOT_RUN'),
                   esmini=consumer.get('esmini', 'NOT_RUN'), checked_positions=consumer.get('checked_positions'),
                   max_position_error_m=consumer.get('max_position_error_m'),
                   research_nonregression_failures=failures,
                   legacy_dynamics=dict(path=str(LEGACY), sha256=digest(LEGACY.read_bytes()),
                                        status=legacy.get('status'), counts=legacy.get('counts'), rerun=False),
                   operating_dynamics='NOT_EVALUATED', absolute_crs='NOT_VALIDATED',
                   artifact_sha256={p.name: digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()})
    verify()
    atomic(DEST/'status.json', json_bytes(summary))
    print(json.dumps(summary, ensure_ascii=False))
    if not good:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
