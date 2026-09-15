"""Seal an actual shared-parent trial and independent readback as REJECTED.

There is intentionally no production-success branch. Historical solver code
changes are recorded by the readback adapter, never silently relabelled.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import _sha256
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged
from scripts.seal_parent_rebuild_diagnostic import require_same_artifact, require_inventory, interface_maxima
from scripts.seal_geometry_review import verify_full_source_budget


def run(directory):
    directory = Path(directory).resolve()
    packet = directory / 'review-packet'
    paths = [directory/'report.json', packet/'report.json', packet/'independent-review/report.json',
             packet/'north-independent/report.json', packet/'shape-review/report.json',
             packet/'shape-review/full-source-turn-budget.json', directory/'esmini-review/esmini-captures.json']
    trial, review, geometry, north, shape, budget, render = [json.loads(p.read_text(encoding='utf8')) for p in paths]
    artifact = Path(trial['artifact'])
    sha = require_same_artifact(artifact, (trial, review, geometry, north, shape, budget, render))
    if not trial['simultaneous_parent_connector_variables'] or not review['simultaneous_parent_connector_variables']:
        raise ValueError('genuine joint trial required')
    if set(trial['affected_roads']) != {r['road'] for r in trial['connectors']}:
        raise ValueError('shared solver omitted an affected turn')
    require_inventory(review, geometry, shape)
    for report, key in ((review, 'source_hashes'), (geometry, 'source_hashes'),
                        (north, 'source_hashes'), (shape, 'hashes')):
        unchanged(report[key])
    failed = verify_full_source_budget(shape, budget, paths[4])
    retained = Path(trial['input'])
    if _sha256(retained) != trial['input_sha256']:
        raise ValueError('retained research artifact changed')
    shots = []
    for row in render['shots']:
        p = (paths[-1].parent / row['file']).resolve()
        if p.parent != paths[-1].parent or not p.is_file():
            raise ValueError('missing or escaped actual simulator capture')
        shots.append(p)
    report = dict(status='REJECTED_NOT_DELIVERY', artifact=str(artifact), sha256=sha,
        previous_research_artifact_retained=str(retained), previous_research_sha256=trial['input_sha256'],
        previous_research_is_delivery=False, candidate_selected=False,
        simultaneous_parent_connector_variables=True, variable_count=trial['variable_count'],
        independent_parent_variables=trial['independent_parent_variables'],
        affected_roads=trial['affected_roads'], construction=trial['final'],
        independent_geometry_failed_roads=geometry['geometry_failed_roads'],
        full_source_turn_failed_roads=failed,
        internal_edge_failed_roads=sorted({r['road'] for r in geometry['all_map_internal_edges']['failures']},key=int),
        internal_edge_maxima=geometry['all_map_internal_edges']['maxima'],
        consumer_interfaces_status=geometry['interfaces']['status'],
        consumer_interfaces_count=len(geometry['interfaces']['rows']),
        consumer_interfaces_failed_count=sum(r['status']!='PASS' for r in geometry['interfaces']['rows']),
        consumer_interfaces_maxima=interface_maxima(geometry['interfaces']),
        north_written_source=north['written_source'],
        north_source_speed_entries_match=north['source_speed_entries_match'],
        north_source_speed_entry_count=north['source_speed_entry_count'],
        east_full_source_aggregate=geometry['east_full_source_aggregate'], xsd_pass=geometry['xsd_pass'],
        code_changes_before_independent_readback=review['code_changes_before_independent_readback'],
        source_priority_stage_ran=False, source_changed=False, source_roles_expanded=False,
        source_speed_dynamics_accepted=False, whole_surface_accepted=False, movement_paths_validated=False,
        MAP_retested=False, production_accepted=False, rendered_images_are_automatic_acceptance=False,
        reason='Joint feasibility is incomplete; actual shape, internal G2 and consumer checks fail.',
        evidence_hashes={str(p):_sha256(p) for p in paths+shots+[artifact,retained,Path(__file__)]})
    target=directory/'FINAL-REVIEW-STATUS.json'
    if target.exists():raise FileExistsError('preserve previous seal')
    unchanged(report['evidence_hashes']);dump(target,report)
    print(report['status'],geometry['geometry_failed_roads'],failed,flush=True)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory')
    run(parser.parse_args().directory)
    raise SystemExit(2)
