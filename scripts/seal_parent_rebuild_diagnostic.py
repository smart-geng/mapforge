"""Bind rejected parent/connector experiments to their actual independent evidence.

This deliberately cannot promote a research map. Dependency coverage is not
joint optimization, nor proof that every incident curve was reconstructed.
"""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump
from scripts.seal_geometry_review import verify_full_source_budget


def require_same_artifact(path, reports):
    path = Path(path).resolve()
    sha = _sha256(path)
    if any(Path(r['artifact']).resolve() != path or r['sha256'] != sha for r in reports):
        raise ValueError('all readbacks must reference the same actual XML')
    return sha


def require_inventory(trial, geometry, shape):
    def ids(rows):
        values = [r['road'] for r in rows]
        if not values or len(values) != len(set(values)):
            raise ValueError('missing or duplicate turn inventory')
        return set(values)
    expected = ids(trial['connectors'])
    if ids(geometry['rows']) != expected or ids(shape['records']) != expected:
        raise ValueError('independent review omitted a declared turn')
    failed = {r['road'] for r in geometry['rows'] if r['geometry_status'] != 'PASS'}
    if failed != set(geometry['geometry_failed_roads']):
        raise ValueError('geometry failure summary disagrees with actual rows')


def interface_maxima(report):
    samples = [sample for row in report['rows'] for sample in row['samples']]
    fields = ('position_m', 'heading_deg', 'curvature_per_m')
    if not samples or any(not math.isfinite(s[k]) or s[k] < 0 for s in samples for k in fields):
        raise ValueError('complete finite consumer measurements required')
    return {k: max(s[k] for s in samples) for k in fields}


def run(directory):
    directory = Path(directory).resolve()
    paths = [directory / p for p in ('report.json', 'independent-review/report.json',
        'north-independent/report.json', 'shape-review/report.json',
        'shape-review/full-source-turn-budget.json', 'esmini-review/esmini-captures.json')]
    trial, geometry, north, shape, budget, render = [json.loads(p.read_text(encoding='utf8')) for p in paths]
    path = Path(trial['artifact'])
    sha = require_same_artifact(path, (trial, geometry, north, shape, budget, render))
    for report, key in ((trial, 'source_hashes'), (geometry, 'source_hashes'),
                        (north, 'source_hashes'), (shape, 'hashes')):
        unchanged(report[key])
    require_inventory(trial, geometry, shape)
    turn_failed = verify_full_source_budget(shape, budget, paths[3])
    retained = Path(trial['input'])
    if _sha256(retained) != trial['input_sha256']:
        raise ValueError('previous research revision changed')
    shots = []
    for row in render['shots']:
        shot = (paths[-1].parent / row['file']).resolve()
        if shot.parent != paths[-1].parent:
            raise ValueError('capture must belong to the declared review folder')
        shots.append(shot)
    result = dict(status='REJECTED_NOT_DELIVERY', artifact=str(path), sha256=sha,
        previous_research_artifact_retained=str(retained), previous_research_sha256=trial['input_sha256'],
        previous_research_is_delivery=False, new_parent_selected=False,
        dependency_attempted_roads=trial['affected_roads'],
        construction_rejected_roads=sorted(trial['rejected_roads'], key=int),
        independent_geometry_failed_roads=geometry['geometry_failed_roads'],
        full_source_turn_failed_roads=turn_failed,
        internal_edge_failed_roads=sorted({r['road'] for r in geometry['all_map_internal_edges']['failures']}, key=int),
        internal_edge_maxima=geometry['all_map_internal_edges']['maxima'],
        consumer_interfaces_status=geometry['interfaces']['status'],
        consumer_interfaces_maxima=interface_maxima(geometry['interfaces']),
        consumer_interfaces_count=len(geometry['interfaces']['rows']),
        consumer_interfaces_failed_count=sum(r['status'] != 'PASS' for r in geometry['interfaces']['rows']),
        east_full_source_aggregate=geometry['east_full_source_aggregate'],
        north_ordinary_written_source_status=north['written_source']['status'],
        north_ordinary_source_speed_entries_match=north['source_speed_entries_match'],
        north_component_is_whole_map_acceptance=False, xsd_pass=geometry['xsd_pass'],
        dependency_attempts_are_joint_optimization=False,
        rendered_images_are_automatic_acceptance=False, source_changed=False,
        source_roles_expanded=False, source_speed_dynamics_accepted=False,
        whole_surface_accepted=False, movement_paths_validated=False,
        MAP_retested=False, production_accepted=False,
        reason='Frozen-parent experiment failed independent source, shape and consumer checks; do not promote.',
        evidence_hashes={str(p): _sha256(p) for p in paths + shots + [path, retained, Path(__file__)]})
    target = directory / 'FINAL-REVIEW-STATUS.json'
    if target.exists():
        raise FileExistsError('preserve previous diagnostic seal')
    unchanged(result['evidence_hashes'])
    dump(target, result)
    print(result['status'], result['independent_geometry_failed_roads'], flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    run(parser.parse_args().directory)
    raise SystemExit(2)
