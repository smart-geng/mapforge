"""Candidate-only MAP lane-center fitting with exact same-leg reflection."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

from lxml import etree
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from spikes.road_boundary_family import solve_road
from scripts.verify_boundary_candidate import verify


class ManifestCenters:
    def __init__(self, manifest):
        self.lanes = {x['source_lane_id']: x['geometry']['coordinates']
                      for x in manifest['lanes'] if x.get('geometry')}

    def lane_center_geometry(self, source_id):
        return self.lanes.get(source_id)


def run(source, target, road_id, tolerance=.35):
    if source.resolve() == target.resolve():
        raise ValueError('candidate must be separate from input')
    manifest_path = source.with_suffix('.source-lanes.json')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    tree = etree.parse(str(source)); road = next(r for r in tree.findall('road') if r.get('id') == road_id)
    # Mirror is a provenance decision. Never reflect a measured departure.
    left = road.findall('lanes/laneSection/left/lane')
    prov = [l.find("userData[@code='mapforge.provenance/v1']") for l in left]
    mirror = bool(left) and all(l.find("userData[@code='mapforge.source_lane']") is None for l in left) and all(p is not None and json.loads(p.get('value')).get('support_kind') in
                               ('mirror', 'lane-transition-ribbon') for p in prov)
    report = solve_road(road, ManifestCenters(manifest), np.asarray,
                        source_mode='centers', mirror=mirror, source_tol=tolerance)
    target.parent.mkdir(parents=True, exist_ok=True)
    report.update(candidate_only=True, input_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    target.with_suffix('.fit.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(report), flush=True)
    if report['status'] != 'CANDIDATE': return False
    tree.write(str(target), encoding='UTF-8', xml_declaration=True, pretty_print=True)
    verify(target, source, manifest_path, road_id)
    return True


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('source', type=Path); p.add_argument('target', type=Path)
    p.add_argument('--road', required=True); p.add_argument('--source-tolerance', type=float, default=.35)
    a = p.parse_args(); sys.exit(0 if run(a.source, a.target, a.road, a.source_tolerance) else 2)
