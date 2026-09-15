"""Read-only counterfactuals for MAP center/width compatibility.

Never emits XODR or returns relaxed parameters. A feasible relaxation is only
a necessary check, not evidence of a source-faithful road or global solution.
"""
import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import CASES
from scripts.recheck_speed_contract import sha, dump
from mapforge.validate.map_source import audit_map_source_manifest
from mapforge.validate.map_width import raw_widths
from spikes.map_lane_family import ManifestCenters
from spikes.road_boundary_family import solve_road


def raw_station_arrays(road, source):
    g = road.find('planView/geometry')
    if len(road.findall('planView/geometry')) != 1 or g.find('line') is None:
        raise ValueError('probe requires one Line chart')
    h = float(g.get('hdg')); e = np.array([np.cos(h), np.sin(h)])
    normal = np.array([-e[1], e[0]])
    origin = np.array([float(g.get('x')), float(g.get('y'))])
    result = {}
    for sid, points in source.lanes.items():
        xy = np.asarray(points)-origin
        result[sid] = np.c_[xy@e, xy@normal]
    return result


def vertex_only(model, stations):
    """Drop interpolated observations, preserve every original vertex row."""
    keep = []
    for label in model.labels:
        kind = label['kind']
        if kind == 'whole-source-center':
            keep.append(False)
        elif kind == 'source-center':
            ss = stations[label['source']][:, 0]
            keep.append(bool(np.any(abs(ss-float(label['s'])) < 1e-7)))
        else:
            keep.append(True)
    mask = np.asarray(keep, bool)
    subset = copy.copy(model)
    subset.C = model.C[mask]; subset.lower = model.lower[mask]
    subset.labels = [label for label, selected in zip(model.labels, mask) if selected]
    return subset, int((~mask).sum())


def near_vertex_pairs(road, stations, widths, max_station_gap=.05):
    """Raw pairs, no invented interpolation or claim of identical cross-section."""
    rows = {}; sections = road.findall('lanes/laneSection')
    ends = [float(s.get('s')) for s in sections[1:]]+[float(road.get('length'))]
    for sec, end in zip(sections, ends):
        start = float(sec.get('s'))
        for side in ('left', 'right'):
            lanes = sorted(sec.findall(side+'/lane'), key=lambda l: abs(int(l.get('id'))))
            ids = [l.find("userData[@code='mapforge.source_lane']") for l in lanes]
            ids = [u.get('value') if u is not None else None for u in ids]
            for i, a in enumerate(ids):
                for j in range(i+1, len(ids)):
                    b = ids[j]; interval = ids[i:j+1]
                    if any(sid not in stations or widths.get(sid) is None for sid in interval):
                        continue
                    expected = .5*(widths[a]+widths[b])+sum(widths[sid] for sid in interval[1:-1])
                    ga, gb = stations[a], stations[b]
                    for ai, (s, t) in enumerate(ga):
                        if not start <= s <= end: continue
                        bi = int(np.argmin(abs(gb[:, 0]-s))); sb, tb = gb[bi]
                        if not start <= sb <= end or abs(sb-s) > max_station_gap: continue
                        measured = abs(t-tb)
                        row = dict(source_a=a, point_index_a=ai, source_b=b, point_index_b=bi,
                                   station_gap_m=float(abs(sb-s)), stations_m=[float(s), float(sb)],
                                   projected_separation_m=float(measured), stacked_width_separation_m=expected,
                                   difference_m=float(measured-expected), side=side)
                        rows[(a, ai, b, bi)] = row
    return sorted(rows.values(), key=lambda r: -abs(r['difference_m']))[:8]


def run(path, output):
    path = path.resolve(); output = output.resolve()
    if output.exists(): raise FileExistsError('use a fresh diagnostic directory')
    manifest_path = path.with_suffix('.source-lanes.json')
    raw = [ROOT/'v2x_map_xml'/p for _, p in CASES]
    tracked = raw+[path, manifest_path]
    for folder in ('mapforge', 'spikes', 'scripts'):
        tracked += list((ROOT/folder).rglob('*.py'))
    hashes = {str(p.resolve()): sha(p) for p in tracked}
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    admission = audit_map_source_manifest(manifest, raw)
    if admission['status'] != 'PASS': raise ValueError('raw source admission failed')
    source = ManifestCenters(manifest); widths = raw_widths(raw)
    rows = []
    for road in etree.parse(str(path)).findall('road'):
        if road.get('junction') != '-1': continue
        stations = raw_station_arrays(road, source)
        left = road.findall('lanes/laneSection/left/lane')
        mirror = bool(left) and all(l.find("userData[@code='mapforge.source_lane']") is None for l in left)
        prov = [l.find("userData[@code='mapforge.provenance/v1']") for l in left]
        mirror &= all(p is not None and json.loads(p.get('value')).get('support_kind')
                      in ('mirror', 'lane-transition-ribbon') for p in prov)
        tests = []
        for mode in ('source-parallel', 'free'):
            model = solve_road(road, source, np.asarray, source_mode='centers', mirror=mirror,
                source_tol=.35, source_error_budget='absolute', all_source_vertices=True,
                model_only=True, source_certificate=True, source_widths=widths,
                junction_endpoint_mode=mode)
            if isinstance(model, dict):
                tests.append(dict(mode=mode, model_rejected=model)); continue
            for support in ('full-polyline', 'original-vertices-only'):
                selected, removed = (model, 0) if support == 'full-polyline' else vertex_only(model, stations)
                _, phase = selected.preflight()
                tests.append(dict(mode=mode, support=support, removed_constraints=removed, **phase))
        rows.append(dict(road=road.get('id'), tests=tests,
                         near_raw_vertex_pairs=near_vertex_pairs(road, stations, widths)))
        print(path.stem, road.get('id'), [(t.get('mode'), t.get('support'), t.get('normalized_phase_slack')) for t in tests], flush=True)
    if any(sha(Path(p)) != digest for p, digest in hashes.items()):
        raise ValueError('input or implementation changed during diagnostic')
    output.mkdir(parents=True)
    dump(output/'report.json', dict(status='DIAGNOSTIC_ONLY', xodr_emitted=False,
         source_and_code_sha256=hashes, source_admission=admission, roads=rows,
         limits=['fixed axis and section stations', 'constant supplied laneWidth interpretation',
                 'vertex-only drops whole-polyline guarantees', 'dynamics and connectors not solved',
                 'near-station raw pairs are not exact common cross-sections', 'not global infeasibility']))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('xodr', type=Path); p.add_argument('output', type=Path)
    a = p.parse_args(); run(a.xodr, a.output)
