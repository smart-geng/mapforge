"""Read-only local MAP/SHP observation overlay; nearest is NOT identity matching."""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import CASES, shp_source
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
from mapforge.ops.map_to_xodr import _project
from mapforge.validate.shp_boundary_fidelity import _point_polyline_distance


def main():
    node = parse_map_xml(str(ROOT/'v2x_map_xml'/dict(CASES)['node4']))
    root = etree.parse(str(ROOT/'out/map-coordinate-v140/node4.xodr'))
    road = root.find("road[@id='10']"); geom = road.find('planView/geometry')
    if geom.find('line') is None or len(road.findall('planView/geometry')) != 1:
        raise ValueError('probe coordinate chart must be one Line')
    origin = np.array([float(geom.get(k)) for k in ('x', 'y')]); h = float(geom.get('hdg'))
    tangent = np.array([np.cos(h), np.sin(h)]); normal = np.array([-tangent[1], tangent[0]])
    def st(points):
        xy = _project(points, node.ref_lat, node.ref_lon)-origin
        return np.column_stack((xy@tangent, xy@normal))
    low, high = 250., 325.
    src = shp_source(); src._load_lanes(); nearby = []
    for lane in src._lane_by_pid.values():
        if len(lane.geometry) < 2:
            continue
        xy = st(lane.geometry)
        if (max(xy[:,0]) < low or min(xy[:,0]) > high
                or min(xy[:,1]) > 20. or max(xy[:,1]) < -10.):
            continue
        nearby.append((lane, xy))
    link = next(lk for lk in node.links if lk.name == 'west')
    rows = []; topology = []; fig, axes = plt.subplots(2, 1, figsize=(13, 8), dpi=160, sharex=True)
    for lane, xy in nearby:
        axes[0].plot(*xy.T, color='#679cc7', lw=1.4, alpha=.65)
        for boundary in src.lane_boundary_geometries(lane.lane_pid):
            axes[0].plot(*st(boundary).T, color='gray', lw=.6, alpha=.5)
        axes[1].plot(*xy.T, color='#a6bacb', lw=.8)
    for lane in link.lanes:
        xy = st(lane.points)
        observations = []
        axes[0].plot(*xy.T, 'o-', lw=1.4, ms=4, label=f'raw MAP Lane{lane.lane_id}')
        axes[1].plot(*xy.T, 'o-', lw=1.4, ms=4, label=f'raw MAP Lane{lane.lane_id}')
        for i, p in enumerate(xy):
            if not low <= p[0] <= high:
                continue
            distances = [(float(_point_polyline_distance(p[None,:], q)[0][0]), other)
                         for other, q in nearby]
            distance, other = min(distances, key=lambda v:v[0])
            candidates = sorted({item.lane_pid for d, item in distances if d <= .02})
            observations.append((i, p, candidates))
            rows.append({'map_lane': lane.lane_id, 'point_index': i, 's_m': float(p[0]), 't_m': float(p[1]),
                         'nearest_shp_center_m': distance, 'nearest_shp_lane': other.lane_pid,
                         'shp_candidates_within_2cm': candidates,
                         'shp_seq': other.seq, 'shp_start_width_mm': other.s_width_mm,
                         'shp_end_width_mm': other.e_width_mm})
            axes[1].annotate(f'{distance:.2f}m', p, xytext=(2,7), textcoords='offset points', fontsize=8)
        for (ia, a, starts), (ib, b, targets) in zip(observations, observations[1:]):
            if ib != ia+1 or not starts or not targets:
                continue
            paths = []; queue = [[sid] for sid in starts]; seen = set(starts)
            while queue:
                chain = queue.pop(0); sid = chain[-1]
                if sid in targets:
                    paths.append(chain); continue
                for next_id in src.topo_out.get(sid, []):
                    if next_id in seen:
                        continue
                    item = src.lane(next_id)
                    if item is None or len(item.geometry) < 2:
                        continue
                    q = st(item.geometry)
                    # This is only a local forward-path check, not a global
                    # routing search that could leave and return through a junction.
                    if (q[-1,0] < q[0,0] or q[0,0] > b[0]+.5
                            or q[-1,0] < a[0]-.5 or min(q[:,1]) > 20. or max(q[:,1]) < -10.):
                        continue
                    seen.add(next_id); queue.append(chain+[next_id])
            topology.append({'map_lane': lane.lane_id, 'point_indices': [ia, ib],
                             's_range_m': [float(a[0]), float(b[0])], 'start_candidates': starts,
                             'target_candidates': targets, 'local_forward_shp_topo_paths': paths,
                             'status': 'PATH_FOUND' if paths else 'REVIEW_REQUIRED',
                             'interpretation': 'SHP local path evidence, not automatic MAP correction'})
    for ax in axes:
        ax.set(xlim=(low, high), ylim=(-8, 18), ylabel='lateral coordinate t (m)')
        ax.grid(alpha=.2); ax.legend(loc='upper left', fontsize=8)
    axes[0].set_title('Original MAP points vs original SHP centers (blue) and boundaries (gray)')
    axes[1].set_title('At MAP vertices: nearest raw SHP center distance; NOT an identity/fidelity certificate')
    axes[1].set_xlabel('same reference coordinate s (m)')
    fig.suptitle('node4 west: no generated lane geometry used as source truth')
    fig.tight_layout()
    output = ROOT/'out/preview/map-source-bend-v141/node4-west'; output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output.with_suffix('.png')); plt.close(fig)
    report = {'scope': 'local raw geometry diagnostic; nearest is not a source ID mapping',
              'source_modified': False, 'nearby_shp_lane_count': len(nearby), 'map_vertices': rows,
              'local_topology_checks': topology,
              'shp_lanes': [{'source_lane_id': lane.lane_pid, 'source_link_id': lane.link_pid,
                            'seq': lane.seq, 'st': xy.tolist(),
                            'successors': src.topo_out.get(lane.lane_pid, [])} for lane, xy in nearby]}
    output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'nearby_shp_lane_count': len(nearby), 'local_topology_checks': topology}, indent=2))


if __name__ == '__main__':
    main()
