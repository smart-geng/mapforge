"""从显式指定的原始 MAP XML 复核 manifest，独立于几何拟合器及其支持域。

只判定来源清单是否完整、同身份、同顺序。目标是否覆盖这些完整来源仍由 G8
判定；两项必须同时通过。不会自动搜索仓库，也不接受旧 manifest 作为原件。
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

import numpy as np

from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all


def audit_map_source_manifest(manifest, raw_paths=None):
    report = {"gate_id": "G8-source-integrity", "status": "UNAVAILABLE",
              "scope": "raw-XML-to-source-manifest; not target-geometry acceptance",
              "inputs": [], "lanes": [], "failure_reasons": []}
    if not raw_paths:
        report["failure_reasons"].append({"code": "raw_map_input_missing"})
        return report
    try:
        nodes, raw = {}, {}
        for path in dict.fromkeys(str(Path(p).resolve()) for p in raw_paths):
            p = Path(path)
            report["inputs"].append({"path": path, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()})
            for node in parse_map_xml_all(path):
                key = (node.region, node.node_id)
                if key in nodes and nodes[key] != node:
                    raise ValueError(f"ambiguous raw node identity: {key}")
                nodes[key] = node
                for link in node.links:
                    for lane in link.lanes:
                        owner = (*key, tuple(link.upstream), link.name, lane.lane_id)
                        pts = np.asarray(lane.points, float).reshape((-1, 2))
                        if owner in raw and not np.array_equal(raw[owner], pts):
                            raise ValueError(f"ambiguous raw lane identity: {owner}")
                        raw[owner] = pts
        crs = manifest["comparison_crs"]
        # Runtime MAP reader uses a spherical local eqc; verify that explicitly
        # rather than importing/reusing the converter's projection helper.
        params = dict(token.lstrip('+').split('=', 1) for token in crs['proj_string'].split()
                      if '=' in token)
        if params.get('proj') != 'eqc' or params.get('units') != 'm':
            raise ValueError('source check supports only explicit metric spherical eqc')
        radius, lat_ts, lat0, lon0 = [float(params[k]) for k in ('R', 'lat_ts', 'lat_0', 'lon_0')]
        if not all(math.isfinite(v) for v in (radius, lat_ts, lat0, lon0)) or radius <= 0:
            raise ValueError('invalid explicit eqc parameters')
        if any(float(params.get(k, 0)) != 0 for k in ('x_0', 'y_0')):
            raise ValueError('unexpected false easting/northing')
        mains = [c for c in manifest["source_contexts"] if c.get("role") == "main"]
        if len(mains) != 1:
            raise ValueError("exactly one main source context is required")
        main = (mains[0]["region"], mains[0]["node_id"])
        origin = crs["origin"]
        if not np.allclose([origin["lon"], origin["lat"]],
                           [nodes[main].ref_lon, nodes[main].ref_lat], atol=1e-9, rtol=0):
            raise ValueError("comparison origin is not the raw main node origin")
        if not np.allclose([lon0, lat0], [origin['lon'], origin['lat']], atol=1e-8, rtol=0):
            raise ValueError('declared eqc projection and origin disagree')
        expected = {key for key in raw if key[:2] == main}
        # Derive known real-exit source scope from ORIGINAL identities as well,
        # not solely from converter-selected contexts. Otherwise a converter
        # could omit an entire available neighboring Link and self-certify a
        # mirror as "no source". Missing neighbor files still permit inference.
        needed = {(c.region, c.node) for link in nodes[main].links for lane in link.lanes
                  for c in lane.connects if c.node is not None}
        legs = {tuple(link.upstream) for link in nodes[main].links}
        available = (needed & legs & nodes.keys()) - {main}
        expected.update(key for key in raw if key[:2] in available and key[2] == main)
        for context in manifest['source_contexts']:
            if context.get('role') == 'neighbor-real-exit':
                remote = (context['region'], context['node_id'])
                if remote not in nodes:
                    raise ValueError(f'missing raw neighboring source: {remote}')
                expected.update(key for key in raw if key[:2] == remote and key[2] == main)
        actual = {}
        failures = report["failure_reasons"]
        for row in manifest["lanes"]:
            owner = row["owner"]
            key = (owner["region"], owner["node"], tuple(owner["upstream"]),
                   owner["link"], owner["lane"])
            if key in actual:
                raise ValueError(f"duplicate manifest source identity: {key}")
            actual[key] = row
            # A selected neighboring real-exit Link must include all its source lanes.
            expected.update(k for k in raw if k[:4] == key[:4])
            raw_points = raw.get(key)
            sid = row["source_lane_id"]
            canonical = f'map:{key[0]}:{key[1]}:from:{key[2][0]}:{key[2][1]}:{key[3]}:lane:{key[4]}'
            if sid != canonical:
                failures.append({'code': 'source_id_owner_mismatch', 'source_lane_id': sid})
            item = {"source_lane_id": sid, "status": "FAIL"}
            report["lanes"].append(item)
            if raw_points is None:
                failures.append({"code": "source_identity_not_in_raw_xml", "source_lane_id": sid})
                continue
            xy = np.column_stack([np.deg2rad(raw_points[:, 0]-lon0)*radius*math.cos(math.radians(lat_ts)),
                                  np.deg2rad(raw_points[:, 1]-lat0)*radius])
            observed = np.asarray(row["geometry"]["coordinates"], float).reshape((-1, 2))
            item.update(raw_vertex_count=len(xy), manifest_vertex_count=len(observed))
            same = xy.shape == observed.shape
            if same:
                item["max_coordinate_delta_m"] = float(np.max(np.linalg.norm(xy-observed, axis=1), initial=0.))
                same = item["max_coordinate_delta_m"] <= 1e-5
            if not same:
                failures.append({"code": "raw_geometry_changed_or_clipped", "source_lane_id": sid})
            elif not row.get("comparison", {}).get("eligible"):
                failures.append({"code": "raw_lane_excluded_from_comparison", "source_lane_id": sid})
            else:
                item["status"] = "PASS"
        for key in sorted(expected - actual.keys(), key=str):
            failures.append({"code": "raw_lane_missing_from_manifest", "owner_identity": list(key)})
        report["status"] = "FAIL" if failures else "PASS"
        report["raw_expected_lane_count"] = len(expected)
    except Exception as exc:
        report["status"] = "UNAVAILABLE"
        report["failure_reasons"].append({"code": "raw_source_check_error",
                                          "message": f"{type(exc).__name__}: {exc}"})
    return report
