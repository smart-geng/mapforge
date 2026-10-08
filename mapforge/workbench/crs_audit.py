"""Reproducible CRS evidence audit; never approves a production CRS by inference.

Declaration checks, reversible local arithmetic, relative source agreement, and
absolute geodetic verification are separate findings. In particular, matching
two products derived from one survey supplies no independent ground truth.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import re
import xml.etree.ElementTree as ET

import numpy as np
import pyproj
from pyproj import CRS, Geod, Transformer
from pyproj.enums import TransformDirection
import shapefile
import shapely
from shapely import STRtree
import yaml

from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.mapir.crs_probe import wgs2gcj

SCHEMA = "mapforge/crs-audit/v1"
_PRIMARY_DOCS = [
    "https://pyproj4.github.io/pyproj/stable/api/transformer.html",
    "https://proj.org/en/stable/operations/projections/aeqd.html",
]


def _hash(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _binding(path: Path, root: Path) -> dict:
    return {"path": path.relative_to(root).as_posix() if path.is_relative_to(root) else path.name,
            "sha256": _hash(path), "size": path.stat().st_size}


def local_transform(lon0: float, lat0: float, *, semi_major: float = 6378137.0,
                    inverse_flattening: float = 298.257223563) -> Transformer:
    """Explicit lon/lat degrees -> local east/north metres on declared ellipsoid.

    This is a numerical model, not an EPSG assignment or datum correction.
    No network, grid, inferred axis swap, or production source mutation is used.
    """
    if not (-180 <= lon0 <= 180 and -90 < lat0 < 90):
        raise ValueError("投影原点必须为有效 lon,lat 度数")
    return Transformer.from_pipeline(
        f"+proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad "
        f"+step +proj=aeqd +lon_0={lon0:.15g} +lat_0={lat0:.15g} "
        f"+a={semi_major:.15g} +rf={inverse_flattening:.15g}")


def projection_check(points: np.ndarray, lon0: float, lat0: float) -> dict:
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points):
        return {"status": "FAILED", "reason": "missing-coordinate-samples"}
    valid = (np.isfinite(points).all(axis=1) & (abs(points[:, 0]) <= 180) & (abs(points[:, 1]) <= 90))
    if not valid.all():
        return {"status": "FAILED", "reason": "invalid-lon-lat-range", "invalid_indices": np.flatnonzero(~valid).tolist()}
    transformer = local_transform(lon0, lat0)
    x, y = transformer.transform(points[:, 0].tolist(), points[:, 1].tolist(), errcheck=True)
    lon, lat = transformer.transform(x, y, direction=TransformDirection.INVERSE, errcheck=True)
    error = np.hypot(np.asarray(lon) - points[:, 0], np.asarray(lat) - points[:, 1])
    geod = Geod(a=6378137.0, rf=298.257223563)
    east_lon, east_lat, _ = geod.fwd(lon0, lat0, 90.0, 100.0)
    north_lon, north_lat, _ = geod.fwd(lon0, lat0, 0.0, 100.0)
    east = transformer.transform(east_lon, east_lat, errcheck=True)
    north = transformer.transform(north_lon, north_lat, errcheck=True)
    zero = transformer.transform(lon0, lat0, errcheck=True)
    axis_ok = abs(east[0] - 100) < 1e-5 and abs(east[1]) < 1e-5 and abs(north[1] - 100) < 1e-5 and abs(north[0]) < 1e-5
    return {"status": "PASS" if float(error.max()) < 1e-9 and axis_ok else "FAILED",
            "scope": "numerical-transform-only", "input_order": "longitude,latitude",
            "input_unit": "degree", "output_order": "east,north", "output_unit": "m",
            "origin": [lon0, lat0], "origin_xy": list(zero),
            "pipeline": transformer.definition, "has_inverse": transformer.has_inverse,
            "network_enabled": transformer.is_network_enabled, "sample_count": len(points),
            "roundtrip_max_deg": float(error.max()), "roundtrip_p95_deg": float(np.percentile(error, 95)),
            "axis_test": {"east_100m_xy": list(east), "north_100m_xy": list(north), "pass": bool(axis_ok)},
            "numerical_roundtrip_tolerance_deg": 1e-9,
            "tolerance_role": "software numeric check, not surveyed map accuracy",
            "absolute_crs_verified": False}


def _raw_map_points(path: Path) -> dict:
    """Enumerate every original RoadPoint, retaining raw integer values/identity."""
    root = ET.parse(path).getroot()
    nodes = root.findall("mapFrame/nodes/Node")
    rows, invalid, origins = [], [], []
    for ni, node in enumerate(nodes):
        lon = node.findtext("refPos/long")
        lat = node.findtext("refPos/lat")
        try:
            origins.append([int(lon) * 1e-7, int(lat) * 1e-7])
        except (TypeError, ValueError):
            invalid.append({"node_index": ni, "code": "invalid-refPos"})
        for li, link in enumerate(node.findall("inLinks/Link")):
            groups = [("link", None, link.find("points"))] + [
                ("lane", lane.findtext("laneID"), lane.find("points")) for lane in link.findall("lanes/Lane")]
            for kind, lane_id, parent in groups:
                for pi, point in enumerate([] if parent is None else parent.findall("RoadPoint")):
                    ident = {"node_index": ni, "node_id": node.findtext("id/id"), "node_region": node.findtext("id/region"),
                             "link_index": li, "link_name": link.findtext("name"),
                             "upstream_node": link.findtext("upstreamNodeId/id"),
                             "upstream_region": link.findtext("upstreamNodeId/region"),
                             "kind": kind, "lane_id": lane_id, "point_index": pi}
                    coord = point.find("posOffset/offsetLL/position-LatLon")
                    if coord is None:
                        invalid.append({**ident, "code": "unsupported-coordinate-branch"})
                        continue
                    try:
                        raw_lon, raw_lat = int(coord.findtext("lon")), int(coord.findtext("lat"))
                        if not (-1800000000 <= raw_lon <= 1800000000 and -900000000 <= raw_lat <= 900000000):
                            raise ValueError("coordinate out of range")
                    except (ValueError, TypeError):
                        invalid.append({**ident, "code": "invalid-coordinate-value"})
                        continue
                    rows.append({**ident, "longitude_e7": raw_lon, "latitude_e7": raw_lat,
                                 "lon": raw_lon * 1e-7, "lat": raw_lat * 1e-7})
    return {"file": path.name, "nodes": len(nodes), "origins": origins, "points": rows, "invalid": invalid}


def _lane_segments(source_dir: Path) -> tuple[np.ndarray, list[dict]]:
    coordinates, identities = [], []
    with shapefile.Reader(str(source_dir / "IBD_LANE_LINK"), encoding="gbk") as reader:
        fields = [f[0] for f in reader.fields[1:]]
        for ri, sr in enumerate(reader.iterShapeRecords()):
            attrs = dict(zip(fields, sr.record))
            for pi, part in enumerate(ProfileSource._raw_parts(sr.shape)):
                for si, (a, b) in enumerate(zip(part, part[1:])):
                    coordinates.append([a[:2], b[:2]])
                    identities.append({"layer": "IBD_LANE_LINK", "record_index": ri,
                                       "part_index": pi, "segment_index": si,
                                       "lane_pid": str(attrs.get("LANE_PID", ""))})
    return np.asarray(coordinates, float), identities


def _nearest(query: np.ndarray, segments: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if not len(query) or not len(segments):
        raise ValueError("最近段计算要求非空点与线段")
    tree = STRtree(shapely.linestrings(segments))
    indices, distances = tree.query_nearest(shapely.points(query), all_matches=False, return_distance=True)
    ordered = np.argsort(indices[0])
    return distances[ordered], indices[1][ordered]


def compare_internal(raw_map: dict, segments: np.ndarray, identities: list[dict], *, shifted_segments=None) -> dict:
    points = np.asarray([[p["lon"], p["lat"]] for p in raw_map["points"]], float)
    if not len(points) or not len(segments) or not raw_map["origins"]:
        return {"file": raw_map["file"], "status": "FAILED", "reason": "missing-source-coordinates"}
    # Search the entire registered layer. A fixed raw-space crop can omit a
    # segment that becomes nearest under a shifted hypothesis.
    selected = segments
    lon0, lat0 = raw_map["origins"][0]
    projection = local_transform(lon0, lat0)

    def project(value):
        flat = np.asarray(value).reshape(-1, 2)
        x, y = projection.transform(flat[:, 0].tolist(), flat[:, 1].tolist(), errcheck=True)
        return np.column_stack([x, y]).reshape(np.asarray(value).shape)

    if shifted_segments is None:
        shifted_segments = np.asarray([wgs2gcj(*p) for p in selected.reshape(-1, 2)]).reshape(selected.shape)
    shifted_points = np.asarray([wgs2gcj(*p) for p in points])
    hypotheses = {"H0_direct": (points, selected), "H1_shift_shp": (points, shifted_segments),
                  "H2_shift_map": (shifted_points, selected)}
    stats, h0_details = {}, []
    for name, (q, s) in hypotheses.items():
        distance, nearest = _nearest(project(q), project(s))
        stats[name] = {"median_m": float(np.median(distance)), "p90_m": float(np.percentile(distance, 90)),
                       "max_m": float(distance.max()), "points": len(distance)}
        if name == "H0_direct":
            for row, d, nearest_index in zip(raw_map["points"], distance, nearest):
                original = int(nearest_index)
                h0_details.append({**row, "distance_m": float(d), "nearest_source": identities[original],
                                   "source_segment_lonlat": segments[original].tolist()})
    best = min(stats, key=lambda k: stats[k]["median_m"])
    return {"file": raw_map["file"], "status": "OBSERVED", "map_points": len(points),
            "invalid_map_points": raw_map["invalid"], "source_segments": len(selected),
            "source_window": "entire-original-IBD_LANE_LINK-layer",
            "projection_pipeline": projection.definition, "hypotheses": stats, "best_median": best,
            "comparison": "exact nearest original segment, not semantic lane correspondence",
            "source_corrections_applied": False, "absolute_crs_verified": False,
            "h0_points": h0_details}


def _correction_crosscheck(root: Path, map_dir: Path, internal: list[dict]) -> list[dict]:
    observed = {item["file"]: item for item in internal}
    results = []
    for path in sorted((root / "profiles/source-corrections").glob("*.yaml")):
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        source = document.get("source", {})
        name = Path(source.get("file", "")).name
        if name not in observed:
            continue
        bound = (map_dir / name).is_file() and _hash(map_dir / name) == source.get("sha256")
        for correction in document.get("corrections", []):
            target = correction["target"]
            rows = [p for p in observed[name].get("h0_points", [])
                    if p["node_id"] == str(target["node"]["id"])
                    and p["node_region"] == str(target["node"]["region"])
                    and p["link_name"] == target["link"]["name"]
                    and p["upstream_node"] == str(target["link"]["upstream"]["id"])
                    and p["upstream_region"] == str(target["link"]["upstream"]["region"])
                    and p["lane_id"] == str(target["lane"])
                    and p["point_index"] == target["point_index_0based"]
                    and p["longitude_e7"] == target["raw"]["lon"] and p["latitude_e7"] == target["raw"]["lat"]]
            results.append({"correction_id": document["id"], "decision_file": _binding(path, root),
                            "source_file": name, "source_hash_matches": bound, "target": target,
                            "original_point_in_audit": len(rows) == 1, "applied_in_audit": False,
                            "h0_distance_m": rows[0]["distance_m"] if len(rows) == 1 else None,
                            "interpretation": "Nearest unrelated source geometry can be close even for a known topology/sequence outlier"})
    return results


def _declarations(source_dir: Path) -> dict:
    prjs, failures = [], []
    for path in sorted(source_dir.glob("*.prj")):
        try:
            wkt = path.read_text(encoding="utf-8-sig")
            crs = CRS.from_wkt(wkt)
            prjs.append({"file": path.name, "sha256": _hash(path), "wkt": wkt,
                         "geographic": crs.is_geographic,
                         "axes": [{"name": a.name, "direction": a.direction, "unit": a.unit_name,
                                   "unit_conversion_factor": a.unit_conversion_factor} for a in crs.axis_info],
                         "semi_major_m": crs.ellipsoid.semi_major_metre,
                         "inverse_flattening": crs.ellipsoid.inverse_flattening,
                         "authority_explicit_in_original": bool(re.search(r"\b(?:AUTHORITY|ID)\s*\[", wkt))})
        except (ValueError, OSError, pyproj.exceptions.CRSError) as exc:
            failures.append({"file": path.name, "error": str(exc)})
    version = []
    with shapefile.Reader(dbf=str(source_dir / "IBD_VERSION.dbf"), encoding="gbk") as reader:
        fields = [f[0] for f in reader.fields[1:]]
        for index, row in enumerate(reader.iterRecords()):
            version.append({"record_index": index, "attributes": dict(zip(fields, row))})
    geometry_bounds = []
    for path in sorted(source_dir.glob("*.shp")):
        with shapefile.Reader(str(path), encoding="gbk") as reader:
            geometry_bounds.append({"file": path.name, "rows": len(reader), "shape_type": reader.shapeType,
                                    "header_bounds": list(getattr(reader, "bbox", []))})
    numeric_ellipsoid = all(p["geographic"] and abs(p["semi_major_m"] - 6378137) < 1e-6
                            and abs(p["inverse_flattening"] - 298.257223563) < 1e-8
                            and p["axes"] and all(a["unit"].lower() == "degree" for a in p["axes"]) for p in prjs)
    version_agrees = bool(version) and all(str(v["attributes"].get("COORD_SYS")) == "84"
                                           and str(v["attributes"].get("COORD_UNIT")) == "2" for v in version)
    return {"status": "DECLARATIONS_CONSISTENT" if prjs and numeric_ellipsoid and version_agrees and not failures else "REVIEW_REQUIRED",
            "prj_files": prjs, "parse_failures": failures, "version_records": version,
            "geometry_headers": geometry_bounds, "absolute_crs_verified": False,
            "warning": "WKT numerical ellipsoid equivalence and metadata declaration do not verify source coordinates against Earth"}


def inventory_independent_evidence(root: Path) -> dict:
    directories = ["docs", "profiles", "ledger", "archives", "shp_0222-0326", "v2x_map_xml"]
    candidates, inspected = [], 0
    excluded = {".git", ".venv", "xml2xodr", "__pycache__"}
    pattern = re.compile(r"control|ground.?truth|gcp|rtk|gnss|survey|georef|crs|坐标|测绘|控制点|测量|基准|正射|影像", re.I)
    formats = {".csv", ".xlsx", ".tif", ".tiff", ".las", ".laz", ".gpkg", ".gpx", ".kml", ".geojson"}
    for directory in directories:
        for path in sorted((root / directory).rglob("*")):
            if not path.is_file() or any(part in excluded for part in path.relative_to(root).parts):
                continue
            inspected += 1
            if pattern.search(path.name) or path.suffix.lower() in formats:
                candidates.append(_binding(path, root))
    return {"search_roots": directories, "excluded_subtrees": sorted(excluded),
            "file_count": inspected, "candidate_files": candidates,
            "method": "filename and geospatial-control artifact extensions; repository evidence review, not external survey",
            "external_authority_source_registered": False,
            "limitation": "Does not prove evidence absent outside listed repository roots or inside unindexed binary documents"}


def control_input_contract() -> dict:
    return {"schema": "mapforge/absolute-crs-control/v1", "status": "INPUT_REQUIRED",
            "purpose": "independent physical correspondence, not curve-control handles or SHP-derived MAP",
            "source_files": [{"relative_path": None, "sha256": None}],
            "authority": {"organization": None, "survey_date": None, "method": None,
                          "independence_from_source": None, "evidence_file": None, "evidence_sha256": None},
            "reference": {"crs_definition": None, "coordinate_order": "longitude,latitude", "coordinate_unit": "degree",
                          "realization_or_epoch": None, "vertical_datum": None},
            "acceptance": {"policy_id": None, "policy_sha256": None, "scope": None,
                           "max_horizontal_residual_m": None, "coverage_requirement": None},
            "points": [{"control_id": None,
                        "source_ref": {"layer": None, "record_index": None, "part_index": None, "point_index": None},
                        "reference_lon": None, "reference_lat": None, "uncertainty_m": None,
                        "physical_feature_description": None}],
            "review_note": "A numerical pass is not acceptance-policy approval or authenticated survey provenance"}


def audit_crs(source_dir: str | Path, map_dir: str | Path, *, repo_root: str | Path) -> dict:
    source_dir, map_dir, root = Path(source_dir).resolve(), Path(map_dir).resolve(), Path(repo_root).resolve()
    used_paths = sorted({path for path in source_dir.iterdir() if path.is_file()} | set(map_dir.glob("map*.xml")))
    before_bindings = [_binding(path, root) for path in used_paths]
    declarations = _declarations(source_dir)
    segments, identities = _lane_segments(source_dir)
    if not len(segments):
        raise ValueError("原始车道无可审核线段")
    points = segments.reshape(-1, 2)
    origin = (points.min(axis=0) + points.max(axis=0)) / 2
    projection = projection_check(points, *origin)
    maps = [_raw_map_points(path) for path in sorted(map_dir.glob("map*.xml"))]
    shifted = np.asarray([wgs2gcj(*p) for p in segments.reshape(-1, 2)]).reshape(segments.shape)
    internal = [compare_internal(raw, segments, identities, shifted_segments=shifted) for raw in maps]
    h0_best = bool(internal) and all(i.get("best_median") == "H0_direct" and not i.get("invalid_map_points") for i in internal)
    if before_bindings != [_binding(path, root) for path in used_paths]:
        raise ValueError("审核期间原件发生字节变化，不能绑定混合证据")
    bound_docs = [root / "docs/资料盘点-金凤示范区数据与标准资料.md",
                  root / "docs/M0作业报告-ASN编译与数据核验.md", root / "out/crs_check_report.md",
                  root / "scripts/m0_crs_check.py", root / "mapforge/mapir/crs_probe.py",
                  Path(__file__).resolve()]
    return {"schema": SCHEMA, "audit_status": "COMPLETED_WITH_BLOCKERS",
            "production_crs_status": "UNVERIFIED", "production_export_allowed": False,
            "absolute_verification": {"status": "NOT_DEMONSTRATED", "reason": "independent-control-evidence-not-registered",
                                      "verified_points": 0, "scope": None},
            "declarations": declarations, "projection": projection,
            "relative_agreement": {"status": "H0_BEST_ALL_FILES" if h0_best else "REVIEW_REQUIRED",
                                   "absolute_crs_verified": False, "file_count": len(internal),
                                   "point_count": sum(len(m["points"]) for m in maps), "files": internal,
                                   "conclusion": "Relative numerical agreement only; same-source products cannot establish absolute datum or origin"},
            "known_source_corrections": _correction_crosscheck(root, map_dir, internal),
            "independent_material_inventory": inventory_independent_evidence(root),
            "control_input_contract": control_input_contract(),
            "source_files": before_bindings,
            "method_files": [_binding(path, root) for path in bound_docs if path.is_file()],
            "runtime": {"python": platform.python_version(), "pyproj": pyproj.__version__,
                        "proj": pyproj.proj_version_str, "numpy": np.__version__, "shapely": shapely.__version__},
            "primary_technical_references": _PRIMARY_DOCS,
            "issues": [{"code": "absolute-crs-not-verified", "severity": "blocking",
                        "message": "缺少与原始具体点对应、独立于SHP/MAP同源数据的测量或权威控制材料"},
                       {"code": "vertical-datum-undeclared", "severity": "blocking-for-height-use",
                        "message": "源SHP高程的垂直基准未声明，本次仅核查水平坐标"},
                       {"code": "formal-accuracy-policy-required", "severity": "blocking",
                        "message": "软件正反算容差与同源叠合距离不是客户用途的正式绝对精度门槛"}]}


def main() -> int:
    parser = argparse.ArgumentParser(description="离线只读 CRS 审核；不批准生产坐标系")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--source", type=Path, default=Path("shp_0222-0326"))
    parser.add_argument("--maps", type=Path, default=Path("v2x_map_xml"))
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = audit_crs(args.root / args.source, args.root / args.maps, repo_root=args.root)
    args.out.mkdir(parents=True, exist_ok=True)
    output = args.out / "report.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (args.out / "control-input-contract.json").write_text(json.dumps(control_input_contract(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(output), "audit_status": report["audit_status"],
                      "production_crs_status": report["production_crs_status"],
                      "projection": report["projection"]["status"],
                      "relative_agreement": report["relative_agreement"]["status"]}, ensure_ascii=False))
    return 2  # Audit complete, but a production gate has not passed.


if __name__ == "__main__":
    raise SystemExit(main())
