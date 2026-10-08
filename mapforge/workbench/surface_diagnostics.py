"""Read-only, evidence-bound auxiliary surface diagnostics.

This is deliberately limited to the source-track reconstruction contract. It
does not score a map, prove a driving surface, edit geometry or accept delivery.
The caller must bind the supplied reconstruction evidence to its source snapshot.
"""
from __future__ import annotations

import hashlib
import json
import math
from collections import Counter

from shapely import wkb
from shapely.errors import GEOSException
from shapely.geometry import Polygon, mapping
from shapely.ops import unary_union

from mapforge.validate import smoothness
from . import source_surface_reconstruction as R
from . import source_surface_support as S
from . import source_surface_tracks as T

SCHEMA = "mapforge/surface-diagnostics/v1"
RECONSTRUCTION_SCHEMA = "mapforge/workbench-source-surface-reconstruction/v1"
# Preserve the existing metric's actual comparison; its name is misleading.
EXISTING_AREA_CUTOFF_M2 = .01  # 100 cm², not 1 cm².


def _polygons(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    return [p for child in getattr(geometry, "geoms", ()) for p in _polygons(child)]


def _holes(geometry):
    return sorted([Polygon(r) for p in _polygons(geometry) for r in p.interiors],
                  key=lambda p: (-p.area, p.bounds))


def _record_geometry(record):
    raw = bytes.fromhex(record["wkb_hex"])
    if hashlib.sha256(raw).hexdigest() != record["sha256"]:
        raise ValueError("source-geometry-record-hash-mismatch")
    geometry = wkb.loads(raw)
    if not geometry.is_valid or geometry.geom_type not in {"Polygon", "MultiPolygon", "GeometryCollection"}:
        raise ValueError("unsupported-source-polygon-record")
    if float(record["area_m2"]) != geometry.area:
        raise ValueError("source-geometry-record-area-mismatch")
    return geometry


def _covered(shape, cover, band):
    return shape.difference(cover.buffer(band) if band else cover).is_empty


def _partition_gap(hole, original_holes, expected, band, *, actual=None):
    """Partition by spatial coverage, never by similar area or nearest object.

    Exact geometry is retained. The prederived serialization band only decides
    whether a remainder can be resolved; it is not a source-cover tolerance.
    """
    resolved = hole if actual is None else hole.difference(actual.buffer(band) if band else actual)
    if _covered(resolved, original_holes, band):
        return [("original-source-gap", hole)]
    if _covered(resolved, expected, band):
        return [("representation-loss", hole)]
    original = hole.intersection(original_holes)
    remainder = hole.difference(original)
    lost = remainder.intersection(expected)
    unknown = remainder.difference(expected)
    rows = [(kind, part) for kind, geometry in (
        ("original-source-gap", original), ("representation-loss", lost),
        ("unattributed-gap", unknown)) for part in _polygons(geometry)]
    return rows


def _association(shape, families, pieces, band):
    adjacent = []
    for family in families:
        geometry = family["geometry"]
        if shape.distance(geometry) <= band:
            adjacent.append({"region_index": family["region_index"], "role": family["role"],
                "lane_type": family["lane_type"], "road_ids": family["road_ids"],
                "source_road": family["source_road"], "source_lane_ids": family["source_lane_ids"],
                "association": "intersects-or-adjacent-within-serialization-band"})
    nearby_pieces = [{"source_lane_id": p["source_lane"], "source_road_id": p["road_id"]}
                     for p in pieces if shape.distance(p["geometry"]) <= band]
    return {"road_ids": sorted({rid for row in adjacent for rid in row["road_ids"]}),
        "source_lane_ids": sorted({sid for row in adjacent for sid in row["source_lane_ids"]}
                                  | {p["source_lane_id"] for p in nearby_pieces}),
        "source_road_ids": sorted({row["source_road"] for row in adjacent if row["source_road"] is not None}
                                  | {p["source_road_id"] for p in nearby_pieces}),
        "roles": sorted({row["role"] for row in adjacent}),
        "adjacent_families": adjacent, "adjacent_source_pieces": nearby_pieces}


_LABELS = {
    "original-source-gap": ("原始来源空区", "实际辅助面空区位于已绑定的原始空区内；不能据此补成可行驶面。"),
    "sampling-artifact": ("采样形成的伪孔", "旧采样形成空区；实际 XML 全部线性断点构成的辅助面已覆盖该空区。"),
    "representation-loss": ("辅助面表达缺失", "来源期望面覆盖此处，实际辅助面存在缺面；需要修复并重新检查。"),
    "unattributed-gap": ("尚未归因的空区", "现有来源空区及期望面不能完整解释此处；需要核查来源。"),
}


def _issue(kind, shape, families, pieces, band, **extra):
    title, detail = _LABELS[kind]
    identity = hashlib.sha256(kind.encode() + shape.wkb).hexdigest()[:20]
    return {"id": "surface-" + identity, "kind": kind, "title": title, "detail": detail,
        "area_m2": float(shape.area), "geometry": mapping(shape),
        "coordinate_system": "OpenDRIVE local XY, meters",
        "bounds_xy": list(shape.bounds), "centroid_xy": list(shape.centroid.coords[0]),
        "numeric_serialization_band_m": band,
        **_association(shape, families, pieces, band), **extra}


def _validate(root, evidence):
    if evidence.get("schema") != RECONSTRUCTION_SCHEMA:
        raise ValueError("unsupported-reconstruction-evidence")
    if evidence.get("intent", {}).get("type") != R.INTENT_TYPE:
        raise ValueError("unsupported-reconstruction-intent")
    rows = evidence["families"]
    if not rows:
        raise ValueError("missing-source-families")
    families, ids, snapshot_ids, hashes = [], [], set(), set()
    for row in rows:
        proof = row["representation"]
        contract = proof["write_contract"]
        provenance = row["provenance"]
        if (provenance != contract["provenance"] or row["lane_type"] != contract["lane_type"]
                or row["role"] != provenance["support_kind"]
                or row["region_index"] != provenance["source_region_index"]
                or provenance["source_region_sha256"] != proof["source_geometry_sha256"]):
            raise ValueError("source-family-metadata-binding-mismatch")
        if row["lane_type"] not in {"restricted", "median"}:
            raise ValueError("unsupported-auxiliary-material")
        snapshot_ids.add(provenance["source_snapshot_id"])
        hashes.add(provenance["source_content_hash"])
        if not T.audit_written(root, proof).get("passed"):
            raise ValueError("actual-track-or-source-contract-audit-failed")
        shape = wkb.loads(bytes.fromhex(proof["source_geometry_wkb_hex"]))
        ids.extend(proof["road_ids"])
        families.append({"region_index": row["region_index"], "role": row["role"],
            "lane_type": row["lane_type"], "source_road": row.get("source_road"),
            "road_ids": proof["road_ids"], "source_lane_ids": provenance["source_lanes"],
            "geometry": shape})
    if len(snapshot_ids) != 1 or len(hashes) != 1 or not all(snapshot_ids | hashes):
        raise ValueError("mixed-or-missing-source-snapshot")
    if (snapshot_ids != {evidence["intent"]["source_snapshot_id"]}
            or hashes != {evidence["intent"]["source_content_hash"]}):
        raise ValueError("source-snapshot-does-not-match-reconstruction-intent")
    if len(ids) != len(set(ids)) or set(ids) != set(evidence["paving_id_diff"]["candidate_ids"]):
        raise ValueError("source-family-id-set-mismatch")
    band = max(float(row["representation"]["numeric_serialization_band_m"]) for row in rows)
    if not math.isfinite(band) or band < 0:
        raise ValueError("invalid-derived-serialization-band")
    support = evidence["source_support"]
    layers = {kind: T.written_geometry(root, [rid for row in families if row["lane_type"] == kind
                                             for rid in row["road_ids"]])
              for kind in ("restricted", "median")}
    source_audit = S.audit_written_support(layers["restricted"], layers["median"], support,
                                           numeric_band_m=band)
    if source_audit.get("passed") is not True:
        raise ValueError("actual-source-support-audit-failed: " + ", ".join(source_audit.get("problems", [])))
    base = _record_geometry(support["source_base"])
    fixed = _record_geometry(support["fixed_support"])
    if support["support_geometry_wkb_hex"].lower() != support["fixed_support"]["wkb_hex"].lower():
        raise ValueError("source-support-domain-mismatch")
    original_holes = _record_geometry(support["original_holes"])
    expected = {kind: _record_geometry(support["expected_" + kind]) for kind in layers}
    pieces = [{"source_lane": p["source_lane"], "road_id": p["road_id"],
               "geometry": _record_geometry(p)} for p in support["source_pieces"]]
    for family in families:
        source_roads = {p["road_id"] for p in pieces if p["source_lane"] in family["source_lane_ids"]}
        known_lanes = {p["source_lane"] for p in pieces if p["road_id"] == family["source_road"]}
        if family["source_road"] is None:
            if family["source_lane_ids"] or family["role"] != "source-polygon":
                raise ValueError("source-family-base-identity-mismatch")
        elif (source_roads != {family["source_road"]} or not family["source_lane_ids"]
              or not set(family["source_lane_ids"]) <= known_lanes):
            raise ValueError("source-family-road-lane-identity-mismatch")
    # Reconstruction's preserved separator is selected by its original intent.
    # This is not the support repair's distinct road-12 fixed scope.
    intent = evidence["intent"]
    matches = [m for m in support["source_mouths"] if m["enter_link"] == intent["enter_link"]
               and m["leave_links"] == intent["leave_links"]]
    if len(matches) != 1:
        raise ValueError("reconstruction-source-scope-not-unique")
    tails = unary_union([f["geometry"] for f in families if f["source_road"] == matches[0]["road_id"]
                         and f["lane_type"] == "restricted"])
    context = {"base": base, "target": matches[0], "tails": tails, "support": fixed,
               "represented_support": unary_union([f["geometry"] for f in families])}
    audit = R.audit_written(root, evidence, context, require_driving_unchanged=False)
    if audit.get("status") != "PASS" or audit.get("material_support_passed") is not True:
        raise ValueError("reconstruction-readback-audit-failed")
    return families, pieces, layers, expected, original_holes, band, audit, snapshot_ids, hashes


def diagnose_surface(root, evidence):
    """Diagnose supported source-track auxiliary geometry; return JSON-safe data.

    DIAGNOSED means this read-only analysis ran, never map acceptance. Unsupported,
    changed or missing evidence returns UNAVAILABLE with no misleading zero count.
    ``paving_holes_gt1cm2`` retains the historical name but its cutoff is 0.01 m².
    Actual resolved gaps have no area cutoff. Minor sampling artifacts are retained
    in ``sampling_details`` instead of multiplying primary UI issues.
    """
    report = {"schema": SCHEMA, "status": "UNAVAILABLE", "issues": [], "layers": [],
        "summary": None, "candidate_accepted": False, "driving_continuity_proven": False,
        "whole_map_score_modified": False, "whole_map_gates_evaluated": False,
        "scope": "source-track auxiliary layers only; restricted and median are not driving proof",
        "source_snapshot_authentication": "caller-must-bind-evidence-to-original-source-snapshot",
        "metric_definition": {"paving_holes_gt1cm2":
            "Historical name; counts sampled holes with area > 0.01 m² (100 cm²).",
            "area_cutoff_m2": EXISTING_AREA_CUTOFF_M2,
            "actual_gap_issue_area_cutoff_m2": 0.0}}
    try:
        families, pieces, layers, expected, original, band, audit, snapshots, hashes = _validate(root, evidence)
        actual = unary_union(list(layers.values()))
        planned = unary_union(list(expected.values()))
        paving = [r for r in root.findall("road") if r.get("name") == "junction_paving"]
        sampled_parts = [smoothness.road_surface_polygon(road) for road in paving]
        if any(p.is_empty or not p.is_valid for p in sampled_parts):
            raise ValueError("legacy-sampled-surface-empty-or-invalid")
        sampled = unary_union(sampled_parts)
        if not sampled.is_valid:
            raise ValueError("legacy-sampled-surface-invalid")
        actual_holes, sampled_holes = _holes(actual), _holes(sampled)
        issues, resolution_details, sampling_details = [], [], []
        for index, hole in enumerate(actual_holes):
            if _covered(hole, actual, band):
                resolution_details.append({"area_m2": hole.area, "geometry": mapping(hole),
                                           "classification": "within-derived-serialization-band"})
                continue
            for kind, part in _partition_gap(hole, original, planned, band, actual=actual):
                issues.append(_issue(kind, part, families, pieces, band,
                    actual_hole_index=index, actual_hole_geometry=mapping(hole)))
        actual_hole_union = unary_union(actual_holes)
        for index, hole in enumerate(sampled_holes):
            covered = _covered(hole, actual, band)
            numerical = covered and _covered(hole, sampled, band)
            # A sampled hole may contain both a real source hole and sampling
            # error at its border. Retain both polygons, never relabel the whole.
            overlap = hole.intersection(actual_hole_union)
            artifact = hole.intersection(actual)
            residual = hole.difference(unary_union([actual, actual_hole_union]).buffer(band))
            row = {"sampled_hole_index": index, "area_m2": hole.area, "geometry": mapping(hole),
                "above_existing_area_cutoff": hole.area > EXISTING_AREA_CUTOFF_M2,
                "entirely_covered_by_actual_with_numeric_band": covered,
                "classification": "within-derived-serialization-band" if numerical else
                    "sampling-artifact" if covered else "mixed-or-actual-gap",
                "actual_gap_overlap_geometry": mapping(overlap),
                "exact_surface_covered_geometry": mapping(artifact),
                "unexplained_geometry": mapping(residual)}
            sampling_details.append(row)
            if covered and not numerical and hole.area > EXISTING_AREA_CUTOFF_M2:
                issues.append(_issue("sampling-artifact", hole, families, pieces, band,
                                     sampled_hole_index=index))
            elif not covered:
                for issue in issues:
                    if "actual_hole_index" in issue and overlap.intersects(
                            actual_holes[issue["actual_hole_index"]]):
                        issue.setdefault("sampling_comparison", []).append(row)
                if hole.area > EXISTING_AREA_CUTOFF_M2:
                    for part in _polygons(residual):
                        issues.append(_issue("unattributed-gap", part, families, pieces, band,
                            sampled_hole_index=index, classification_scope="sampled-hole-outside-exact-surface-and-exact-holes"))
        report.update(status="DIAGNOSED", issues=issues, sampling_details=sampling_details,
            numeric_resolution_details=resolution_details,
            layers=[{"id": prefix + "-" + kind, "title": title + ("受限铺面" if kind == "restricted" else "分隔带"),
                     "lane_type": kind, "geometry": mapping(shape), "driving_surface_proven": False}
                    for prefix, title, group in [("source", "来源期望", expected), ("actual", "实际写出", layers)]
                    for kind, shape in group.items()],
            source_proof={"snapshot_id": next(iter(snapshots)), "content_hash": next(iter(hashes)),
                "support_evidence_sha256": evidence["source_support"]["evidence_sha256"],
                "original_holes_geometry": mapping(original), "family_count": len(families)},
            actual_proof={"readback": "analytic XML line and all paired offset/width breakpoints",
                "numeric_serialization_band_m": band, "reconstruction_audit": audit,
                "auxiliary_road_ids": sorted(r.get("id") for r in paving),
                "whole_map_score_untouched": True},
            summary={"issue_count": len(issues), "by_kind": dict(Counter(i["kind"] for i in issues)),
                "paving_holes_gt1cm2": sum(h.area > EXISTING_AREA_CUTOFF_M2 for h in sampled_holes),
                "exact_holes_gt_existing_0_01m2": sum(h.area > EXISTING_AREA_CUTOFF_M2 for h in actual_holes),
                "exact_holes_including_numeric_roundoff": len(actual_holes),
                "numeric_resolution_holes": len(resolution_details),
                "sampled_holes_including_numeric_roundoff": len(sampled_holes),
                "exact_auxiliary_union_area_m2": actual.area,
                "source_family_union_area_m2": planned.area,
                "exact_vs_source_symdiff_m2": actual.symmetric_difference(planned).area,
                "sampled_vs_exact_symdiff_m2": sampled.symmetric_difference(actual).area})
        # Normalize tuples and reject non-finite values before returning to an API.
        return json.loads(json.dumps(report, ensure_ascii=False, allow_nan=False))
    except (ValueError, KeyError, TypeError, AttributeError, IndexError, GEOSException) as exc:
        report.update(status="UNAVAILABLE", issues=[], layers=[], summary=None,
                      reason=str(exc), error_code="unsupported-or-unverified-surface-evidence")
        return report
