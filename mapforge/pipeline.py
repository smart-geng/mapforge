"""Default XODR pipeline: generate, post-process, run the gates (2026-10-03).

* SHP: junction mouths moved before the earliest real lane ends (shp_mouth_envelope, 3 m) with the
  gap-aware surface rebuild; separate departure carriageways started 3 m past their via joints and the
  curb-return flare of their lanes (shp_leave_mouth, 2026-10-05); then the post-process variant; the comparison
  windows of the source manifest are then cut at the written road ends (window_align, 2026-10-05), and road
  lanes whose SHP centreline is off its boundaries are compared on the boundary midpoint (window_midpoint).
* MAP XML: approved source corrections (profiles/source-corrections) applied to a derived copy, the
  converter's XODR, then the same post-process variant (lane refit is a no-op for MAP; the mouth
  alignment applies).
* Post-process (lane_refit.VARIANTS[DEFAULT_VARIANT] = "g2-k04-c2"): reference primitives < 5 m merged;
  SHP road-side boundaries refitted to the source with every boundary C2 (G2 corners, end blends,
  births/deaths, repair bumps; inner-side births/deaths re-chained); curb-return shoulders; connector
  mouth alignment with spread / local / source-guided candidates, exact edge headings at the mouths
  (local width-slope corrections) and no lane-centre curvature jump above 1e-3 /m.

The gates (G8, G11, edge contacts, MAP source integrity) always run on the written result, with the
generator's source manifest, exactly as the CLI did before. The legacy paths stay available
(``mapforge.cli convert --shp-mouth legacy --post none``) to replay old evidence.
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
G8_POLICY = ROOT / "profiles" / "validation" / "g8-opendrive-jinfeng-v1.yaml"
CODE = "mapforge.pipeline/v1"
DEFAULT_VARIANT = "g2-k04-c2"
SHP_MOUTH_MARGIN_M = 3.0


def variant_name(post: str | None):
    """CLI value -> lane_refit variant name (None: no post-processing)."""
    from mapforge.ops.lane_refit import VARIANTS
    if post in (None, "", "none"):
        return None
    for name in (post, f"g2-k04-{post}"):
        if name in VARIANTS:
            return name
    raise ValueError(f"unknown post-process variant: {post} (choose from {sorted(VARIANTS)} or none)")


def _dump(path: Path, obj) -> None:
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=1, default=str), encoding="utf-8")


def _drop(raw: Path) -> None:
    """Remove the intermediate <stem>.raw.* files of one conversion."""
    for p in raw.parent.glob(raw.stem + ".*"):
        p.unlink()


def postprocess(raw_xodr, out_xodr, manifest: dict, variant: str = DEFAULT_VARIANT, raw_map_paths=None,
                align_windows: bool = False, midpoint_source=None) -> dict:
    """Apply a registered post-process variant to a generated XODR and run the gates on the result.
    ``align_windows`` (SHP): the comparison windows are cut at the written road ends before the gates
    (mapforge.ops.window_align). ``midpoint_source`` (SHP: (shp_dir, profile)): road lanes whose source centreline
    is off its boundaries are compared on the boundary midpoint (mapforge.ops.window_midpoint)."""
    from mapforge.ops import lane_refit
    from mapforge.report.decision import finalize_opendrive_g8
    raw_xodr, out_xodr = Path(raw_xodr), Path(out_xodr)
    _dump(raw_xodr.with_suffix(".source-lanes.json"), manifest)
    report = lane_refit.apply_with_mouths(raw_xodr, out_xodr, lane_refit.VARIANTS[variant])
    if align_windows:
        import xml.etree.ElementTree as ET
        from mapforge.ops import window_align
        report["window_align"] = window_align.apply(ET.parse(str(out_xodr)).getroot(), manifest)
    if midpoint_source is not None:
        from mapforge.adapters.shp.profile_source import ProfileSource
        from mapforge.ops import window_midpoint
        report["window_midpoint"] = window_midpoint.apply(manifest, ProfileSource(str(midpoint_source[0]),
                                                                                   midpoint_source[1]))
    final = finalize_opendrive_g8(out_xodr, manifest, G8_POLICY, connect_mode="data", raw_map_paths=raw_map_paths)
    _dump(out_xodr.with_suffix(".transform.json"), {"schema": CODE, "variant": variant, **report})
    return final


def convert_shp(shp_dir, out, *, like=None, at=None, margin_m: float = SHP_MOUTH_MARGIN_M,
                variant: str | None = DEFAULT_VARIANT, profile: str = "ibd-smarteditor-v1",
                leave_mouths: bool = True, align_windows: bool = True, midpoint_windows: bool = True) -> dict:
    """SHP -> XODR along the default path; returns the gate result (plus the envelope record)."""
    from mapforge.ops import shp_mouth_envelope
    out = Path(out)
    if variant is None:
        record = shp_mouth_envelope.convert(shp_dir, like, out, margin_m, profile, at=at)
        manifest_path = out.with_suffix(".source-lanes.json")
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            _dump(out.with_suffix(".source-review.json"), _shp_source_review(shp_dir, profile, manifest, out))
        return {"record": record}
    raw = out.with_name(out.stem + ".raw.xodr")
    record = shp_mouth_envelope.convert(shp_dir, like, raw, margin_m, profile, finalize=False, at=at)
    manifest = json.loads(raw.with_suffix(".source-lanes.json").read_text(encoding="utf-8"))
    if leave_mouths:
        # departure carriageways start 3 m past their via joints and curb-return flare (mapforge.ops.shp_leave_mouth)
        from mapforge.ops import shp_leave_mouth
        if like is not None:
            from mapforge.adapters.v2xmap.xml_reader import parse_map_xml
            ref = parse_map_xml(str(like))
            locate = (ref.ref_lon, ref.ref_lat)
        else:
            locate = at
        record["leave_mouth"] = shp_leave_mouth.apply_file(raw, manifest, shp_dir, profile, locate,
                                                           record["stats"].get("mouth_envelope_decisions", []))
    # comparison windows cut at the written road ends (window_align) and, for road lanes whose source centreline is
    # off its boundaries, on the boundary midpoint (window_midpoint); user decisions 2026-10-05
    final = postprocess(raw, out, manifest, variant, align_windows=align_windows,
                        midpoint_source=(shp_dir, profile) if midpoint_windows else None)
    _dump(out.with_suffix(".mouth-envelope.json"), record)
    _dump(out.with_suffix(".source-review.json"), _shp_source_review(shp_dir, profile, manifest, out))
    _drop(raw)
    return {"record": record, **final}


def _shp_source_review(shp_dir, profile, manifest, xodr=None) -> dict:
    """Read-only SHP source consistency review of the lanes this conversion used (shp_source_review): where the
    SHP centreline, the next lane object or the lane placed alongside contradicts the boundaries. Recorded under
    the boundary-priority decision (2026-10-04), nothing corrected."""
    from mapforge.validate import shp_source_review
    return shp_source_review.review_manifest(manifest, shp_dir, profile, xodr=xodr)


def _source_review(xml_path, decisions) -> dict:
    """Read-only point-list review of the original MAP source; each finding names the applied decision covering
    it, if any. An open finding is a suspected source error nobody has decided on (reported, not blocking)."""
    from mapforge.validate import map_source_review
    review = map_source_review.review_file(xml_path)
    covered = {}
    for decision in decisions:
        for c in decision["corrections"]:
            t = c["target"]
            covered[(t["link"]["name"], t["link"]["upstream"]["region"], t["link"]["upstream"]["id"], t["lane"],
                     t["point_index_0based"], t["raw"]["lon"], t["raw"]["lat"])] = decision["id"]
    for f in review["findings"]:
        t = f["target"]
        f["decision"] = covered.get((t["link"]["name"], t["link"]["upstream"]["region"], t["link"]["upstream"]["id"],
                                     t["lane"], t["point_index_0based"], t["raw"]["lon"], t["raw"]["lat"]))
    review["open"] = sum(1 for f in review["findings"] if not f["decision"])
    return review


def corrected_map_source(xml_path, out) -> tuple[Path, dict | None]:
    """Approved MAP source corrections: derived XML next to ``out`` and a DROPPED loss report.

    Always writes ``<out>.source-review.json`` too (``_source_review``), so a new suspicious point shows up in
    every conversion instead of being fitted silently."""
    from mapforge.ops import map_source_corrections as corrections
    xml_path, out = Path(xml_path), Path(out)
    decisions = corrections.applicable(xml_path)
    _dump(out.with_suffix(".source-review.json"), _source_review(xml_path, decisions))
    if not decisions:
        return xml_path, None
    derived = out.with_name(out.stem + ".source-corrected.xml")
    record = corrections.apply(xml_path, decisions, derived)
    _dump(out.with_suffix(".source-corrections.json"), record)
    corrections.write_loss_report(record, out.with_suffix(".loss-report.json"))
    return derived, record
