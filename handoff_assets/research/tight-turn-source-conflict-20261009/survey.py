"""Read-only survey for the user decision of 2026-10-09: tight-turn connector source vias are a source conflict.

For every SHP case with G8 output (7 scoreboard cases, the generalization cases, the frozen 0621 candidate) this
measures how sharply each comparable junction-via source window turns, and what the lane-centre statistics
would be if connector lanes whose source turns tighter than a candidate radius were left out on top of the
existing mouth-curb-flare zones. Nothing is changed: no geometry, policy, threshold or scoreboard output. The
candidate radii are for the user's choice; none is adopted here.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
from lxml import etree
from scipy.spatial import cKDTree

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT))
from mapforge.validate import lane_centre_flare as LCF  # noqa: E402
from mapforge.validate.lane_fidelity import _resample, extract_target_components  # noqa: E402

RADII = (4.0, 5.0, 6.0, 8.0, 10.0)
WINDOW_M = 3.0


def cases():
    for base in ("out/scoreboard/20261008-safe-mouth-recovery-v3", "out/generalize/20261008-safe-mouth-recovery-v3"):
        for manifest in sorted((ROOT / base).glob("shp-*.source-lanes.json")):
            stem = manifest.name[: -len(".source-lanes.json")]
            xodr, review = manifest.with_name(stem + ".xodr"), manifest.with_name(stem + ".source-review.json")
            if xodr.exists() and review.exists() and manifest.with_name(stem + ".g8.json").exists():
                yield base.split("/")[1] + "/" + stem, xodr, manifest, review
    w = ROOT / "out/workbench/wb11-source-tracks-20261009-v2"
    yield "0621/candidate", w / "candidate.xodr", w / "candidate.source-lanes.json", w / "candidate.source-review.json"


def turn_shape(coords):
    p = np.asarray(coords, float)[:, :2]
    seg = np.diff(p, axis=0)
    keep = np.hypot(*seg.T) > 1e-9
    seg = seg[keep]
    length = np.hypot(*seg.T)
    s = np.concatenate([[0.0], np.cumsum(length)])
    heading = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    mid = (s[:-1] + s[1:]) / 2
    if s[-1] <= WINDOW_M:
        k3 = abs(heading[-1] - heading[0]) / max(s[-1], 1e-9)
    else:
        probes = np.linspace(WINDOW_M / 2, s[-1] - WINDOW_M / 2, max(2, int(s[-1] / 0.1)))
        k3 = float(np.max(np.abs(np.interp(probes + WINDOW_M / 2, mid, heading)
                                 - np.interp(probes - WINDOW_M / 2, mid, heading)) / WINDOW_M))
    turns = np.degrees(np.abs(np.diff(heading))) if len(heading) > 1 else np.zeros(1)
    return {"length_m": float(s[-1]), "total_turn_deg": float(np.degrees(heading[-1] - heading[0])),
            "kappa_3m_max": float(k3), "radius_3m_min_m": float(1 / k3) if k3 > 0 else None,
            "vertex_turn_max_deg": float(turns.max())}


def lane_samples(root, manifest, review):
    """Per comparable lane: (source-to-target, target-to-source) nearest-sample distances with the flare zones
    already left out, exactly as lane_centre_flare.distances does for the pooled numbers."""
    sources = {lane["source_lane_id"]: lane for lane in manifest.get("lanes", [])
               if (lane.get("comparison") or {}).get("eligible")}
    comps = {}
    for comp in extract_target_components(root, ds=1.0, zero_width_epsilon_m=0.05)["components"]:
        comps.setdefault(comp["source_lane_id"], []).append(comp)
    flared = LCF.zones(review)
    out = {}
    for sid in sorted(set(sources) & set(comps)):
        if len(comps[sid]) != 1:
            continue
        src = _resample((sources[sid].get("geometry") or {}).get("coordinates", []), 1.0)
        tgt = _resample(comps[sid][0]["points"], 1.0)
        if len(src) < 2 or len(tgt) < 2:
            continue
        s2t, t2s = cKDTree(tgt).query(src)[0], cKDTree(src).query(tgt)[0]
        if sid in flared:
            mouths = np.asarray(flared[sid], float)
            s2t = s2t[np.linalg.norm(src[:, None, :2] - mouths[None], axis=2).min(axis=1) > LCF.FLARE_ZONE_M]
            t2s = t2s[np.linalg.norm(tgt[:, None, :2] - mouths[None], axis=2).min(axis=1) > LCF.FLARE_ZONE_M]
        out[sid] = (s2t, t2s, comps[sid][0].get("road_id"))
    return out


def pooled(samples, leave_out=()):
    s = [v[0] for k, v in samples.items() if k not in leave_out]
    t = [v[1] for k, v in samples.items() if k not in leave_out]
    s, t = np.concatenate(s), np.concatenate(t)
    return {"p95_m": float(max(np.percentile(s, 95), np.percentile(t, 95))),
            "max_m": float(max(s.max(), t.max())), "samples": int(len(s) + len(t))}


def main():
    rows, lanes = [], []
    for name, xodr, manifest_path, review_path in cases():
        root = etree.parse(str(xodr)).getroot()
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        review = json.loads(review_path.read_text(encoding="utf-8"))
        samples = lane_samples(root, manifest, review)
        reference = LCF.audit(root, manifest, review)
        base = pooled(samples)
        assert abs(base["p95_m"] - reference["lane_center_noflare_p95_m"]) < 1e-12, (name, base, reference)
        assert abs(base["max_m"] - reference["lane_center_noflare_max_m"]) < 1e-12, (name, base, reference)
        vias = {}
        for lane in manifest["lanes"]:
            sid = lane["source_lane_id"]
            if lane.get("role") != "junction-via" or sid not in samples:
                continue
            shape = turn_shape(lane["geometry"]["coordinates"])
            s2t, t2s, road = samples[sid]
            worst = np.concatenate([s2t, t2s])
            vias[sid] = shape
            lanes.append({"case": name, "source_lane_id": sid, "road": road, **shape,
                          "support_reason": (lane.get("support") or {}).get("reason"),
                          "lane_p95_m": float(max(np.percentile(s2t, 95), np.percentile(t2s, 95))),
                          "lane_max_m": float(worst.max()), "samples_above_015": int((t2s > 0.15).sum())})
        by_radius = {}
        for radius in RADII:
            flagged = sorted(sid for sid, shape in vias.items()
                             if shape["radius_3m_min_m"] is not None and shape["radius_3m_min_m"] < radius)
            by_radius[str(radius)] = {"flagged_lanes": len(flagged), **pooled(samples, flagged)}
        rows.append({"case": name, "comparable_lanes": len(samples), "junction_via_lanes": len(vias),
                     "current_noflare": base, "if_tight_turns_left_out": by_radius})
        print(json.dumps({"case": name, "vias": len(vias), "p95": round(base["p95_m"], 4),
                          **{r: (v["flagged_lanes"], round(v["p95_m"], 4)) for r, v in by_radius.items()}}), flush=True)
    summary = {"schema": "mapforge/research/tight-turn-source-conflict-survey/v1", "window_m": WINDOW_M,
               "candidate_radii_m": list(RADII), "cases": rows, "junction_via_lanes": lanes,
               "reproduces_lane_center_noflare": True, "policy_changed": False, "geometry_changed": False}
    (OUT / "survey.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf8")


if __name__ == "__main__":
    main()
