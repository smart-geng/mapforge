"""Merge reference-line primitives shorter than MIN_SEG_M on ordinary roads (T2 leg_seg_min).

A few converter reference lines carry one short primitive (node18 road 10 after the moved mouth:
3.97 m; MAP node13 road 13: 4.88 m). The short primitive and its longer neighbour are refitted as
one span with the converter's own few-segment G2 fitter: both span end poses and curvatures are kept
(so the rest of the road and every road/junction contact stay as they are), every new primitive is
at least MIN_SEG_M, and the new reference may deviate from the old one by at most MERGE_TOL. Lanes
keep their records; the road length changes by the span's length change (reported, millimetres).
"""
from __future__ import annotations

import math

import numpy as np

CODE = "mapforge.refline_merge/v1"
MIN_SEG_M = 5.0
MERGE_TOL = {"endpoint_tol": 0.005, "median_tol": 0.01, "p95_tol": 0.03, "max_dev_tol": 0.05}


def _span_samples(road, s_lo, s_hi, step=0.1):
    from mapforge.validate.smoothness import sample_road_ref
    pts, ss, hh = sample_road_ref(road, step)
    keep = (ss >= s_lo - 1e-9) & (ss <= s_hi + 1e-9)
    return pts[keep], ss[keep], hh[keep]


def merge_road(road) -> list[dict]:
    """Merge every short primitive of one road; returns one record per merge (or a failure record)."""
    from mapforge.ops.mouth_frame_align import _write_planview
    from mapforge.ops.refline_fit import fit_connector_minimal
    from mapforge.validate.smoothness import _geoms
    records = []
    for _ in range(10):
        geoms = _geoms(road)
        short = [i for i, g in enumerate(geoms) if g[4] < MIN_SEG_M - 1e-9]
        if not short or len(geoms) < 2:
            break
        i = short[0]
        j = i + 1 if i + 1 < len(geoms) else i - 1
        lo, hi = min(i, j), max(i, j)
        s_lo = sum(g[4] for g in geoms[:lo])
        s_hi = s_lo + sum(g[4] for g in geoms[lo:hi + 1])
        pts, ss, hh = _span_samples(road, s_lo, s_hi)
        p0 = (geoms[lo][1], geoms[lo][2], geoms[lo][3])
        p1 = (float(pts[-1, 0]), float(pts[-1, 1]), float(hh[-1]))
        fit = fit_connector_minimal(pts, p0, p1, k0=geoms[lo][5], k1=geoms[hi][6], max_segments=3, max_kappa=0.25,
                                    sharpness_cap=0.20, min_segment_m=MIN_SEG_M, **MERGE_TOL)
        if fit is None:
            records.append({"road": road.get("id"), "span": [round(s_lo, 3), round(s_hi, 3)],
                            "old_lengths_m": [round(g[4], 3) for g in geoms[lo:hi + 1]], "status": "NOT_MERGED"})
            break
        new = list(geoms[:lo]) + [tuple(p) for p in fit.primitives] + list(geoms[hi + 1:])
        old_length = float(road.get("length"))
        _write_planview(road, new)
        records.append({"road": road.get("id"), "span": [round(s_lo, 3), round(s_hi, 3)],
                        "old_lengths_m": [round(g[4], 3) for g in geoms[lo:hi + 1]],
                        "new_lengths_m": [round(p[4], 3) for p in fit.primitives],
                        "max_deviation_m": round(float(fit.metrics["source_to_target"]["max_m"]), 4),
                        "length_change_m": round(float(road.get("length")) - old_length, 5),
                        "status": "MERGED"})
    return records


def merge_tree(root) -> dict:
    rows = []
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1") or road.get("name") == "junction_paving":
            continue
        rows += merge_road(road)
    return {"schema": CODE, "merged": sum(r["status"] == "MERGED" for r in rows), "rows": rows}
