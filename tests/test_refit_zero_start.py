"""Second boundary refit of a road the first refit leaves unchanged for a negative width (2026-10-12). Shaped like
0630 road 13 lane -3: the lane opens from zero width 0.40 m into a free road start; its outer boundary, fitted on its
own, runs on back to the road start and crosses the inner one there (the crossing repair cannot lift a road end)."""
import json

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_zero_start as Z
from mapforge.validate.smoothness import lane_edges_at

XODR = """<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0=29.59 +lon_0=106.30]]></geoReference></header>
<road id="7" length="60" junction="-1"><link/>
<planView><geometry s="0" x="0" y="0" hdg="0" length="60"><line/></geometry></planView>
<lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/>
<laneSection s="0"><center><lane id="0" type="none"/></center><right>
<lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><userData code="mapforge.source_lane" value="20001"/></lane>
<lane id="-2" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><userData code="mapforge.source_lane" value="20002"/></lane>
<lane id="-3" type="driving"><width sOffset="0" a="1.0" b="0" c="0" d="0"/><userData code="mapforge.source_lane" value="20003"/></lane>
</right></laneSection></lanes></road></OpenDRIVE>"""
P = {"mode": "g2", "kappa_cap": 0.04, "seg_min": 6.0, "rdp_tol": 0.05, "c2_ends": True, "short_kappa": True,
     "lsq_refine": True, "protect_unobserved_ends": True}
S = np.arange(0.0, 60.0 + 1e-9, 0.5)
OPEN = 0.4                          # the source lane opens here from zero width, 0.3 /m up to 3.5 m


def _outer(s):
    return -7.0 - np.minimum(0.3 * (s - OPEN), 3.5)


def _boundaries(overlap=0.0):
    """(s, t) source samples of right boundary k. ``overlap``: lane -2 a 0.5 m strip whose outer boundary runs
    inward over s 25-35 to ``overlap`` across its inner one (two source lanes on top of each other mid-road, not a
    free-end problem; 0627 / 0707104 road 10), lane -3 there from the road start."""
    def flat(t):
        return np.column_stack([S, np.full(len(S), t)])
    if overlap:
        push = np.interp(S, [15.0, 25.0, 35.0, 45.0], [0.0, 0.5 + overlap, 0.5 + overlap, 0.0])
        return {0: flat(0.0), 1: flat(-3.5), 2: np.column_stack([S, -4.0 + push]), 3: flat(-7.5)}
    out = {k: flat(-3.5 * k) for k in range(3)}
    s = np.concatenate([[OPEN], S[S > OPEN]])
    out[3] = np.column_stack([s, _outer(s)])
    return out


@pytest.fixture
def source(monkeypatch):
    def use(overlap=0.0):
        bounds = _boundaries(overlap)
        monkeypatch.setattr(LR, "_source_observations",
                            lambda road, src, origin, side, i, k: bounds[k] if side == "right" else None)
        return etree.fromstring(XODR.encode())
    return use


def test_the_first_refit_crosses_at_the_free_road_start(source):
    root = source()
    road, fitted, report = LR.refit_road(root.find("road"), None, (29.59, 106.30), P)
    (side, i, k, low), = Z.negative_lanes(road, fitted)
    assert (side, i, k) == ("right", 0, 3) and -0.15 < low < -LR.NEGATIVE_WIDTH_TOL
    assert not LR.write_road(road, fitted, report)          # a road end: the repair bump is zero there


def test_the_retry_holds_the_lane_at_zero_width_from_the_road_start_and_is_written(source):
    root = source()
    report = LR.refit_tree(root, None, P)
    assert report["rewritten"] == 1 and report["skipped"] == []
    road = root.find("road")
    row = json.loads(road.find(f"userData[@code='{LR.CODE}']").get("value"))
    (lane,) = row["zero_start_retry"]["lanes"]
    assert (lane["side"], lane["section"], lane["lane"]) == ("right", 0, 3) and lane["min_width_m"] < 0
    assert row["zero_start_retry"]["first_refit"].startswith("negative width")
    assert row["advanced_events"] == 1
    widths = [lane_edges_at(road, s, "right")[2] - lane_edges_at(road, s, "right")[3] for s in np.arange(0, 60, 0.1)]
    assert min(widths) >= -1e-9
    assert widths[0] == pytest.approx(0.0, abs=1e-9)
    # inner boundaries unchanged, the outer one on its source within the corner rounding of the curvature target
    for s in np.arange(0.0, 60.0, 0.5):
        e = lane_edges_at(road, s, "right")
        assert e[:3] == pytest.approx([0.0, -3.5, -7.0], abs=1e-9)
        assert abs(e[3] - (_outer(s) if s >= OPEN else -7.0)) < 0.35


def test_a_crossing_that_is_not_at_a_free_end_stays_skipped_with_a_note(source):
    root = source(overlap=0.5)
    before = etree.tostring(root)
    report = LR.refit_tree(root, None, P)
    assert report["rewritten"] == 0
    (entry,) = report["skipped"]
    assert entry["reason"].startswith("negative width")
    assert [(x["side"], x["lane"]) for x in entry["zero_start_retry"]["lanes"]] == [("right", 2)]
    assert entry["zero_start_retry"]["lanes"][0]["min_width_m"] < -LR.REPAIR_MAX_M
    assert entry["zero_start_retry"]["result"] == entry["reason"]       # nothing at a free end to hold at zero
    assert etree.tostring(root) == before                    # road unchanged


def test_no_retry_without_the_shp_boundary_step(source):
    root = source()
    before = etree.tostring(root)
    plain = {k: v for k, v in P.items() if k != "protect_unobserved_ends"}
    report = LR.refit_tree(root, None, plain)
    assert report["rewritten"] == 0
    assert report["skipped"] == [{"road": "7", "reason": report["skipped"][0]["reason"]}]
    assert etree.tostring(root) == before


def test_the_unobserved_minimum_still_holds_for_lanes_of_a_first_refit(source):
    root = source()
    road, fitted, report = LR.refit_road(root.find("road"), None, (29.59, 106.30), P)
    assert "advanced_events" not in report                  # 0.4 m < UNOBSERVED_END_M: no zero start
    road, fitted, report = LR.refit_road(root.find("road"), None, (29.59, 106.30),
                                         {**P, "zero_start_lanes": {("right", 0, 3)}})
    assert report["advanced_events"] == 1 and Z.negative_lanes(road, fitted) == []
