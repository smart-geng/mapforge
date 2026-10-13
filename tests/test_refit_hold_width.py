"""Third boundary refit of a road both earlier refits leave unchanged for a negative width (2026-10-13). Shaped like
0627 road 10: a free road start where the median's outer boundary and the departure lane beyond it have no source for
the first 20 m; the centre boundary, observed there, drifts 0.8 m toward the median over the first 12 m. Carried on
from where its source starts, the median's outer edge crosses the centre boundary at the road start; the median is
0.5 m wide where its source starts, so there is no zero-width start to hold."""
import json

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_hold_width as H
from mapforge.ops import refit_zero_start as Z
from mapforge.validate.smoothness import lane_edges_at

XODR = """<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0=29.59 +lon_0=106.30]]></geoReference></header>
<road id="7" length="60" junction="-1"><link/>
<planView><geometry s="0" x="0" y="0" hdg="0" length="60"><line/></geometry></planView>
<lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/>
<laneSection s="0"><left>
<lane id="2" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><userData code="mapforge.source_lane" value="20002"/></lane>
<lane id="1" type="median"><width sOffset="0" a="0.5" b="0" c="0" d="0"/></lane>
</left><center><lane id="0" type="none"/></center><right>
<lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><userData code="mapforge.source_lane" value="20001"/></lane>
</right></laneSection></lanes></road></OpenDRIVE>"""
P = {"mode": "g2", "kappa_cap": 0.04, "seg_min": 6.0, "rdp_tol": 0.05, "c2_ends": True, "short_kappa": True,
     "lsq_refine": True, "protect_unobserved_ends": True}
S = np.arange(0.0, 60.0 + 1e-9, 0.5)
FIRST = 20.0                        # the median and the departure lane have source from here on
DRIFT = 12.0                        # the centre boundary drifts over the first DRIFT metres
MEDIAN = 0.5


def _centre(s):
    return np.where(s < DRIFT, 0.8 * (1.0 - s / DRIFT), 0.0)


def _boundaries(at_end=False, crossing_mid_road=False):
    """(side, k) -> (s, t) source samples. ``at_end``: mirrored, the source of the median and the departure lane stops
    at 60 - FIRST and the centre drifts over the last DRIFT metres (a free road end). ``crossing_mid_road``: source
    everywhere, the median's outer boundary pushed across the centre over s 25-35 (two source lanes on top of each
    other, 0707104 road 10)."""
    s = 60.0 - S if at_end else S
    centre = _centre(s)
    seen = S[(60.0 - S if at_end else S) >= FIRST]
    if crossing_mid_road:
        seen = S
    median = np.full(len(seen), MEDIAN)
    if crossing_mid_road:
        median = MEDIAN - np.interp(seen, [15.0, 25.0, 35.0, 45.0], [0.0, MEDIAN + 0.4, MEDIAN + 0.4, 0.0])
    return {("right", 0): np.column_stack([S, centre]), ("left", 0): np.column_stack([S, centre]),
            ("right", 1): np.column_stack([S, centre - 3.5]),
            ("left", 1): np.column_stack([seen, median]),
            ("left", 2): np.column_stack([seen, np.full(len(seen), MEDIAN + 3.5)])}


@pytest.fixture
def source(monkeypatch):
    def use(**kw):
        bounds = _boundaries(**kw)
        monkeypatch.setattr(LR, "_source_observations",
                            lambda road, src, origin, side, i, k: bounds.get((side, k)))
        return etree.fromstring(XODR.encode())
    return use


def _widths(road, s):
    left = lane_edges_at(road, s, "left")       # centre, median outer, departure outer
    return left[1] - left[0], left[2] - left[1]


def test_the_extrapolated_median_edge_crosses_and_the_zero_start_retry_cannot_help(source):
    root = source()
    road, fitted, report = LR.refit_road(root.find("road"), None, (29.59, 106.30), P)
    (side, i, k, low), = Z.negative_lanes(road, fitted)
    assert (side, i, k) == ("left", 0, 1) and low == pytest.approx(MEDIAN - 0.8, abs=0.05)
    assert not LR.write_road(road, fitted, report)
    again, note = Z.retry(root.find("road"), None, (29.59, 106.30), P, road, fitted, {**report, "params": P})
    assert again is None and note["result"].startswith("negative width")      # 0.5 m wide: no zero start


def test_the_hold_retry_keeps_the_median_width_over_the_start_without_source(source):
    root = source()
    report = LR.refit_tree(root, None, P)
    assert report["rewritten"] == 1 and report["skipped"] == []
    road = root.find("road")
    row = json.loads(road.find(f"userData[@code='{LR.CODE}']").get("value"))
    (lane,) = row["hold_width_retry"]["lanes"]
    assert (lane["side"], lane["section"], lane["lane"]) == ("left", 0, 1) and lane["min_width_m"] < 0
    assert row["hold_width_retry"]["first_refit"].startswith("negative width")
    assert row["hold_width_retry"]["zero_start_retry"].startswith("negative width")
    (held,) = row["held_widths"]
    assert (held["side"], held["lane"], held["kind"]) == ("left", 1, "birth")
    assert held["s"] == pytest.approx(FIRST, abs=0.01) and held["width_m"] == pytest.approx(MEDIAN, abs=0.01)
    for s in np.arange(0.0, 60.0, 0.25):
        median, departure = _widths(road, s)
        assert median >= -1e-9 and departure > 2.5
        centre = lane_edges_at(road, s, "left")[0]
        assert abs(centre - float(_centre(np.array(s)))) < 0.1      # the centre boundary keeps its own fit
        if s <= FIRST:
            assert median == pytest.approx(MEDIAN, abs=0.01)          # held, not carried on as a position
        else:
            assert lane_edges_at(road, s, "left")[1] == pytest.approx(MEDIAN, abs=0.1)   # on its source


def test_a_free_road_end_is_held_the_same_way(source):
    root = source(at_end=True)
    report = LR.refit_tree(root, None, P)
    assert report["rewritten"] == 1 and report["skipped"] == []
    road = root.find("road")
    (held,) = json.loads(road.find(f"userData[@code='{LR.CODE}']").get("value"))["held_widths"]
    assert held["kind"] == "death" and held["s"] == pytest.approx(60.0 - FIRST, abs=0.01)
    for s in np.arange(60.0 - FIRST, 60.0, 0.25):
        assert _widths(road, s)[0] == pytest.approx(MEDIAN, abs=0.01)


def test_a_crossing_with_source_on_both_sides_stays_skipped_with_a_note(source):
    root = source(crossing_mid_road=True)
    before = etree.tostring(root)
    report = LR.refit_tree(root, None, P)
    assert report["rewritten"] == 0
    (entry,) = report["skipped"]
    assert entry["reason"].startswith("negative width")
    assert entry["zero_start_retry"]["result"] == entry["reason"]
    assert entry["hold_width_retry"] == {"held": [], "result": "no free end without source"}
    assert etree.tostring(root) == before                    # road unchanged


def test_no_hold_retry_without_the_shp_boundary_step_or_a_failed_zero_start_retry(source):
    root = source()
    plain = {k: v for k, v in P.items() if k != "protect_unobserved_ends"}
    assert H.retry(root.find("road"), None, (29.59, 106.30), plain, {"params": plain},
                   {"lanes": [{"side": "left", "section": 0, "lane": 1, "min_width_m": -0.3}]}) == (None, None)
    assert H.retry(root.find("road"), None, (29.59, 106.30), P, {"params": P}, None) == (None, None)
    before = etree.tostring(root)
    report = LR.refit_tree(root, None, plain)
    assert report["rewritten"] == 0 and set(report["skipped"][0]) == {"road", "reason"}
    assert etree.tostring(root) == before


def test_roads_written_by_their_first_refit_are_not_held(source, monkeypatch):
    root = source()
    bounds = _boundaries()
    bounds[("left", 0)] = bounds[("right", 0)] = np.column_stack([S, np.zeros(len(S))])     # no drift: no crossing
    bounds[("right", 1)] = np.column_stack([S, np.full(len(S), -3.5)])
    monkeypatch.setattr(LR, "_source_observations", lambda road, src, origin, side, i, k: bounds.get((side, k)))
    report = LR.refit_tree(root, None, P)
    assert report["rewritten"] == 1
    row = json.loads(root.find(f"road/userData[@code='{LR.CODE}']").get("value"))
    assert "held_widths" not in row and "hold_width_retry" not in row


def test_holds_needs_more_than_a_grid_step_without_source(source):
    root = source()
    road, fitted, _ = LR.refit_road(root.find("road"), None, (29.59, 106.30), P)
    chain = next(c for c in LR._chains(road, "left") if c["members"] == {0: 1})
    obs = _boundaries()[("left", 1)]
    assert set(H.holds(road, "left", chain, obs, fitted, None, None, 0.0, 60.0)) == {"birth"}
    near = obs[obs[:, 0] >= FIRST]
    near = np.vstack([[LR.GRID * 0.5, MEDIAN], near])
    assert H.holds(road, "left", chain, near, fitted, None, None, 0.0, 60.0) == {}
