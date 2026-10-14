"""Lane swaps fitted together (2026-10-14). Shaped like generalization 0711 road 11 ("inner-birth": the inner lane opens
from zero width while the outer one closes) and 0411 road 13 ("outer-birth": the outer lane opens on the inner one,
which closes): two lanes on the right side swap between the source events at s 28 and 32 and the outer edge runs on at
-7 m. Each width fitted on its own, the opening and the closing do not add up and the outer edge dents; fitted
together, the outer edge stays on its source and the divider carries the swap."""
import json

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_lane_swap as W
from mapforge.ops.lane_birth_advance import CODE as ADVANCE_CODE
from mapforge.validate.smoothness import lane_edges_at

P = {"mode": "g2", "kappa_cap": 0.04, "seg_min": 6.0, "rdp_tol": 0.05, "c2_ends": True, "short_kappa": True,
     "lsq_refine": True, "rechain": True, "protect_unobserved_ends": True}
S_B, S_D = 28.0, 32.0                # source events: the opening lane's birth, the closing lane's death
ADVANCE = 8.0                        # lane_birth_advance: zero-width extensions over whole sections, as in 0711 / 0411
LENGTH = 60.0
EDGE = -7.0
ORIGIN = (29.59, 106.30)


def _secs(s_b=S_B, s_d=S_D):
    return [0.0, s_b - ADVANCE, s_b, s_d, s_d + ADVANCE, LENGTH]


def _lane(lid, width, src, pred=None, mark=None):
    link = f'<link><predecessor id="{pred}"/></link>' if pred is not None else "<link/>"
    data = f"<userData code='{ADVANCE_CODE}' value='{json.dumps(mark)}'/>" if mark else ""
    return (f'<lane id="{lid}" type="driving">{link}<width sOffset="0" a="{width}" b="0" c="0" d="0"/>'
            f'<userData code="mapforge.source_lane" value="{src}"/>{data}</lane>')


def _xodr(kind, s_b=S_B, s_d=S_D):
    """Lane A (inner), and X (outer) and Y (inner) swapping: "inner" = Y born inside X, which dies; "outer" = X born
    outside Y, which dies. The opening lane exists from ADVANCE before its birth with zero width, the closing one up
    to ADVANCE after its death (lane_birth_advance), each over sections of their own as in the source data."""
    born = {"source_birth_s": s_b, "advance_m": ADVANCE}
    dies = {"source_death_s": s_d, "advance_m": ADVANCE}
    a = [_lane(-1, 3.5, "A")] + [_lane(-1, 3.5, "A", -1)] * 4
    if kind == "inner":
        rest = [[_lane(-2, 3.5, "X")],
                [_lane(-2, 0.0, "Y", None, born), _lane(-3, 3.5, "X", -2)],
                [_lane(-2, 1.0, "Y", -2), _lane(-3, 2.5, "X", -3)],
                [_lane(-2, 3.5, "Y", -2), _lane(-3, 0.0, "X", -3, dies)],
                [_lane(-2, 3.5, "Y", -2)]]
    else:
        rest = [[_lane(-2, 3.5, "Y")],
                [_lane(-2, 3.5, "Y", -2), _lane(-3, 0.0, "X", None, born)],
                [_lane(-2, 2.5, "Y", -2), _lane(-3, 1.0, "X", -3)],
                [_lane(-2, 0.0, "Y", -2, dies), _lane(-3, 3.5, "X", -3)],
                [_lane(-2, 3.5, "X", -3)]]
    secs = "".join(f'<laneSection s="{s}"><center><lane id="0" type="none"/></center><right>{x}{"".join(ls)}</right>'
                   f'</laneSection>' for s, x, ls in zip(_secs(s_b, s_d), a, rest))
    return f"""<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0=29.59 +lon_0=106.30]]></geoReference></header>
<road id="7" length="{LENGTH}" junction="-1"><link/>
<planView><geometry s="0" x="0" y="0" hdg="0" length="{LENGTH}"><line/></geometry></planView>
<lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/>{secs}</lanes></road></OpenDRIVE>"""


def _boundaries(kind, s_b=S_B, s_d=S_D, y_width=3.5):
    """(section, k) -> (s, t) source samples of right boundary k. The divider runs straight across between the events;
    ``y_width`` (inner-birth): the opening lane's full width, the outer edge stepping in after the swap below 3.5.
    The closing (opening) lane's outer boundary has no source after its death (before its birth)."""
    out, secs = {}, _secs(s_b, s_d)
    for i in range(5):
        s = np.arange(secs[i], secs[i + 1] + 1e-9, 0.25)
        out[(i, 0)] = np.column_stack([s, np.zeros(len(s))])
        out[(i, 1)] = np.column_stack([s, np.full(len(s), -3.5)])
        ramp = np.clip((s - s_b) / (s_d - s_b), 0.0, 1.0)
        if i in (1, 2, 3):
            seen = s <= s_d + 1e-9 if kind == "inner" else s >= s_b - 1e-9
            out[(i, 2)] = np.column_stack([s, -3.5 - y_width * ramp if kind == "inner" else EDGE + 3.5 * ramp])
            if seen.sum() > 1:
                out[(i, 3)] = np.column_stack([s[seen], np.full(int(seen.sum()), EDGE)])
        else:
            out[(i, 2)] = np.column_stack([s, np.full(len(s), -3.5 - y_width if i == 4 and kind == "inner" else EDGE)])
    return out


@pytest.fixture
def source(monkeypatch):
    def use(kind, **kw):
        bounds = _boundaries(kind, **kw)
        monkeypatch.setattr(LR, "_source_observations",
                            lambda road, src, origin, side, i, k: bounds.get((i, k)) if side == "right" else None)
        return etree.fromstring(_xodr(kind, **{x: v for x, v in kw.items() if x != "y_width"}).encode())
    return use


def _section(s):
    secs = _secs()
    return max(i for i in range(5) if secs[i] <= s + 1e-9)


def _t(fitted, s, k):
    return LR._at(fitted[("right", _section(s), k)], s)


def _outer_edge(fitted, s):
    return _t(fitted, s, 3 if _section(s) in (1, 2, 3) else 2)


STATIONS = np.arange(0.0, 60.0, 0.1)


def test_fitted_apart_the_outer_edge_dents(source):
    for kind in ("inner", "outer"):
        root = source(kind)
        _, fitted, report = LR.refit_road(root.find("road"), None, ORIGIN, {**P, "lane_swaps": False})
        assert "lane_swaps" not in report
        assert max(abs(_outer_edge(fitted, s) - EDGE) for s in STATIONS) > 0.3


def test_an_inner_birth_swap_keeps_the_outer_edge_on_its_source(source):
    root = source("inner")
    _, fitted, report = LR.refit_road(root.find("road"), None, ORIGIN, P)
    (row,) = report["lane_swaps"]
    assert (row["type"], row["side"], row["s_b"], row["s_d"]) == ("inner-birth", "right", S_B, S_D)
    assert row["edge"] == {"0": 2, "1": 3, "2": 3, "3": 3, "4": 2} and row["divider"] == {"1": 2, "2": 2, "3": 2}
    assert row["h_m"] == [2.0, 2.0] and row["span"] == [26.0, 34.0]          # corners: half the run between events
    assert max(abs(_outer_edge(fitted, s) - EDGE) for s in STATIONS) < 0.01
    for s in np.arange(_secs()[1], _secs()[4], 0.05):
        inner, divider, outer = (_t(fitted, s, k) for k in (1, 2, 3))
        y, x = inner - divider, divider - outer                       # Y opens, X closes (right side: t falls outward)
        assert -1e-9 <= y <= 3.5 + LR.NEGATIVE_WIDTH_TOL and -1e-9 <= x <= 3.5 + LR.NEGATIVE_WIDTH_TOL
        if s <= S_B - 2.0:
            assert y == pytest.approx(0.0, abs=1e-12)                 # exactly zero before Y's corner
        if s >= S_D + 2.0:
            assert x == pytest.approx(0.0, abs=1e-12)                 # ... and after X's
    assert _t(fitted, 30.0, 2) == pytest.approx(-5.25, abs=0.01)      # the middle of the swap on the source
    assert row["divider_dev_m"] < 0.35
    pieces = [p for i in (1, 2, 3) for p in fitted[("right", i, 2)]]
    assert max(abs(LR._eval(p, p[1], o) - LR._eval(q, q[0], o)) for p, q in zip(pieces[:-1], pieces[1:])
               for o in (0, 1, 2)) < 1e-9                             # the divider is C2


def test_an_outer_birth_swap_the_other_way_round(source):
    root = source("outer")
    _, fitted, report = LR.refit_road(root.find("road"), None, ORIGIN, P)
    (row,) = report["lane_swaps"]
    assert (row["type"], row["s_b"], row["s_d"]) == ("outer-birth", S_B, S_D)
    assert row["edge"] == {"0": 2, "1": 3, "2": 3, "3": 3, "4": 2} and row["divider"] == {"1": 2, "2": 2, "3": 2}
    assert max(abs(_outer_edge(fitted, s) - EDGE) for s in STATIONS) < 0.01
    for s in np.arange(_secs()[1], _secs()[4], 0.05):
        inner, divider, outer = (_t(fitted, s, k) for k in (1, 2, 3))
        y, x = inner - divider, divider - outer                       # Y closes, X opens
        assert -1e-9 <= y <= 3.5 + LR.NEGATIVE_WIDTH_TOL and -1e-9 <= x <= 3.5 + LR.NEGATIVE_WIDTH_TOL
        if s <= S_B - 2.0:
            assert x == pytest.approx(0.0, abs=1e-12)
        if s >= S_D + 2.0:
            assert y == pytest.approx(0.0, abs=1e-12)
    assert _t(fitted, 30.0, 2) == pytest.approx(-5.25, abs=0.01)


def test_the_swap_is_written_and_reported(source):
    root = source("inner")
    report = LR.refit_tree(root, None, P)
    assert report["rewritten"] == 1 and report["skipped"] == []
    road = root.find("road")
    row = json.loads(road.find(f"userData[@code='{LR.CODE}']").get("value"))
    assert [x["type"] for x in row["lane_swaps"]] == ["inner-birth"] and "lane_swaps_failed" not in row
    for s in STATIONS:
        edges = lane_edges_at(road, s, "right")
        assert edges[-1] == pytest.approx(EDGE, abs=0.01)
        assert min(np.diff(edges) * -1) >= -1e-9


def test_a_swap_the_outer_edge_does_not_run_across_is_left_alone(source):
    root = source("inner", y_width=2.5)                               # the outer edge steps in by 1 m at X's death
    _, plain, _ = LR.refit_road(root.find("road"), None, ORIGIN, {**P, "lane_swaps": False})
    _, fitted, report = LR.refit_road(root.find("road"), None, ORIGIN, P)
    assert "lane_swaps" not in report and fitted == plain


def test_a_divider_without_room_for_its_corners_leaves_the_road_fitted_without_swaps(source):
    root = source("inner", s_b=29.8, s_d=30.4)                        # 0.6 m between the events: no frame join
    _, plain, plain_report = LR.refit_road(root.find("road"), None, ORIGIN, {**P, "lane_swaps": False})
    _, fitted, report = LR.refit_road(root.find("road"), None, ORIGIN, P)
    assert "lane_swaps" not in report and fitted == plain
    assert report["lane_swaps_failed"]["type"] == "inner-birth"
    assert {k: v for k, v in report.items() if k != "lane_swaps_failed"} == plain_report


def test_no_swap_fit_without_the_shp_boundary_step(source):
    root = source("inner")
    plain = {k: v for k, v in P.items() if k != "protect_unobserved_ends"}
    _, _, report = LR.refit_road(root.find("road"), None, ORIGIN, plain)
    assert "lane_swaps" not in report
    _, _, report = LR.refit_road(root.find("road"), None, ORIGIN, {**plain, "lane_swaps": True})
    assert [x["type"] for x in report["lane_swaps"]] == ["inner-birth"]


def test_find_needs_the_closing_lane_on_both_sides_of_the_swap(source):
    root = source("inner")
    road = LR._Road(root.find("road"))
    bounds = _boundaries("inner")

    def obs_at(i, k):
        return bounds.get((i, k))
    chains = LR._fit_order(LR._chains(road, "right"))
    found = W.find(road, "right", chains, obs_at, {})
    assert sorted(c["swap"]["role"] for c in found if c.get("swap")) == ["divider", "edge"]
    # the opening lane born in the closing lane's first section: no lane runs on outside the swap's start
    x_chain = {0: 2, 1: 3, 2: 3, 3: 3}
    late = [dict(c, members={i: k for i, k in c["members"].items() if i >= 1}, birth=True)
            if c["members"] == x_chain else c for c in chains]
    assert not any(c.get("swap") for c in W.find(road, "right", late, obs_at, {}))
