"""Opening slopes of births/deaths in sections too short to measure them, continued along the lane links; a road
the refit skips, that would lose a curb-return shoulder or that would lie farther from its source gets back the
advance of its own sections."""
import numpy as np
import pytest
from lxml import etree

from mapforge.ops import lane_birth_advance as BA
from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_birth_window as BW


class _Road:
    """The part of lane_refit._Road the window needs: section stations and lanes by side."""

    def __init__(self, stations, length, sections):
        self.s = list(stations) + [length]
        self.sections = []
        for lanes in sections:
            body = "".join(
                f'<lane id="{lid}"><link>'
                + (f'<predecessor id="{pred}"/>' if pred is not None else "")
                + (f'<successor id="{succ}"/>' if succ is not None else "")
                + "</link></lane>" for lid, pred, succ in lanes)
            self.sections.append(etree.fromstring(f"<laneSection><right>{body}</right></laneSection>"))

    def lanes(self, i, side):
        return sorted(self.sections[i].findall(f"{side}/lane"), key=lambda ln: abs(int(ln.get("id"))))


def _road():
    """Lane -2 born at s = 20 in a 1.5 m section; the next section carries it on as lane -2."""
    return _Road([0.0, 20.0, 21.5], 60.0, [
        [(-1, None, -1)],
        [(-1, -1, -1), (-2, None, -2)],
        [(-1, -1, None), (-2, -2, None)],
    ])


def _observe(road, calls=None):
    """Boundary 1 straight at 3.5 m, boundary 2 opening at 0.2 /m from s = 20, every 0.5 m inside each section."""
    def observe(side, i, k):
        if calls is not None:
            calls.append((i, k))
        if k > len(road.lanes(i, side)):
            return None
        s = np.arange(road.s[i] + 0.25, road.s[i + 1], 0.5)
        t = np.full(len(s), 3.5) if k == 1 else 3.5 + 0.2 * (s - 20.0) if k == 2 else np.zeros(len(s))
        return np.column_stack([s, t])
    return observe


def test_continued_follows_the_lane_links():
    road = _road()
    assert BW.continued(road, "right", 1, 2, 1) == 2
    assert BW.continued(road, "right", 1, 1, -1) == 1
    assert BW.continued(road, "right", 1, 2, -1) is None      # born here: no predecessor
    assert BW.continued(road, "right", 1, 0, 1) == 0          # the centre stays the centre
    assert BW.continued(road, "right", 0, 3, 1) is None       # no such lane


def test_short_birth_section_is_measured_through_the_next_section():
    road = _road()
    (ev,) = [e for e in BA.events(_lanes_xml(road)) if e["kind"] == "birth"]
    assert ev["section"] == 1 and ev["lane"] == -2
    plain = _observe(road)
    assert BA._opening_slope(plain, ev) is None              # 1.5 m: too short on its own
    assert BA._opening_slope(BW.observer(road, plain), ev) == pytest.approx(0.2, abs=1e-9)


def test_measurable_sections_see_what_they_saw_before():
    road = _Road([0.0, 20.0, 30.0], 60.0, [
        [(-1, None, -1)],
        [(-1, -1, -1), (-2, None, -2)],
        [(-1, -1, None), (-2, -2, None)],
    ])
    calls = []
    plain = _observe(road, calls)
    ext = BW.observer(road, plain)
    for k in (1, 2):
        got = ext("right", 1, k)
        assert np.array_equal(got, plain("right", 1, k))
    assert calls == [(1, 1), (1, 1), (1, 2), (1, 2)]            # no neighbouring section looked at


def test_continuation_stops_at_the_window():
    road = _Road([0.0, 20.0, 21.5, 24.0, 30.0], 60.0, [
        [(-1, None, -1)],
        [(-1, -1, -1), (-2, None, -2)],
        [(-1, -1, -1), (-2, -2, -2)],
        [(-1, -1, -1), (-2, -2, -2)],
        [(-1, -1, None), (-2, -2, None)],
    ])
    got = BW.observer(road, _observe(road))("right", 1, 2)
    # sections 2 and 3 start within SLOPE_WINDOW_M of the section end (21.5); section 4 (30.0) does not
    assert got[:, 0].max() < 30.0
    assert got[:, 0].max() > 29.0
    assert got[:, 0].min() > 20.0                             # nothing from before the birth


def test_missing_observations_stay_missing():
    road = _road()
    ext = BW.observer(road, lambda side, i, k: None)
    assert ext("right", 1, 2) is None


def _stations(road_el):
    return [round(float(sec.get("s")), 6) for sec in road_el.findall("lanes/laneSection")]


def test_advance_keeps_the_own_section_road_only_where_the_window_adds():
    road = _road()
    el = _lanes_xml(road)
    events, own = BW.advance(el, road, _observe(road), 0.04)
    assert [(e["kind"], e["lane"], e["advanced_m"]) for e in events] == [("birth", -2, 5.0)]   # 0.2 / 0.04
    assert _stations(el) == [0.0, 15.0, 20.0, 21.5]
    own_el, own_events = own
    assert own_events == [] and _stations(own_el) == [0.0, 20.0, 21.5]     # 1.5 m: not measured on its own
    long = _Road([0.0, 20.0, 30.0], 60.0, [
        [(-1, None, -1)],
        [(-1, -1, -1), (-2, None, -2)],
        [(-1, -1, None), (-2, -2, None)],
    ])
    events, own = BW.advance(_lanes_xml(long), long, _observe(long), 0.04)
    assert [e["advanced_m"] for e in events] == [5.0] and own is None   # the window adds nothing there


GEO = "<header><geoReference>+proj=tmerc +lat_0=29.5 +lon_0=106.3 +k=1 +x_0=0 +y_0=0</geoReference></header>"


def _skipped_case():
    road = _road()
    root = etree.fromstring(f"<OpenDRIVE>{GEO}</OpenDRIVE>")
    el = _lanes_xml(road)
    root.append(el)
    events, own = BW.advance(el, road, _observe(road), 0.04)
    return root, events, own


def test_a_skipped_road_gets_its_own_section_advance_back_and_is_refitted_again(monkeypatch):
    root, events, own = _skipped_case()
    advanced = {"10": events}
    rows = [{"road": "11"}, {"road": "12"}, {"road": "13"}]
    report = {"rewritten": 3, "skipped": [{"road": "10", "reason": "negative width -0.5 m"}], "rows": list(rows)}
    seen = []

    def refit_tree(alone, src, params):
        seen.append((_stations(alone.find("road")), alone.find("header") is not None, params))
        return {"rewritten": 0, "skipped": [{"road": "10", "reason": "negative width -0.4 m"}], "rows": []}
    monkeypatch.setattr(LR, "refit_tree", refit_tree)
    notes = BW.keep_own_way(root, None, {"p": 1}, report, advanced, {"10": own})
    assert seen == [([0.0, 20.0, 21.5], True, {"p": 1})]
    assert _stations(root.find("road")) == [0.0, 20.0, 21.5]               # the window's split is gone
    assert len(root.findall("road")) == 1
    assert report == {"rewritten": 3, "skipped": [{"road": "10", "reason": "negative width -0.4 m"}], "rows": rows}
    assert advanced == {}
    assert notes == [{"road": "10", "reason": "skipped", "window_events": events, "refit": "skipped"}]


def test_a_second_refit_that_writes_the_road_is_kept(monkeypatch):
    root, events, own = _skipped_case()
    report = {"rewritten": 0, "skipped": [{"road": "10", "reason": "negative width -0.5 m"}], "rows": []}
    monkeypatch.setattr(LR, "refit_tree", lambda alone, src, params: {
        "rewritten": 1, "skipped": [], "rows": [{"road": alone.find("road").get("id")}]})
    notes = BW.keep_own_way(root, None, {}, report, {"10": events}, {"10": own})
    assert report == {"rewritten": 1, "skipped": [], "rows": [{"road": "10"}]}
    assert notes[0]["refit"] == "written"


def _worst_by_version(monkeypatch, window_m, own_m):
    """``worst`` of the window's road (split at s 15) and of the own-section road (not split)."""
    seen = []

    def worst(road_el, src, origin):
        seen.append(origin)
        return window_m if 15.0 in _stations(road_el) else own_m
    monkeypatch.setattr(BW, "worst", worst)
    return seen


@pytest.mark.parametrize("window_shoulders, own_shoulders, window_m, own_m", [
    (0, 0, 0.375, 0.375), (0, 0, 0.300, 0.375), (0, 0, 0.3759, 0.375), (1, 0, 0.5, 0.375), (1, 1, 0.375, 0.375)])
def test_a_written_road_keeps_the_window_advance_unless_it_loses_a_shoulder_or_its_source(
        monkeypatch, window_shoulders, own_shoulders, window_m, own_m):
    root, events, own = _skipped_case()
    row = {"road": "10", "curb_return_shoulders": [{"side": "right"}] * window_shoulders}
    report = {"rewritten": 1, "skipped": [], "rows": [row]}
    own_row = {"road": "10", "curb_return_shoulders": [{"side": "right"}] * own_shoulders}
    monkeypatch.setattr(LR, "refit_tree", lambda alone, src, params: {"rewritten": 1, "skipped": [], "rows": [own_row]})
    seen = _worst_by_version(monkeypatch, window_m, own_m)
    advanced = {"10": events}
    assert BW.keep_own_way(root, None, {}, report, advanced, {"10": own}) == []
    assert _stations(root.find("road")) == [0.0, 15.0, 20.0, 21.5]
    assert report == {"rewritten": 1, "skipped": [], "rows": [row]} and report["rows"][0] is row
    assert advanced == {"10": events}
    assert seen == ([] if window_shoulders > own_shoulders else [(29.5, 106.3)] * 2)


def test_a_written_road_that_would_lose_a_curb_shoulder_goes_back(monkeypatch):
    root, events, own = _skipped_case()
    report = {"rewritten": 2, "skipped": [], "rows": [{"road": "9"}, {"road": "10", "curb_return_shoulders": []}]}
    own_row = {"road": "10", "curb_return_shoulders": [{"side": "right", "lane": -3}]}
    monkeypatch.setattr(LR, "refit_tree", lambda alone, src, params: {"rewritten": 1, "skipped": [], "rows": [own_row]})
    advanced = {"10": events}
    notes = BW.keep_own_way(root, None, {}, report, advanced, {"10": own})
    assert _stations(root.find("road")) == [0.0, 20.0, 21.5]
    assert report == {"rewritten": 2, "skipped": [], "rows": [{"road": "9"}, own_row]}
    assert advanced == {}
    assert notes == [{"road": "10", "reason": "curb shoulder", "window_events": events, "refit": "written"}]


def test_a_written_road_farther_from_its_source_goes_back_in_its_place(monkeypatch):
    root, events, own = _skipped_case()
    rows = [{"road": "10", "curb_return_shoulders": []}, {"road": "11"}]
    report = {"rewritten": 2, "skipped": [], "rows": list(rows)}
    own_row = {"road": "10", "curb_return_shoulders": [], "own": True}
    monkeypatch.setattr(LR, "refit_tree", lambda alone, src, params: {"rewritten": 1, "skipped": [], "rows": [own_row]})
    _worst_by_version(monkeypatch, 0.394, 0.375)
    advanced = {"10": events, "11": []}
    notes = BW.keep_own_way(root, None, {}, report, advanced, {"10": own})
    assert _stations(root.find("road")) == [0.0, 20.0, 21.5]
    assert report == {"rewritten": 2, "skipped": [], "rows": [own_row, {"road": "11"}]}   # the entry stays first
    assert advanced == {"11": []}
    assert notes == [{"road": "10", "reason": "source deviation", "window_events": events, "refit": "written",
                      "worst_m": {"window": 0.394, "own": 0.375}}]


class _Source:
    """lane_boundary_geometries of a ProfileSource: (lon, lat) lines by source lane."""

    def __init__(self, lines):
        self.lines = lines

    def lane_boundary_geometries(self, sid):
        return self.lines.get(sid, [])


def test_worst_is_the_farthest_written_boundary_from_its_source_observations():
    from mapforge.validate.shp_boundary_fidelity import R_EARTH, _origin
    lat0, lon0 = 29.5, 106.3

    def lonlat(x, y):
        return (lon0 + np.degrees(x / (R_EARTH * np.cos(np.radians(lat0)))), lat0 + np.degrees(y / R_EARTH))
    root = etree.fromstring(
        f'<OpenDRIVE>{GEO}<road id="1" length="50" junction="-1">'
        '<planView><geometry s="0" x="0" y="0" hdg="0" length="50"><line/></geometry></planView>'
        '<lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/><laneSection s="0">'
        '<center><lane id="0" type="none"/></center>'
        '<right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>'
        '<userData code="mapforge.source_lane" value="L1"/></lane></right>'
        '</laneSection></lanes></road></OpenDRIVE>')
    # the source lane's inner boundary on the reference line, its outer one 3.6 m out and 3.65 m at the far end
    src = _Source({"L1": [[lonlat(x, 0.0) for x in (0.0, 50.0)], [lonlat(0.0, -3.6), lonlat(50.0, -3.65)]]})
    got = BW.worst(root.find("road"), src, _origin(root))
    assert got == pytest.approx(0.15, abs=0.01)


def _lanes_xml(road):
    """An OpenDRIVE road element with the fake road's sections, for lane_birth_advance.events."""
    secs = []
    for s, sec in zip(road.s, road.sections):
        lanes = []
        for ln in sec.findall("right/lane"):
            lid = int(ln.get("id"))
            pred, succ = ln.find("link/predecessor"), ln.find("link/successor")
            width = "0.0" if pred is None and lid == -2 else "3.5"
            link = ("" if pred is None else f'<predecessor id="{pred.get("id")}"/>') + \
                   ("" if succ is None else f'<successor id="{succ.get("id")}"/>')
            lanes.append(f'<lane id="{lid}"><link>{link}</link>'
                         f'<width sOffset="0" a="{width}" b="0" c="0" d="0"/></lane>')
        secs.append(f'<laneSection s="{s}"><right>{"".join(lanes)}</right></laneSection>')
    return etree.fromstring(f'<road id="10" length="{road.s[-1]}"><lanes>{"".join(secs)}</lanes></road>')
