"""Lane births/deaths moved ahead of their source events: section splits, short sections taken whole, links."""
import json

import pytest
from lxml import etree

from mapforge.ops import lane_birth_advance as BA


def _lane(lid, width, pred=None, succ=None, src="L"):
    a, b = width
    link = "".join([f'<predecessor id="{pred}"/>' if pred is not None else "",
                    f'<successor id="{succ}"/>' if succ is not None else ""])
    prov = json.dumps({"eligibility": "comparable", "status": "TRANSFORMED", "support_kind": "shp-field-centerline"})
    return (f'<lane id="{lid}" type="driving" level="false"><link>{link}</link>'
            f'<width sOffset="0" a="{a}" b="{b}" c="0" d="0"/><roadMark sOffset="0" type="solid"/>'
            f'<userData code="mapforge.source_lane" value="{src}{abs(lid)}"/>'
            f"<userData code=\"mapforge.provenance/v1\" value='{prov}'/></lane>")


def _road(sections, length):
    body = "".join(f'<laneSection s="{s}"><left/><center><lane id="0" type="none" level="false"/></center>'
                   f'<right>{"".join(lanes)}</right></laneSection>' for s, lanes in sections)
    return etree.fromstring(f'<road id="10" length="{length}" junction="-1"><lanes>{body}</lanes></road>')


def _links_ok(road):
    secs = road.findall("lanes/laneSection")
    for i, sec in enumerate(secs):
        for ln in sec.findall("right/lane"):
            lid = int(ln.get("id"))
            for tag, back, j in (("predecessor", "successor", i - 1), ("successor", "predecessor", i + 1)):
                el = ln.find(f"link/{tag}")
                if el is None or not 0 <= j < len(secs):
                    continue
                other = next(o for o in secs[j].findall("right/lane") if int(o.get("id")) == int(el.get("id")))
                assert int(other.find(f"link/{back}").get("id")) == lid
    return True


def _widths(road, lid):
    out = []
    for sec in road.findall("lanes/laneSection"):
        ln = next((x for x in sec.findall("right/lane") if int(x.get("id")) == lid), None)
        out.append(None if ln is None else BA._width_at(ln, 0.0))
    return out


def test_birth_next_to_a_short_section_runs_through_it_and_splits_the_one_before():
    road = _road([(0.0, [_lane(-1, (3.5, 0), succ=-1)]),
                  (20.0, [_lane(-1, (3.5, 0), pred=-1, succ=-1)]),
                  (22.0, [_lane(-1, (3.5, 0), pred=-1), _lane(-2, (0.0, 0.2))])], 60.0)
    (ev,) = BA.events(road)
    assert ev["kind"] == "birth" and ev["lane"] == -2 and ev["s"] == pytest.approx(22.0)
    made = BA.extend(road, ev, 5.0)
    assert made == pytest.approx(5.0)
    secs = road.findall("lanes/laneSection")
    assert [float(s.get("s")) for s in secs] == pytest.approx([0.0, 17.0, 20.0, 22.0])
    assert _widths(road, -2) == [None, 0.0, 0.0, 0.0]
    assert _links_ok(road)
    first = next(x for x in secs[1].findall("right/lane") if x.get("id") == "-2")
    assert first.find("link/predecessor") is None
    mark = json.loads(first.find(f"userData[@code='{BA.CODE}']").get("value"))
    assert mark == {"source_birth_s": 22.0, "advance_m": 5.0}
    prov = json.loads(first.find("userData[@code='mapforge.provenance/v1']").get("value"))
    assert prov["eligibility"] == "excluded" and prov["exclusion_code"] == "source-extension"
    assert prov["status"] == "INFERRED"
    # the continuing lane keeps its width and source through the new section
    lane1 = next(x for x in secs[1].findall("right/lane") if x.get("id") == "-1")
    assert BA._width_at(lane1, 0.0) == pytest.approx(3.5)
    assert lane1.find("userData[@code='mapforge.source_lane']").get("value") == "L1"


def test_inner_birth_takes_no_whole_section():
    road = _road([(0.0, [_lane(-1, (3.5, 0), succ=-1), _lane(-2, (3.5, 0), succ=-2)]),
                  (20.0, [_lane(-1, (3.5, 0), pred=-1, succ=-1), _lane(-2, (3.5, 0), pred=-2, succ=-3)]),
                  (22.0, [_lane(-1, (3.5, 0), pred=-1), _lane(-2, (0.0, 0.2)), _lane(-3, (3.5, 0), pred=-2)])], 60.0)
    before = etree.tostring(road)
    (ev,) = BA.events(road)
    assert BA.extend(road, ev, 5.0) == 0.0
    assert etree.tostring(road) == before


def test_death_next_to_a_short_section_is_mirrored():
    road = _road([(0.0, [_lane(-1, (3.5, 0), succ=-1), _lane(-2, (3.0, -0.1))]),
                  (30.0, [_lane(-1, (3.5, 0), pred=-1, succ=-1)]),
                  (32.0, [_lane(-1, (3.5, 0), pred=-1)])], 60.0)
    (ev,) = BA.events(road)
    assert ev["kind"] == "death" and ev["s"] == pytest.approx(30.0)
    assert BA.extend(road, ev, 4.0) == pytest.approx(4.0)
    secs = road.findall("lanes/laneSection")
    assert [float(s.get("s")) for s in secs] == pytest.approx([0.0, 30.0, 32.0, 34.0])
    assert _widths(road, -2)[1:] == [0.0, 0.0, None]
    assert _links_ok(road)
    last = next(x for x in secs[2].findall("right/lane") if x.get("id") == "-2")
    assert last.find("link/successor") is None
    assert json.loads(last.find(f"userData[@code='{BA.CODE}']").get("value")) == {"source_death_s": 30.0,
                                                                                    "advance_m": 4.0}


def test_advance_sizes_by_the_source_opening_and_skips_gentle_ones():
    road = _road([(0.0, [_lane(-1, (3.5, 0), succ=-1)]),
                  (30.0, [_lane(-1, (3.5, 0), pred=-1), _lane(-2, (0.0, 0.2))])], 60.0)

    def observe(side, i, k):
        import numpy as np
        s = np.arange(30.0, 40.0, 0.5)
        t = -3.5 - (0.2 * (s - 30.0) if k == 2 else 0.0 * s)
        return np.column_stack([s, t])
    report = BA.advance(road, observe, 0.04)
    assert report[0]["advanced_m"] == pytest.approx(5.0)         # |dm| / kappa = 0.2 / 0.04
    gentle = _road([(0.0, [_lane(-1, (3.5, 0), succ=-1)]),
                    (30.0, [_lane(-1, (3.5, 0), pred=-1), _lane(-2, (0.0, 0.01))])], 60.0)

    def flat(side, i, k):
        import numpy as np
        s = np.arange(30.0, 40.0, 0.5)
        return np.column_stack([s, -3.5 - (0.01 * (s - 30.0) if k == 2 else 0.0 * s)])
    assert BA.advance(gentle, flat, 0.04) == []
