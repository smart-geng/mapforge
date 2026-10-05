"""Gap-aware junction surface after moving the mouths (mapforge.ops.envelope_surface)."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from mapforge.ops import envelope_surface as ES

ROOT = Path(__file__).resolve().parents[1]


def test_numerical_slivers_are_filled_but_real_islands_stay():
    outer = [(0, 0), (20, 0), (20, 10), (0, 10)]
    sliver = [(5, 5), (5.5, 5), (5.5, 5.00001), (5, 5.00001)]
    island = [(12, 3), (15, 3), (15, 6), (12, 6)]
    filled, n = ES._fill_slivers(Polygon(outer, [sliver, island]))
    assert n == 1 and len(filled.interiors) == 1
    assert Polygon(filled.interiors[0]).area == pytest.approx(9.0)


def _spec():
    # mouth at the origin, travel along +x into the junction; normal (+y) is the cross-section axis
    return {"road_id": "10", "pose": [0.0, 0.0, 0.0]}


def _family():
    # two carriageway tails 0..6 m long, separated by a 1 m strip (y 4..5)
    return unary_union([box(-0.5, 0.0, 6.0, 4.0), box(-0.5, 5.0, 6.0, 9.0)])


def test_gap_covered_by_the_junction_polygon_is_not_a_hole():
    from mapforge.adapters.opendrive import writer as W
    doc = W.XodrDoc("t")
    cover = box(-1.0, -1.0, 7.0, 10.0)  # the junction polygon covers the strip
    merged = ES._append_surface_family(doc, _family(), Polygon(), _spec(), 60, 1, ["a", "b"], cover)
    assert merged > 0 and len(doc.roads) == 1


def test_gap_outside_the_junction_polygon_still_raises():
    from mapforge.adapters.opendrive import writer as W
    doc = W.XodrDoc("t")
    cover = box(5.5, -1.0, 7.0, 10.0)  # the polygon starts after the tails: the strip is a real separator
    with pytest.raises(ValueError, match="unresolved gap"):
        ES._append_surface_family(doc, _family(), Polygon(), _spec(), 60, 1, ["a", "b"], cover)


@pytest.mark.slow
def test_real_node3_envelope_surface_is_generated_without_holes(tmp_path):
    shp = ROOT / "shp_0222-0326"
    like = ROOT / "v2x_map_xml" / "map含金路-金玥路node3.xml"
    if not shp.exists() or not like.exists():
        pytest.skip("local SHP/MAP samples not installed")
    from mapforge.ops.shp_mouth_envelope import convert
    from mapforge.validate.smoothness import surface_continuity
    out = tmp_path / "shp-node3.xodr"
    record = convert(shp, like, out, margin_m=3.0)
    assert record["status"] == "GENERATED" and record["surface"]["source_surface_status"] == "GENERATED"
    assert record["converter_surface"]["status"] == "FAIL"  # the converter's own builder refuses node3
    surface = surface_continuity(ET.parse(out).getroot())
    assert surface["paving_holes_gt1cm2"] == 0 and surface["paving_components"] == 1
    assert surface["paving_leg_overlap_min"] > 1.0
