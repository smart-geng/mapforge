"""MAP mouths at the farthest stop line: arm extended, connectors cut back to the new mouth line."""
import json
import math

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import map_stop_line_mouth as S
from mapforge.validate.smoothness import lane_edges_at, sample_road_ref

# arm 10 along +x ends at x = 50 into junction 1; connector 100 leaves its end turning left (spirals)
DOC = """<OpenDRIVE>
<road id="10" length="50" junction="-1">
 <link><successor elementType="junction" elementId="1"/></link>
 <planView><geometry s="0" x="0" y="0" hdg="0" length="50"><line/></geometry></planView>
 <lanes><laneSection s="0"><center><lane id="0" type="none"/></center>
  <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>
   <userData code="mapforge.source_lane" value="S1"/>
   <userData code="mapforge.provenance/v1" value='{{"eligibility":"comparable","support_s":[0.0,49.9999941],"travel_direction":"with_s"}}'/></lane>
  <lane id="-2" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>
   <userData code="mapforge.source_lane" value="S2"/>
   <userData code="mapforge.provenance/v1" value='{{"eligibility":"comparable","support_s":[0.0,48.5],"travel_direction":"with_s"}}'/></lane></right>
 </laneSection></lanes>
</road>
<road id="100" length="30" junction="1">
 <link><predecessor elementType="road" elementId="10" contactPoint="end"/></link>
 <planView>
  <geometry s="0" x="50" y="-1.75" hdg="0" length="10"><spiral curvStart="0" curvEnd="0.05"/></geometry>
  <geometry s="10" x="{x1}" y="{y1}" hdg="{h1}" length="20"><arc curvature="0.05"/></geometry>
 </planView>
 <lanes>
  <laneOffset s="0" a="0" b="0.01" c="0" d="0"/>
  <laneSection s="0"><center><lane id="0" type="none"/></center>
  <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0.02" c="0" d="0"/>
   <width sOffset="25" a="4.0" b="0" c="0" d="0"/>
   <roadMark sOffset="0" type="none" weight="standard" color="standard" width="0"/></lane></right>
 </laneSection></lanes>
</road>
</OpenDRIVE>"""


def _doc():
    probe = etree.fromstring(DOC.format(x1=0, y1=0, h1=0))
    pts, ss, hh = sample_road_ref(probe.find("road[@id='100']"), 0.001)
    i = int(np.argmin(np.abs(ss - 10.0)))
    return etree.fromstring(DOC.format(x1=pts[i][0], y1=pts[i][1], h1=hh[i]))


def test_arm_reaches_its_farthest_stop_line_and_the_connector_starts_on_the_new_mouth():
    root = _doc()
    old_conn = sample_road_ref(etree.fromstring(etree.tostring(root)).find("road[@id='100']"), 0.001)
    # lane -1's stop line lies 0.8 m beyond the arm end, lane -2's 1.5 m before it
    centres = {"S1": np.array([[0.0, -1.75], [50.8, -1.75]]), "S2": np.array([[0.0, -5.25], [48.5, -5.25]])}
    record = S.apply(root, centres)
    assert record["roads"][0]["end"]["extended_m"] == pytest.approx(0.8)
    arm, conn = root.find("road[@id='10']"), root.find("road[@id='100']")
    assert float(arm.get("length")) == pytest.approx(50.8)
    lanes = arm.findall("lanes/laneSection/right/lane")
    supports = [json.loads(ln.find("userData[@code='mapforge.provenance/v1']").get("value"))["support_s"] for ln in lanes]
    assert supports[0][1] == pytest.approx(50.8, abs=1e-3) and supports[1][1] == pytest.approx(48.5)
    # the connector now starts on the new mouth line, exactly where its old curve crossed it
    (cut,) = record["connector_cuts"]
    assert cut["at"] == "start" and cut["cut_m"] == pytest.approx(0.8, abs=1e-3)
    assert float(conn.get("length")) == pytest.approx(30.0 - cut["cut_m"])
    pts, ss, hh = sample_road_ref(conn, 0.001)
    assert pts[0][0] == pytest.approx(50.8, abs=1e-6)
    j = int(np.argmin(np.abs(old_conn[1] - cut["cut_m"])))
    assert np.linalg.norm(pts[0] - old_conn[0][j]) < 2e-3 and abs(hh[0] - old_conn[2][j]) < 1e-5
    assert np.linalg.norm(pts[-1] - old_conn[0][-1]) < 1e-6                 # the far end does not move
    spiral = conn.find("planView/geometry/spiral")
    assert float(spiral.get("curvStart")) == pytest.approx(0.05 * cut["cut_m"] / 10.0, rel=1e-6)
    # lane records restart at the new start with the same values
    lo = conn.find("lanes/laneOffset")
    assert float(lo.get("a")) == pytest.approx(0.01 * cut["cut_m"])
    widths = conn.findall("lanes/laneSection/right/lane/width")
    assert float(widths[0].get("a")) == pytest.approx(3.5 + 0.02 * cut["cut_m"])
    assert float(widths[1].get("sOffset")) == pytest.approx(25.0 - cut["cut_m"])
    assert lane_edges_at(conn, 5.0, "right") == pytest.approx(
        lane_edges_at(etree.fromstring(DOC.format(x1=0, y1=0, h1=0)).find("road[@id='100']"), 5.0 + cut["cut_m"], "right"))


def test_cut_end_shortens_the_last_geometry_and_drops_records_beyond_it():
    root = _doc()
    conn = root.find("road[@id='100']")
    S.cut_end(conn, 6.0)
    assert float(conn.get("length")) == pytest.approx(24.0)
    last = conn.findall("planView/geometry")[-1]
    assert float(last.get("length")) == pytest.approx(14.0)
    widths = conn.findall("lanes/laneSection/right/lane/width")
    assert [float(w.get("sOffset")) for w in widths] == [0.0]                # the record from 25 m is gone


def test_arms_without_a_stop_line_beyond_their_end_are_left_alone():
    root = _doc()
    before = etree.tostring(root)
    record = S.apply(root, {"S1": np.array([[0.0, -1.75], [50.02, -1.75]])})
    assert record["roads"] == [] and record["connector_cuts"] == [] and etree.tostring(root) == before


@pytest.mark.slow
def test_node18_arms_reach_their_stop_lines_and_the_paving_is_rebuilt(tmp_path):
    import os
    import subprocess
    import sys
    import xml.etree.ElementTree as ET
    from pathlib import Path
    from mapforge.ops import map_lane_refit as M
    from mapforge.validate.smoothness import surface_continuity
    root_dir = Path(__file__).resolve().parents[1]
    source = root_dir / "v2x_map_xml" / "map凤苑路-金剑路node18.xml"
    if not source.exists():
        pytest.skip("local MAP sample not installed")
    out = tmp_path / "map-node18.xodr"
    subprocess.run([sys.executable, "-m", "mapforge.cli", "convert", str(source), "--to", "xodr", "--post", "none",
                    "-o", str(out)], cwd=str(root_dir), env=dict(os.environ, PYTHONIOENCODING="utf-8"), capture_output=True)
    root = etree.parse(str(out)).getroot()
    record = S.apply(root, M.load_centres(out.with_suffix(".source-lanes.json")))
    assert {r["road"]: r["end"]["extended_m"] for r in record["roads"]} == pytest.approx(
        {"10": 0.848, "11": 0.887, "12": 0.314, "13": 1.378}, abs=2e-3)
    assert len(record["connector_cuts"]) == 24 and record["junctions"] == ["1"]
    paving = S.rebuild_paving(root, record["junctions"])
    assert paving == [{"junction": "1", "rebuilt": True, "paving_roads": ["90", "91"]}]
    holes = surface_continuity(ET.fromstring(etree.tostring(root)))
    assert holes["paving_holes_gt1cm2"] == 0
