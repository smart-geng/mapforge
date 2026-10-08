"""Lane-edge curvature jumps at written joins (scoreboard diagnostic for the D2 edge bound)."""
import xml.etree.ElementTree as ET

import pytest

from mapforge.validate.lane_edge_joins import audit, road_edge_jumps

ROAD = """<OpenDRIVE><road id="1" length="30" junction="-1">
 <planView>
  <geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry>
  <geometry s="10" x="10" y="0" hdg="0" length="20"><spiral curvStart="0" curvEnd="0.02"/></geometry>
 </planView>
 <lanes>
  <laneOffset s="0" a="0" b="0.1" c="0" d="0"/>
  <laneSection s="0">
   <center><lane id="0" type="none"/></center>
   <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
  </laneSection>
 </lanes>
</road></OpenDRIVE>"""


def test_edge_jump_at_a_sharpness_change_is_t_times_slope_times_the_sharpness_step():
    root = ET.fromstring(ROAD)
    jumps = sorted(road_edge_jumps(root.find("road")), reverse=True)
    # line -> spiral at s = 10: sharpness 0 -> 0.001 /m^2; offset t = 1.0, outer edge t = -2.5, both t' = 0.1
    # (rel 1e-4: the one-sided samples 1e-6 m off the join see 1e-9 /m of the spiral itself)
    speed3 = 1.01 ** 1.5
    assert [j[1] for j in jumps] == pytest.approx([10.0, 10.0])
    assert jumps[0][0] == pytest.approx(2.5 * 0.1 * 0.001 / speed3, rel=1e-4)
    assert jumps[1][0] == pytest.approx(1.0 * 0.1 * 0.001 / speed3, rel=1e-4)
    assert audit(root) == {"lane_edge_join_curvature_jump_max_per_m": pytest.approx(jumps[0][0]),
                           "lane_edge_join_jumps_gt_1e-03": 0, "conn_edge_join_curvature_jump_max_per_m": 0.0}
    # with C2 lateral functions only the reference joins can jump: the short scan finds the same
    assert sorted(road_edge_jumps(root.find("road"), reference_only=True), reverse=True) == jumps
