"""Point-to-polyline lane-centre deviation beside G8 (scoreboard diagnostic)."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial import cKDTree

from mapforge.validate.lane_centre_poly import audit, deviations
from mapforge.validate.lane_fidelity import _resample

ROAD = """<OpenDRIVE><road id="12" length="50" junction="-1">
 <planView><geometry s="0" x="0" y="0" hdg="0" length="50"><line/></geometry></planView>
 <lanes><laneSection s="0"><center><lane id="0" type="none"/></center>
  <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/>
   <userData code="mapforge.source_lane" value="S1"/></lane></right>
 </laneSection></lanes>
</road></OpenDRIVE>"""


def test_a_lane_on_its_source_that_starts_later_has_no_deviation_along_the_lane():
    root = ET.fromstring(ROAD)
    # the MAP lane begins 0.45 m before the road: G8's 1 m samples are 0.45 m out of phase all along the lane
    manifest = {"lanes": [{"source_lane_id": "S1", "comparison": {"eligible": True},
                           "geometry": {"coordinates": [[-0.45, -1.75], [50.0, -1.75]]}}]}
    s2t, t2s = deviations(root, manifest)["S1"]
    src = _resample(manifest["lanes"][0]["geometry"]["coordinates"])
    tgt = _resample([[0.0, -1.75], [50.0, -1.75]])
    nearest = cKDTree(tgt).query(src)[0]
    assert np.median(nearest) == pytest.approx(0.45, abs=1e-6)          # what G8 measures
    assert np.median(s2t) == pytest.approx(0.0, abs=1e-9)               # the lane lies on its source
    assert s2t.max() == pytest.approx(0.45, abs=1e-6)                   # only the first point is off
    assert t2s.max() == pytest.approx(0.0, abs=1e-9)
    out = audit(root, manifest)
    assert out["lane_center_poly_median_m"] == pytest.approx(0.0, abs=1e-9)
    assert out["lane_center_poly_max_m"] == pytest.approx(0.45, abs=1e-6)


def test_ineligible_or_missing_lanes_are_not_measured():
    root = ET.fromstring(ROAD)
    manifest = {"lanes": [{"source_lane_id": "S1", "comparison": {"eligible": False},
                           "geometry": {"coordinates": [[0.0, -1.75], [50.0, -1.75]]}},
                          {"source_lane_id": "S9", "comparison": {"eligible": True},
                           "geometry": {"coordinates": [[0.0, 5.0], [50.0, 5.0]]}}]}
    assert deviations(root, manifest) == {}
    assert audit(root, manifest) == {}
