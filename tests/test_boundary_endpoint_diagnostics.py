"""Report-only diagnostics: lateral endpoint gaps and boundary deviation on the common extent (2026-10-04)."""
from pathlib import Path

import numpy as np
import pytest

from mapforge.validate import endpoint_lateral as EL

ROOT = Path(__file__).resolve().parents[1]


def test_an_along_track_endpoint_gap_has_no_lateral_part():
    src = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    shorter = src.copy()
    shorter[-1] = [2.64, 0.0]                         # the target lane ends 0.36 m earlier on the same line
    assert EL._lateral(src, shorter, at_start=False) == pytest.approx(0.0, abs=1e-12)
    shifted = src + [0.0, 0.25]                       # the same length, 0.25 m to the side
    assert EL._lateral(src, shifted, at_start=True) == pytest.approx(0.25)
    assert EL._lateral(src, shifted, at_start=False) == pytest.approx(0.25)


@pytest.mark.slow
def test_boundary_inside_only_leaves_out_the_continuation_past_the_road_ends():
    run = ROOT / "out" / "scoreboard" / "20261004-default-c2-shpfit-v2"
    xodr = run / "shp-node16.xodr"
    shp = ROOT / "shp_0222-0326"
    if not xodr.exists() or not shp.exists():
        pytest.skip("scoreboard run or SHP sample not available")
    from mapforge.adapters.shp.profile_source import ProfileSource
    from mapforge.validate.boundary_inside import audit
    from mapforge.validate.shp_boundary_fidelity import evaluate_shp_outer_edges
    src = ProfileSource(str(shp), "ibd-smarteditor-v1")
    full = evaluate_shp_outer_edges(shp, xodr, src=src)
    inside = audit(xodr, shp, src=src)
    full_max = max(full["source_to_target"]["max_m"], full["target_to_source"]["max_m"])
    full_p95 = max(full["source_to_target"]["p95_m"], full["target_to_source"]["p95_m"])
    assert inside["boundary_inside_max_m"] < 0.5 < full_max
    assert inside["boundary_inside_p95_m"] <= full_p95
