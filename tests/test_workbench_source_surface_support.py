"""Source identity and loss guards for the fixed experimental visual surface."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from shapely import wkb
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from mapforge.workbench import source_surface_support as S


class Source:
    def __init__(self):
        self.rows = {}
        for lane, ref in S.BOUNDARIES.items():
            end, y = (2.98, .1) if lane == S.ENTRY_LANE else (2.85, -.1)
            self.rows[lane] = [{"source_lane_id": lane, "boundary_id": ref["boundary_id"],
                "relation_layer": ref["relation_layer"], "relation_index": ref["relation_index"],
                "records": [{"boundary_id": ref["boundary_id"], "layer": ref["layer"],
                    "record_index": ref["record_index"], "parts": [[[-2., y], [end, y]]]}]}]

    def lanes_of(self, link):
        return [SimpleNamespace(lane_pid=S.ENTRY_LANE if link == S.ENTER else S.DEPARTURE_LANE)]

    def lane_boundary_records(self, lane):
        return deepcopy(self.rows[lane])


def inputs():
    base = box(3, -5, 10, 5)
    pieces = [{"road_id": S.ROAD, "source_lane": S.ENTRY_LANE, "geometry": box(-.75, .1, 2.98, 3)},
              {"road_id": S.ROAD, "source_lane": S.DEPARTURE_LANE, "geometry": box(-.75, -3, 2.85, -.1)}]
    mouths = [{"road_id": S.ROAD, "enter_link": S.ENTER, "leave_links": [S.LEAVE],
               "pose": [0., 0., 0.], "median_interval": [-.1, .1]}]
    return base, pieces, mouths


def build():
    base, pieces, mouths = inputs()
    pieces = S.bind_source_boundaries(Source(), np.asarray, pieces)
    tails, medians, evidence = S.build_support(base, pieces, mouths)
    restricted = unary_union([base, *[x["geometry"] for x in tails]])
    median = unary_union([x["geometry"] for x in medians])
    return restricted, median, evidence, tails


def test_only_unsupported_departure_withdrawn_all_raw_and_old_median_preserved():
    restricted, median, evidence, tails = build()
    result = S.audit_written_support(restricted, median, evidence, numeric_band_m=1e-12)
    assert result["passed"], result
    assert result["outside_fixed_support_m2"] == 0
    assert result["raw_source_missing_m2"] == 0
    assert result["old_median_missing_m2"] == 0
    assert not result["driving_continuity_proven"] and not result["candidate_accepted"]
    assert evidence["departure_raw_asphalt_base_gap_m"] == pytest.approx(.15)
    assert len(evidence["withdrawn_inferred_sweeps"]) == 1
    assert evidence["support_geometry_wkb_hex"] == evidence["fixed_support"]["wkb_hex"]
    assert all(r["verified_source_binding"]["source_lane"] == r["source_lane"]
               for r in evidence["median_boundary_evidence"])
    entry = next(t for t in tails if t["source_lanes"] == [S.ENTRY_LANE])
    departure = next(t for t in tails if t["source_lanes"] == [S.DEPARTURE_LANE])
    assert entry["sweep_m"] > 0 and entry["status"] == "INFERRED"
    assert departure["sweep_m"] == 0
    assert departure["geometry"].equals(inputs()[1][1]["geometry"])
    assert evidence["new_median_added_raw_overlap_m2"] <= evidence["new_median_raw_overlap_roundoff_bound_m2"]
    assert all(m["scope"].endswith("not driving full-mouth coverage") for m in result["mouths"])


@pytest.mark.parametrize("mutation", ["relation-index", "record-index", "boundary-id", "duplicate-relation", "duplicate-part", "wrong-lane", "wrong-link"])
def test_source_references_are_read_and_verified(mutation):
    source = Source()
    row = source.rows[S.ENTRY_LANE][0]
    if mutation == "relation-index": row["relation_index"] += 1
    elif mutation == "record-index": row["records"][0]["record_index"] += 1
    elif mutation == "boundary-id": row["records"][0]["boundary_id"] = "other"
    elif mutation == "duplicate-relation": source.rows[S.ENTRY_LANE].append(deepcopy(row))
    elif mutation == "duplicate-part": row["records"][0]["parts"].append(deepcopy(row["records"][0]["parts"][0]))
    elif mutation == "wrong-lane": row["source_lane_id"] = "other"
    elif mutation == "wrong-link": source.lanes_of = lambda _: []
    with pytest.raises(S.SupportRejected):
        S.bind_source_boundaries(source, np.asarray, inputs()[1])


def test_missing_source_binding_is_rejected_instead_of_constant_identity_claim():
    with pytest.raises(S.SupportRejected, match="binding-missing"):
        S.build_support(*inputs())


def test_changed_projected_source_boundary_cannot_self_certify_polygon_chain():
    source = Source()
    source.rows[S.ENTRY_LANE][0]["records"][0]["parts"][0][1][1] += .0001
    base, pieces, mouths = inputs()
    pieces = S.bind_source_boundaries(source, np.asarray, pieces)
    with pytest.raises(S.SupportRejected, match="does-not-match-tail-inner-chain"):
        S.build_support(base, pieces, mouths)


def test_changed_binding_hash_is_rejected():
    base, pieces, mouths = inputs()
    pieces = S.bind_source_boundaries(Source(), np.asarray, pieces)
    pieces[0]["source_inner_boundary"]["record_index"] += 1
    with pytest.raises(S.SupportRejected, match="binding-missing-or-changed"):
        S.build_support(base, pieces, mouths)


@pytest.mark.parametrize("change", ["missing-raw", "fill-gap", "reassign-median", "remove-entry-bridge", "outside-source"])
def test_actual_written_loss_fill_reassignment_or_gap_is_rejected(change):
    restricted, median, evidence, _ = build()
    if change == "missing-raw": restricted = restricted.difference(box(1, -2, 1.001, -1))
    elif change == "fill-gap": restricted = restricted.union(box(2, -.05, 2.5, .05))
    elif change == "reassign-median": restricted, median = restricted.union(median), Polygon()
    elif change == "remove-entry-bridge": restricted = restricted.difference(box(2.985, .2, 2.995, 2.9))
    elif change == "outside-source": restricted = restricted.union(box(20, 20, 21, 21))
    result = S.audit_written_support(restricted, median, evidence, numeric_band_m=1e-12)
    assert not result["passed"]
    assert not result["candidate_accepted"] and not result["driving_continuity_proven"]


@pytest.mark.parametrize("band", [-1., float("nan"), float("inf"), .05])
def test_source_tolerance_cannot_be_reused_as_numeric_band(band):
    restricted, median, evidence, _ = build()
    result = S.audit_written_support(restricted, median, evidence, numeric_band_m=band)
    assert not result["passed"] and "invalid-numerical-serialization-band" in result["problems"]


def test_changed_evidence_fails_before_actual_comparison():
    restricted, median, evidence, _ = build()
    evidence["source_mouths"][0]["pose"][0] += 1
    result = S.audit_written_support(restricted, median, evidence, numeric_band_m=1e-12)
    assert result["problems"] == ["support-evidence-binding-mismatch"]


def test_other_positive_width_median_guard_is_preserved(monkeypatch):
    base, pieces, mouths = inputs()
    pieces = S.bind_source_boundaries(Source(), np.asarray, pieces)
    mouths.append({"road_id": "13", "enter_link": "other", "leave_links": [],
                   "pose": [0., 0., 0.], "median_interval": [-.1, .1]})
    original_bridge = S.E._bridge_tails
    original_median = S.E._median_tails
    monkeypatch.setattr(S.E, "_bridge_tails", lambda b, c, p, m: original_bridge(b, c, p, m[:1]))
    monkeypatch.setattr(S.E, "_median_tails", lambda b, c, p, m: original_median(b, c, p, m[:1]))
    with pytest.raises(S.SupportRejected, match="positive-width-median-unresolved"):
        S.build_support(base, pieces, mouths)


def test_cap_overlap_does_not_jump_across_base_hole():
    base = box(1, -1, 4, 1).difference(box(1.1, -.5, 1.15, .5))
    with pytest.raises(S.SupportRejected, match="overlap-not-inside"):
        S._extend_to_base([np.array([0., 0.])], base, np.array([1., 0.]))


def test_cap_overlap_cannot_extend_beyond_existing_sweep_budget():
    with pytest.raises(S.SupportRejected, match="exceeds-existing-sweep-budget"):
        S._extend_to_base([np.array([0., 0.])], box(1.4, -1, 2, 1), np.array([1., 0.]))


def test_already_supported_departure_is_not_relabelled_as_fixed_failure(monkeypatch):
    base, pieces, mouths = inputs()
    source = Source()
    source.rows[S.DEPARTURE_LANE][0]["records"][0]["parts"][0][1][0] = 2.98
    pieces[1]["geometry"] = box(-.75, -3, 2.98, -.1)
    pieces = S.bind_source_boundaries(source, np.asarray, pieces)
    with pytest.raises(S.SupportRejected, match="sweep-is-not-unsupported"):
        S.build_support(base, pieces, mouths)
