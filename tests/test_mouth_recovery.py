"""Strict admission of recovered connector candidates using existing quality verdicts."""
import copy

import pytest

from mapforge.ops.mouth_recovery import filter_recovery_candidates


def _entry(name, p95=0.1):
    shape = {"end_curvature": {end: {"edge_t2_residual": [0.0, 0.0]} for end in ("start", "end")}}
    candidate = (name, None, None, None, None, None, shape, None)
    fidelity = {"median_m": 0.05, "p95_m": p95, "reverse_median_m": 0.05,
                "reverse_p95_m": p95, "coverage": 0.99}
    scored = (candidate, None, None, fidelity, 0.0001)
    flags = {"within_g8": True, "counter_curvature": False, "lane_centre_jump": False,
             "edge_residual": False, "edge_residual_per_m": 0.0001,
             "counter_curvature_per_m": 0.009, "counter_mid_per_m": 0.009, "counter_ends_per_m": 0.019}
    return scored, flags


def _filter(entries, flags, **options):
    defaults = dict(recovery_required=True, full_g8_checked=True, turn_required=True, turn_end_zone=True)
    return filter_recovery_candidates(entries, flags, **{**defaults, **options})


def test_filter_every_candidate_before_ranking_so_good_alternative_survives():
    bad, bad_flags = _entry("closer-but-reversing", 0.01)
    good, good_flags = _entry("farther-admissible", 0.15)
    bad_flags.update(counter_curvature=True, counter_mid_per_m=0.0318, counter_ends_per_m=0.205896)
    entries = [bad, good]
    before = copy.deepcopy(entries)
    accepted, rejected = _filter(entries, {bad[0][0]: bad_flags, good[0][0]: good_flags})
    assert accepted == [good]
    assert min(accepted, key=lambda row: row[3]["p95_m"]) is good
    assert rejected[0]["reasons"] == ["counter_curvature"]
    assert entries == before


def test_061310_130_has_no_candidate_that_is_both_faithful_and_non_reversing():
    free, free_flags = _entry("source-fit-3m", 1.259311)
    free_flags.update(counter_curvature=True, counter_mid_per_m=0.0317879, counter_ends_per_m=0.205896)
    bounded, bounded_flags = _entry("source-fit-6m-ends", 2.004085)
    bounded_flags.update(within_g8=False, counter_mid_per_m=0.0077476, counter_ends_per_m=0.0196803)
    bounded[3]["coverage"] = 0.871795
    accepted, rejected = _filter([free, bounded], {free[0][0]: free_flags, bounded[0][0]: bounded_flags})
    assert not accepted
    assert rejected[0]["reasons"] == ["counter_curvature"]
    assert rejected[1]["reasons"] == ["within_g8"]


def test_straight_candidate_with_null_turn_measurements_is_not_missing_evidence():
    candidate, checks = _entry("source-fit-4m")
    checks.update(counter_curvature_per_m=None, counter_mid_per_m=None, counter_ends_per_m=None)
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks}, turn_required=False)
    assert accepted == [candidate] and not rejected


@pytest.mark.parametrize("key", ["lane_centre_jump", "edge_residual"])
def test_recovery_must_not_trade_another_existing_smoothness_bound(key):
    candidate, checks = _entry("new-fit")
    checks[key] = True
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks})
    assert not accepted and rejected[0]["reasons"] == [key]


@pytest.mark.parametrize("key", ["within_g8", "counter_curvature", "lane_centre_jump", "edge_residual"])
def test_absent_required_flag_is_not_treated_as_false(key):
    candidate, checks = _entry("new-fit")
    del checks[key]
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks})
    assert not accepted and "missing:" + key in rejected[0]["reasons"]


@pytest.mark.parametrize("field", ["reverse_p95_m", "reverse_median_m", "coverage"])
@pytest.mark.parametrize("value", [None, float("nan"), float("inf")])
def test_g8_forward_only_or_nonfinite_evidence_is_rejected(field, value):
    candidate, checks = _entry("new-fit")
    candidate[3][field] = value
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks})
    assert not accepted and "unmeasured:" + field in rejected[0]["reasons"]


def test_turn_measurement_is_required_when_turn_rule_applies():
    candidate, checks = _entry("new-fit")
    checks["counter_mid_per_m"] = None
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks})
    assert not accepted and "unmeasured:counter_mid_per_m" in rejected[0]["reasons"]


def test_full_g8_evidence_must_be_explicit():
    candidate, checks = _entry("new-fit")
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks}, full_g8_checked=False)
    assert not accepted and "incomplete:bidirectional_g8_and_coverage" in rejected[0]["reasons"]


def test_missing_centre_jump_is_not_implicitly_zero():
    candidate, checks = _entry("new-fit")
    candidate = (*candidate[:4], None)
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks})
    assert not accepted and "unmeasured:lane_centre_jump" in rejected[0]["reasons"]


def test_missing_end_curvature_is_not_implicitly_zero():
    candidate, checks = _entry("new-fit")
    candidate[0][6]["end_curvature"] = None
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks})
    assert not accepted and "unmeasured:edge_residual_start" in rejected[0]["reasons"]
    assert "unmeasured:edge_residual_end" in rejected[0]["reasons"]


def test_integer_zero_is_not_an_explicit_boolean_verdict():
    candidate, checks = _entry("new-fit")
    checks["counter_curvature"] = 0
    accepted, rejected = _filter([candidate], {candidate[0][0]: checks})
    assert not accepted and "invalid:counter_curvature" in rejected[0]["reasons"]


def test_normal_successful_path_is_identical_even_without_guard_evidence():
    candidate, _ = _entry("old-normal-path")
    original = [candidate]
    accepted, rejected = _filter(original, None, recovery_required=False, full_g8_checked=False)
    assert accepted is original and rejected == []
