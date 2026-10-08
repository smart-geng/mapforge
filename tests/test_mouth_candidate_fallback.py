"""A rejected old-path fit must not disable independent mouth/source candidates."""
import json

import numpy as np
import pytest
from lxml import etree

from mapforge.ops import mouth_frame_align as MFA
from mapforge.ops import refline_fit as RF
from mapforge.validate.junction_edges import audit


def _junction(shift=0.2):
    """Straight lane centres at y=0; the old connector is shifted sideways."""
    root = etree.Element("OpenDRIVE")
    for rid, x, y, length, junction in (("1", -20, 1.75, 20, "-1"),
                                       ("100", 0, shift, 30, "1"),
                                       ("2", 30, 1.75, 20, "-1")):
        road = etree.SubElement(root, "road", id=rid, name="", length=str(length), junction=junction)
        if rid == "100":
            links = etree.SubElement(road, "link")
            etree.SubElement(links, "predecessor", elementType="road", elementId="1", contactPoint="end")
            etree.SubElement(links, "successor", elementType="road", elementId="2", contactPoint="start")
        pv = etree.SubElement(road, "planView")
        for s in (range(0, length, 10) if rid == "100" else [0]):
            g = etree.SubElement(pv, "geometry", s=str(s), x=str(x + s), y=str(y), hdg="0",
                                 length=str(10 if rid == "100" else length))
            etree.SubElement(g, "line")
        lanes = etree.SubElement(road, "lanes")
        etree.SubElement(lanes, "laneOffset", s="0", a="1.75" if rid == "100" else "0", b="0", c="0", d="0")
        sec = etree.SubElement(lanes, "laneSection", s="0")
        etree.SubElement(etree.SubElement(sec, "center"), "lane", id="0", type="none", level="false")
        lane = etree.SubElement(etree.SubElement(sec, "right"), "lane", id="-1", type="driving", level="false")
        links = etree.SubElement(lane, "link")
        etree.SubElement(links, "predecessor", id="-1")
        etree.SubElement(links, "successor", id="-1")
        etree.SubElement(lane, "width", sOffset="0", a="3.5", b="0", c="0", d="0")
        if rid == "100":
            etree.SubElement(lane, "userData", code="mapforge.source_lane", value="via")
            etree.SubElement(lane, "userData", code="mapforge.provenance/v1",
                             value=json.dumps({"eligibility": "comparable", "support_s": [0.0, 30.0]}))
    jn = etree.SubElement(root, "junction", id="1", name="")
    conn = etree.SubElement(jn, "connection", id="0", incomingRoad="1", connectingRoad="100", contactPoint="start")
    etree.SubElement(conn, "laneLink", attrib={"from": "-1", "to": "-1"})
    x = np.linspace(0, 30, 121)
    return root, {"via": np.column_stack([x, np.zeros_like(x)])}


def _reject_old_path(monkeypatch, mode="exception"):
    original = MFA._structure_refit
    calls = 0

    def first_fails(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            if mode == "exception":
                raise ValueError("old path has no acceptable fit")
            prims, error, _ = original(*args, **kwargs)
            return prims, error, 0.412
        return original(*args, **kwargs)

    monkeypatch.setattr(MFA, "_structure_refit", first_fails)
    if mode == "tolerance":
        minimal = RF.fit_connector_minimal
        attempts = 0

        def reject_first(*args, **kwargs):
            nonlocal attempts
            attempts += 1
            return None if attempts == 1 else minimal(*args, **kwargs)

        monkeypatch.setattr(RF, "fit_connector_minimal", reject_first)


def _assert_aligned(root, report):
    assert report["rewritten"] == 1 and not report["skipped"]
    edges = audit(root)
    assert edges["count"] == 4
    assert edges["maxima"]["position_m"] < 1e-7
    assert edges["maxima"]["heading_deg"] < 1e-6
    assert report["rows"][0]["min_primitive_m"] >= 3.0 - 1e-8


@pytest.mark.parametrize("mode", ["exception", "tolerance"])
def test_rejected_old_path_still_uses_independent_aligned_fit(monkeypatch, mode):
    root, _ = _junction()
    assert audit(root)["maxima"]["position_m"] == pytest.approx(0.2)
    _reject_old_path(monkeypatch, mode)
    report = MFA.align_tree(root, aligned_frame=True, mouth_blend_kappa=0.01)
    _assert_aligned(root, report)
    assert report["rows"][0]["fit_method"] == "aligned"
    assert report["candidate_failures"][0]["candidate"] == "old-path"


@pytest.mark.parametrize("family", ["source-guided", "source-fit"])
def test_rejected_old_path_still_uses_real_source_candidate(monkeypatch, family):
    root, source = _junction()
    _reject_old_path(monkeypatch)
    report = MFA.align_tree(root, source, source_guided=family == "source-guided",
                            source_fit=family == "source-fit", mouth_blend_kappa=0.01)
    _assert_aligned(root, report)
    assert report["rows"][0]["fit_method"].startswith(family)
    assert report["candidate_failures"][0]["candidate"] == "old-path"


def test_all_reference_families_fail_without_changing_input(monkeypatch):
    root, source = _junction()
    before = etree.tostring(root)

    def fail(*args, **kwargs):
        raise ValueError("no admissible candidate")

    for name in ("_structure_refit", "_aligned_refits", "_source_guided", "_source_fits"):
        monkeypatch.setattr(MFA, name, fail)
    report = MFA.align_tree(root, source, aligned_frame=True, source_guided=True, source_fit=True)
    assert report["rewritten"] == 0 and len(report["skipped"]) == 1
    assert {r["candidate"] for r in report["candidate_failures"]} == {
        "old-path", "aligned", "source-guided", "source-fit"}
    assert etree.tostring(root) == before


def test_failed_scoring_restores_original_geometry(monkeypatch):
    root, _ = _junction()
    before = etree.tostring(root)

    def fail(*args, **kwargs):
        raise ValueError("cannot measure candidate")

    monkeypatch.setattr(MFA, "_source_fidelity", fail)
    report = MFA.align_tree(root)
    assert report["rewritten"] == 0 and len(report["skipped"]) == 1
    assert report["candidate_failures"][0]["candidate"] == "spread:score"
    assert etree.tostring(root) == before


def test_all_end_curvature_failures_are_not_reported_as_narrow_width(monkeypatch):
    root, _ = _junction()
    before = etree.tostring(root)

    def fail(*args, **kwargs):
        raise ValueError("endpoint curvature could not be matched")

    monkeypatch.setattr(MFA, "_match_end_curvature", fail)
    report = MFA.align_tree(root, match_end_curvature=True)
    assert report["rewritten"] == 0
    assert report["skipped"] == [{"connecting_road": "100",
                                  "reason": "all reference candidates failed end-curvature matching"}]
    assert report["candidate_failures"][0]["candidate"] == "spread:end-curvature"
    assert etree.tostring(root) == before


def test_one_failed_score_keeps_the_remaining_candidate(monkeypatch):
    root, _ = _junction()
    original = MFA._source_fidelity
    calls = 0

    def fail_once(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise ValueError("cannot measure first candidate")
        return original(*args, **kwargs)

    monkeypatch.setattr(MFA, "_source_fidelity", fail_once)
    report = MFA.align_tree(root, mouth_blend_kappa=0.01)
    _assert_aligned(root, report)
    assert [r["candidate"] for r in report["candidate_failures"]] == ["spread:score"]


def test_normal_old_path_fit_still_aligns_without_candidate_failures():
    root, _ = _junction()
    report = MFA.align_tree(root, mouth_blend_kappa=0.01)
    _assert_aligned(root, report)
    assert report["rows"][0]["fit_method"] == "structure-preserving"
    assert "candidate_failures" not in report


DEFAULT_C2 = dict(mouth_blend_kappa=0.01, source_guided=True, width_local_slopes=True,
                  match_end_curvature=True, monotone_turns=True, aligned_frame=True,
                  source_fit=True, turn_end_zone=(10.0, 0.02))


def test_default_recovery_rejects_all_unfaithful_candidates_and_restores_xml(monkeypatch):
    root, source = _junction()
    before = etree.tostring(root)
    _reject_old_path(monkeypatch)
    original = MFA._source_fidelity

    def outside_g8(*args, **kwargs):
        measured = original(*args, **kwargs)
        return {**measured, "p95_m": 2.0, "reverse_p95_m": 2.0, "coverage": 0.8}

    # Real refits and provisional XML writes run; deterministic evaluator values
    # represent the no-faithful-candidate condition without an expensive real map.
    monkeypatch.setattr(MFA, "_source_fidelity", outside_g8)
    report = MFA.align_tree(root, source, **DEFAULT_C2)
    assert report["rewritten"] == 0
    assert report["skipped"] == [{"connecting_road": "100", "reason": "recovery-guard: no admissible candidate"}]
    assert report["recovery_rejections"]
    assert all("within_g8" in r["reasons"] for r in report["recovery_rejections"])
    assert etree.tostring(root) == before


def test_default_straight_source_recovery_accepts_null_turn_measurements(monkeypatch):
    root, source = _junction()
    _reject_old_path(monkeypatch)
    report = MFA.align_tree(root, source, **DEFAULT_C2)
    _assert_aligned(root, report)
    assert report["rows"][0]["turn"]["sign"] == 0
    assert report["rows"][0]["turn"]["lane_centre_counter_curvature_per_m"] is None
    assert report["recovery_checks"] == [{"connecting_road": "100", "causes": ["old-path"],
                                           "scope": "guided-c2-source", "full_g8_checked": True,
                                           "turn_required": False}]


def test_geometry_only_recovery_does_not_claim_full_g8(monkeypatch):
    root, _ = _junction()
    _reject_old_path(monkeypatch)
    report = MFA.align_tree(root, aligned_frame=True, mouth_blend_kappa=0.01)
    _assert_aligned(root, report)
    assert report["recovery_checks"][0]["full_g8_checked"] is False
    assert report["recovery_checks"][0]["scope"] == "simplified-geometry"


def test_old_frame_edge_failure_also_enters_strict_recovery(monkeypatch):
    root, source = _junction()
    original = MFA._edge_model
    calls = 0

    def reject_first(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise ValueError("old frame cannot construct admissible edges")
        return original(*args, **kwargs)

    monkeypatch.setattr(MFA, "_edge_model", reject_first)
    report = MFA.align_tree(root, source, **DEFAULT_C2)
    _assert_aligned(root, report)
    assert report["recovery_checks"][0]["causes"] == ["old-path:edges"]
    assert report["recovery_checks"][0]["full_g8_checked"] is True


@pytest.mark.parametrize("unsafe_trim", [False, True])
def test_derived_trim_is_scored_and_admitted_again_before_writing(monkeypatch, unsafe_trim):
    root, source = _junction()
    before = etree.tostring(root)
    _reject_old_path(monkeypatch)
    original_match = MFA._match_end_curvature
    original_fidelity = MFA._source_fidelity
    trimming = False

    def measured_curvature(*args, **kwargs):
        off, centre, width, curv = original_match(*args, **kwargs)
        # Force an edge-only admission failure for every untrimmed candidate.
        if not trimming:
            for end in ("start", "end"):
                curv[end]["edge_t2_residual"] = [0.002, 0.002]
        return off, centre, width, curv

    def trim_geometry(prims, *args, **kwargs):
        nonlocal trimming
        trimming = True
        return prims, 0.0

    def measure_fidelity(*args, **kwargs):
        result = original_fidelity(*args, **kwargs)
        if trimming and unsafe_trim:
            return {**result, "p95_m": 2.0, "reverse_p95_m": 2.0, "coverage": 0.8}
        return result

    monkeypatch.setattr(MFA, "_match_end_curvature", measured_curvature)
    monkeypatch.setattr(MFA, "_end_rate_trim", trim_geometry)
    monkeypatch.setattr(MFA, "_source_fidelity", measure_fidelity)
    report = MFA.align_tree(root, source, **DEFAULT_C2)
    assert trimming
    if unsafe_trim:
        assert report["rewritten"] == 0
        assert etree.tostring(root) == before
        rejected = [r for r in report["recovery_rejections"] if r["stage"] == "trim-admission"]
        assert rejected and all("within_g8" in r["reasons"] for r in rejected)
    else:
        _assert_aligned(root, report)
        row = report["rows"][0]
        chosen = (row["mouth_blend"] or {}).get("chosen", row["fit_method"])
        assert chosen.endswith("-trim")


def test_nonfinite_score_evidence_is_rejected_before_candidate_ranking(monkeypatch):
    root, source = _junction()
    before = etree.tostring(root)
    _reject_old_path(monkeypatch)
    original = MFA._source_fidelity

    def unavailable_reverse(*args, **kwargs):
        return {**original(*args, **kwargs), "reverse_p95_m": float("nan")}

    monkeypatch.setattr(MFA, "_source_fidelity", unavailable_reverse)
    report = MFA.align_tree(root, source, **DEFAULT_C2)
    assert report["rewritten"] == 0 and etree.tostring(root) == before
    assert all(r["stage"] == "score-evidence" for r in report["recovery_rejections"])
    assert all("unmeasured:reverse_p95_m" in r["reasons"] for r in report["recovery_rejections"])


def _force_unfaithful_scores(monkeypatch):
    original = MFA._source_fidelity

    def measured(*args, **kwargs):
        return {**original(*args, **kwargs), "p95_m": 2.0, "reverse_p95_m": 2.0, "coverage": 0.8}

    monkeypatch.setattr(MFA, "_source_fidelity", measured)


@pytest.mark.parametrize("failure", ["source-guided", "source-guided:edges", "source-fit",
                                     "source-fit:edges", "source-fit:local", "end-curvature", "score"])
def test_every_new_exception_recovery_gates_all_surviving_candidates(monkeypatch, failure):
    from mapforge.validate import smoothness
    root, source = _junction()
    before = etree.tostring(root)
    _force_unfaithful_scores(monkeypatch)

    if failure in ("source-guided", "source-fit"):
        def fail(*args, **kwargs):
            raise ValueError("injected candidate family failure")
        monkeypatch.setattr(MFA, "_source_guided" if failure == "source-guided" else "_source_fits", fail)
    elif failure.endswith(":edges") or failure == "source-fit:local":
        factory_name = "_source_guided" if failure.startswith("source-guided") else "_source_fits"
        original_factory = getattr(MFA, factory_name)
        target_name = "_local" if failure.endswith(":local") else "_edge_model"
        original_target = getattr(MFA, target_name)
        armed = False

        def factory(*args, **kwargs):
            nonlocal armed
            value = original_factory(*args, **kwargs)
            assert value
            armed = True
            return value

        def fail_once(*args, **kwargs):
            nonlocal armed
            if armed:
                armed = False
                # ValueError here had a pre-existing benign catch; division by
                # zero instead exercises the genuinely new local recovery.
                error = ZeroDivisionError if target_name == "_local" else ValueError
                raise error("injected post-fit candidate failure")
            return original_target(*args, **kwargs)

        monkeypatch.setattr(MFA, factory_name, factory)
        monkeypatch.setattr(MFA, target_name, fail_once)
    else:
        owner = MFA if failure == "end-curvature" else smoothness
        name = "_match_end_curvature" if failure == "end-curvature" else "sample_road_ref"
        original = getattr(owner, name)
        calls = 0

        def fail_once(*args, **kwargs):
            nonlocal calls
            calls += 1
            # sample call 1 measures the old road; call 3 follows one already
            # successful candidate score and must retroactively guard that one.
            if calls == (1 if failure == "end-curvature" else 3):
                raise ValueError("injected candidate measurement failure")
            return original(*args, **kwargs)

        monkeypatch.setattr(owner, name, fail_once)

    report = MFA.align_tree(root, source, **DEFAULT_C2)
    assert report["rewritten"] == 0
    assert report["skipped"] == [{"connecting_road": "100", "reason": "recovery-guard: no admissible candidate"}]
    assert report["recovery_checks"][0]["full_g8_checked"] is True
    causes = report["recovery_checks"][0]["causes"]
    assert any(c == failure or c.endswith(":" + failure) for c in causes)
    assert report["recovery_rejections"]
    assert etree.tostring(root) == before


def test_late_score_failure_rechecks_earlier_incomplete_score_evidence(monkeypatch):
    root, source = _junction()
    before = etree.tostring(root)
    original = MFA._source_fidelity
    calls = 0

    def measure(*args, **kwargs):
        nonlocal calls
        calls += 1
        value = {**original(*args, **kwargs), "p95_m": 2.0, "reverse_p95_m": 2.0, "coverage": 0.8}
        if calls == 2:  # first candidate score, after the pre-source-fit trigger
            value["reverse_p95_m"] = None
        elif calls == 3:
            raise ValueError("second candidate score failed")
        return value

    monkeypatch.setattr(MFA, "_source_fidelity", measure)
    report = MFA.align_tree(root, source, **DEFAULT_C2)
    assert report["rewritten"] == 0 and etree.tostring(root) == before
    first = next(r for r in report["recovery_rejections"] if r["candidate"] == "spread")
    assert first["stage"] == "score-evidence"
    assert "unmeasured:reverse_p95_m" in first["reasons"]
    assert "local:score" in report["recovery_checks"][0]["causes"]


@pytest.mark.parametrize("error", [ValueError, np.linalg.LinAlgError])
def test_preexisting_source_fit_local_skip_does_not_enable_strict_recovery(monkeypatch, error):
    root, source = _junction()
    _force_unfaithful_scores(monkeypatch)
    original_factory, original_local = MFA._source_fits, MFA._local
    armed = False

    def factory(*args, **kwargs):
        nonlocal armed
        value = original_factory(*args, **kwargs)
        assert value
        armed = True
        return value

    def local(*args, **kwargs):
        nonlocal armed
        if armed:
            armed = False
            raise error("pre-existing per-candidate local skip")
        return original_local(*args, **kwargs)

    monkeypatch.setattr(MFA, "_source_fits", factory)
    monkeypatch.setattr(MFA, "_local", local)
    report = MFA.align_tree(root, source, **DEFAULT_C2)
    assert report["rewritten"] == 1
    assert "recovery_checks" not in report and "recovery_rejections" not in report


def test_preexisting_aligned_helper_catch_does_not_enable_strict_recovery(monkeypatch):
    root, source = _junction()
    _force_unfaithful_scores(monkeypatch)

    def fail(*args, **kwargs):
        raise ValueError("pre-existing optional aligned refit failure")

    monkeypatch.setattr(MFA, "_aligned_refits", fail)
    report = MFA.align_tree(root, source, **DEFAULT_C2)
    assert report["rewritten"] == 1
    assert "recovery_checks" not in report and "recovery_rejections" not in report


@pytest.mark.parametrize("failure", ["trim", "trim-score"])
def test_normal_path_trim_exceptions_preserve_legacy_abort(monkeypatch, failure):
    root, source = _junction()
    original_match, original_fidelity = MFA._match_end_curvature, MFA._source_fidelity
    trimming = False

    def match(*args, **kwargs):
        off, centre, width, curv = original_match(*args, **kwargs)
        if not trimming:
            for end in ("start", "end"):
                curv[end]["edge_t2_residual"] = [0.002, 0.002]
        return off, centre, width, curv

    def trim(prims, *args, **kwargs):
        nonlocal trimming
        trimming = True
        if failure == "trim":
            raise ValueError("legacy trim exception")
        return prims, 0.0

    def fidelity(*args, **kwargs):
        if trimming:
            raise ValueError("legacy trim-score exception")
        return original_fidelity(*args, **kwargs)

    monkeypatch.setattr(MFA, "_source_guided", lambda *args, **kwargs: None)
    monkeypatch.setattr(MFA, "_match_end_curvature", match)
    monkeypatch.setattr(MFA, "_end_rate_trim", trim)
    monkeypatch.setattr(MFA, "_source_fidelity", fidelity)
    with pytest.raises(ValueError, match="legacy trim"):
        MFA.align_tree(root, source, **{**DEFAULT_C2, "source_fit": False, "aligned_frame": False})
