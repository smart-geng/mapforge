"""Independent shape-counterexamples on self-contained written OpenDRIVE.

These fixtures never load research out/ artifacts. Curvature checks use known
line/circle geometry and elementary piecewise cubics, not the checked module's
Frenet or root helpers to manufacture expected answers.
"""
import copy
from fractions import Fraction
import json
import math
from xml.etree import ElementTree as ET

import numpy as np
from numpy.polynomial import Polynomial as P
import pytest

from mapforge.workbench.local_shape import compare, make_contract
from mapforge.workbench.local_shape_math import analyze_road, UnavailableError


def _document(*, length=32.0, curvature=None):
    root = ET.Element("OpenDRIVE")
    ET.SubElement(root, "header", name="synthetic-shape-evidence", revMajor="1", revMinor="5")
    road = ET.SubElement(root, "road", id="10", name="fixture", length=str(length), junction="-1")
    geometry = ET.SubElement(ET.SubElement(road, "planView"), "geometry",
                             s="0", x="0", y="0", hdg="0", length=str(length))
    if curvature is None:
        ET.SubElement(geometry, "line")
    else:
        ET.SubElement(geometry, "arc", curvature=str(curvature))
    lane_parent = ET.SubElement(road, "lanes")
    ET.SubElement(lane_parent, "laneOffset", s="0", a="0", b="0", c="0", d="0")
    section = ET.SubElement(lane_parent, "laneSection", s="0")
    left = ET.SubElement(section, "left")
    center = ET.SubElement(section, "center")
    ET.SubElement(center, "lane", id="0", type="none")
    right = ET.SubElement(section, "right")
    for lid in (1, 2, 3, -1, -2):
        lane = ET.SubElement(left if lid > 0 else right, "lane", id=str(lid),
                             type="median" if lid == 1 else "driving", level="false")
        ET.SubElement(lane, "width", sOffset="0", a="1.25" if lid == 1 else "3",
                      b="0", c="0", d="0")
        ET.SubElement(lane, "speed", sOffset="0", max="50", unit="km/h")
        ET.SubElement(lane, "userData", code="mapforge.source_lane", value=f"raw-lane-{lid}")
        ET.SubElement(lane, "userData", code="mapforge.provenance/v1",
                      value='{"status":"TRANSFORMED","eligibility":"excluded","exclusion_code":"lane-transition-taper"}')
    return root


def _bytes(root):
    return ET.tostring(root, encoding="utf-8")


def _lane(root, lid):
    return root.find(f"road/lanes/laneSection/{'left' if lid > 0 else 'right'}/lane[@id='{lid}']")


def _record(parent, tag, attribute, station, coeffs):
    ET.SubElement(parent, tag, {attribute: format(station, ".17g"),
                              **{key: format(float(value), ".17g") for key, value in zip("abcd", coeffs)}})


def _profile(root, lid, bumps=(), *, offset=False):
    """Add disjoint cardinal cubic bumps with zero endpoint position/slope/curvature.

    A bump (start, unit_span, height) comprises four known cubics. No scipy
    spline construction or production coefficient helper participates.
    """
    parent = root.find("road/lanes") if offset else _lane(root, lid)
    tag, attr = ("laneOffset", "s") if offset else ("width", "sOffset")
    base = 0.0 if offset else (1.25 if lid == 1 else 3.0)
    for old in list(parent.findall(tag)):
        parent.remove(old)
    # Local-u polynomials for a cardinal cubic, normalized to peak one.
    forms = ((0, 0, 0, 1), (1, 3, 3, -3), (4, 0, -6, 3), (1, -3, 3, -1))
    records = {0.0: [base, 0, 0, 0]}
    for start, span, height in bumps:
        for i, coeffs in enumerate(forms):
            c = np.array(coeffs, dtype=float) * height / 4
            c /= np.array([1, span, span**2, span**3])
            c[0] += base
            records[start + i * span] = c
        records[start + 4 * span] = [base, 0, 0, 0]
    for s, c in sorted(records.items()):
        _record(parent, tag, attr, s, c)
    if offset:
        # Preserve valid laneOffset-before-laneSection ordering.
        ordered = list(parent.findall(tag))
        for e in ordered:
            parent.remove(e)
        for i, e in enumerate(ordered):
            parent.insert(i, e)
    else:
        ordered = list(parent.findall(tag))
        for e in ordered:
            parent.remove(e)
        for i, e in enumerate(ordered):
            parent.insert(i, e)


def _contract(data, *, domain=(2.0, 30.0), cells=(2.0, 15.0, 30.0), ids=(-2,), offset=False):
    return make_contract(data, "10", domain, cells,
                         editable_width_lane_ids=ids, allow_lane_offset=offset)


def _issues(report):
    return [item["code"] for item in report["issues"]]


def _curve(analysis, lane_id, name):
    return next(row for row in analysis["lanes"] if row["lane_id"] == lane_id)["curves"][name]


def test_fixed_cells_expose_local_peak_hidden_by_unchanged_domain_maximum():
    baseline = _document()
    _profile(baseline, -2, [(4.0, 1.0, 0.5)])
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(4.0, 1.0, 0.5), (20.0, 1.0, 0.0625)])
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "REGRESSION", report["issues"]
    row = next(r for r in report["curves"] if r["lane_id"] == -2 and r["curve"] == "outer")
    maxima = [max(abs(c[side]["kappa"]["value"]) for c in row["cells"]) for side in ("before", "after")]
    assert maxima[0] == pytest.approx(maxima[1], abs=1e-12)
    assert any(i["code"] == "local-kappa-regression" and i["domain"] == [15.0, 30.0]
               for i in report["issues"])


@pytest.mark.parametrize("height", [0.5, 0.4])
def test_reducing_existing_bump_is_comparable_but_never_a_product_pass(height):
    baseline = _document()
    _profile(baseline, -2, [(4.0, 1.0, height)])
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(4.0, 1.0, height / 2)])
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "NO_REGRESSION_FOUND", report["issues"]
    assert report["formal_delivery"] is False
    assert report["editing_capability_granted"] is False
    assert report["source_fidelity_evaluated"] is False
    assert report["whole_map_gates_replaced"] is False
    assert len(report["curves"]) == 16


@pytest.mark.parametrize("mode", ["median", "offset"])
def test_median_and_lane_zero_are_included_even_when_caller_only_authorizes_one_edit(mode):
    baseline = _document()
    candidate = copy.deepcopy(baseline)
    _profile(candidate, 1, [(8.0, 1.0, 0.125)], offset=mode == "offset")
    data = _bytes(baseline)
    contract = _contract(data, ids=(1,) if mode == "median" else (), offset=mode == "offset")
    report = compare(data, _bytes(candidate), contract)
    assert report["status"] == "REGRESSION", report["issues"]
    assert {(r["lane_id"], r["curve"]) for r in report["curves"]} == {
        *((lid, role) for lid in (-2, -1, 1, 2, 3) for role in ("inner", "center", "outer")),
        (0, "center"),
    }
    target = 1 if mode == "median" else 0
    assert any(i.get("lane_id") == target and i["code"] == "local-kappa-regression" for i in report["issues"])


@pytest.mark.parametrize("format_only", [False, True])
def test_no_geometric_change_is_unavailable_not_positive_evidence(format_only):
    data = _bytes(_document())
    candidate = data.replace(b" />", b"/>") if format_only else data
    report = compare(data, candidate, _contract(data))
    assert report["status"] == "UNAVAILABLE"
    assert "no-geometric-change" in report["issues"][0]["reason"]


def test_large_world_origin_cannot_hide_a_local_position_jump():
    baseline = _document()
    baseline.find("road/planView/geometry").set("y", "1e16")
    candidate = copy.deepcopy(baseline)
    lane = _lane(candidate, -2)
    # A 12.5cm width step is real even though binary64 world y loses it.
    for index, (station, width) in enumerate(((8.0, 3.125), (9.0, 3.0)), 1):
        _record(lane, "width", "sOffset", station, [width, 0, 0, 0])
        record = lane[-1]
        lane.remove(record)
        lane.insert(index, record)
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "REGRESSION", report["issues"]
    assert any(issue["code"] == "joint-t-regression"
               and issue["lane_id"] == -2 and issue["curve"] == "outer"
               and issue["s_m"] == 8.0
               and issue["before"] == 0.0 and issue["after"] == 0.125
               for issue in report["issues"])
    outer = _curve(report["mathematical_analysis"][1], -2, "outer")
    joint = next(item for item in outer["breakpoints"] if item["s_m"] == 8.0)
    assert joint["left_jet"]["y"] == joint["right_jet"]["y"]
    assert joint["right_jet"]["t"] - joint["left_jet"]["t"] == -0.125


@pytest.mark.parametrize("change", ["outside-domain", "metadata", "source-id", "speed", "reference"])
def test_geometry_scope_and_source_metadata_changes_refuse_comparison(change):
    baseline = _document()
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(8.0, 1.0, 0.1)])
    lane = _lane(candidate, -2)
    if change == "outside-domain":
        lane.find("width").set("a", "3.1")
    elif change == "metadata":
        lane.find("userData[@code='mapforge.provenance/v1']").set("value", '{"eligibility":"comparable"}')
    elif change == "source-id":
        lane.find("userData[@code='mapforge.source_lane']").set("value", "different-source")
    elif change == "speed":
        lane.find("speed").set("max", "10")
    else:
        candidate.find("road/planView/geometry").set("hdg", "0.1")
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "UNAVAILABLE"
    expected = "outside-domain-polynomial-changed" if change == "outside-domain" else "outside-editable-xml-structure-changed"
    assert expected in report["issues"][0]["reason"]


def test_small_outside_cubic_coefficient_cannot_hide_centimeter_geometry_change():
    baseline = _document(length=1000.0)
    _profile(baseline, -2, [(4.0, 1.0, 0.5)])
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(4.0, 1.0, 0.25)])
    # In-domain smoothing is valid, but a small *coefficient* causes 4.55cm
    # displacement outside the editable domain. Coefficient units differ.
    _record(_lane(candidate, -2), "width", "sOffset", 31.0, [3, 0, 0, 5e-11])
    assert 5e-11 * (1000 - 31)**3 > 0.04
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "UNAVAILABLE", report["issues"]
    assert "outside-domain" in report["issues"][0]["reason"]


@pytest.mark.parametrize("offset", [False, True], ids=["width", "laneOffset"])
@pytest.mark.parametrize("decoration", ["attribute", "child", "text"])
def test_editable_record_metadata_is_not_erased_by_scope_masking(offset, decoration):
    baseline = _document()
    _profile(baseline, -2, [(4.0, 1.0, 0.5)], offset=offset)
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(4.0, 1.0, 0.25)], offset=offset)
    record = candidate.find("road/lanes/laneOffset") if offset else _lane(candidate, -2).find("width")
    if decoration == "attribute":
        record.set("sourceNote", "must-not-disappear")
    elif decoration == "child":
        ET.SubElement(record, "userData", code="source-extension", value="must-not-disappear")
    else:
        record.text = "must-not-disappear"
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data, ids=() if offset else (-2,), offset=offset))
    assert report["status"] == "UNAVAILABLE", report["issues"]


@pytest.mark.parametrize("mutation", ["baseline-hash", "missing-domain", "missing-cells", "gap", "nan", "boolean"])
def test_incomplete_or_tampered_comparison_contract_is_unavailable(mutation):
    baseline = _document()
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(8.0, 1.0, 0.1)])
    data = _bytes(baseline)
    contract = _contract(data)
    if mutation == "baseline-hash":
        contract["baseline_sha256"] = "0" * 64
    elif mutation == "missing-domain":
        del contract["domain"]
    elif mutation == "missing-cells":
        del contract["comparison_stations_m"]
    elif mutation == "gap":
        contract["comparison_stations_m"][0] = 3.0
    elif mutation == "nan":
        contract["comparison_stations_m"][1] = float("nan")
    else:
        contract["allow_lane_offset"] = 1
    assert compare(data, _bytes(candidate), contract)["status"] == "UNAVAILABLE"


def test_nonfinite_contract_still_produces_a_standard_json_unavailable_receipt():
    data = _bytes(_document())
    contract = _contract(data)
    contract["comparison_stations_m"][1] = float("nan")
    report = compare(data, data, contract)
    assert report["status"] == "UNAVAILABLE"
    assert json.loads(json.dumps(report, allow_nan=False))["status"] == "UNAVAILABLE"


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "not-a-number"])
def test_nonfinite_written_polynomial_never_becomes_no_regression(value):
    baseline = _document()
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(8.0, 1.0, 0.1)])
    _lane(candidate, -2).findall("width")[1].set("d", value)
    data = _bytes(baseline)
    assert compare(data, _bytes(candidate), _contract(data))["status"] == "UNAVAILABLE"


def test_one_ulp_wide_changed_record_is_not_merged_away():
    baseline = _document()
    candidate = copy.deepcopy(baseline)
    lane = _lane(candidate, -2)
    first, second = 8.0, float(np.nextafter(8.0, math.inf))
    for w in list(lane.findall("width")):
        lane.remove(w)
    for start, width in ((0.0, 3.0), (first, 3.1), (second, 3.0)):
        _record(lane, "width", "sOffset", start, [width, 0, 0, 0])
    # Keep scope structure identical apart from allowed width records.
    widths = lane.findall("width")
    for w in widths:
        lane.remove(w)
    for i, w in enumerate(widths):
        lane.insert(i, w)
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "REGRESSION", report["issues"]
    assert first in report["computation_partition"] and second in report["computation_partition"]
    assert any(i["code"] == "joint-y-regression" for i in report["issues"])


def test_straight_parallel_tracks_have_zero_curvature_and_rate():
    report = analyze_road(_document().find("road"), (2.0, 30.0))
    for lane in report["lanes"]:
        for curve in lane["curves"].values():
            assert curve["kappa"]["max_abs"]["value"] == 0.0
            assert curve["rate"]["max_abs"]["value"] == 0.0
            assert curve["sign_sequence"] == []
            assert curve["sign_change_count"] == 0


def test_parallel_arc_radius_and_lane_arclength_rate_match_geometry():
    k = 0.02
    report = analyze_road(_document(curvature=k).find("road"), (2.0, 30.0))
    # Right -2 outer edge has t=-6 and radius 50+6, center has t=-4.5.
    for role, t in (("inner", -3.0), ("center", -4.5), ("outer", -6.0)):
        curve = _curve(report, -2, role)
        assert curve["kappa"]["max_abs"]["value"] == pytest.approx(1 / (50 - t), abs=1e-14)
        assert curve["rate"]["max_abs"]["value"] == 0.0
        assert curve["sign_sequence"] == [1]


def test_analytic_rate_uses_lane_arclength_not_reference_station():
    root = _document(length=2.0)
    root.find("road/lanes/laneOffset").set("d", "0.125")
    report = analyze_road(root.find("road"), (0.0, 1.0))
    jet = _curve(report, 0, "center")["pieces"][-1]["end_jet"]
    # r(s)=(s, alpha*s^3). Independent differentiation gives these values.
    alpha = 0.125
    expected_curvature = 6*alpha / (1+9*alpha**2)**1.5
    expected_rate = 6*alpha*(1-45*alpha**2) / (1+9*alpha**2)**3
    assert jet["kappa"] == pytest.approx(expected_curvature, abs=1e-13)
    assert jet["rate"] == pytest.approx(expected_rate, abs=1e-13)
    assert jet["rate"] != pytest.approx(expected_rate * math.sqrt(1+9*alpha**2), abs=1e-4)


def test_tiny_nonzero_arc_is_not_rounded_to_a_straight_curve():
    report = analyze_road(_document(curvature=1e-12).find("road"), (2.0, 30.0))
    curve = _curve(report, 0, "center")
    assert curve["kappa"]["max_abs"]["value"] == pytest.approx(1e-12, rel=1e-12, abs=0)
    assert curve["sign_sequence"] == [1]


def test_positive_width_endpoints_do_not_hide_an_interior_negative_width():
    baseline = _document()
    candidate = copy.deepcopy(baseline)
    lane = _lane(candidate, -2)
    for width in list(lane.findall("width")):
        lane.remove(width)
    for station, coeffs in ((0.0, [3, 0, 0, 0]), (8.0, [0.75, -4, 4, 0]), (9.0, [3, 0, 0, 0])):
        _record(lane, "width", "sOffset", station, coeffs)
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "UNAVAILABLE"
    assert "negative-width" in report["issues"][0]["reason"]


def test_two_curvature_reversals_between_coarse_samples_are_detected():
    root = _document()
    # A 0.125m compact bump lives entirely between coarse stations 8 and 8.25.
    _profile(root, -2, [(8.0625, 0.03125, 1 / 65536)])
    report = analyze_road(root.find("road"), (8.0, 8.25))
    curve = _curve(report, -2, "outer")
    assert curve["sign_change_count"] == 2
    assert curve["sign_sequence"] == [-1, 1, -1]
    assert all(8.0625 < r["domain"][0] <= r["domain"][1] < 8.1875 for r in curve["reversals"])


def test_same_sign_touching_zero_at_written_join_is_not_a_reversal():
    root = _document(length=2.0)
    parent = root.find("road/lanes")
    parent.remove(parent.find("laneOffset"))
    # t'' is positive on both sides, approaches zero at s=1, and is C2 there:
    # t=(1-s)^3/8 for s<1 and t=(s-1)^3/8 for s>1. Binary-exact
    # coefficients make this an actual zero in the declared float model.
    _record(parent, "laneOffset", "s", 0.0, [1/8, -3/8, 3/8, -1/8])
    _record(parent, "laneOffset", "s", 1.0, [0, 0, 0, 1/8])
    report = analyze_road(root.find("road"), (0.0, 2.0))
    curve = _curve(report, 0, "center")
    assert curve["sign_sequence"] == [1]
    assert curve["sign_change_count"] == 0


def test_exact_double_root_is_accounted_once_not_as_two_crossings():
    from mapforge.workbench.local_shape_math import _roots
    brackets, zero = _roots(P([0.25, -1.0, 1.0]))
    assert zero is False
    assert len(brackets) == 1
    assert float(brackets[0][0]) <= 0.5 <= float(brackets[0][1])


def test_uncertain_near_multiple_root_is_not_silently_accepted():
    # The positive root is about 1e-100, far narrower than the declared fixed
    # isolation bracket. Its derivative cannot be certified away from zero.
    from mapforge.workbench.local_shape_math import _roots
    with pytest.raises(UnavailableError, match="conditioning"):
        _roots(P([-1e-200, 0.0, 1.0]))


def test_near_double_quadratic_with_proved_negative_discriminant_has_no_roots():
    from mapforge.workbench.local_shape_math import _roots
    a, b = (3 / 7)**2, -6 / 7
    # Exact float-coefficient discriminant distinguishes this from a touch.
    assert Fraction(b)**2 - 4*Fraction(a) < 0
    brackets, zero = _roots(P([a, b, 1.0]))
    assert not brackets
    assert zero is False


def test_global_root_station_bracket_encloses_exact_affine_mapping():
    from mapforge.workbench.local_shape_math import _station_bracket
    lo, length = 4652.779030379938, 72362.6275869074
    a = Fraction(462899688131140677019740545649787, 1298074214633706907132624082305024)
    b = Fraction(1851598752524562708078962182599149, 5192296858534827628530496329220096)
    lower, upper = _station_bracket(a, b, lo, length)
    # Rounding u, multiplying, adding, then growing by one ULP did not
    # enclose this root bracket. Test exact containment, not float equality.
    assert Fraction(lower) <= Fraction(lo) + Fraction(length)*a
    assert Fraction(upper) >= Fraction(lo) + Fraction(length)*b


def test_invalid_xml_refuses_comparison_without_a_positive_result():
    data = _bytes(_document())
    for candidate in (b"<OpenDRIVE>", b"<!DOCTYPE OpenDRIVE><OpenDRIVE/>", b"", None):
        assert compare(data, candidate, _contract(data))["status"] == "UNAVAILABLE"


def test_utf16_dtd_does_not_bypass_the_xml_input_contract():
    baseline = _document()
    _profile(baseline, -2, [(4.0, 1.0, 0.5)])
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(4.0, 1.0, 0.25)])
    document = ('<?xml version="1.0" encoding="utf-16"?>\n'
                '<!DOCTYPE OpenDRIVE [<!ENTITY fixture "unused">]>\n'
                + _bytes(candidate).decode("utf-8")).encode("utf-16")
    # A byte search for ASCII <!DOCTYPE misses this; ET itself can parse it.
    assert ET.fromstring(document).tag == "OpenDRIVE"
    assert b"<!DOCTYPE" not in document
    data = _bytes(baseline)
    report = compare(data, document, _contract(data))
    assert report["status"] == "UNAVAILABLE", report["issues"]


def test_utf8_bom_keeps_a_valid_actual_geometry_comparison_available():
    baseline = _document()
    _profile(baseline, -2, [(4.0, 1.0, 0.5)])
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(4.0, 1.0, 0.25)])
    data = b"\xef\xbb\xbf" + _bytes(baseline)
    report = compare(data, b"\xef\xbb\xbf" + _bytes(candidate), _contract(data))
    assert report["status"] == "NO_REGRESSION_FOUND", report["issues"]


@pytest.mark.parametrize("feature", ["elevation", "superelevation", "lane-height"])
def test_nonplanar_input_is_unavailable_even_if_the_3d_records_are_unchanged(feature):
    baseline = _document()
    road = baseline.find("road")
    if feature == "elevation":
        ET.SubElement(ET.SubElement(road, "elevationProfile"), "elevation", s="0", a="1", b="0", c="0", d="0")
    elif feature == "superelevation":
        ET.SubElement(ET.SubElement(road, "lateralProfile"), "superelevation", s="0", a="0.1", b="0", c="0", d="0")
    else:
        ET.SubElement(_lane(baseline, -2), "height", sOffset="0", inner="0", outer="0.1")
    _profile(baseline, -2, [(4.0, 1.0, 0.5)])
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(4.0, 1.0, 0.25)])
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "UNAVAILABLE", report["issues"]


def test_complete_identically_zero_elevation_is_a_supported_planar_input():
    baseline = _document()
    ET.SubElement(ET.SubElement(baseline.find("road"), "elevationProfile"),
                  "elevation", s="0", a="0", b="0", c="0", d="0")
    _profile(baseline, -2, [(4.0, 1.0, 0.5)])
    candidate = copy.deepcopy(baseline)
    _profile(candidate, -2, [(4.0, 1.0, 0.25)])
    data = _bytes(baseline)
    report = compare(data, _bytes(candidate), _contract(data))
    assert report["status"] == "NO_REGRESSION_FOUND", report["issues"]
