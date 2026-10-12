"""surface_seams (2026-10-12): junction paving retried only where envelope_surface fails, for seams the source explains.
Synthetic geometry in a local metric frame, mouth at the origin heading +x."""
import hashlib
from pathlib import Path

import numpy as np
import pytest
import yaml
from shapely.geometry import LineString, Polygon, box
from shapely.ops import unary_union

from mapforge.ops import envelope_surface as E
from mapforge.ops import surface_seams as SS

ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------- road-surface seam (0630 road 30 shape)
# a tail whose 1.5 m sweep reaches the junction polygon above and below a shallow notch but not inside it
NOTCHED = Polygon([(4.2, -5), (20, -5), (20, 5), (4.2, 5), (4.2, 1.5), (4.62, 0), (4.2, -1.5)])
TAIL = box(0, -2.5, 3, 2.5)
AHEAD = np.array([1.0, 0.0])
SURFACE = box(4.0, -2.6, 4.9, 2.6)


def _seam(rs, others=()):
    return SS.bridge_one(NOTCHED, NOTCHED.buffer(E.COVER_TOL_M), TAIL, AHEAD, rs, [("rs1", rs)], list(others))


def test_the_builder_itself_rejects_the_notch_as_an_enclosed_gap():
    result, issue = E._bridge_one(NOTCHED, NOTCHED.buffer(E.COVER_TOL_M), TAIL, AHEAD)
    assert result is None and issue["reason"] == "end-cap-encloses-gap"


def test_a_seam_the_source_road_surface_covers_is_paved_and_inferred():
    result, issue = _seam(SURFACE)
    assert issue is None
    assert result["geometry"].geom_type == "Polygon" and not result["geometry"].interiors
    assert result["sweep_m"] == 1.5                                  # the sweep budget is not raised
    assert result["status"] == "INFERRED" and result["support_kind"] == "source-road-surface-seam"
    assert result["road_surface_seam"][0]["road_surface_objects"] == ["rs1"]


def test_a_seam_without_road_surface_stays_a_failure():
    result, issue = _seam(box(10, -1, 11, 1))
    assert result is None and issue["reason"] == "end-cap-encloses-gap"
    assert issue["road_surface_seam"][0]["road_surface_uncovered_m2"] > E.SLIVER_M2


def test_a_seam_another_legs_tail_overlaps_stays_a_failure():
    result, issue = _seam(SURFACE, [box(4.3, -0.5, 4.7, 0.5)])
    assert result is None and issue["reason"] == "end-cap-encloses-gap"


# ---------------------------------------------------------------- median takeover (0621 road 10 shape)
SPEC = {"road_id": 10, "pose": (0.0, 0.0, 0.0), "median_interval": (-0.5, 0.5)}
RIGHT = box(-1, -6.5, 15, -0.5)             # tails running on to the junction polygon at s = 15
LEFT = box(-1, 0.5, 4, 6.5)                 # tails ending at s = 4 inside the junction area
JUNCTION = unary_union([box(4, 0.5, 25, 12), box(15, -12, 25, 12)])
TAILS = [{"road_id": 10, "geometry": RIGHT}, {"road_id": 10, "geometry": LEFT}]
PAVED = unary_union([RIGHT, LEFT, JUNCTION])   # road surface everywhere but the separator
BARRIER = [("b1", LineString([(1, 0), (14, 0)]))]


def _takeover(rs, base=JUNCTION, spec=SPEC):
    return SS.median_takeover(base, base.buffer(E.COVER_TOL_M), TAILS, spec, rs, BARRIER)


def test_a_median_continued_along_the_junction_polygon_is_a_median_island():
    island, evidence = _takeover(PAVED)
    assert island is not None and abs(island.area - 15.5) < 1e-6
    assert evidence["status"] == "INFERRED" and evidence["closed_at_s"] == 15.0
    lo, hi = evidence["takeover_s_range"]
    assert 4.0 < lo < 4.5 and 14.5 < hi < 15.0                      # the polygon took over where the tail ended
    assert evidence["takeover_width_m"] == [1.0, 1.0]
    assert evidence["barriers"] == [{"object": "b1", "length_inside_m": 13.0}]


def test_a_strip_that_is_road_surface_is_not_a_median():
    island, evidence = _takeover(unary_union([PAVED, box(-1, -0.5, 15, 0.5)]))
    assert island is None and evidence["reason"] == "median-takeover-is-road-surface"


def test_without_road_surface_data_there_is_no_takeover():
    island, evidence = _takeover(Polygon())
    assert island is None and evidence["reason"] == "median-takeover-no-road-surface-data"


def test_a_strip_that_jumps_wider_than_the_median_is_not_taken_over():
    wide = unary_union([box(4, 1.5, 25, 12), box(15, -12, 25, 12)])     # polygon edge 1 m further out
    island, evidence = _takeover(unary_union([RIGHT, LEFT, wide]), base=wide)
    assert island is None


# ---------------------------------------------------------------- the object layers come from the Profile's companion
SURFACE_SPEC = {"file": "IBD_OBJECT_ROAD_SURFACE", "fields": {"id": "OBJECT_PID"}}


class _Src:
    def __init__(self, layers, path=None):
        self.L = layers
        self.p = {"_path": str(path)} if path else {}


def _src(tmp_path, companion=None, own=None, name="p"):
    """A source whose Profile file is tmp_path/<name>.yaml, with an optional companion next to it."""
    profile = tmp_path / f"{name}.yaml"
    profile.write_text("layers: {}\n", encoding="utf-8")
    if companion is not None:
        (tmp_path / f"{name}.objects.yaml").write_text(yaml.safe_dump(companion), encoding="utf-8")
    return _Src(own or {}, profile)


def test_the_companion_declares_the_object_layers(tmp_path):
    src = _src(tmp_path, {"extends": "p", "layers": {"road_surface": SURFACE_SPEC}})
    assert SS.object_layers(src) == {"road_surface": SURFACE_SPEC} and SS.has_road_surface(src)


def test_without_a_companion_there_is_no_road_surface(tmp_path):
    assert not SS.has_road_surface(_src(tmp_path)) and not SS.has_road_surface(_Src({}))


def test_a_companion_for_another_profile_is_refused(tmp_path):
    with pytest.raises(ValueError, match="extends"):
        SS.object_layers(_src(tmp_path, {"extends": "other", "layers": {"road_surface": SURFACE_SPEC}}))


def test_the_profiles_own_declaration_wins(tmp_path):
    own = {"file": "OWN_SURFACE", "fields": {"id": "PID"}}
    src = _src(tmp_path, {"extends": "p", "layers": {"road_surface": SURFACE_SPEC}}, own={"road_surface": own})
    assert SS.object_layers(src)["road_surface"] == own


def test_a_layer_without_an_id_field_is_refused(tmp_path):
    with pytest.raises(ValueError, match="fields.id"):
        SS.object_layers(_src(tmp_path, {"extends": "p", "layers": {"road_surface": {"file": "X"}}}))


def test_the_v1_profile_is_untouched_and_its_companion_declares_the_two_layers():
    from mapforge.workbench import compiler
    profile = ROOT / "profiles/shp/ibd-smarteditor-v1.yaml"
    # the workbench registers its capabilities on the v1 Profile's bytes
    assert hashlib.sha256(profile.read_bytes()).hexdigest() == compiler.PROFILE_SHA256
    assert SS.ROAD_SURFACE not in yaml.safe_load(profile.read_text(encoding="utf-8-sig"))["layers"]
    companion = yaml.safe_load((ROOT / "profiles/shp/ibd-smarteditor-v1.objects.yaml").read_text(encoding="utf-8"))
    assert companion["extends"] == "ibd-smarteditor-v1"
    assert companion["layers"][SS.ROAD_SURFACE] == SURFACE_SPEC
    assert companion["layers"][SS.TRAFFIC_BARRIER] == {"file": "IBD_OBJECT_TRAFFIC_BARRIER", "fields": {"id": "OBJECT_PID"}}


@pytest.mark.skipif(not (ROOT / "shp_0222-0326" / "IBD_OBJECT_ROAD_SURFACE.shp").exists(), reason="source data absent")
def test_the_companion_layers_read_from_the_source():
    from mapforge.adapters.shp.profile_source import ProfileSource
    src = ProfileSource(str(ROOT / "shp_0222-0326"), "ibd-smarteditor-v1")
    for key, spec in SS.object_layers(src).items():
        record, shp = next(src._iter(spec))
        assert record[spec["fields"]["id"]] and shp.points, key


# ---------------------------------------------------------------- the fallback only engages where the builder fails
class _Doc:
    def __init__(self):
        self.roads = ["kept"]


@pytest.fixture
def with_surface(tmp_path):
    return _src(tmp_path, {"extends": "p", "layers": {"road_surface": SURFACE_SPEC}})


@pytest.fixture
def without_surface(tmp_path):
    return _src(tmp_path)


def _patch(monkeypatch, builder, rules):
    calls = {"rules": 0}

    def fake_rules(doc, *a, **k):
        calls["rules"] += 1
        return rules(doc)
    monkeypatch.setattr(SS.E, "append_source_paving", builder)
    monkeypatch.setattr(SS, "_append_with_rules", fake_rules)
    return calls


def _ok_stats():
    return {"mouth_tail_source_issues": [], "mouth_tail_bridge_issues": [], "median_tail_issues": []}


def test_a_successful_build_is_returned_untouched(monkeypatch, with_surface):
    stats = _ok_stats()

    def builder(doc, *a, **k):
        doc.roads.append("paving")
        return stats
    calls = _patch(monkeypatch, builder, lambda doc: pytest.fail("no retry expected"))
    doc = _Doc()
    assert SS.append_source_paving(doc, with_surface, None, None, [], []) is stats
    assert doc.roads == ["kept", "paving"] and calls["rules"] == 0


def test_without_the_road_surface_layer_the_original_failure_is_raised(monkeypatch, without_surface):
    def builder(doc, *a, **k):
        raise ValueError("unresolved gap inside source surface family: road=10")
    calls = _patch(monkeypatch, builder, lambda doc: pytest.fail("no retry expected"))
    with pytest.raises(ValueError, match="unresolved gap"):
        SS.append_source_paving(_Doc(), without_surface, None, None, [], [])
    assert calls["rules"] == 0


def test_a_retry_that_explains_the_seam_replaces_the_failed_build(monkeypatch, with_surface):
    def builder(doc, *a, **k):
        doc.roads.append("partial")
        raise ValueError("unresolved gap inside source surface family: road=10")

    def rules(doc):
        doc.roads.append("rebuilt")
        return {**_ok_stats(), "seam_rules": [{"rule": "median-takeover"}]}
    _patch(monkeypatch, builder, rules)
    doc = _Doc()
    stats = SS.append_source_paving(doc, with_surface, None, None, [], [])
    assert stats["surface_builder_fallback"] == SS.CODE
    assert doc.roads == ["kept", "rebuilt"]                        # the failed build's roads are gone


def test_a_retry_that_explains_nothing_leaves_the_original_failure(monkeypatch, with_surface):
    def builder(doc, *a, **k):
        doc.roads.append("partial")
        raise ValueError("unresolved gap inside source surface family: road=10")

    def rules(doc):
        doc.roads.append("retry")
        return {**_ok_stats(), "seam_rules": []}
    _patch(monkeypatch, builder, rules)
    doc = _Doc()
    with pytest.raises(ValueError, match="unresolved gap"):
        SS.append_source_paving(doc, with_surface, None, None, [], [])
    assert doc.roads == ["kept"]
