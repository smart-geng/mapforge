"""MAP road-side refit on a real junction (map-node13): fidelity, mouths kept, mirror exact, never crossing."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
from lxml import etree

from mapforge.ops import lane_refit as LR
from mapforge.ops import map_lane_refit as M
from mapforge.validate.lane_centre_poly import audit as poly_audit
from mapforge.validate.smoothness import lane_edges_at

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "v2x_map_xml" / "map凤阁路-金玥路node13.xml"


@pytest.fixture(scope="module")
def raw(tmp_path_factory):
    if not SOURCE.exists():
        pytest.skip("local MAP sample not installed")
    out = tmp_path_factory.mktemp("mapfit") / "map-node13.xodr"
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    subprocess.run([sys.executable, "-m", "mapforge.cli", "convert", str(SOURCE), "--to", "xodr", "--post", "none",
                    "-o", str(out)], cwd=str(ROOT), env=env, capture_output=True)
    assert out.exists() and out.with_suffix(".source-lanes.json").exists()
    return out


def _ordinary(root):
    return [r for r in root.findall("road") if r.get("junction") in (None, "-1") and r.get("name") != "junction_paving"]


@pytest.mark.slow
def test_map_refit_puts_lanes_on_the_point_lists_and_keeps_mouths_and_mirror(raw, tmp_path):
    manifest = json.loads(raw.with_suffix(".source-lanes.json").read_text(encoding="utf-8"))
    before = etree.parse(str(raw)).getroot()
    out = tmp_path / "map-node13.xodr"
    out.with_suffix(".source-lanes.json").write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
    raw_copy = tmp_path / "map-node13.raw.xodr"
    raw_copy.write_bytes(raw.read_bytes())
    raw_copy.with_suffix(".source-lanes.json").write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
    params = dict(LR.VARIANTS["g2-k04-c2"])
    for key in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
                "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
        params.pop(key)
    report = LR.apply(raw_copy, out, params)
    assert report["rewritten"] == len(_ordinary(before)) and not report["skipped"]
    after = etree.parse(str(out)).getroot()

    old, new = poly_audit(before, manifest), poly_audit(after, manifest)
    assert new["lane_center_poly_p95_m"] < 0.06 < old["lane_center_poly_p95_m"]
    assert new["lane_center_poly_max_m"] < 0.1

    for road_old, road_new in zip(_ordinary(before), _ordinary(after)):
        assert road_new.get("id") == road_old.get("id")
        # junction mouths stay where the converter put them (the connectors start there)
        link = road_new.find("link")
        if link is not None and link.find("successor[@elementType='junction']") is not None:
            l0, l1 = float(road_old.get("length")), float(road_new.get("length"))
            for side in ("right", "left"):
                assert lane_edges_at(road_new, l1, side) == pytest.approx(lane_edges_at(road_old, l0, side), abs=1e-3)
        rec = json.loads(road_new.find(f"userData[@code='{M.CODE}']").get("value"))
        assert rec["min_width_m"] >= -LR.NEGATIVE_WIDTH_TOL
        # the departure lanes the converter mirrored still have exactly the (refitted) approach widths
        mirrored = M.mirrored_lanes(road_old)
        assert mirrored and rec["mirrored_lanes"] == len(mirrored)
        secs = road_new.findall("lanes/laneSection")
        for i, lid in mirrored:
            lane = secs[i].find(f"left/lane[@id='{lid}']")
            twin = secs[i].find(f"right/lane[@id='-{lid}']")
            assert M._widths(lane) == M._widths(twin)
    assert etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd"))).validate(etree.parse(str(out)))
