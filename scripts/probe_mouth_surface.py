"""无需重跑拟合器，按既有候选 XODR 复核入口尾带和路口物理面。"""
import argparse
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import mapping

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import shp_source
from mapforge.ops.junction_surface import source_tail_regions
from mapforge.ops.junction_surface import (replace_source_paving, written_mouth_specs,
                                         bridge_source_tails, source_median_tails)
from mapforge.adapters.opendrive import writer as W
from mapforge.validate.shp_boundary_fidelity import _origin, _project
from mapforge.validate.smoothness import sample_road_ref, road_surface_polygon
from mapforge.validate.smoothness import surface_continuity, audit_file
from mapforge.validate.smoothness import lane_edges_kinematics_at
from shapely.ops import unary_union
from shapely.geometry import Polygon


def inspect(path, output, patched_path=None):
    root = ET.parse(path).getroot()
    lat0, lon0 = _origin(root)
    proj = lambda p: _project(p, lat0, lon0)
    src = shp_source()
    junc, distance = src.find_junction(lon0, lat0)
    assert distance < 50
    probe = json.loads(path.with_suffix(".probe.json").read_text(encoding="utf-8"))
    mouths = written_mouth_specs(root, probe["stats"]["mouth_envelope_decisions"])
    base, pieces, union, report = source_tail_regions(src, junc, proj, mouths)
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), dpi=150)
    x, y = base.exterior.xy
    for ax in axes:
        ax.fill(x, y, color="#cccccc", label="source INTERSECTION")
    for p in pieces:
        geoms = list(p["geometry"].geoms) if p["geometry"].geom_type == "MultiPolygon" else [p["geometry"]]
        for pg in geoms:
            x, y = pg.exterior.xy
            axes[0].fill(x, y, color="#0088bb", alpha=.2)
            axes[0].plot(x, y, color="#0088bb", lw=.6)
    parts = list(union.geoms) if union.geom_type == "MultiPolygon" else [union]
    for i, p in enumerate(parts):
        x, y = p.exterior.xy
        axes[1].plot(x, y, lw=1.2)
        pt = p.representative_point()
        axes[1].text(pt.x, pt.y, str(i), fontsize=8)
    for spec in mouths:
        x, y, h = spec["pose"]
        axes[0].plot([x-20*np.sin(h), x+20*np.sin(h)],
                     [y+20*np.cos(h), y-20*np.cos(h)], "r--", lw=.8)
    for ax in axes:
        ax.set_aspect("equal")
        ax.set_xlabel("local x (m)")
        ax.set_ylabel("local y (m)")
        ax.grid(alpha=.2)
    axes[0].set_title("Measured lane tails / candidate mouth planes")
    axes[1].set_title(f"Union: {len(parts)} components (no hull, no hole filling)")
    fig.suptitle(path.parent.name + " / " + path.stem)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output)
    plt.close(fig)
    report["pieces"] = [{k:v for k,v in p.items() if k != "geometry"} for p in pieces]
    output.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k != "pieces"}))
    print(output)
    if patched_path is not None:
        from mapforge.report.decision import finalize_opendrive_g8
        from mapforge.validate.g11 import audit_file as g11_audit, load_policy
        from lxml import etree
        from scripts.gen_all import POLICY
        st = replace_source_paving(root, src, junc, proj, probe["stats"]["mouth_envelope_decisions"])
        tails, _ = bridge_source_tails(base, pieces, mouths)
        medians, median_issues = source_median_tails(base, tails, mouths)
        st["median_tail_issues"] = median_issues
        st["median_tail_areas_m2"] = [p["geometry"].area for p in medians]
        patched_path.parent.mkdir(parents=True, exist_ok=True)
        ET.indent(root)
        ET.ElementTree(root).write(patched_path, encoding="utf-8", xml_declaration=True)
        manifest = json.loads(path.with_suffix(".source-lanes.json").read_text(encoding="utf-8"))
        final = finalize_opendrive_g8(patched_path, manifest, POLICY, connect_mode="data")
        schema = etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))
        xsd_ok = schema.validate(etree.parse(str(patched_path)))
        g11 = g11_audit(patched_path, load_policy(ROOT / "profiles/validation/g11-opendrive-v1.draft.yaml"))
        result = {**probe, "stats":{**probe["stats"], **st}, "surface":surface_continuity(root),
                  "xsd": xsd_ok, "g8":final["gate"]["status"], "g11":g11,
                  "smoothness":audit_file(patched_path), "patched_from": str(path)}
        patched_path.with_suffix(".probe.json").write_text(json.dumps(result, ensure_ascii=False,
                 indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps({"patched":str(patched_path), "xsd":xsd_ok,
                          "g8":result["g8"], "g11":g11["status"], "surface":result["surface"],
                          "bridge_issues":st["mouth_tail_bridge_issues"]}))
        # 独立复算最终 XODR 路面，标明真实缺口与有依据的分隔带。
        generated = unary_union([road_surface_polygon(rd) for rd in root.findall("road")
                                if rd.get("name") == "junction_paving"])
        target = unary_union([base, *[t["geometry"] for t in tails]])
        fig, ax = plt.subplots(figsize=(10, 9), dpi=170)
        for p in getattr(generated, "geoms", [generated]):
            ax.fill(*p.exterior.xy, color="#dddddd", alpha=.8)
            for ring in p.interiors:
                hole = Polygon(ring)
                ax.fill(*hole.exterior.xy, color="red")
                print("hole", hole.area, hole.bounds, "supported", hole.intersection(target).area)
        for p in getattr(target, "geoms", [target]):
            ax.plot(*p.exterior.xy, color="#ff8800", lw=.8)
        for p in medians:
            ax.fill(*p["geometry"].exterior.xy, color="#448866")
        ax.set_aspect("equal"); ax.grid(alpha=.2)
        ax.set(xlabel="local x (m)", ylabel="local y (m)",
               title="Written paving (gray), source-supported outline (orange), median candidate (green)")
        fig.tight_layout()
        fig.savefig(output.with_name(output.stem+"-written.png"))
        plt.close(fig)
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("xodr", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--patched-xodr", type=Path)
    args = p.parse_args()
    inspect(args.xodr, args.output, args.patched_xodr)
