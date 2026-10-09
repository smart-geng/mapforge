"""End-to-end check that scoreboard.evaluate reads candidate.surface-evidence.json next to the XODR, the way the
workbench single-candidate check now lays out its bundle. Works on copies of the 0621 run; esmini is skipped on Linux,
so only the paving-hole figures are compared."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT))
from mapforge.validate import scoreboard as sb  # noqa: E402

W0621 = ROOT / "out/workbench/wb11-source-tracks-20261009-v2"
KEYS = ("paving_holes_gt1cm2", "paving_holes_resampled", "paving_holes_source_void", "paving_holes_counted",
        "paving_hole_counted_area_max_m2", "paving_source_void_area_m2", "paving_source_void_evidence")


def run(name, with_evidence):
    d = OUT / name
    shutil.rmtree(d, ignore_errors=True)
    d.mkdir()
    for p in W0621.glob("candidate.*"):
        shutil.copyfile(p, d / p.name)
    if with_evidence:
        shutil.copyfile(W0621 / "surface-evidence.json", d / "candidate.surface-evidence.json")
    m = sb.evaluate(d / "candidate.xodr", "shp")
    return {k: m.get(k) for k in KEYS}


def main():
    result = {"with_evidence": run("e2e-with-evidence", True), "without_evidence": run("e2e-without-evidence", False)}
    print(json.dumps(result, ensure_ascii=False, indent=1))
    (OUT / "sidecar_check.json").write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf8")


if __name__ == "__main__":
    main()
