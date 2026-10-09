"""End-to-end check that scoreboard.evaluate reports the stop-line figures for a stored conversion, building the
SHP source itself when none is passed. Works on copies of the 0621 run (with its bound surface evidence) and of the
generalization case 0412; esmini is skipped on Linux, so only the G8 / stop-line / paving figures are compared."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT))
from mapforge.validate import scoreboard as sb  # noqa: E402

KEYS = ("g8_status", "g8_status_stopline_resolved", "stopline_matched", "stopline_source_absent",
        "stopline_match_distance_max_m", "stopline_matched_delta_max_m", "g8_stopline_requirements_left_out",
        "paving_holes_counted")
CASES = {"0621": (ROOT / "out/workbench/wb11-source-tracks-20261009-v2", "candidate", True),
         "0412": (ROOT / "out/generalize/20261008-safe-mouth-recovery-v3", "shp-2023041216222042093", False)}


def main():
    result = {}
    for name, (directory, stem, evidence) in CASES.items():
        d = OUT / f"e2e-{name}"
        shutil.rmtree(d, ignore_errors=True)
        d.mkdir()
        for p in directory.glob(stem + ".*"):
            shutil.copyfile(p, d / ("candidate" + p.name[len(stem):]))
        if evidence:
            shutil.copyfile(directory / "surface-evidence.json", d / "candidate.surface-evidence.json")
        m = sb.evaluate(d / "candidate.xodr", "shp")
        result[name] = {k: m.get(k) for k in KEYS}
    print(json.dumps(result, ensure_ascii=False, indent=1))
    (OUT / "scoreboard_check.json").write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf8")


if __name__ == "__main__":
    main()
