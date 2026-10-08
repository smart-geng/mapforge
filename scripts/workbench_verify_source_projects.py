"""Verify actual failed/successful junction sources can persist without XODR."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import yaml
from mapforge.workbench.sources import SourceCatalog, verify_source_snapshot
from mapforge.workbench.store import ProjectStore


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("use a new output directory; previous evidence is retained")
    args.out.mkdir(parents=True)
    source = ROOT / "shp_0222-0326"
    profile = ROOT / "profiles/shp/ibd-smarteditor-v1.yaml"
    catalog = SourceCatalog(source, profile)
    cases = yaml.safe_load((ROOT / "profiles/validation/generalization-set-v1.yaml").read_text(encoding="utf-8"))
    registered = ROOT / "out/generalize/20261008-safe-mouth-recovery-v3/generation.json"
    prior = {r["pid"]: r for r in json.loads(registered.read_text(encoding="utf-8"))}
    store = ProjectStore(args.out / "projects")
    records = []
    for entry in cases["junctions"]:
        pid = entry["pid"]
        snapshot = catalog.snapshot(pid)
        project = store.create(snapshot, f"{pid} 源工程复验")
        target = next(o for o in snapshot["objects"] if o["points"])
        command = {"command_id": "source-acceptance-note", "type": "annotation",
                   "source_ref": target["id"], "scope": {"feature_ids": [target["id"]]},
                   "parameters": {"text": "源工程保存复验；未运行转换或修补几何", "status": "unresolved"}}
        after = store.commit(project["project_id"], project["revision"], command)
        reopened = ProjectStore(args.out / "projects").load(project["project_id"])
        assert reopened["content_hash"] == after["content_hash"] and len(reopened["intents"]) == 1
        assert reopened["source_snapshot"] == snapshot and reopened["candidate"] is None
        records.append({"junction_id": pid, "objects": len(snapshot["objects"]),
                        "prior_generated": prior[pid]["generated"], "project_id": project["project_id"],
                        "draft_sha256": after["content_hash"], "reopened": True, "xodr_required": False,
                        "formal_export_available": reopened["status"]["formal_export_available"]})
    integrity = verify_source_snapshot(snapshot, source, profile)
    assert integrity["matches"], integrity
    report = {"schema": "mapforge/source-project-verification/v1", "created_at": datetime.now(timezone.utc).isoformat(),
              "source_snapshot_id": catalog.snapshot_id, "source_file_count": len(catalog.files),
              "source_bytes": sum(f["size"] for f in catalog.files), "source_integrity": integrity,
              "registered_generation_sha256": hashlib.sha256(registered.read_bytes()).hexdigest(),
              "cases": records, "opened": len(records),
              "prior_generation_failed_opened": sum(not r["prior_generated"] for r in records),
              "interpretation": "Source/project persistence only; not geometry or product acceptance"}
    (args.out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"opened": report["opened"], "prior_generation_failed_opened": report["prior_generation_failed_opened"],
                      "source_integrity": integrity, "out": str(args.out)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
