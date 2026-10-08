"""Capture/verify immutable pre-workbench code and existing conversion artifacts."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def file_hash(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--verify", type=Path)
    args = parser.parse_args()
    if bool(args.out) == bool(args.verify):
        parser.error("choose exactly one of --out / --verify")
    if args.verify:
        record = json.loads(args.verify.read_text(encoding="utf-8"))
        changed = [r for r in record["files"] if not (ROOT / r["path"]).is_file()
                   or file_hash(ROOT / r["path"]) != r["sha256"]]
        print(json.dumps({"files": len(record["files"]), "changed": [r["path"] for r in changed]},
                         ensure_ascii=False))
        return int(bool(changed))
    tracked = subprocess.check_output(["git", "ls-files", "-z", "mapforge", "scripts", "tests",
                                       "profiles", "pyproject.toml", "uv.lock", "AGENTS.md", "CLAUDE.md"],
                                      cwd=ROOT).decode("utf-8").split("\0")
    paths = {ROOT / p for p in tracked if p and (ROOT / p).is_file()}
    for base in (ROOT / "out/scoreboard/20261008-safe-mouth-recovery-v3",
                 ROOT / "out/generalize/20261008-safe-mouth-recovery-v3"):
        if base.is_dir():
            for path in base.iterdir():
                if path.is_file() and (path.suffix == ".xodr" or path.name.endswith(
                        ("source.json", "source-lanes.json", "scoreboard.json", "generalization.json",
                         "generation.json", "control.json", "delivery-decision.json"))):
                    paths.add(path)
    record = {
        "schema": "mapforge/workbench-baseline/v1", "created_at": datetime.now(timezone.utc).isoformat(),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT).decode().strip(),
        "scope": "Existing tracked code/profiles and selected registered output artifacts; new workbench code excluded",
        "files": [{"path": p.relative_to(ROOT).as_posix(), "size": p.stat().st_size, "sha256": file_hash(p)}
                  for p in sorted(paths)],
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8") as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({"path": str(args.out), "files": len(record["files"]), "git_head": record["git_head"]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
